// MQTT plumbing: subscribes to the configured topics, dispatches inbound
// messages through the pure parsers in mqtt_parsers.c, and pushes parsed
// results onto FreeRTOS queues for the rest of the system to consume.
//
// Action publication is done out of `mqtt_out_queue`, which the inference
// task fills.

#include "mqtt_handler.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "sdkconfig.h"

#include "led_blink.h"
#include "messages.h"
#include "mqtt_parsers.h"

extern char *TAG;

static esp_mqtt_client_handle_t s_client;

static uint8_t s_mqtt_led_gpio = CONFIG_BLINK_GPIO + 1;
static TaskHandle_t s_mqtt_led_task;

QueueHandle_t action_queue;
QueueHandle_t mocap_queue;
QueueHandle_t raw_observations_queue;
QueueHandle_t mqtt_out_queue;
QueueHandle_t settings_queue;

static SettingsData s_settings_data = {.header = 0x00, .is_running = false};
static MocapDataWithRotation s_mocap_state = {0};

static void log_error_if_nonzero(const char *message, int error_code) {
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void handle_action_payload(const char *data) {
    char cmd = '\0';
    float channels[16];
    int n = parse_action_string(data, &cmd, channels);
    (void)n;
    // The inference firmware itself only PUBLISHES actions; receiving an
    // action on the wire (e.g. from another agent) is currently a no-op but
    // we still drain the payload so it doesn't pile up in MQTT buffers.
    (void)cmd;
}

static void handle_settings_payload(const char *data) {
    ParsedSettings parsed;
    if (!parse_settings_payload(data, &parsed)) {
        return;
    }

    switch (parsed.cmd) {
        case SETTINGS_CMD_RUN:
            s_settings_data.is_running = true;
            break;
        case SETTINGS_CMD_STOP:
            s_settings_data.is_running = false;
            break;
        case SETTINGS_CMD_MODEL:
            strncpy(s_settings_data.model, parsed.model_name,
                    sizeof(s_settings_data.model) - 1);
            s_settings_data.model[sizeof(s_settings_data.model) - 1] = '\0';
            break;
        case SETTINGS_CMD_SET_PIVOT:
            s_mocap_state.theta = parsed.theta_rad;
            s_mocap_state.pivot_x = s_mocap_state.no_rotation_x;
            s_mocap_state.pivot_y = s_mocap_state.no_rotation_y;
            ESP_LOGI(TAG, "Setting theta=%.4f, pivot=(%d, %d)",
                     s_mocap_state.theta,
                     s_mocap_state.pivot_x, s_mocap_state.pivot_y);
            xQueueOverwrite(mocap_queue, &s_mocap_state);
            break;
        default:
            return;
    }
    xQueueOverwrite(settings_queue, &s_settings_data);
}

static void handle_observations_payload(const char *data, int len) {
    if (len > 0 && (uint8_t)data[0] == BIN_OBSERVATIONS_HEADER) {
        StateObservations obs;
        if (parse_bin_observations(data, (size_t)len, &obs)) {
            xQueueOverwrite(raw_observations_queue, &obs);
        }
    } else {
        StateObservations obs;
        if (parse_observations_string(data, &obs)) {
            xQueueOverwrite(raw_observations_queue, &obs);
        }
    }
}

static void handle_mocap_payload(const char *data, int len) {
    MocapDataWithRotation next;
    if (parse_bin_mocap(data, (size_t)len, &s_mocap_state, &next)) {
        s_mocap_state = next;
        xQueueOverwrite(mocap_queue, &s_mocap_state);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event base=%s, id=%ld", base, event_id);
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            xTaskCreate(blink_led, "MQTT LED TASK", 4096, &s_mqtt_led_gpio, 1,
                        &s_mqtt_led_task);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGE(TAG, "MQTT_EVENT_DISCONNECTED");
            if (s_mqtt_led_task) {
                vTaskDelete(s_mqtt_led_task);
                s_mqtt_led_task = NULL;
            }
            gpio_set_level(s_mqtt_led_gpio, 0);
            break;

        case MQTT_EVENT_DATA: {
            // event->topic / event->data are NOT null-terminated, copy
            // defensively before passing to text parsers.
            char topic[64];
            int tlen = event->topic_len < (int)sizeof(topic) - 1
                           ? event->topic_len
                           : (int)sizeof(topic) - 1;
            memcpy(topic, event->topic, tlen);
            topic[tlen] = '\0';

            if (strstr(topic, "set") != NULL) {
                char buf[64];
                int dlen = event->data_len < (int)sizeof(buf) - 1
                               ? event->data_len
                               : (int)sizeof(buf) - 1;
                memcpy(buf, event->data, dlen);
                buf[dlen] = '\0';
                handle_settings_payload(buf);
            } else if (strstr(topic, "mocap") != NULL) {
                handle_mocap_payload(event->data, event->data_len);
            } else if (strstr(topic, "obs") != NULL) {
                handle_observations_payload(event->data, event->data_len);
            } else {
                char buf[256];
                int dlen = event->data_len < (int)sizeof(buf) - 1
                               ? event->data_len
                               : (int)sizeof(buf) - 1;
                memcpy(buf, event->data, dlen);
                buf[dlen] = '\0';
                handle_action_payload(buf);
            }
            break;
        }

        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls",
                                     event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack",
                                     event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as socket errno",
                                     event->error_handle->esp_transport_sock_errno);
            }
            break;

        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

static void mqtt_app_start(void) {
    esp_mqtt_client_config_t cfg = {.broker.address.uri = CONFIG_MQTT_BROKER_URL};
    s_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_client);

    // Give the client a moment to connect before subscribing.  The
    // subscriptions will be retried automatically by the broker on the
    // CONNECTED event; this is just an early subscribe for the common path.
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_mqtt_client_subscribe(s_client, CONFIG_ACTION_TOPIC, 0);
    esp_mqtt_client_subscribe(s_client, CONFIG_OBSERVATIONS_TOPIC, 0);
    esp_mqtt_client_subscribe(s_client, CONFIG_SETTINGS_TOPIC, 0);
    esp_mqtt_client_subscribe(s_client, CONFIG_MOCAP_TOPIC, 0);
}

static void time_sync_notification_cb(struct timeval *tv) {
    (void)tv;
    time_t now;
    time(&now);
    ESP_LOGI(TAG, "Time synchronised (%lld)", (long long)now);
}

static void initialize_sntp(void) {
    ESP_LOGI(TAG, "Initialising SNTP");
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, CONFIG_NTP_IP);
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
}

void mqtt_task(void *pvParams) {
    (void)pvParams;

    mocap_queue            = xQueueCreate(1, sizeof(MocapDataWithRotation));
    action_queue           = xQueueCreate(1, sizeof(Action));
    raw_observations_queue = xQueueCreate(1, sizeof(StateObservations));
    settings_queue         = xQueueCreate(1, sizeof(SettingsData));
    mqtt_out_queue         = xQueueCreate(10, sizeof(MqttOutData));

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    example_connect();
    initialize_sntp();
    mqtt_app_start();

    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t last_wake;
    MqttOutData out;

    while (1) {
        last_wake = xTaskGetTickCount();
        if (xQueueReceive(mqtt_out_queue, &out, 0) == pdTRUE) {
            esp_mqtt_client_publish(s_client, CONFIG_ACTION_TOPIC, out.msg,
                                    strlen(out.msg), 0, 0);
        }
        vTaskDelayUntil(&last_wake, period);
    }
}
