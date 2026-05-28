// ESP-DL inference loop.
//
// Loads the embedded `.espdl` model at startup, classifies the actor as
// non-recurrent / R- / RA- by inspecting its inputs, and from then on pulls
// normalised observations off `normalized_observations_queue`, runs the
// model, and publishes the resulting action vector on `mqtt_out_queue`.

#include "inference_handler.hpp"

#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "dl_model_base.hpp"
#include "dl_tool.hpp"

#include "led_blink.h"
#include "messages.h"
#include "model_variant.h"
#include "observations.h"

// Disabled bool Kconfig options are absent from sdkconfig.h, so supply a 0
// fallback to keep the runtime check below well-formed when the option is off.
#ifndef CONFIG_LOG_INFERENCE_TIME_PER_STEP
#define CONFIG_LOG_INFERENCE_TIME_PER_STEP 0
#endif

extern "C" const char *TAG;
extern "C" const uint8_t model_espdl[] asm("_binary_model_espdl_start");

extern "C" {
extern QueueHandle_t normalized_observations_queue;
extern QueueHandle_t mqtt_out_queue;
extern QueueHandle_t settings_queue;
}

static uint8_t s_inference_led_gpio = CONFIG_BLINK_GPIO + 2;

// Default hidden-state shape used as a fallback when the model exposes no
// `h_t` input.  Real recurrent models override this with the actual shape.
static constexpr int kDefaultHtLayers = 3;
static constexpr int kDefaultHtBatch  = 1;
static constexpr int kDefaultHtHidden = 64;

namespace {

// Pull the first dimension of a tensor (used to discover observation size and
// recurrent layer count).  Falls back to 0 if shape is empty.
int first_dim_of(dl::TensorBase *t) {
    if (t == nullptr) return 0;
    auto shape = t->get_shape();
    return shape.empty() ? 0 : shape.back();  // ESP-DL flattens single-batch
                                              // tensors; pick the trailing
                                              // dim for vector inputs.
}

}  // namespace

extern "C" void inference_handler(void *pvParameters) {
    (void)pvParameters;
    vTaskDelay(pdMS_TO_TICKS(7000));

    gpio_reset_pin(static_cast<gpio_num_t>(s_inference_led_gpio));
    gpio_set_direction(static_cast<gpio_num_t>(s_inference_led_gpio),
                       GPIO_MODE_OUTPUT);
    gpio_set_level(static_cast<gpio_num_t>(s_inference_led_gpio), 1);

    dl::Model *model = new dl::Model(reinterpret_cast<const char *>(model_espdl),
                                     fbs::MODEL_LOCATION_IN_FLASH_RODATA);
    ESP_LOGI(TAG, "Model loaded.");
    model->test();
    model->profile();

    std::map<std::string, dl::TensorBase *> inputs  = model->get_inputs();
    std::map<std::string, dl::TensorBase *> outputs = model->get_outputs();

    dl::TensorBase *observation_input = inputs.at("observations");
    dl::TensorBase *action_output     = outputs.at("action");

    const bool has_h_t_input  = inputs.count("h_t_in") > 0;
    const bool has_h_t_output = outputs.count("h_t") > 0;

    // Vector inputs are exported as either {features} or {1, features}; pull
    // the trailing dim to get the feature count either way.
    const int observation_size = first_dim_of(observation_input);
    const ModelVariant variant =
        detect_model_variant(observation_size, has_h_t_input);

    ESP_LOGI(TAG,
             "Detected model variant: %s (obs_size=%d, h_t in/out=%d/%d)",
             model_variant_name(variant), observation_size,
             has_h_t_input, has_h_t_output);

    if (variant == MODEL_VARIANT_UNKNOWN) {
        ESP_LOGE(TAG,
                 "Cannot classify model: observation tensor first-dim=%d, "
                 "h_t input %s — refusing to run.",
                 observation_size, has_h_t_input ? "present" : "absent");
        vTaskDelete(nullptr);
        return;
    }

    // Carry-over hidden state for recurrent variants.  Shape comes from the
    // model when available so different hidden sizes work without recompile.
    dl::TensorBase *h_t_input  = nullptr;
    dl::TensorBase *h_t_output = nullptr;
    if (has_h_t_input) {
        h_t_input  = inputs.at("h_t_in");
        h_t_output = has_h_t_output
                         ? outputs.at("h_t")
                         : new dl::TensorBase(h_t_input->get_shape(), nullptr,
                                              0, dl::DATA_TYPE_FLOAT);
    } else {
        // No-op placeholder so the rest of the code can branch uniformly.
        h_t_output = new dl::TensorBase(
            {kDefaultHtLayers, kDefaultHtBatch, kDefaultHtHidden}, nullptr, 0,
            dl::DATA_TYPE_FLOAT);
    }

    dl::TensorBase *actions = new dl::TensorBase({8}, nullptr, 0,
                                                 dl::DATA_TYPE_FLOAT);
    dl::TensorBase *obs_tensor = new dl::TensorBase(
        {1, observation_size}, nullptr, 0, dl::DATA_TYPE_FLOAT);

    NormalizedStateObs state = {};
    MqttOutData mqtt_out = {};
    float obs_with_actions[25] = {};

    SettingsData settings = {.header = 0x00, .is_running = false};
    TaskHandle_t led_task = nullptr;
    bool led_on = false;
    gpio_set_level(static_cast<gpio_num_t>(s_inference_led_gpio), 0);

    uint32_t total_latency = 0;
    uint32_t latency_count = 0;
    DL_LOG_LATENCY_INIT();

    const TickType_t period = pdMS_TO_TICKS(50);
    TickType_t last_wake;

    while (true) {
        last_wake = xTaskGetTickCount();

        if (xQueueReceive(settings_queue, &settings, 0) == pdTRUE) {
            if (led_on && !settings.is_running) {
                vTaskDelete(led_task);
                led_task = nullptr;
                led_on = false;
                gpio_set_level(static_cast<gpio_num_t>(s_inference_led_gpio), 0);
                ESP_LOGI(TAG, "Stopping inference...");
            } else if (!led_on && settings.is_running) {
                xTaskCreate(blink_led, "INFERENCE LED", 4096,
                            &s_inference_led_gpio, 1, &led_task);
                led_on = true;
                ESP_LOGI(TAG, "Starting inference...");
            }
        }

        if (settings.is_running) {
            if (xQueueReceive(normalized_observations_queue, &state, 0) ==
                pdTRUE) {
                if (variant == MODEL_VARIANT_RECURRENT_RA) {
                    for (int i = 0; i < 8; ++i) {
                        obs_with_actions[i] = actions->get_element<float>(i);
                    }
                    for (int i = 0; i < NORMALIZED_OBS_LEN; ++i) {
                        obs_with_actions[8 + i] = state.observations[i];
                    }
                    obs_tensor->data = obs_with_actions;
                } else {
                    obs_tensor->data = state.observations;
                }

                observation_input->assign(obs_tensor);

                if (variant == MODEL_VARIANT_RECURRENT ||
                    variant == MODEL_VARIANT_RECURRENT_RA) {
                    h_t_input->assign(h_t_output);
                }

                DL_LOG_LATENCY_START();
                model->run();
                DL_LOG_LATENCY_END();
                total_latency += DL_LOG_LATENCY_GET();
                ++latency_count;
                if (CONFIG_LOG_INFERENCE_TIME_PER_STEP) {
                    printf("%lu\n",
                           static_cast<unsigned long>(DL_LOG_LATENCY_GET()));
                }
            }

            actions->assign(action_output);

            // 16-channel PCA9685 layout: only servos 0..1, 4..5, 8..9, 12..13
            // are populated on the Quaid; the rest are 0.
            std::snprintf(mqtt_out.msg, sizeof(mqtt_out.msg),
                          "a%f,%f,0,0,%f,%f,0,0,%f,%f,0,0,%f,%f,0,0",
                          actions->get_element<float>(0),
                          actions->get_element<float>(1),
                          actions->get_element<float>(2),
                          actions->get_element<float>(3),
                          actions->get_element<float>(4),
                          actions->get_element<float>(5),
                          actions->get_element<float>(6),
                          actions->get_element<float>(7));
            xQueueSend(mqtt_out_queue, &mqtt_out, 0);

            if (latency_count == 100) {
                ESP_LOGI(TAG,
                         "Mean inference latency over 100 steps: %fus",
                         total_latency / (float)latency_count);
                total_latency = 0;
                latency_count = 0;
            }
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
