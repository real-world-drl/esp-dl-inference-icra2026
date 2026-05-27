// Simple status-LED heartbeat.  GPIO is passed as a pointer-to-uint8 in the
// task argument so multiple tasks can share this body with different pins.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "led_blink.h"

extern char *TAG;

void blink_led(void *pvParameters) {
    uint8_t gpio = *(uint8_t *)pvParameters;
    ESP_LOGI(TAG, "Starting LED on GPIO %d!", gpio);
    gpio_reset_pin(gpio);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    uint8_t pattern[8] = {0, 2, 2, 2, 0, 2, 1, 2};

    while (1) {
        for (uint8_t i = 0; i < 8; ++i) {
            uint8_t delay = pattern[i];
            if (delay != 0) {
                gpio_set_level(gpio, 1);
                if (delay == 2) {
                    vTaskDelay(300 / portTICK_PERIOD_MS);
                } else if (delay == 1) {
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
                gpio_set_level(gpio, 0);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            } else {
                gpio_set_level(gpio, 0);
                vTaskDelay(200 / portTICK_PERIOD_MS);
            }
        }
    }
}
