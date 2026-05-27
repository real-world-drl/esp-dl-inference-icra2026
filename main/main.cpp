// Application entry point.  Spawns the four cooperating FreeRTOS tasks:
//
//   - LED        : status heartbeat (one solid pattern; per-event LEDs are
//                  created/destroyed from the MQTT and inference tasks)
//   - MQTT       : MQTT client + dispatch into queues
//   - Observation: normalises raw observations into the 17-float vector the
//                  actor network expects
//   - Inference  : runs the embedded .espdl model and publishes actions

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "inference_handler.hpp"
#include "led_blink.h"
#include "mqtt_handler.h"
#include "observation_task.h"

extern "C" const char *TAG = "ESP-DL-INFERENCE";

extern "C" const uint8_t model_espdl[] asm("_binary_model_espdl_start");

extern "C" void app_main(void) {
    static uint8_t basic_led_gpio = CONFIG_BLINK_GPIO;

    xTaskCreate(blink_led, "LED Task", 4096, &basic_led_gpio, 1, NULL);

    xTaskCreatePinnedToCore(mqtt_task, "MQTT Task", 4096, NULL, 5, NULL, 1);
    xTaskCreate(observation_task, "Observation Task", 4096, NULL, 5, NULL);
    xTaskCreatePinnedToCore(inference_handler, "Inference Task", 4096, NULL, 5,
                            NULL, 0);
}
