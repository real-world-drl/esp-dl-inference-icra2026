// FreeRTOS MQTT task entry point.  See docs/MQTT_PROTOCOL.md for the
// topic-by-topic message contract.

#ifndef ESP_DL_INFERENCE_MQTT_HANDLER_H
#define ESP_DL_INFERENCE_MQTT_HANDLER_H

#include "messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void mqtt_task(void *pvParams);

#ifdef __cplusplus
}
#endif

#endif  // ESP_DL_INFERENCE_MQTT_HANDLER_H
