// FreeRTOS task that drives ESP-DL inference.
//
// Lifecycle: blocked until SETTINGS_CMD_RUN, then per-tick pulls a normalised
// observation off `normalized_observations_queue`, runs the model, publishes
// the action over MQTT.  Variant (non-recurrent / R- / RA-) is auto-detected
// at model-load time.

#ifndef ESP_DL_INFERENCE_INFERENCE_HANDLER_HPP
#define ESP_DL_INFERENCE_INFERENCE_HANDLER_HPP

#ifdef __cplusplus
extern "C" {
#endif

void inference_handler(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif  // ESP_DL_INFERENCE_INFERENCE_HANDLER_HPP
