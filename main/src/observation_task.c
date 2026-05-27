#include "observation_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "observations.h"

extern QueueHandle_t raw_observations_queue;   // owned by mqtt_handler
extern QueueHandle_t mocap_queue;              // owned by mqtt_handler
QueueHandle_t normalized_observations_queue;

void observation_task(void *params) {
    (void)params;

    StateObservations raw = {0};
    NormalizedStateObs normalized = {0};
    MocapDataWithRotation mocap = {0};

    normalized_observations_queue = xQueueCreate(1, sizeof(normalized));

    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t last_wake;

    while (true) {
        last_wake = xTaskGetTickCount();

        xQueuePeek(mocap_queue, &mocap, 0);

        if (xQueueReceive(raw_observations_queue, &raw, 0) == pdTRUE) {
            normalize_observation(&raw, &mocap, &kStateObsMaxesDefault,
                                  CONFIG_USE_IMU_FOR_YAW != 0, &normalized);
            xQueueOverwrite(normalized_observations_queue, &normalized);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
