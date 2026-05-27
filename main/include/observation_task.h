// FreeRTOS task that normalises raw observations.  Pulls from
// `raw_observations_queue` (StateObservations) and `mocap_queue`
// (MocapDataWithRotation), pushes NormalizedStateObs onto
// `normalized_observations_queue`.

#ifndef ESP_DL_INFERENCE_OBSERVATION_TASK_H
#define ESP_DL_INFERENCE_OBSERVATION_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void observation_task(void *params);

#ifdef __cplusplus
}
#endif

#endif  // ESP_DL_INFERENCE_OBSERVATION_TASK_H
