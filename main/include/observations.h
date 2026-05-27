// Pure observation-normalisation logic.
//
// `normalize_observation()` is the testable core: it takes raw sensor +
// mocap state and writes a 17-float observation vector matching what the
// training environment fed the actor network.  The FreeRTOS task that drives
// it lives in observation_task.c.

#ifndef ESP_DL_INFERENCE_OBSERVATIONS_H
#define ESP_DL_INFERENCE_OBSERVATIONS_H

#include "messages.h"

#ifdef __cplusplus
extern "C" {
#endif

// Number of floats the normalised observation occupies.  Matches the actor
// network's `observations` input.
#define NORMALIZED_OBS_LEN 17

// Per-dimension max values used to scale raw sensor readings into roughly
// [-1, 1].  Changing these must match the corresponding training-time
// normalisation in quaid-env.
typedef struct {
    float x_distance;
    float current;
    float current_per_leg;
    float yaw_delta;
    float yaw;
    float pitch;
    float roll;
    float servo_position;
} StateObsMaxes;

typedef struct {
    float observations[NORMALIZED_OBS_LEN];
} NormalizedStateObs;

// Default scaling constants (mirror the training environment).
extern const StateObsMaxes kStateObsMaxesDefault;

// When `|mocap.theta| > kMocapThetaActiveEps` the rotated mocap yaw is
// considered authoritative; below that we fall back to either the IMU yaw or
// the unrotated mocap yaw, see `use_imu_for_yaw`.
#define MOCAP_THETA_ACTIVE_EPS 0.001f

// Normalises `raw` + `mocap` into `out` using `maxes`.
//
// - `use_imu_for_yaw` controls fallback yaw when mocap rotation is inactive.
// - Layout of `out->observations` matches the original quaid_env mapping:
//     [0..3]  per-leg + total current (FL, FR, BL, total)
//     [4]     yaw (rotated mocap, unrotated mocap, or IMU per fallback)
//     [5]     accel-z (carried through unscaled)
//     [6..7]  pitch, roll (normalised by pi/2)
//     [8]     back-right current (placed here for historical reasons)
//     [9..12] back-leg servo positions
//     [13..16] front-leg servo positions
void normalize_observation(const StateObservations *raw,
                           const MocapDataWithRotation *mocap,
                           const StateObsMaxes *maxes,
                           bool use_imu_for_yaw,
                           NormalizedStateObs *out);

#ifdef __cplusplus
}
#endif

#endif  // ESP_DL_INFERENCE_OBSERVATIONS_H
