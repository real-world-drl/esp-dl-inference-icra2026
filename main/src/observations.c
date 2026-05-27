#include <math.h>

#include "observations.h"

const StateObsMaxes kStateObsMaxesDefault = {
    .x_distance = 2.0f,
    .current = 8.0f,
    .current_per_leg = 2.0f,
    .yaw_delta = 1.0f,
    .yaw = (float)(M_PI / 2.0),
    .pitch = (float)(M_PI / 2.0),
    .roll = (float)(M_PI / 2.0),
    .servo_position = 500.0f,
};

void normalize_observation(const StateObservations *raw,
                           const MocapDataWithRotation *mocap,
                           const StateObsMaxes *maxes,
                           bool use_imu_for_yaw,
                           NormalizedStateObs *out) {
    float *o = out->observations;

    o[0] = raw->current_front_left  / maxes->current_per_leg;
    o[1] = raw->current_front_right / maxes->current_per_leg;
    o[2] = raw->current_back_left   / maxes->current_per_leg;
    o[8] = raw->current_back_right  / maxes->current_per_leg;
    o[3] = raw->current             / maxes->current;

    if (fabsf(mocap->theta) > MOCAP_THETA_ACTIVE_EPS) {
        o[4] = mocap->yaw / maxes->yaw;
    } else if (use_imu_for_yaw) {
        o[4] = raw->yaw / maxes->yaw;
    } else {
        o[4] = mocap->no_rotation_yaw / maxes->yaw;
    }

    o[5] = raw->acc_z;
    o[6] = raw->pitch / maxes->pitch;
    o[7] = raw->roll  / maxes->roll;

    o[9]  = raw->position_knee_back_left   / maxes->servo_position;
    o[10] = raw->position_thigh_back_left  / maxes->servo_position;
    o[11] = raw->position_knee_back_right  / maxes->servo_position;
    o[12] = raw->position_thigh_back_right / maxes->servo_position;

    o[13] = raw->position_knee_front_right  / maxes->servo_position;
    o[14] = raw->position_thigh_front_right / maxes->servo_position;
    o[15] = raw->position_knee_front_left   / maxes->servo_position;
    o[16] = raw->position_thigh_front_left  / maxes->servo_position;
}
