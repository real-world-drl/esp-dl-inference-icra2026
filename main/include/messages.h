// MQTT wire-format structs shared between parser, MQTT handler, and observation
// pipeline.  Kept in one header so the parsers can be compiled host-side
// without dragging in FreeRTOS/ESP-IDF.

#ifndef ESP_DL_INFERENCE_MESSAGES_H
#define ESP_DL_INFERENCE_MESSAGES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RAD_TO_DEG 57.295779513082321

// Action vector inferred by the model (one float per servo).
typedef struct {
    float servo0, servo1, servo2, servo3, servo4, servo5, servo6, servo7;
} __attribute__((packed)) Action;

// Raw state-observation telemetry as published by the robot (or the sim) on
// the OBSERVATIONS topic.  `header = 0x0A` for the binary form.
typedef struct {
    uint8_t header;

    int16_t time_delta;
    float distance;
    float yaw;
    float pitch;
    float roll;

    float voltage;
    float current;

    int16_t position_knee_front_left;
    int16_t position_thigh_front_left;
    int16_t position_knee_front_right;
    int16_t position_thigh_front_right;

    int16_t position_knee_back_left;
    int16_t position_thigh_back_left;
    int16_t position_knee_back_right;
    int16_t position_thigh_back_right;

    float current_front_left;
    float current_front_right;
    float current_back_left;
    float current_back_right;

    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
} __attribute__((packed)) StateObservations;

// Mocap pose as published by the mocap bridge (header byte 0x0E).
typedef struct {
    uint8_t header;
    bool degrees;
    uint8_t rigid_body_no;

    int16_t x;
    int16_t y;
    int16_t z;

    float yaw;
    float pitch;
    float roll;

    float qr;
    float qi;
    float qj;
    float qk;
} __attribute__((packed)) MocapData;

// Mocap with the user-defined rotation offset applied (this is the form the
// inference pipeline consumes).  Theta != 0 means the world frame is rotated
// by `theta` around the pivot (px, py).
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;

    int16_t no_rotation_x;
    int16_t no_rotation_y;

    float no_rotation_yaw;
    float yaw;
    float pitch;
    float roll;

    float theta;
    int16_t pivot_x;
    int16_t pivot_y;
} __attribute__((packed)) MocapDataWithRotation;

// Runtime settings sent over the SETTINGS topic.
typedef struct {
    uint8_t header;
    bool is_running;
    char model[20];
} __attribute__((packed)) SettingsData;

// Outbound buffer used by the inference task to publish action strings.
typedef struct {
    char msg[350];
} __attribute__((packed)) MqttOutData;

#ifdef __cplusplus
}
#endif

#endif  // ESP_DL_INFERENCE_MESSAGES_H
