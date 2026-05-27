#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "mqtt_parsers.h"

#define ACTION_MAX_CHANNELS 16
#define ACTION_SCRATCH_BYTES 200
#define OBSERVATIONS_SCRATCH_BYTES 200

int parse_action_string(const char *payload, char *cmd_out, float channels_out[16]) {
    if (payload == NULL || channels_out == NULL) {
        return 0;
    }

    if (cmd_out != NULL) {
        *cmd_out = payload[0];
    }
    if (payload[0] == '\0') {
        return 0;
    }

    char scratch[ACTION_SCRATCH_BYTES];
    strncpy(scratch, payload + 1, sizeof(scratch) - 1);
    scratch[sizeof(scratch) - 1] = '\0';

    for (int i = 0; i < ACTION_MAX_CHANNELS; ++i) {
        channels_out[i] = 0.0f;
    }

    int count = 0;
    char *saveptr = NULL;
    char *tok = strtok_r(scratch, ",", &saveptr);
    while (tok != NULL && count < ACTION_MAX_CHANNELS) {
        channels_out[count++] = (float)atof(tok);
        tok = strtok_r(NULL, ",", &saveptr);
    }
    return count;
}

bool parse_settings_payload(const char *payload, ParsedSettings *out) {
    if (payload == NULL || out == NULL || payload[0] == '\0') {
        return false;
    }

    out->cmd = SETTINGS_CMD_UNKNOWN;
    out->is_running = false;
    out->model_name[0] = '\0';
    out->theta_rad = 0.0f;

    switch (payload[0]) {
        case 'x':
            out->cmd = SETTINGS_CMD_RUN;
            out->is_running = true;
            return true;

        case 'y':
            out->cmd = SETTINGS_CMD_STOP;
            out->is_running = false;
            return true;

        case 'm': {
            out->cmd = SETTINGS_CMD_MODEL;
            const char *name = payload + 1;
            strncpy(out->model_name, name, sizeof(out->model_name) - 1);
            out->model_name[sizeof(out->model_name) - 1] = '\0';
            return true;
        }

        case 'p': {
            // Historical format: 'p' + 2 chars of padding + degrees-as-text.
            if (strlen(payload) < 4) {
                return false;
            }
            out->cmd = SETTINGS_CMD_SET_PIVOT;
            out->theta_rad = (float)(atof(payload + 3) / RAD_TO_DEG);
            return true;
        }

        default:
            return false;
    }
}

bool parse_observations_string(const char *payload, StateObservations *out) {
    if (payload == NULL || out == NULL || payload[0] == '\0') {
        return false;
    }

    char scratch[OBSERVATIONS_SCRATCH_BYTES];
    strncpy(scratch, payload + 1, sizeof(scratch) - 1);
    scratch[sizeof(scratch) - 1] = '\0';

    float fields[22];
    for (int i = 0; i < 22; ++i) {
        fields[i] = 0.0f;
    }

    int count = 0;
    char *saveptr = NULL;
    char *tok = strtok_r(scratch, ",", &saveptr);
    while (tok != NULL && count < 22) {
        fields[count++] = (float)atof(tok);
        tok = strtok_r(NULL, ",", &saveptr);
    }

    if (count < 17) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->header = BIN_OBSERVATIONS_HEADER;
    out->time_delta = (int16_t)fields[0];
    out->current    = fields[3];
    out->yaw        = fields[4];
    out->pitch      = fields[6];
    out->roll       = fields[7];

    out->position_knee_back_left   = (int16_t)fields[9];
    out->position_thigh_back_left  = (int16_t)fields[10];
    out->position_knee_back_right  = (int16_t)fields[11];
    out->position_thigh_back_right = (int16_t)fields[12];

    out->position_knee_front_right  = (int16_t)fields[13];
    out->position_thigh_front_right = (int16_t)fields[14];
    out->position_knee_front_left   = (int16_t)fields[15];
    out->position_thigh_front_left  = (int16_t)fields[16];

    return true;
}

bool parse_bin_observations(const void *payload, size_t payload_len,
                            StateObservations *out) {
    if (payload == NULL || out == NULL) {
        return false;
    }
    if (payload_len < sizeof(StateObservations)) {
        return false;
    }
    const uint8_t *bytes = (const uint8_t *)payload;
    if (bytes[0] != BIN_OBSERVATIONS_HEADER) {
        return false;
    }
    memcpy(out, payload, sizeof(StateObservations));
    return true;
}

bool parse_bin_mocap(const void *payload, size_t payload_len,
                     const MocapDataWithRotation *state_in,
                     MocapDataWithRotation *state_out) {
    if (payload == NULL || state_out == NULL || state_in == NULL) {
        return false;
    }
    if (payload_len < sizeof(MocapData)) {
        return false;
    }
    const uint8_t *bytes = (const uint8_t *)payload;
    if (bytes[0] != BIN_MOCAP_HEADER) {
        return false;
    }

    MocapData md;
    memcpy(&md, payload, sizeof(md));

    MocapDataWithRotation result = *state_in;
    result.no_rotation_x = md.x;
    result.no_rotation_y = md.y;
    result.z = md.z;
    result.no_rotation_yaw = md.yaw;
    result.yaw = md.yaw;
    result.pitch = md.pitch;
    result.roll = md.roll;

    if (fabsf(result.theta) > 0.001f) {
        float rotated_yaw = md.yaw - result.theta;
        if (rotated_yaw > (float)M_PI) {
            result.yaw = rotated_yaw - 2.0f * (float)M_PI;
        } else if (rotated_yaw < -(float)M_PI) {
            result.yaw = rotated_yaw + 2.0f * (float)M_PI;
        } else {
            result.yaw = rotated_yaw;
        }
    }

    *state_out = result;
    return true;
}
