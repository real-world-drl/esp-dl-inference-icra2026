// Pure parsing functions for inbound MQTT payloads.
//
// All of these are deliberately free of FreeRTOS / esp_mqtt dependencies so
// they can be unit-tested on the host.  The thin MQTT handler in
// mqtt_handler.c calls them and then routes results into FreeRTOS queues.
//
// All parsers return `true` on a recognised, well-formed payload and `false`
// otherwise; outputs are written into caller-provided structs.

#ifndef ESP_DL_INFERENCE_MQTT_PARSERS_H
#define ESP_DL_INFERENCE_MQTT_PARSERS_H

#include <stdbool.h>
#include <stdint.h>

#include "messages.h"

#ifdef __cplusplus
extern "C" {
#endif

// Header bytes used on the wire by the binary message variants.
#define BIN_OBSERVATIONS_HEADER 0x0A
#define BIN_MOCAP_HEADER        0x0E

// Commands accepted on the SETTINGS topic.
typedef enum {
    SETTINGS_CMD_UNKNOWN = 0,
    SETTINGS_CMD_RUN,           // 'x'
    SETTINGS_CMD_STOP,          // 'y'
    SETTINGS_CMD_MODEL,         // 'm<name>'
    SETTINGS_CMD_SET_PIVOT,     // 'p<sp><sp>theta_deg'
} SettingsCommand;

// Result of decoding a settings payload.  When `cmd == SETTINGS_CMD_SET_PIVOT`
// `theta_rad` carries the pivot rotation; otherwise the field is 0.
typedef struct {
    SettingsCommand cmd;
    bool is_running;        // mirror of cmd for convenience
    char model_name[20];    // populated for SETTINGS_CMD_MODEL
    float theta_rad;        // populated for SETTINGS_CMD_SET_PIVOT
} ParsedSettings;

// Parses an action string of the form "a<f>,<f>,...,<f>" or "b<f>,<f>,..."
// into up to 16 float channels.  Cmd byte is the first character; any unused
// channels are zeroed.  Returns the number of channels actually decoded.
int parse_action_string(const char *payload, char *cmd_out, float channels_out[16]);

// Decodes a SETTINGS-topic payload.  Returns false for an unrecognised
// command byte.
bool parse_settings_payload(const char *payload, ParsedSettings *out);

// Decodes the legacy CSV-form observations stream ("S<td>,<dist>,<yaw>,...").
// Maps the available fields into a StateObservations struct (fields not
// present in the CSV are zeroed).  Returns false if the payload is too short.
bool parse_observations_string(const char *payload, StateObservations *out);

// Decodes the binary StateObservations payload.  Returns false if the header
// byte does not match `BIN_OBSERVATIONS_HEADER`.
bool parse_bin_observations(const void *payload, size_t payload_len,
                            StateObservations *out);

// Applies the configured rotation/pivot from `state_in` to a freshly-received
// `MocapData`, producing the rotated `MocapDataWithRotation` consumed by the
// observation pipeline.  Returns false if the header byte does not match
// `BIN_MOCAP_HEADER`.  `state_in` and `state_out` may alias safely.
bool parse_bin_mocap(const void *payload, size_t payload_len,
                     const MocapDataWithRotation *state_in,
                     MocapDataWithRotation *state_out);

#ifdef __cplusplus
}
#endif

#endif  // ESP_DL_INFERENCE_MQTT_PARSERS_H
