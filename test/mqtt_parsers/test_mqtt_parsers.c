#include <math.h>
#include <string.h>

#include "messages.h"
#include "mqtt_parsers.h"
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

// --- parse_action_string ----------------------------------------------------

static void test_action_string_extracts_command_byte_and_channels(void) {
    float channels[16] = {0};
    char cmd = '\0';
    int n = parse_action_string("a0.1,0.2,-0.3,0.4", &cmd, channels);

    TEST_ASSERT_EQUAL_INT(4, n);
    TEST_ASSERT_EQUAL_CHAR('a', cmd);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, channels[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.2f, channels[1]);
    TEST_ASSERT_EQUAL_FLOAT(-0.3f, channels[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.4f, channels[3]);
    // Unused channels are zeroed.
    for (int i = 4; i < 16; ++i) {
        TEST_ASSERT_EQUAL_FLOAT(0.0f, channels[i]);
    }
}

static void test_action_string_caps_at_sixteen_channels(void) {
    float channels[16] = {0};
    char cmd = '\0';
    // Eighteen values; expect 16 to be recorded and the rest dropped.
    int n = parse_action_string("a1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18",
                                &cmd, channels);
    TEST_ASSERT_EQUAL_INT(16, n);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, channels[0]);
    TEST_ASSERT_EQUAL_FLOAT(16.0f, channels[15]);
}

static void test_action_string_empty_payload(void) {
    float channels[16];
    char cmd = 'X';
    int n = parse_action_string("", &cmd, channels);
    TEST_ASSERT_EQUAL_INT(0, n);
    TEST_ASSERT_EQUAL_CHAR('\0', cmd);
}

// --- parse_settings_payload -------------------------------------------------

static void test_settings_x_is_run(void) {
    ParsedSettings out = {0};
    TEST_ASSERT_TRUE(parse_settings_payload("x", &out));
    TEST_ASSERT_EQUAL_INT(SETTINGS_CMD_RUN, out.cmd);
    TEST_ASSERT_TRUE(out.is_running);
}

static void test_settings_y_is_stop(void) {
    ParsedSettings out = {0};
    TEST_ASSERT_TRUE(parse_settings_payload("y", &out));
    TEST_ASSERT_EQUAL_INT(SETTINGS_CMD_STOP, out.cmd);
    TEST_ASSERT_FALSE(out.is_running);
}

static void test_settings_m_carries_model_name(void) {
    ParsedSettings out = {0};
    TEST_ASSERT_TRUE(parse_settings_payload("mmodel.bin", &out));
    TEST_ASSERT_EQUAL_INT(SETTINGS_CMD_MODEL, out.cmd);
    TEST_ASSERT_EQUAL_STRING("model.bin", out.model_name);
}

static void test_settings_m_truncates_long_model_name(void) {
    ParsedSettings out = {0};
    TEST_ASSERT_TRUE(
        parse_settings_payload("m0123456789012345678901234567890", &out));
    TEST_ASSERT_EQUAL_INT(SETTINGS_CMD_MODEL, out.cmd);
    // Result is null-terminated within model_name[20].
    TEST_ASSERT_EQUAL_INT(19, (int)strlen(out.model_name));
}

static void test_settings_p_decodes_theta_from_degrees(void) {
    ParsedSettings out = {0};
    // Format: 'p' + 2 chars + degrees value -> 'p  90' = pi/2 rad
    TEST_ASSERT_TRUE(parse_settings_payload("p  90", &out));
    TEST_ASSERT_EQUAL_INT(SETTINGS_CMD_SET_PIVOT, out.cmd);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, (float)(M_PI / 2.0), out.theta_rad);
}

static void test_settings_unknown_command_rejected(void) {
    ParsedSettings out;
    TEST_ASSERT_FALSE(parse_settings_payload("Q", &out));
    TEST_ASSERT_FALSE(parse_settings_payload("", &out));
}

// --- parse_observations_string ---------------------------------------------

static void test_observations_string_maps_csv_fields(void) {
    // Indices the parser actually reads (the leading 'S' is the cmd byte):
    //   0 = time_delta, 3 = current, 4 = yaw, 6 = pitch, 7 = roll,
    //   9..16 = servo positions.  Other slots are ignored.
    const char *payload =
        "S15,0,0,3.0,0.50,0,0,0,0,10,20,30,40,50,60,70,80";
    StateObservations obs;
    memset(&obs, 0xAA, sizeof(obs));
    TEST_ASSERT_TRUE(parse_observations_string(payload, &obs));

    TEST_ASSERT_EQUAL_INT16(15, obs.time_delta);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, obs.current);
    TEST_ASSERT_EQUAL_FLOAT(0.50f, obs.yaw);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, obs.pitch);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, obs.roll);
    TEST_ASSERT_EQUAL_INT16(10, obs.position_knee_back_left);
    TEST_ASSERT_EQUAL_INT16(20, obs.position_thigh_back_left);
    TEST_ASSERT_EQUAL_INT16(30, obs.position_knee_back_right);
    TEST_ASSERT_EQUAL_INT16(40, obs.position_thigh_back_right);
    TEST_ASSERT_EQUAL_INT16(50, obs.position_knee_front_right);
    TEST_ASSERT_EQUAL_INT16(60, obs.position_thigh_front_right);
    TEST_ASSERT_EQUAL_INT16(70, obs.position_knee_front_left);
    TEST_ASSERT_EQUAL_INT16(80, obs.position_thigh_front_left);
}

static void test_observations_string_rejects_short_payload(void) {
    StateObservations obs;
    TEST_ASSERT_FALSE(parse_observations_string("S1,2,3", &obs));
    TEST_ASSERT_FALSE(parse_observations_string("", &obs));
}

// --- parse_bin_observations -------------------------------------------------

static void test_bin_observations_round_trips_struct(void) {
    StateObservations in = {0};
    in.header = BIN_OBSERVATIONS_HEADER;
    in.time_delta = 42;
    in.yaw = 1.25f;
    in.pitch = -0.5f;
    in.position_thigh_front_left = 123;

    StateObservations out;
    TEST_ASSERT_TRUE(parse_bin_observations(&in, sizeof(in), &out));
    TEST_ASSERT_EQUAL_MEMORY(&in, &out, sizeof(in));
}

static void test_bin_observations_rejects_wrong_header(void) {
    StateObservations in = {0};
    in.header = 0x42;
    StateObservations out;
    TEST_ASSERT_FALSE(parse_bin_observations(&in, sizeof(in), &out));
}

static void test_bin_observations_rejects_short_payload(void) {
    uint8_t buf[4] = {BIN_OBSERVATIONS_HEADER, 0, 0, 0};
    StateObservations out;
    TEST_ASSERT_FALSE(parse_bin_observations(buf, sizeof(buf), &out));
}

// --- parse_bin_mocap --------------------------------------------------------

static void test_bin_mocap_preserves_state_when_theta_inactive(void) {
    MocapData wire = {0};
    wire.header = BIN_MOCAP_HEADER;
    wire.x = 10;
    wire.y = 20;
    wire.z = 30;
    wire.yaw = 0.75f;
    wire.pitch = -0.1f;
    wire.roll = 0.05f;

    MocapDataWithRotation state_in = {0};   // theta = 0 → no rotation applied
    MocapDataWithRotation state_out;
    TEST_ASSERT_TRUE(
        parse_bin_mocap(&wire, sizeof(wire), &state_in, &state_out));

    TEST_ASSERT_EQUAL_INT16(10, state_out.no_rotation_x);
    TEST_ASSERT_EQUAL_INT16(20, state_out.no_rotation_y);
    TEST_ASSERT_EQUAL_INT16(30, state_out.z);
    TEST_ASSERT_EQUAL_FLOAT(0.75f, state_out.no_rotation_yaw);
    TEST_ASSERT_EQUAL_FLOAT(0.75f, state_out.yaw);  // unchanged: theta == 0
    TEST_ASSERT_EQUAL_FLOAT(-0.1f, state_out.pitch);
    TEST_ASSERT_EQUAL_FLOAT(0.05f, state_out.roll);
}

static void test_bin_mocap_subtracts_theta_when_active(void) {
    MocapData wire = {0};
    wire.header = BIN_MOCAP_HEADER;
    wire.yaw = 1.0f;

    MocapDataWithRotation state_in = {0};
    state_in.theta = 0.25f;             // active
    MocapDataWithRotation state_out;
    TEST_ASSERT_TRUE(
        parse_bin_mocap(&wire, sizeof(wire), &state_in, &state_out));
    TEST_ASSERT_EQUAL_FLOAT(0.75f, state_out.yaw);  // 1.0 - 0.25
    TEST_ASSERT_EQUAL_FLOAT(1.0f, state_out.no_rotation_yaw);
}

static void test_bin_mocap_wraps_to_pi_range(void) {
    MocapData wire = {0};
    wire.header = BIN_MOCAP_HEADER;
    wire.yaw = (float)(M_PI - 0.1);

    MocapDataWithRotation state_in = {0};
    state_in.theta = -0.5f;             // shifts yaw above pi → should wrap
    MocapDataWithRotation state_out;
    TEST_ASSERT_TRUE(
        parse_bin_mocap(&wire, sizeof(wire), &state_in, &state_out));
    // rotated = (pi - 0.1) - (-0.5) = pi + 0.4 → wrap to pi + 0.4 - 2*pi
    TEST_ASSERT_FLOAT_WITHIN(1e-4f,
                             (float)(M_PI - 0.1 + 0.5 - 2.0 * M_PI),
                             state_out.yaw);
    TEST_ASSERT_TRUE(state_out.yaw >= -(float)M_PI);
    TEST_ASSERT_TRUE(state_out.yaw <= (float)M_PI);
}

static void test_bin_mocap_rejects_wrong_header(void) {
    MocapData wire = {0};
    wire.header = 0x55;
    MocapDataWithRotation state_in = {0};
    MocapDataWithRotation state_out;
    TEST_ASSERT_FALSE(
        parse_bin_mocap(&wire, sizeof(wire), &state_in, &state_out));
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_action_string_extracts_command_byte_and_channels);
    RUN_TEST(test_action_string_caps_at_sixteen_channels);
    RUN_TEST(test_action_string_empty_payload);

    RUN_TEST(test_settings_x_is_run);
    RUN_TEST(test_settings_y_is_stop);
    RUN_TEST(test_settings_m_carries_model_name);
    RUN_TEST(test_settings_m_truncates_long_model_name);
    RUN_TEST(test_settings_p_decodes_theta_from_degrees);
    RUN_TEST(test_settings_unknown_command_rejected);

    RUN_TEST(test_observations_string_maps_csv_fields);
    RUN_TEST(test_observations_string_rejects_short_payload);

    RUN_TEST(test_bin_observations_round_trips_struct);
    RUN_TEST(test_bin_observations_rejects_wrong_header);
    RUN_TEST(test_bin_observations_rejects_short_payload);

    RUN_TEST(test_bin_mocap_preserves_state_when_theta_inactive);
    RUN_TEST(test_bin_mocap_subtracts_theta_when_active);
    RUN_TEST(test_bin_mocap_wraps_to_pi_range);
    RUN_TEST(test_bin_mocap_rejects_wrong_header);

    return UNITY_END();
}
