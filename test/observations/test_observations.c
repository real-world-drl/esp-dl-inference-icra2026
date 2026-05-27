#include <math.h>
#include <string.h>

#include "observations.h"
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

static void zero_raw(StateObservations *r) {
    memset(r, 0, sizeof(*r));
}

static void zero_mocap(MocapDataWithRotation *m) {
    memset(m, 0, sizeof(*m));
}

// Zeroed input → all observation channels at 0.0 except acc_z (carried
// through unscaled) and yaw, which is divided by maxes->yaw = pi/2 anyway.
static void test_zero_inputs_yield_zero_observations(void) {
    StateObservations raw;
    MocapDataWithRotation mocap;
    NormalizedStateObs out;

    zero_raw(&raw);
    zero_mocap(&mocap);
    memset(&out, 0xAA, sizeof(out));

    normalize_observation(&raw, &mocap, &kStateObsMaxesDefault, false, &out);

    for (int i = 0; i < NORMALIZED_OBS_LEN; ++i) {
        TEST_ASSERT_EQUAL_FLOAT(0.0f, out.observations[i]);
    }
}

static void test_per_leg_currents_normalize_against_current_per_leg_max(void) {
    StateObservations raw;
    MocapDataWithRotation mocap;
    NormalizedStateObs out;

    zero_raw(&raw);
    zero_mocap(&mocap);

    raw.current_front_left  = 1.0f;
    raw.current_front_right = 2.0f;  // == max → exactly 1.0
    raw.current_back_left   = -2.0f;
    raw.current_back_right  = 0.5f;

    normalize_observation(&raw, &mocap, &kStateObsMaxesDefault, false, &out);

    TEST_ASSERT_EQUAL_FLOAT(0.5f, out.observations[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, out.observations[1]);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, out.observations[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.25f, out.observations[8]);
}

static void test_total_current_normalizes_against_current_max(void) {
    StateObservations raw;
    MocapDataWithRotation mocap;
    NormalizedStateObs out;

    zero_raw(&raw);
    zero_mocap(&mocap);
    raw.current = 4.0f;  // current max is 8 → expect 0.5
    normalize_observation(&raw, &mocap, &kStateObsMaxesDefault, false, &out);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, out.observations[3]);
}

static void test_yaw_uses_rotated_mocap_when_theta_is_active(void) {
    StateObservations raw;
    MocapDataWithRotation mocap;
    NormalizedStateObs out;

    zero_raw(&raw);
    zero_mocap(&mocap);

    raw.yaw = 9.0f;                    // would be selected if IMU fallback
    mocap.theta = 0.5f;                // > eps → rotated mocap wins
    mocap.yaw = (float)(M_PI / 4.0);   // = max/2 → expect 0.5
    mocap.no_rotation_yaw = 7.0f;      // unused

    normalize_observation(&raw, &mocap, &kStateObsMaxesDefault, false, &out);

    TEST_ASSERT_EQUAL_FLOAT(0.5f, out.observations[4]);
}

static void test_yaw_falls_back_to_imu_when_theta_inactive_and_imu_flag(void) {
    StateObservations raw;
    MocapDataWithRotation mocap;
    NormalizedStateObs out;

    zero_raw(&raw);
    zero_mocap(&mocap);

    raw.yaw = (float)(M_PI / 2.0);     // exactly the max → 1.0
    mocap.theta = 0.0f;                // inactive
    mocap.no_rotation_yaw = 0.0f;

    normalize_observation(&raw, &mocap, &kStateObsMaxesDefault,
                          /*use_imu_for_yaw=*/true, &out);

    TEST_ASSERT_EQUAL_FLOAT(1.0f, out.observations[4]);
}

static void
test_yaw_falls_back_to_unrotated_mocap_when_theta_inactive_and_no_imu(void) {
    StateObservations raw;
    MocapDataWithRotation mocap;
    NormalizedStateObs out;

    zero_raw(&raw);
    zero_mocap(&mocap);

    raw.yaw = 9.0f;                    // would dominate if IMU flag set
    mocap.theta = 0.0f;                // inactive
    mocap.no_rotation_yaw = (float)(M_PI / 4.0);  // -> 0.5

    normalize_observation(&raw, &mocap, &kStateObsMaxesDefault,
                          /*use_imu_for_yaw=*/false, &out);

    TEST_ASSERT_EQUAL_FLOAT(0.5f, out.observations[4]);
}

static void test_pitch_roll_normalize_by_pi_over_2(void) {
    StateObservations raw;
    MocapDataWithRotation mocap;
    NormalizedStateObs out;

    zero_raw(&raw);
    zero_mocap(&mocap);

    raw.pitch = (float)(M_PI / 4.0);   // 0.5
    raw.roll  = -(float)(M_PI / 2.0);  // -1.0

    normalize_observation(&raw, &mocap, &kStateObsMaxesDefault, false, &out);

    TEST_ASSERT_EQUAL_FLOAT(0.5f, out.observations[6]);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, out.observations[7]);
}

static void test_servo_positions_normalize_by_servo_max(void) {
    StateObservations raw;
    MocapDataWithRotation mocap;
    NormalizedStateObs out;

    zero_raw(&raw);
    zero_mocap(&mocap);

    raw.position_knee_back_left   = 250;   // -> 0.5
    raw.position_thigh_back_left  = 500;   // -> 1.0
    raw.position_knee_back_right  = 100;   // -> 0.2
    raw.position_thigh_back_right = 0;     // -> 0.0

    raw.position_knee_front_right  = -250;   // -> -0.5
    raw.position_thigh_front_right = 500;    // -> 1.0
    raw.position_knee_front_left   = 125;    // -> 0.25
    raw.position_thigh_front_left  = -500;   // -> -1.0

    normalize_observation(&raw, &mocap, &kStateObsMaxesDefault, false, &out);

    TEST_ASSERT_EQUAL_FLOAT(0.5f, out.observations[9]);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, out.observations[10]);
    TEST_ASSERT_EQUAL_FLOAT(0.2f, out.observations[11]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.observations[12]);

    TEST_ASSERT_EQUAL_FLOAT(-0.5f, out.observations[13]);
    TEST_ASSERT_EQUAL_FLOAT(1.0f,  out.observations[14]);
    TEST_ASSERT_EQUAL_FLOAT(0.25f, out.observations[15]);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, out.observations[16]);
}

static void test_acc_z_is_carried_through_unscaled(void) {
    StateObservations raw;
    MocapDataWithRotation mocap;
    NormalizedStateObs out;

    zero_raw(&raw);
    zero_mocap(&mocap);
    raw.acc_z = -0.42f;
    normalize_observation(&raw, &mocap, &kStateObsMaxesDefault, false, &out);
    TEST_ASSERT_EQUAL_FLOAT(-0.42f, out.observations[5]);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_inputs_yield_zero_observations);
    RUN_TEST(test_per_leg_currents_normalize_against_current_per_leg_max);
    RUN_TEST(test_total_current_normalizes_against_current_max);
    RUN_TEST(test_yaw_uses_rotated_mocap_when_theta_is_active);
    RUN_TEST(test_yaw_falls_back_to_imu_when_theta_inactive_and_imu_flag);
    RUN_TEST(test_yaw_falls_back_to_unrotated_mocap_when_theta_inactive_and_no_imu);
    RUN_TEST(test_pitch_roll_normalize_by_pi_over_2);
    RUN_TEST(test_servo_positions_normalize_by_servo_max);
    RUN_TEST(test_acc_z_is_carried_through_unscaled);
    return UNITY_END();
}
