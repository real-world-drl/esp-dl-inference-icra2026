#include <string.h>

#include "model_variant.h"
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

static void test_obs17_no_h_t_is_non_recurrent(void) {
    TEST_ASSERT_EQUAL_INT(MODEL_VARIANT_NON_RECURRENT,
                          detect_model_variant(17, false));
}

static void test_obs17_with_h_t_is_recurrent_r(void) {
    TEST_ASSERT_EQUAL_INT(MODEL_VARIANT_RECURRENT,
                          detect_model_variant(17, true));
}

static void test_obs25_with_h_t_is_recurrent_ra(void) {
    TEST_ASSERT_EQUAL_INT(MODEL_VARIANT_RECURRENT_RA,
                          detect_model_variant(25, true));
}

static void test_obs25_without_h_t_is_unknown(void) {
    // RA-shape without recurrent state is nonsensical (RA = recurrent +
    // previous-action feedback).
    TEST_ASSERT_EQUAL_INT(MODEL_VARIANT_UNKNOWN,
                          detect_model_variant(25, false));
}

static void test_unexpected_observation_size_is_unknown(void) {
    TEST_ASSERT_EQUAL_INT(MODEL_VARIANT_UNKNOWN,
                          detect_model_variant(0, false));
    TEST_ASSERT_EQUAL_INT(MODEL_VARIANT_UNKNOWN,
                          detect_model_variant(13, true));
    TEST_ASSERT_EQUAL_INT(MODEL_VARIANT_UNKNOWN,
                          detect_model_variant(33, true));
}

static void test_model_variant_name_strings(void) {
    TEST_ASSERT_EQUAL_STRING("non-recurrent",
                             model_variant_name(MODEL_VARIANT_NON_RECURRENT));
    TEST_ASSERT_EQUAL_STRING("recurrent (R-)",
                             model_variant_name(MODEL_VARIANT_RECURRENT));
    TEST_ASSERT_EQUAL_STRING("recurrent (RA-)",
                             model_variant_name(MODEL_VARIANT_RECURRENT_RA));
    TEST_ASSERT_EQUAL_STRING("unknown",
                             model_variant_name(MODEL_VARIANT_UNKNOWN));
    // Defensive: name() of out-of-enum value should fall back to "unknown",
    // not crash.
    TEST_ASSERT_EQUAL_STRING("unknown",
                             model_variant_name((ModelVariant)42));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_obs17_no_h_t_is_non_recurrent);
    RUN_TEST(test_obs17_with_h_t_is_recurrent_r);
    RUN_TEST(test_obs25_with_h_t_is_recurrent_ra);
    RUN_TEST(test_obs25_without_h_t_is_unknown);
    RUN_TEST(test_unexpected_observation_size_is_unknown);
    RUN_TEST(test_model_variant_name_strings);
    return UNITY_END();
}
