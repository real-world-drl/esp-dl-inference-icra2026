// Pure model-variant classification.
//
// Auto-detection rule (no #ifdef, no Kconfig):
//   - observation tensor first-dim == 17  ⇒  R-/non-recurrent layout
//   - observation tensor first-dim == 25  ⇒  RA- layout (17 obs + 8 prev acts)
//   - presence of an h_t input            ⇒  recurrent
//
// The detector is split out so it can be unit-tested without spinning up an
// ESP-DL model.

#ifndef ESP_DL_INFERENCE_MODEL_VARIANT_H
#define ESP_DL_INFERENCE_MODEL_VARIANT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MODEL_VARIANT_UNKNOWN = 0,
    MODEL_VARIANT_NON_RECURRENT,  // act_net: 17-dim obs, no h_t
    MODEL_VARIANT_RECURRENT,      // aug_act_net R-*: 17-dim obs + h_t
    MODEL_VARIANT_RECURRENT_RA,   // aug_act_net RA-*: 25-dim obs + h_t
} ModelVariant;

// Classify a model from the shape of its inputs.
//
// `observation_size` is the number of floats in the actor's `observations`
// input (17 or 25 in practice).  `has_h_t_input` is true if the actor has an
// `h_t_in` input alongside `observations`.
//
// Returns MODEL_VARIANT_UNKNOWN for nonsensical combinations
// (e.g. RA-shaped observations without h_t).
ModelVariant detect_model_variant(int observation_size, bool has_h_t_input);

// String name for logs (never NULL).
const char *model_variant_name(ModelVariant v);

#ifdef __cplusplus
}
#endif

#endif  // ESP_DL_INFERENCE_MODEL_VARIANT_H
