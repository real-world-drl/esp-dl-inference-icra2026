#include "model_variant.h"

extern "C" ModelVariant detect_model_variant(int observation_size,
                                             bool has_h_t_input) {
    if (observation_size == 17) {
        return has_h_t_input ? MODEL_VARIANT_RECURRENT
                             : MODEL_VARIANT_NON_RECURRENT;
    }
    if (observation_size == 25 && has_h_t_input) {
        return MODEL_VARIANT_RECURRENT_RA;
    }
    return MODEL_VARIANT_UNKNOWN;
}

extern "C" const char *model_variant_name(ModelVariant v) {
    switch (v) {
        case MODEL_VARIANT_NON_RECURRENT: return "non-recurrent";
        case MODEL_VARIANT_RECURRENT:     return "recurrent (R-)";
        case MODEL_VARIANT_RECURRENT_RA:  return "recurrent (RA-)";
        default:                          return "unknown";
    }
}
