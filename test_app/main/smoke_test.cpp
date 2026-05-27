// On-target smoke test for the ESP-DL inference path.
//
// Reuses the *real* model.espdl that the main firmware embeds, so this catches
// breakage in the model loader, variant detection, and forward pass without
// any of the MQTT plumbing.
//
// Pass criterion: all 8 action outputs must be finite and within [-1.5, 1.5]
// (Tanh-action heads can momentarily overshoot slightly when run on a
// zero observation, hence the looser bound).

#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dl_model_base.hpp"
#include "dl_tool.hpp"

#include "model_variant.h"

extern "C" const uint8_t model_espdl[] asm("_binary_model_espdl_start");

static const char *TAG = "smoke";

static int first_dim_of(dl::TensorBase *t) {
    if (t == nullptr) return 0;
    auto shape = t->get_shape();
    return shape.empty() ? 0 : shape.back();
}

static void abort_test(const char *why) {
    ESP_LOGE(TAG, "FAIL: %s", why);
    esp_system_abort(why);
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Loading embedded model...");
    auto *model = new dl::Model(reinterpret_cast<const char *>(model_espdl),
                                fbs::MODEL_LOCATION_IN_FLASH_RODATA);
    model->test();

    auto inputs  = model->get_inputs();
    auto outputs = model->get_outputs();

    if (inputs.count("observations") == 0) abort_test("no `observations` input");
    if (outputs.count("action") == 0)       abort_test("no `action` output");

    dl::TensorBase *obs_in = inputs.at("observations");
    dl::TensorBase *act_out = outputs.at("action");

    const bool has_h_t = inputs.count("h_t_in") > 0;
    const int  obs_n   = first_dim_of(obs_in);
    const ModelVariant v = detect_model_variant(obs_n, has_h_t);
    ESP_LOGI(TAG, "Detected variant: %s (obs=%d, h_t=%d)",
             model_variant_name(v), obs_n, has_h_t);
    if (v == MODEL_VARIANT_UNKNOWN) abort_test("model variant classified as UNKNOWN");

    // Zero-initialised observation/hidden-state for a deterministic check.
    float obs_buf[25] = {0};
    auto *obs_tensor = new dl::TensorBase({1, obs_n}, obs_buf, 0,
                                          dl::DATA_TYPE_FLOAT);
    obs_in->assign(obs_tensor);

    if (has_h_t) {
        dl::TensorBase *h_t_in = inputs.at("h_t_in");
        // Allocate a zero hidden state matching the model's declared shape.
        auto shape = h_t_in->get_shape();
        size_t n = 1;
        for (int d : shape) n *= (size_t)d;
        float *zeros = new float[n]();
        auto *h_t = new dl::TensorBase(shape, zeros, 0, dl::DATA_TYPE_FLOAT);
        h_t_in->assign(h_t);
        delete h_t;
        delete[] zeros;
    }

    DL_LOG_LATENCY_INIT();
    DL_LOG_LATENCY_START();
    model->run();
    DL_LOG_LATENCY_END();
    ESP_LOGI(TAG, "Forward pass latency: %luus",
             static_cast<unsigned long>(DL_LOG_LATENCY_GET()));

    dl::TensorBase *actions = new dl::TensorBase({8}, nullptr, 0,
                                                 dl::DATA_TYPE_FLOAT);
    actions->assign(act_out);

    for (int i = 0; i < 8; ++i) {
        float a = actions->get_element<float>(i);
        ESP_LOGI(TAG, "  action[%d] = %f", i, a);
        if (!std::isfinite(a)) abort_test("non-finite action element");
        if (a < -1.5f || a > 1.5f) abort_test("action element out of range");
    }

    ESP_LOGI(TAG, "PASS — model loaded, classified as %s, forward pass OK.",
             model_variant_name(v));

    // Park forever so the LOG line stays on the monitor and the watchdog
    // doesn't complain.
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
