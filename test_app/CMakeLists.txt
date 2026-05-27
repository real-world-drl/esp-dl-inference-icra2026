# On-device smoke test.
#
# A standalone ESP-IDF project that:
#   1. Loads the .espdl model embedded in flash,
#   2. Auto-detects the variant via detect_model_variant(),
#   3. Runs one forward pass with a deterministic, all-zeros observation,
#   4. Asserts the output action vector is finite and within [-1, 1],
#   5. Logs latency, then exits the test.
#
# Build with:
#   idf.py -C test_app set-target esp32s3
#   idf.py -C test_app build flash monitor

cmake_minimum_required(VERSION 3.16)

set(ENV{IDF_TARGET} "esp32s3")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_dl_inference_smoke_test)
