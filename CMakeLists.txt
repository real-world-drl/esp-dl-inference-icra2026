# Top-level ESP-IDF project file.
#
# Companion firmware to esp-dl-quant-icra2026: receives observations over MQTT,
# runs an .espdl actor on-device, publishes actions over MQTT.

cmake_minimum_required(VERSION 3.16)

set(ENV{IDF_TARGET} "esp32s3")

add_compile_options(-fdiagnostics-color=always)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(esp-dl-inference-icra2026)
