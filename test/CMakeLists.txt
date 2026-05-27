# Host-side Unity test build for pure C/C++ modules under ../main/src.
#
# These tests link against the modules *directly* (no ESP-IDF, no FreeRTOS) so
# they can run in CI without an attached board.
#
# Usage:
#     cmake -S test -B test/build
#     cmake --build test/build
#     ctest --test-dir test/build --output-on-failure

cmake_minimum_required(VERSION 3.16)
project(esp_dl_inference_host_tests LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

# ---------------------------------------------------------------------------
# Unity (single-translation-unit C testing framework).
#
# Fetched once at configure time so the repo stays self-contained without
# vendoring 200 kB of source.  Pinned to a release tag.
# ---------------------------------------------------------------------------
include(FetchContent)
FetchContent_Declare(
    unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.6.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(unity)

# ---------------------------------------------------------------------------
# Under-test sources (only the pure modules — anything that touches FreeRTOS
# or ESP-IDF stays out of host builds).
# ---------------------------------------------------------------------------
set(MAIN_INCLUDE ${CMAKE_SOURCE_DIR}/../main/include)
set(MAIN_SRC     ${CMAKE_SOURCE_DIR}/../main/src)

add_library(esp_dl_inference_pure STATIC
    ${MAIN_SRC}/observations.c
    ${MAIN_SRC}/mqtt_parsers.c
    ${MAIN_SRC}/model_variant.cpp
)
target_include_directories(esp_dl_inference_pure PUBLIC ${MAIN_INCLUDE})

# Treat -Wall -Wextra as errors for the host build so test breakage is loud.
if (NOT MSVC)
    target_compile_options(esp_dl_inference_pure PRIVATE
        -Wall -Wextra -Werror -Wno-unused-parameter
    )
endif()

# ---------------------------------------------------------------------------
# Test executables.  Each test_*.c file becomes its own runner and ctest case.
# ---------------------------------------------------------------------------
function(add_host_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE esp_dl_inference_pure unity)
    target_include_directories(${name} PRIVATE ${MAIN_INCLUDE})
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_host_test(test_observations  observations/test_observations.c)
add_host_test(test_mqtt_parsers  mqtt_parsers/test_mqtt_parsers.c)
add_host_test(test_model_variant model_detect/test_model_variant.c)
