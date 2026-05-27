# Host-side unit tests

These cover the pure (FreeRTOS-free) modules under `main/src/`:

| Module             | Test file                            |
|--------------------|--------------------------------------|
| `observations.c`   | `observations/test_observations.c`   |
| `mqtt_parsers.c`   | `mqtt_parsers/test_mqtt_parsers.c`   |
| `model_variant.cpp`| `model_detect/test_model_variant.c`  |

## Build & run

```bash
cmake -S test -B test/build
cmake --build test/build
ctest --test-dir test/build --output-on-failure
```

Unity is fetched at configure time (pinned to v2.6.0), so the only host
prerequisites are a C11 toolchain, CMake ≥ 3.16, and network access on the
first configure.

For the on-device smoke test that exercises the actual ESP-DL forward pass,
see `../test_app/README.md`.
