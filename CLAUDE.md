# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

ESP-IDF firmware (ESP32-S3 target) that runs a quantised DRL actor `.espdl` on-device. Observations come in over MQTT, the actor runs locally via ESP-DL, and 16-channel PCA9685 action CSVs go back out over MQTT. Companion to the ICRA 2026 paper *Quantization of DRL Models for Embedded Microcontrollers* and to `esp-dl-quant-icra2026` (which produces the models) and `quaid-sim-cpp` (one possible MQTT driver).

## Build / flash / run

```bash
cp /path/to/<name>.espdl main/models/model.espdl     # required; gitignored
idf.py set-target esp32s3
idf.py build flash monitor
idf.py menuconfig                                    # MQTT broker URL, topics, GPIO, etc.
```

Tested hardware: ESP32-S3-WROOM-1-N16R8 (octal PSRAM @ 80 MHz, 16 MB flash). PSRAM is **mandatory** — Aug-GRU activations don't fit in internal SRAM. `sdkconfig.defaults` carries the required PSRAM/cache/240 MHz config and a custom 8 MB factory partition (`partitions.csv`).

## Tests

Two completely separate test layers:

```bash
# Host-side Unity tests for pure (FreeRTOS-free) modules. No board needed.
cmake -S test -B test/build && cmake --build test/build
ctest --test-dir test/build --output-on-failure

# Run a single host test:
ctest --test-dir test/build -R test_observations --output-on-failure
# or directly:
./test/build/test_observations
```

Host tests build `main/src/{observations.c,mqtt_parsers.c,model_variant.cpp}` directly with `-Wall -Wextra -Werror` and link Unity (FetchContent, pinned v2.6.0, needs network on first configure). The same source files are compiled into the firmware — there are no host-only forks.

```bash
# On-device smoke test: loads the same main/models/model.espdl, runs variant
# detection + one forward pass with zero input, asserts the action is finite
# and in [-1.5, 1.5]. No MQTT.
idf.py -C test_app set-target esp32s3
idf.py -C test_app build flash monitor
```

## Architecture

Four cooperating FreeRTOS tasks, no shared mutable state. All inter-task traffic is through single-slot `xQueueOverwrite` queues (latest-value semantics) except `mqtt_out_queue` which is depth-10.

```
MQTT task  ─raw_obs─►  Observation task ─normalized_obs─► Inference task ─mqtt_out─► MQTT task
(core 1)    ─mocap──►  (any core)                         (core 0)
            ─settings─────────────────────────────────────►
```

- **MQTT task** (`main/src/mqtt_handler.c`, core 1) — owns the esp_mqtt client. Event handler dispatches by topic, hands payloads to pure parsers, and pushes structs into the relevant queue. Also drains `mqtt_out_queue` and publishes.
- **Observation task** (`main/src/observation_task.c`, any core) — joins the latest `StateObservations` and `MocapDataWithRotation`, calls `normalize_observation()`, emits a 17-float `NormalizedStateObs`.
- **Inference task** (`main/src/inference_handler.cpp`, core 0) — loads the embedded `.espdl` once, classifies the model, then runs at 50 ms (20 Hz). Latency is logged either per-step (`CONFIG_LOG_INFERENCE_TIME_PER_STEP`) or mean-of-100.
- **LED task** — heartbeat only, not on the data path.

CPU pinning isolates MQTT (which can block on Wi-Fi/DNS) from inference (which must hit 20 Hz). The observation task is intentionally unpinned.

### Pure vs task split (critical convention)

Every task is a thin wrapper around pure functions that have **no FreeRTOS, no ESP_LOG, no global state**. That is what lets `test/` compile them host-side. Keep this split when adding code:

| Pure function (in `main/src/*.c[pp]`)    | Lives in                | Driven by                        |
|------------------------------------------|-------------------------|----------------------------------|
| `normalize_observation`                  | `observations.c`        | `observation_task.c`             |
| `parse_bin_observations`, `parse_observations_string`, `parse_bin_mocap`, `parse_settings_payload`, `parse_action_string` | `mqtt_parsers.c` | MQTT event handler in `mqtt_handler.c` |
| `detect_model_variant`, `model_variant_name` | `model_variant.cpp` | Inference task                   |

If you add logic that needs unit tests, put it in one of these (or a new pure module) and add it to the `esp_dl_inference_pure` static library in `test/CMakeLists.txt`.

### Model variant auto-detection (runtime, not build-time)

The firmware classifies the loaded model from its tensor shapes at startup — no Kconfig flag, no `#define`, no relink when switching variants:

| `observations` first-dim | `h_t_in` present | Variant                       |
|--------------------------|------------------|-------------------------------|
| 17                       | no               | `MODEL_VARIANT_NON_RECURRENT` |
| 17                       | yes              | `MODEL_VARIANT_RECURRENT`     (R-) |
| 25                       | yes              | `MODEL_VARIANT_RECURRENT_RA`  (RA-, observation prefixed with 8 prev-action floats) |
| anything else            | -                | `MODEL_VARIANT_UNKNOWN` — task aborts |

The hidden-state shape is read from the model (`h_t_in.get_shape()`), so hidden size and layer count vary without recompile. `with_gru_act_net_*` (native `nn.GRU`) does **not** survive ESP-DL import — don't use those.

### Model embedding

`main/CMakeLists.txt` embeds `main/models/model.espdl` via `target_add_aligned_binary_data()` so it's flash-resident read-only data referenced as `_binary_model_espdl_start`. `.espdl` is gitignored. Swapping models = file copy + rebuild.

## MQTT protocol

Topics are Kconfig-configurable (`main/Kconfig.projbuild`), default `quaid/{obs,mocap,set,act}/r100`. The trailing number identifies the entity — `/r1..r9` for real robots, `/r100+` for sim. The observation topic accepts **both** binary `StateObservations` (header `0x0A`) and a legacy CSV form starting with `S`; mocap is always binary (header `0x0E`). See `docs/MQTT_PROTOCOL.md` for the byte layout and `main/include/messages.h` for the source-of-truth structs.

## Configuration

Important Kconfig options under "ESP-DL Inference Configuration":
- `USE_IMU_FOR_YAW` — fall back to IMU yaw when mocap rotation is zero.
- `LOG_INFERENCE_TIME_PER_STEP` — emit a latency line per inference (needed to compute std-dev; otherwise mean over 100 is logged).
- `MQTT_BROKER_URL`, `{OBSERVATIONS,ACTION,SETTINGS,MOCAP}_TOPIC` — broker + topics.

## Documentation pointers

- `docs/ARCHITECTURE.md` — task/queue diagram, CPU pinning, memory layout
- `docs/MQTT_PROTOCOL.md` — wire format per topic
- `docs/MODELS.md` — variant table, swap procedure, filename conventions from `esp-dl-quant-icra2026`
