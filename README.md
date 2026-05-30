# esp-dl-inference-icra2026

ESP-IDF firmware that runs the quantised DRL actor networks produced by
[`esp-dl-quant-icra2026`](https://github.com/real-world-drl/esp-dl-quant-icra2026)
directly on an ESP32-S3.  Companion repository for the ICRA 2026 paper
*Quantization of DRL Models for Embedded Microcontrollers*.

The firmware is deliberately decoupled from any specific hardware setup: it
**receives observations over MQTT, runs ESP-DL inference on-device, and
publishes actions over MQTT**.  Anything that speaks the documented MQTT
protocol — the [`quaid-sim-cpp`](https://github.com/real-world-drl/quaid-sim-cpp)
simulator, the real Quaid robot, or a recorded replay — can drive it.

```
                ┌──────────────────────────────────────────────────────┐
                │                ESP32-S3 (this repo)                  │
   observations │                                                      │ actions
   ────────────►│  MQTT IN ──► normalise ──► .espdl actor ──► MQTT OUT │────────────►
       mocap    │              (17-float)                              │
   ────────────►│                                                      │
                └──────────────────────────────────────────────────────┘
```

## Contents

| Path                     | Purpose                                                 |
|--------------------------|---------------------------------------------------------|
| `main/`                  | Firmware sources (the `main` ESP-IDF component)         |
| `main/include/`          | Public headers — pure modules + task entry points       |
| `main/src/`              | Implementation, split into pure C/C++ + FreeRTOS wrappers |
| `main/models/`           | Drop `model.espdl` here (gitignored)                    |
| `test/`                  | Host-side Unity tests for the pure modules              |
| `test_app/`              | Standalone on-device smoke test                         |
| `docs/`                  | Architecture, MQTT protocol, model-loading reference    |
| `partitions.csv`         | Custom partition table (8 MB factory + NVS)             |
| `sdkconfig.defaults`     | Checked-in PSRAM + perf defaults                        |

## Quickstart (firmware)

```bash
# 1. Drop your quantised actor in place.
cp /path/to/act_net_<name>.espdl main/models/model.espdl

# 2. Build and flash.
idf.py set-target esp32s3
idf.py build flash monitor
```

The model variant is **auto-detected at startup** from the inputs of the
embedded `.espdl`:

- Single observation input, no hidden state → non-recurrent actor (act_net).
- `observations` + `h_t_in`, 17-float observation → recurrent R-* (Aug-GRU,
  observation-only).
- `observations` + `h_t_in`, 25-float observation → recurrent RA-* (Aug-GRU
  with previous-action feedback).

No #ifdef, no Kconfig flag, no relink needed when you switch model variant.

## MQTT topics (defaults — change in `idf.py menuconfig`)

| Topic               | Direction | Payload                                  |
|---------------------|-----------|------------------------------------------|
| `quaid/obs/r100`    | in        | Binary `StateObservations` (header 0x0A) or CSV `"S<td>,<dist>,<yaw>,..."` |
| `quaid/mocap/r100`  | in        | Binary `MocapData` (header 0x0E)         |
| `quaid/set/r100`    | in        | Single-char command: `x`=run, `y`=stop, `m<name>`=set model, `p<sp><sp><deg>`=pivot |
| `quaid/act/r100`    | out       | CSV action string for a 16-channel PCA9685 (only servos 0..1, 4..5, 8..9, 12..13 are populated; others are 0) |

See [`docs/MQTT_PROTOCOL.md`](docs/MQTT_PROTOCOL.md) for the full byte-level
contract.

## Hardware

Tested on the ESP32-S3-WROOM-1-N16R8 (octal PSRAM at 80 MHz, 16 MB flash).
The default `sdkconfig.defaults` enables PSRAM and 240 MHz CPU because the
Aug-GRU activations exceed internal SRAM otherwise.

The firmware does **not** drive any peripheral directly — no I²C, no PWM, no
GPIO beyond a status LED.  All robot-specific hardware (BNO055 IMU, INA226
current sensors, PCA9685 servo driver, motor controllers) lives in a
companion firmware project; the inference firmware talks to it (or to a
simulator) only via the MQTT broker.

## Testing

Two levels:

1. **Host-side Unity tests** — pure C/C++ logic (observation normalisation,
   MQTT parsing, model-variant detection):
   ```bash
   cmake -S test -B test/build && cmake --build test/build
   ctest --test-dir test/build --output-on-failure
   ```

2. **On-device smoke test** — loads the embedded `.espdl`, runs a forward
   pass with a deterministic zero input, asserts the action vector is finite
   and within range:
   ```bash
   idf.py -C test_app build flash monitor
   ```

See [`test/README.md`](test/README.md) and
[`test_app/README.md`](test_app/README.md).

## Documentation

- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — tasks, queues, message flow
- [`docs/MQTT_PROTOCOL.md`](docs/MQTT_PROTOCOL.md) — wire format for each topic
- [`docs/MODELS.md`](docs/MODELS.md) — model variants, swap procedure

## License

MIT.  See [`LICENSE`](LICENSE).

## Citation

If you use this firmware in academic work, please cite the ICRA 2026 paper
(see the parent `esp-dl-quant-icra2026` repository for the canonical BibTeX
entry).
