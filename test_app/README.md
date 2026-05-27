# On-target smoke test

A standalone ESP-IDF project that exercises the **model load + variant
detection + one forward pass** path against the same `model.espdl` the main
firmware embeds.  It runs no MQTT and needs no broker — just a Quaid /
ESP32-S3 board and a USB cable.

## What it checks

1. The `.espdl` blob in `main/models/model.espdl` loads cleanly.
2. `detect_model_variant()` returns one of `non-recurrent / R- / RA-` for the
   loaded model (and not `UNKNOWN`).
3. A forward pass with all-zero observation runs without error.
4. The eight returned action floats are finite and within `[-1.5, 1.5]`.

Latency is logged for reference.

## Run

```bash
# From the repo root.
idf.py -C test_app set-target esp32s3
idf.py -C test_app build flash monitor
```

Expected last lines:

```
I (xxxx) smoke: Detected variant: non-recurrent (obs=17, h_t=0)
I (xxxx) smoke: Forward pass latency: NNNus
I (xxxx) smoke:   action[0] = -0.034512
...
I (xxxx) smoke: PASS — model loaded, classified as non-recurrent, forward pass OK.
```

Any `FAIL: <reason>` line followed by an `esp_system_abort()` and reboot
indicates the assertion failed.

## Note on the action range

The bound is `[-1.5, 1.5]` rather than `[-1, 1]` because quantised Tanh heads
occasionally cross the saturation point by a small margin on synthetic inputs.
Production observations are normalised into a regime where the network was
trained, where the output is reliably inside `[-1, 1]`.
