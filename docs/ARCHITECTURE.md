# Architecture

The firmware is four cooperating FreeRTOS tasks plus a handful of shared
queues.  All inter-task data movement is through queues; there is no shared
mutable state.

## Task / queue diagram

```
                     ╔═══════════════════════════════════════╗
                     ║          MQTT broker (LAN)            ║
                     ╚═══════════════════════════════════════╝
                          ▲ act/             │ obs/ mocap/ set/
                          │                  ▼
   ┌──────────────────────┴──────────────────────────────────┐
   │                       MQTT task                          │
   │      (core 1, prio 5, src/mqtt_handler.c + mqtt_parsers) │
   │                                                          │
   │   event_handler → parse_settings  → settings_queue       │
   │                 → parse_observations → raw_observations_queue │
   │                 → parse_bin_mocap   → mocap_queue        │
   │                                                          │
   │   mqtt_out_queue ──► publish on ACTION topic             │
   └──────────────┬───────────────────────┬─────────────────┬─┘
                  │ raw_obs               │ mocap           │ settings
                  ▼                       ▼                 │
   ┌──────────────────────────────────────┐                 │
   │           Observation task           │                 │
   │     (prio 5, src/observation_task.c) │                 │
   │  normalize_observation() — pure      │                 │
   └──────────────┬───────────────────────┘                 │
                  │ normalized_observations_queue           │
                  ▼                                          ▼
   ┌──────────────────────────────────────────────────────────┐
   │                    Inference task                         │
   │     (core 0, prio 5, src/inference_handler.cpp)           │
   │                                                          │
   │   on first run:                                          │
   │     dl::Model(model_espdl) → detect_model_variant()      │
   │                                                          │
   │   per tick (50 ms):                                      │
   │     pull NormalizedStateObs → set tensor → model->run()  │
   │     format action CSV → mqtt_out_queue                   │
   └──────────────────────────────────────────────────────────┘
```

A fourth task (LED heartbeat) runs at low priority and is independent of the
data path.

## Queue ownership

| Queue                                | Created by                         | Producers           | Consumers               | Depth |
|--------------------------------------|------------------------------------|---------------------|-------------------------|-------|
| `mocap_queue`                        | MQTT task                          | MQTT task           | Observation task        | 1     |
| `raw_observations_queue`             | MQTT task                          | MQTT task           | Observation task        | 1     |
| `settings_queue`                     | MQTT task                          | MQTT task           | Inference task          | 1     |
| `action_queue`                       | MQTT task (currently unused)       | (none)              | (none)                  | 1     |
| `mqtt_out_queue`                     | MQTT task                          | Inference task      | MQTT task               | 10    |
| `normalized_observations_queue`      | Observation task                   | Observation task    | Inference task          | 1     |

Single-slot queues with `xQueueOverwrite` give us "always the latest value"
semantics, which is what the control loop needs.  `mqtt_out_queue` is the
only multi-slot queue because the publisher can briefly run slower than the
inference loop.

## Why split pure logic from tasks

Every task is a thin wrapper around one or more **pure functions**:

| Pure function                  | Wrapper task                     |
|--------------------------------|----------------------------------|
| `normalize_observation`        | `observation_task`               |
| `parse_action_string`          | MQTT event handler               |
| `parse_settings_payload`       | MQTT event handler               |
| `parse_observations_string`    | MQTT event handler               |
| `parse_bin_observations`       | MQTT event handler               |
| `parse_bin_mocap`              | MQTT event handler               |
| `detect_model_variant`         | Inference task                   |

The pure functions live in `main/src/observations.c`,
`main/src/mqtt_parsers.c`, and `main/src/model_variant.cpp`.  They have no
FreeRTOS, no `ESP_LOG`, and no global state, which is what lets `test/`
compile them host-side under Unity.

## CPU pinning

Inference is pinned to **core 0** and the MQTT task to **core 1**.  The
observation task floats — it's mostly memcpy and a handful of divisions so
the scheduler picks whatever core it wants.  Splitting MQTT off the inference
core avoids stalls when MQTT keepalives or DNS lookups block on the network.

## Memory

- The `.espdl` is embedded as flash-resident read-only data via
  `target_add_aligned_binary_data` (see `main/CMakeLists.txt`).
- Aug-GRU activations are allocated in PSRAM by ESP-DL (PSRAM is mandatory —
  see `sdkconfig.defaults`).
- Queue items are passed by value; nothing on the data path heap-allocates
  per frame.
