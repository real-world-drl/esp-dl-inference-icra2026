# MQTT protocol

All topic names are configurable in `idf.py menuconfig` (see
`main/Kconfig.projbuild`).  The defaults documented below are the values
checked into `sdkconfig.defaults`.

## Topic summary

| Topic              | Direction | QoS | Body                       |
|--------------------|-----------|-----|----------------------------|
| `quaid/obs/r100`   | sub       | 0   | binary `StateObservations` OR CSV string |
| `quaid/mocap/r100` | sub       | 0   | binary `MocapData`         |
| `quaid/set/r100`   | sub       | 0   | ASCII command (see below)  |
| `quaid/act/r100`   | pub       | 0   | ASCII action CSV           |

### Convention used by the wider Quaid ecosystem

The trailing number in each topic identifies the entity.  Real robots use
`/r1`..`/r9`; the simulator uses `/r100` and up.  This firmware ships with
`r100` so the same binary can connect to a sim without modification.

## Observations topic — two accepted formats

The firmware accepts either form on the same topic, distinguishing by the
first byte.

### Binary form (header `0x0A`)

The exact packed `StateObservations` struct from
`main/include/messages.h`.  Byte layout (little endian, packed):

| Offset | Field                              | Type     |
|--------|------------------------------------|----------|
| 0      | `header` (= `0x0A`)                | u8       |
| 1      | `time_delta`                       | i16      |
| 3      | `distance`                         | f32      |
| 7      | `yaw, pitch, roll`                 | f32 × 3  |
| 19     | `voltage, current`                 | f32 × 2  |
| 27     | 8 × servo position                 | i16 × 8  |
| 43     | 4 × per-leg current                | f32 × 4  |
| 59     | `acc_xyz, gyro_xyz`                | f32 × 6  |

The full struct is the source of truth; the parser is `parse_bin_observations`
in `main/src/mqtt_parsers.c`.

### CSV form

```
S<time_delta>,<dist>,<yaw_unused>,<current>,<yaw>,<acc_z>,<pitch>,<roll>, ...
  ^                                                                       ...
  command byte 'S'                                                        ...
```

Only a subset of the fields the parser reads (see field-index table below);
the rest are ignored.  Indices are positions in the CSV after the command
byte.

| Index | StateObservations field         |
|-------|---------------------------------|
| 0     | `time_delta`                    |
| 3     | `current`                       |
| 4     | `yaw`                           |
| 6     | `pitch`                         |
| 7     | `roll`                          |
| 9     | `position_knee_back_left`       |
| 10    | `position_thigh_back_left`      |
| 11    | `position_knee_back_right`      |
| 12    | `position_thigh_back_right`     |
| 13    | `position_knee_front_right`     |
| 14    | `position_thigh_front_right`    |
| 15    | `position_knee_front_left`     |
| 16    | `position_thigh_front_left`    |

> The CSV is a legacy compatibility path; new producers should use the binary
> form, which round-trips losslessly.

## Mocap topic

Always binary, always header byte `0x0E`.  Layout matches `MocapData` in
`main/include/messages.h`.  The firmware applies the configured rotation
(see settings command `p` below) and writes a `MocapDataWithRotation` into
`mocap_queue` for the observation pipeline.

## Settings topic

Single-byte ASCII command followed by an optional argument string.

| Command             | Effect                                                     |
|---------------------|------------------------------------------------------------|
| `x`                 | Start inference.  Action publishing begins.                |
| `y`                 | Stop inference.  Action publishing pauses.                 |
| `m<name>`           | Record the model name in `SettingsData.model`.  Cosmetic in this firmware (the embedded model is fixed). |
| `p` `<sp><sp><deg>` | Set the mocap pivot rotation to `<deg>` degrees, using the *current* mocap position as the pivot.  Used to align the mocap frame with the world frame after placing the robot. |

The `p` command's two-space padding is a legacy of the original CSV
producers; the parser tolerates any 3-char prefix and reads `atof()` from
byte 3.

## Action topic

ASCII string with command byte and 16 comma-separated floats:

```
a<f0>,<f1>,0,0,<f2>,<f3>,0,0,<f4>,<f5>,0,0,<f6>,<f7>,0,0
```

Only servos 0..1, 4..5, 8..9, 12..13 are populated — these are the PCA9685
channels the Quaid actually uses.  The unused channels are sent as `0`.

Float format: `%f` — six decimal places by default; consumers should treat
values outside `[-1, 1]` as a degenerate clip.
