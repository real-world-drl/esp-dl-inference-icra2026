# Models

## Where the model lives

The build embeds `main/models/model.espdl` into flash as a read-only blob
via `target_add_aligned_binary_data()`.  Switching the deployed model is a
file copy plus rebuild — no source changes:

```bash
cp /path/to/<new>.espdl main/models/model.espdl
idf.py build flash
```

`*.espdl` is gitignored so the repo stays small.

## Variant auto-detection

`detect_model_variant()` (in `main/src/model_variant.cpp`) classifies the
loaded model on a single rule:

| `observations` first-dim | `h_t_in` input present | Classified as            |
|--------------------------|------------------------|--------------------------|
| 17                       | no                     | `MODEL_VARIANT_NON_RECURRENT` |
| 17                       | yes                    | `MODEL_VARIANT_RECURRENT`     |
| 25                       | yes                    | `MODEL_VARIANT_RECURRENT_RA`  |
| anything else            | -                      | `MODEL_VARIANT_UNKNOWN` (refuses to run) |

The inference task branches on the result:

- **Non-recurrent** — feed the 17-float observation in, run, done.
- **Recurrent (R-)** — feed the 17-float observation + the previous frame's
  `h_t` output back into `h_t_in`, then run.
- **Recurrent (RA-)** — first concatenate `[prev_actions(8), observation(17)]`
  into a 25-float buffer (the layout the network was trained on), then run as
  per R-.

There is no Kconfig flag, no `#define` switch, no rebuild required to change
variants.

## Naming conventions (filename → variant mapping)

Models exported by [`esp-dl-quant-icra2026`](
https://github.com/real-world-drl/esp-dl-quant-icra2026) follow these prefixes:

| Filename starts with    | Variant                       |
|-------------------------|-------------------------------|
| `act_net_`              | non-recurrent (TD3/SAC)       |
| `aug_act_net_*_R-{TD3,SAC}_` | recurrent R- (Aug-GRU)   |
| `aug_act_net_*_RA-{TD3,SAC}_`| recurrent RA- (Aug-GRU)  |
| `with_gru_act_net_`     | **DO NOT USE** — native `nn.GRU` does not survive ESP-DL import |
| `aug_rnn_`              | GRU-only export, not a full actor |

The on-device variant detector does **not** read the filename — only the
ONNX/ESPDL graph structure matters.  The table above is for the human
choosing which file to copy in.

## Hidden-state shape

For recurrent models the hidden state is whatever shape the actor exports —
the inference code uses the model's declared `h_t_in` shape rather than a
hardcoded `(3, 1, 64)`, so different layer counts or hidden sizes work
without a recompile.

## On-device verification

After flashing a new model, run the smoke test once:

```bash
idf.py -C test_app build flash monitor
```

It logs the detected variant and runs one forward pass; if the model loads
but classifies as `UNKNOWN`, the test aborts before doing anything more.
