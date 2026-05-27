# Model placement

The build embeds whatever lives at `main/models/model.espdl` into flash via
`target_add_aligned_binary_data()` (see `main/CMakeLists.txt`).

The variant is auto-detected at load time, so the **same firmware build works
for**:

| Variant       | Filename pattern (from esp-dl-quant-icra2026) | Observation length | Hidden state |
|---------------|-----------------------------------------------|--------------------|--------------|
| Non-recurrent | `act_net_*.espdl`                             | 17 floats          | none         |
| Recurrent R-  | `aug_act_net_*_R-{TD3,SAC}_*.espdl`           | 17 floats          | h_t          |
| Recurrent RA- | `aug_act_net_*_RA-{TD3,SAC}_*.espdl`          | 25 floats          | h_t          |

## Procedure

1. Build a model with the toolchain in `../esp-dl-quant-icra2026` (see that
   repo's `README.md` and `GRU_QUICKSTART.md`).
2. **Copy** (don't symlink) the resulting `.espdl` to `main/models/model.espdl`.
   CMake sometimes misses changes to symlink targets and you end up flashing a
   stale model.
3. Rebuild: `idf.py build` — the binary embedder picks up the new model.

`.espdl` files are deliberately `.gitignored` so the repo stays small.
