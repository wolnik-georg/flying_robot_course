# NS2 in C for the Omar C controller tree — module specification (Phase 1)

Date 2026-10-14. Plan: `docs/plan_omar_tree_ns2_2026-10-14.md`. Status: **spec for review, no code written.**
Reference implementation (must be reproduced exactly): `flying_drone_stack/firmware_app/src/residual_nn.rs` (`ResidualNet::eval`), peer handling in `src/lib.rs:rnn_predict_eval` / `rnn_predict`, params/logs in `firmware_app/traj_iface.c`. Numerical reference: `neuralswarm.py` (see `docs/52`).

## 1. Scope
- **In:** the NS2 forward pass, its gate/clamp/units, per-drone peer-velocity history, evaluation decimation, flash-resident weights, a hook in `controller_omar_indi.c`.
- **Out (unchanged):** all of Omar's control law, controller 6 / `lib.rs`, controllers 10, 11, 12, the Rust `residual_nn` module, the weight upload protocol (not used on the C tree), `upload_residual_weights.py`.

## 2. Where the code lives (and why)
| File | Tree | Content | Preserved as |
|---|---|---|---|
| `src/modules/interface/ns2_residual.h` | crazyflie-firmware (core) | types, constants, API | `firmware_patches/ns2_residual_module.patch` |
| `src/modules/src/ns2_residual.c` | crazyflie-firmware (core) | forward pass, gate, peer history, decimation | same patch (+ `Kbuild` line) |
| `firmware_app/ns2_weights.c` | our app | `const float ns2_w_phi_s[3675]`, `ns2_w_phi_g[3625]`, `ns2_w_rho_s[4161]` + sha256 string, **generated** | repo (regenerated per retrain) |
| `tools/residual/export_weights_c.py` | repo | npz → `ns2_weights.c` | repo |
| hook lines in `controller_omar_indi.c` + 1 field in `controller_omar_indi.h` | core | see §6 | extends `controller_omar_indi.c.patch` / `.h.patch` |
| bindings (`setup.py`, `cffirmware.i`) | core | compile `ns2_residual.c` and the app's `ns2_weights.c` into the host build | extends `cffirmware_bindings.patch` |

Reason for core placement of the logic: the Omar header/controller are core code and must include the module without depending on the app include path; the weights are data that change per retrain, so they stay in the app and are linked as `extern const float[]` (same linking mechanism as `peer_get_all`, `g_rnn_*`, `rnn_pred_write`, which already live in `traj_iface.c`).

## 3. Constants (identical to `residual_nn.rs`)
| Name | Value |
|---|---|
| `NS2_MAX_NEIGHBOURS` | 3 |
| `PHI_IN_S` / `PHI_IN_G` | 6 / 4 |
| `PHI_L1, L2, L3` | 25, 40, 40 |
| `HIDDEN` | 20 |
| `RHO_L1, L2, L3, OUT` | 40, 40, 40, 1 |
| weights phi_S / phi_G / rho_S | 3675 / 3625 / 4161 (= 11 461 floats, 45.8 kB flash) |
| gate | `|dx| < 0.2`, `|dy| < 0.2`, `|dvx| < 1.5` (m, m, m/s); dz, dvy, dvz not gated |
| `GRAMS_TO_NEWTONS` | 9.81 / 1000 |
| `OUT_CLAMP` | 8.0 m/s² |

Weight layout of the source vector (19 297 floats, `npz["weights"]`): `phi_S | phi_L | phi_G | rho_S | rho_L` at offsets 0, 3675, 7350, 10975, 15136. The generator slices **phi_S [0:3675], phi_G [7350:10975], rho_S [10975:15136]** only (phi_L / rho_L are unused: every vehicle in the project is "small"); it asserts length 19 297, all finite, and writes the sha256 of the npz into the file.

Per-layer layout (PyTorch `nn.Linear`): `n_out × n_in` weights row-major, then `n_out` biases; ReLU on layers 1–3, **linear** layer 4.

## 4. Algorithm (one evaluation)
Inputs: own position `p` (m, world), own velocity `v` (m/s, world), mass `m` (kg), peers via `peer_get_all(xs, ys, zs, ts, 3)` (positions in m, `ts` in ms).
1. For each peer slot `k < n`: velocity of the peer by differencing (exactly the Rust rules): `pt == 0` → store, v = 0; `ts == pt` → reuse stored v; `0 < dt_ms < 500` → v = (p − p_prev)·1000/dt_ms, store; otherwise (gap ≥ 500 ms, wrap, backwards) → store, v = 0. Relative state `dp = p_peer − p`, `dv = v_peer − v`.
2. `rho_in[20] = 0`. Ground term (always): `phi_G([0 − p.z, −v.x, −v.y, −v.z])` added to `rho_in`.
3. For each peer: if gated → `phi_S([dp.x, dp.y, dp.z, dv.x, dv.y, dv.z])` added to `rho_in`.
4. `faz_g = rho_S(rho_in)` (scalar, grams).
5. `a = faz_g · GRAMS_TO_NEWTONS / m`. If not finite → output 0 and `clamped = 1`. Else vector `(0, 0, a)`; if `|a| > 8` → scale to 8, `clamped = 1`.
Output: acceleration (m/s²), world frame, **z component only**, positive = the network says the net disturbance acceleration is up.

Decimation (as `rnn_predict`): `do_eval = (tick_since_reset == first) || (main_tick % rnn.div == 0)`; on non-eval ticks the held value and clamp flag are re-published. The C hook counts **main-loop ticks** (the `tick` argument, 1 kHz); Omar's controller body runs on even ticks only, so `rnn.div = 10` ⇒ 100 Hz as before. `rnn.div` must be even for the intended rate (an odd value would evaluate on every even tick that is a multiple of it, i.e. at a lower rate than the label suggests); the study value stays 10.

State per drone instance (lives in `controllerOmarIndi_t`, one per drone in the host simulator): `peer_prev[3] = {x, y, z, ts}`, `peer_vel[3]`, `hold[3]`, `hold_clamped`, `first` flag. `ns2_reset()` clears it (called from `controllerOmarIndiReset`/Init so a re-arm starts clean, as `reset()` in Rust).

## 5. Public API (`ns2_residual.h`)
```c
#define NS2_MAX_NEIGHBOURS 3
typedef struct { float x, y, z; uint32_t ts; } ns2_peer_prev_t;
typedef struct {
  ns2_peer_prev_t prev[NS2_MAX_NEIGHBOURS];
  float vel[NS2_MAX_NEIGHBOURS][3];
  float hold[3];
  uint8_t hold_clamped;
  uint8_t started;
} ns2_state_t;

void ns2_reset(ns2_state_t *s);
/* Full evaluation from peers (peer_get_all). Pure function of (state, inputs, weights). */
void ns2_eval(ns2_state_t *s, const float pos[3], const float vel[3], float mass,
              float out_accel[3], uint8_t *clamped);
/* Same, with the relative states given explicitly (used by the equivalence tests; no peer history). */
void ns2_eval_rel(const float rel_p[][3], const float rel_v[][3], int n,
                  float own_z, const float own_vel[3], float mass,
                  float out_accel[3], uint8_t *clamped);
```
`ns2_eval` = steps 1–5; `ns2_eval_rel` = steps 2–5 with caller-supplied relative states. Both use the same internal forward function so the tests cover the flight path. No dynamic memory, static scratch (`phi` 25/40/40/20, `rho` 40/40/40/1, `rho_in` 20 floats), non-reentrant per image (as the Rust module; the host runs drones sequentially in one thread).

## 6. Hook in Omar C (additive, no-op when `rnn.en = 0`)
In `controllerOmarIndi()`, inside the position-controller branch, after `a_d` is computed and before the thrust is formed:
```c
/* NS2 (2026-10-14): learned residual, subtracted like the measured residual. */
struct vec a_nn = vzero();
ns2_hook_update(self, state, tick);          // decimated ns2_eval + rnn_pred_write + timing; sets g_rnn_ready = 1
if (g_rnn_en != 0) a_nn = mkvec(self->ns2_hold[0], self->ns2_hold[1], self->ns2_hold[2]);
... a_d = a_d - a_nn;                         // equivalent to adding to a_indi with the sign Omar's residual has
```
- The hook is called every executed controller tick but evaluates only on `tick % rnn.div == 0` (plus the first call). It runs **in all strategies** (S0–S3), logs `rnn.pred_*`, `rnn.clamped`, and updates `rnn.us_last/max/avg`.
- `g_rnn_ready` is set to 1 by the hook (the Rust `rnn_service` that does this for controller 6 does not run when controller 9 is selected).
- `g_rnn_en`, `g_rnn_div`, `g_rnn_rst`, `g_rnn_us_*`, `rnn_pred_write()` are the existing symbols in `traj_iface.c` (host: `host/oot_host.c`). No new parameter, no new log name.
- Sign: `a_nn` estimates the measured residual (`a_meas − a_model`). Omar's INDI term is `a_rpm_f − a_imu_f = −(that)`, added to `a_d`. NS2 therefore enters as `a_d − a_nn`; thrust and the desired rotation use `a_d + a_indi` as before. With `indi = 0` only the `a_nn` term is active (S2); with `indi = 3` both (S3).
- `a_nn` is not used if the position controller branch is not taken (attitude/thrust-only setpoints), exactly as `a_indi`.
- No change to any other line of Omar's law.

## 7. Invariants to test (Phase 4)
1. `rnn.en = 0` ⇒ thrust and torque outputs bit-identical to the current build on logged inputs (replay).
2. `ns2_eval_rel` equals Rust `ResidualNet::eval` for the same weights and inputs within the agreed tolerance: random relative states, 0–3 neighbours, gate boundaries (|dx| = 0.199 / 0.201, |dvx| = 1.49 / 1.51), ground term only, output clamp, NaN/Inf inputs.
3. Peer-history rules reproduce `lib.rs` on a scripted timestamp sequence (first seen, repeated ts, normal, gap ≥ 500 ms, backwards).
4. Decimation: with `rnn.div = 10` and main ticks 0…99, exactly the first call plus every 10th main tick evaluate (and the held value is published in between).
5. Generator: `ns2_weights.c` re-parsed equals the npz slices bit-for-bit; sha256 embedded.

## 8. Not decided here (flagged)
- **Tolerance for invariant 2 and the bench timing limit** — you define (the plan proposes ~1e-5 relative, float32).
- Whether the weights of the **retrained** network keep exactly this layout: yes by decision (same architecture); a different layout means a new generator, not a new module.
