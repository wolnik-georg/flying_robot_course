# DiffCoTune code (experiments/diffcotune)

All code of the DiffCoTune auto-tuning (plan `docs/plan_diffcotune_2026-11-11.md`, spec `docs/diffcotune_spec.md`, inventory `docs/diffcotune_inventory.md`).
The old tuning pipeline (`experiments/tuning/`) is the record and is not edited; files copied from it carry a header naming the source.

State of 11 Nov 2026 (Stage 2, S0 only): the differentiable S0 model, its checks, the cost functions and an ingest sketch. The loop itself
(fit beta, tune theta, round command) is Stage 3 / 4.

| File | What |
|---|---|
| `s0_model.py` | differentiable S0 on the frsim plant: exact Omar inertia, thrust-law gain / motor lag / downwash scale / plume amplitude scale as differentiable beta (log-factors relative to the calibration), 10 S0 gains theta (log-factors relative to `simulator/gains/defaults.yaml`), no noise, full-state output |
| `reference.py` | reference of any frsim scenario cell on the 1 ms grid, built with frsim's own `FlightPlan` (cached in `cache/`), and frsim's float-rounding jitter of the 4 ms command delay |
| `costs.py` | J_task and J_sysId of the spec (units / window / attitude form are open choices of the spec, placeholders documented in the file) |
| `ingest.py` | ingest sketch: a full-state trace (frsim now, uSD log in Stage 4) -> J_sysId target on the model's time grid |
| `s0_torch.py`, `ns2_torch.py` | copies of `experiments/tuning/diff/s0_torch.py` (controller + mixer) and `ns2_torch.py` (downwash network), header names the source |
| `checks/forward_check.py` | forward check model vs frsim (6 scenarios x 5 parameter rows), gate: relative 3-D RMS difference <= 0.5 % |
| `checks/gradient_check.py` | autograd vs central finite differences (float64) for all 14 parameters, J_task and J_sysId, 3 cells, pass <= 1 % |
| `checks/runtime.py`, `checks/runtime_batch.py` | runtime of one rollout (frsim; model forward / forward + backward on CPU and GPU; against the number of rows) |
| `checks/plot_references.py`, `checks/references_6.png` | the six references |
| `checks/*.json`, `checks/*.log` | results of the last runs |

## Conventions
* Parameters are log-factors: `phi` (B, 10) for the gains (order of `frsim.controllers.TREE_GAIN_NAMES`), `psi` (B, 4) for beta (order `s0_model.BETA_NAMES`: `thrust_gain`, `motor_lag_ms`, `downwash_scale`, `plume_scale`). `0` = defaults / frozen calibration. Positivity is by construction (exp), no clipping is needed; the spec's rule "clip to stay positive" is therefore automatically satisfied.
* Output state = frsim's state trace: state at the start of tick i, rows every 10 ms, quaternion (x, y, z, w), body rates in rad/s, scenario time t = 0 at the scenario start.
* Run with the system `python3` (3.10, torch 2.5.1, numpy, scipy; the frsim firmware bindings are needed for frsim episodes only). Plots with `~/.pyenv/versions/flying_robots/bin/python`.

## Results of the last checks (11 Nov 2026)
* Forward check (30 cells, noise off): worst relative 3-D RMS difference 0.015 % (gate 0.5 %), worst max position difference 0.47 mm, no crash anywhere: PASS (`checks/forward_check.log`).
* Gradient check (h = 1e-6 in log-parameters): A8 and circle pass (worst 0.47 %); A1 fails the 1 % gate for Dz, motor lag and plume scale (marginal vertical equilibrium of the noise-free A1 flight, see `docs/diffcotune_spec.md` section 8 item 6 and `checks/gradient_a1_*.log`): FAIL overall (`checks/gradient_check.log`).
* Runtime: `checks/runtime.json`.

## Stage 5 (14 Nov 2026): S1, S2, S3
* Models on the frsim plant: `s2_model.py` (S0 + NS2), `s1_model.py` (INDI mode 3, fractional rpm delay as a fifth β), `s3_model.py` (S1 + NS2). `dryrun/loop_lib.py` selects them with `DIFFCOTUNE_CONTROLLER` (default S0; plants P1 and P1RD = P1 + rpm delay 8 ms). The fractional rpm delay in frsim (`Plant.rpm_delay_fractional`, off by default) leaves all earlier results unchanged (`tests/test_equivalence.py`: 199/199).
* Checks (`dryrun/stage5/checks5.py`): forward PASS for S1, S2, S3; noise gate PASS; gradient check FAIL on A1 for all three (S1/S3: explosion 1e20–1e24, S2: mismatch). Dry runs with criteria a–c: S2 yes, S1 yes, S3 yes (formally; flat to round 3). **Lab-ready on the desk side: S0 yes, S1 no, S2 no, S3 no.**
* Report: `dryrun/stage5/report_stage5.md` (diagnostics `jac1.py`, `jac0.py`, `eig1.py`, `tangent.py`, `revfwd.py`).

## Stage 5b (14 Nov 2026): forward-mode gradient
* `dryrun/loop_lib.py`: `GRAD_MODE` (`DIFFCOTUNE_GRAD_MODE`, default `reverse`) and kinds `task_F` / `sysid_F` / `task_R` / `sysid_R`. Forward mode is exact (gradient gate passed for S0–S3) but about 20× slower; not adopted. Reverse mode is wrong on A1 only (noise-free).
* Report: `dryrun/stage5b/report_stage5b.md`. `lab/round.py` default `--n-rounds` 5.

## Stage 5c (15 Nov 2026): exact gradient at normal speed; "Stage 5c, RPM delay fixed (user, 14 Nov)"
* `fsens.py`: compiled exact forward sensitivity. `loop_lib.GRAD_MODE` `mixed` (S1, S2, S3; S0 stays `reverse`), `FIX_RD` (S1, S3 keep the RPM delay at 4.0502 ms). Same gradient, other way of computing it; not a deviation.
* Round times S1 22.5, S2 25.9, S3 28.8 min. Lab-ready on the desk side: S0, S1, S2 yes; S3 no (θ_best = θ_0). Reports: `dryrun/stage5c/report_stage5c.md`, `dryrun/stage5c_rdfix/report_rdfix.md`.

## Stage 6 (15 Nov 2026): fast lab session
Desk-speed levers tested, none adopted (identity gate); two controllers at once measured (S0 24.8 / S2 30.7 min against 19.4 / 26.0); interleaved session tooling and timeline proposed, not decided. See `dryrun/stage6/report_stage6.md`, `lab/INTERLEAVED_LAB_CARD.md`, `dryrun/stage6/scenario_decision.md`.

## Stage 7 and Stage 8 (15 Nov 2026)
* Stage 7: five tuning scenarios (A1, A8, A4 cell 1, CIRCLE, FIG8), forward gradient everywhere (`cfwd`), S3 diagnosis (the noisy A1 cost of S3 is not smooth), NS2 weights check in `lab/preflight.py`. See `dryrun/stage7/report_stage7.md`.
* Stage 8: paper-faithful variant `DIFFCOTUNE_METHOD=paper` (full-state task cost, J_sysId with body rates, the paper's stop rule, no noise in the tuning step, no trust region, 6 rounds); `lab/round.py` defaults to it (`--method ours` for the frozen method). See `dryrun/stage8/report_stage8.md` and `dryrun/stage8/LAB_TOMORROW.md`.

## Stage 8b (15 Nov 2026, evening): lab variant frozen
The paper-faithful variant did not work (Stage 8); the lab variant is "ours", frozen, the same for S0–S3 (`lab/round.py` default `--method ours`): DiffCoTune's co-tuning loop with DiffTune's quadrotor settings, 5 tuning scenarios, 6 rounds, forward gradient. Every new gain set gets a frsim safety preview (`lab/preview.py`, a check only). See `docs/diffcotune_spec.md` section 24 and `dryrun/stage8/LAB_TOMORROW.md`.
