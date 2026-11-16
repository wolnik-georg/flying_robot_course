# DiffCoTune, Stage 2 for S0: simulator work, forward and gradient checks (2026-11-11)

Paste everything below the line into the tuning (main) agent.

---

Your Stage 0 + 1 work was checked against the files and the paper. It holds up:
- the thrust-cap finding is right (`real_thrust` = min(quadratic law, cap));
- the integer RPM delay, the rounded S0–S3 inertia and the oval files on the flight side are right;
- θ_l as the start of each update is right (Alg. 1 line 4);
- the stop-rule unit issue is right.

## User decisions (11 Nov evening), now in plan section 5b

1. **Thrust β = thrust-law gain** (`THRUST_LAW_GAIN_N`), not the force cap. The cap stays fixed.
2. **β per controller = only the values that act on it:**
   - RPM delay only for S1 / S3 (later, as a smooth fractional delay);
   - downwash and plume scales are fitted from A1 / A8 of the same round;
   - circle and figure 8 are flown solo.
3. **FBL:** flash `fbl_v2b` later (lab item), the image the simulator represents.
4. **4 FBL variants kept:** a net-only model is trained later.
5. **S0 first end to end:** this prompt is Stage 2 **for S0 only**, plus the shared parts every controller needs later. The other controllers follow after the S0 dry run.
6. Each update starts from θ_l (paper). The cost is rescaled so the 1e-3 stop rule is meaningful; the unit is chosen in the dry run, not now.

**S0 β set:**
- thrust-law gain;
- motor lag;
- downwash scale;
- plume amplitude scale.

There is no RPM delay in S0.

## Where the code goes

- **New package `experiments/diffcotune/`** for all DiffCoTune code: the differentiable S0 model, the loop parts, the checks, a README.
- **Do not edit the old tuning files** (`experiments/tuning/diff/*_torch.py`, `run_job*.py`, `optimize*.py`); they are the record. Import them, or copy into `experiments/diffcotune/` with a header naming the source file. Prefer a copy where you must change behaviour.
- **`simulator/` (frsim)** gets the options the dry run needs.
  - **Every new option defaults to today's behaviour.** `tests/test_equivalence.py` must still give 199 / 199 identical after your changes.
  - No commit or push of `simulator/`; it stays local until the user says so.

## Do

### 1. frsim as the "real drone" (in `simulator/`)

1. **β overrides:** thrust-law gain, motor lag, downwash scale, plume amplitude scale, settable from Python and from `run_scenario.py` / `evaluate_gains.py` (e.g. `--plant thrust_gain=…,motor_lag_ms=…`). The overrides are recorded in `summary.json`.
2. **Full state in the trace:** position, velocity, attitude quaternion, body rates and the reference, per time step (CSV + `summary.json` fields).
3. **Noise switch:** `--no-noise` (all sensor noise off). Default stays noise on.
4. **New scenarios, solo:**
   - **circle:** existing `Circle` curve;
   - **figure 8:** existing `Lemniscate` curve;
   - **oval:** new curve.

   Take the parameters (size, period, height, laps, takeoff / landing ramps) from the existing crazyswarm2 flight CSVs / meta files (read only); name the source file per scenario in `docs/SCENARIOS.md`. If several versions exist, list them and use the one closest to the A-scenario speeds; say which you chose and why. Add tuning / test cells for them. A4 stays as it is.
5. Docs: README / `docs/SCENARIOS.md` / `docs/MODEL.md` updated for the new options and scenarios (short).

### 2. Differentiable S0 model (in `experiments/diffcotune/`)

1. **Plant:**
   - exact Omar inertia J = 16.571710e-6 / 16.655602e-6 / 29.261652e-6 kg·m² and mass 0.0427 kg, the same as frsim;
   - the same thrust law as frsim, with the **thrust-law gain as a differentiable β**;
   - motor lag, downwash scale and plume amplitude scale as differentiable β.
2. **No noise** in the tuning model (spec).
3. **Same scenarios as frsim:** A1, A4, A8, circle, figure 8, oval, with the same reference.
4. **Output:** the full state per step (position, velocity, quaternion, body rates) for J_sysId, and position for J_task.
5. **Gains:**
   - θ = the S0 gain set (same names and defaults as `simulator/gains/defaults.yaml`);
   - positivity by a log parametrisation or clipping (state which);
   - β parametrised relative to the start value (e.g. log-scale), as the spec's open point 7 suggests.

### 3. Forward check: differentiable S0 model vs frsim (the gate for Stage 3)

- **Cells:** S0 × {A1, A4, A8, circle, figure 8, oval} × default gains and 2 other gain sets (e.g. the old S0 A8 pick and a random set within ±30 % of the defaults), with noise off in both.
- **Shifted plants:** repeat for 2 shifted β sets (e.g. thrust gain × 0.8 with motor lag 45 ms, and downwash scale × 1.3 with plume scale × 0.7).
- **Report per cell:**
  - 3-D RMS of each model and their relative difference;
  - max position difference over the flight (mm);
  - max attitude difference (deg);
  - crash flag.
- **Pass:**
  - relative 3-D RMS difference ≤ 0.5 % (the tolerance of the earlier copies' checks) and no crash-flag mismatch;
  - the max position difference is reported, not gated.

  If a cell fails, find the cause; do not loosen the gate yourself. Report and stop.

### 4. Gradient check

- Autograd vs central finite differences (float64) for every θ and every β, on 3 cells (A1, A8, circle), for J_task and J_sysId (J_sysId against a frsim trace with shifted β).
- Report the relative error per parameter.
- Pass: ≤ 1 % for every parameter (our choice, stated as such).
- Flag any parameter whose gradient is zero or not finite.

### 5. Ingest sketch (no lab data yet)

- A function that turns one frsim trace (stand-in for a uSD log) into the J_sysId target on the model's time grid: state alignment, time base, takeoff / landing cut.
- Use the same alignment rule the real uSD path will need (Stage 1 item 4): scenario time t = 0.
- The real uSD reader comes in Stage 4.

## Checks (report each)

1. `simulator/tests/test_equivalence.py` full: 199 / 199 identical after your frsim changes.
2. `simulator/tests/test_smoke.py` plus a smoke run of each new scenario with S0 default gains in frsim (no crash, plot saved in the results folder).
3. Forward check table (all cells), pass yes / no.
4. Gradient check table, pass yes / no.
5. Runtime: one S0 rollout of each scenario in the differentiable model (forward and forward + backward, CPU and GPU), and one frsim episode. This sizes the Stage 3 dry run.

## Report back (short)

1. Files created / changed (tree with line counts); `simulator/` changes listed separately.
2. The new scenarios: source file, parameters, a plot of the 6 references.
3. Equivalence 199 / 199 yes / no; smoke yes / no.
4. Forward check: the table, pass yes / no, causes of any failure.
5. Gradient check: the table, pass yes / no.
6. Runtimes.
7. Anything in the spec or plan that must change after this stage.

## Rules

- No old jobs run; resources are free, but stay polite:
  - `nice -n 19`;
  - at most 16 CPU processes;
  - one GPU job at a time;
  - load ≤ 22, MemAvailable ≥ 3 GB before each start.
- Edits only in the new `experiments/diffcotune/` and in `simulator/` (new options, scenarios, docs). Do not edit or delete anything else in `experiments/tuning/`, `out/`, the old docs, `docs/papers/`, the meeting docs or the thesis. You may update `docs/diffcotune_spec.md` / `docs/diffcotune_inventory.md` where this stage changes them, marked "Stage 2".
- Firmware trees (`~/Desktop/crazyflie-firmware`, `~/Desktop/fbl_v2_firmware`) and `~/Desktop/crazyswarm2`: read only, read-only git commands only. No firmware rebuild (the frsim build in `simulator/_build/` stays as it is).
- No branch switch, checkout, stash, commit or push. No Co-Authored-By. Nothing flashed or flown. No reboot; do not touch the NVIDIA driver.
- No `pkill`, `killall` or kill by pattern; stop only your own processes by exact PID.
- Never invent numbers or rules; write "not stated" and list it as open.
- Time stamps only from the real `date` output. Method names in full in docs; FBL is just "FBL"; say "test error", never "study score".
