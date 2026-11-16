# DiffCoTune, Stage 3 for S0: four fixes, then the simulator-to-simulator dry run (2026-11-11)

Paste everything below the line into the tuning (main) agent.

---

Your Stage 2 work was checked against the files:
- **Forward check:** confirmed (30 / 30, worst 0.015 %).
- **Equivalence:** rerun by the user's side with `--quick`, 22 / 22 identical.
- **Old `experiments/tuning/`:** untouched.
- **Scenario numbers:** confirmed from the cache: peak acceleration CIRCLE 2.5, FIG8 3.4, OVAL 3.0 m/s² against ≤ 0.32 for A1 / A4 / A8; OVAL y ±2.2 m.
- **`frsim/flightplan.py`:** also changed; it was missing from your list. Please list it.
- **The blocked kill:** never derive a PID from a pattern. Only exact PIDs you started yourself, checked with `ps -o pid,ppid,args -p <PID>`.

## User decisions (11 Nov evening)

1. **Gradient gate** on the cost the optimiser actually uses: the mean over A1, A8, CIRCLE, FIG8 (J_task and J_sysId).
   - Pass per parameter: same sign **and** (relative error ≤ 1 % **or** absolute error ≤ 0.1 % of the largest gradient component of that cost).
   - Single-scenario tables stay in the report, not gated.
2. **CIRCLE and FIG8:** keep the shape and lap time of the flight files, but use smooth start / end ramps of about 2 s, so the peak acceleration drops to about the curve's own level.
3. **OVAL:** long axis scaled to 1.8 m (0.2 m margin to the ±2.0 m flight volume), same speed along the path. It stays a test scenario.
4. **Stop rule:** each inner loop's cost is normalised by its value at the start of that inner loop, so 1e-3 means a 0.1 % change. Confirm or reject this in the dry run (item B4).

## Part A: fixes (in `simulator/` and `experiments/diffcotune/`)

1. **Ramps and oval:**
   - Ramps per decision 2, oval per decision 3, in frsim (`solo.py`) and in the model's reference.
   - Report the new peak speed / acceleration of CIRCLE, FIG8, OVAL and the new oval extent.
   - Note in `docs/SCENARIOS.md` that the ramps / oval differ from the flight files, and that Stage 4 must export matching flight files.
2. **Re-run the forward check** on all 30 cells (the references changed). The gate stays at 0.5 %.
3. **Gradient check per decision 1:**
   - combined-cost table, pass yes / no;
   - single-scenario tables reported.
4. **`simulator/tests/test_equivalence.py --quick`:** still 22 / 22 identical (the A-scenario cells must not change).

Stop and report if the forward check or the combined gradient gate fails.

## Part B: the S0 dry run (no flights)

### Setup

**"Real drone"** = frsim (the real firmware code) with **noise on** and shifted β, two plants:
- **P1:** thrust-law gain × 0.8, motor lag 45 ms, downwash scale × 1.3, plume scale × 0.7.
- **P2:** P1 plus a mismatch no β covers: state delay `md10` (the measured one, `frsim/variants.py`). This shows what happens when the model cannot fully match the real drone, as on hardware.

**Rest of the setup:**
- **Tuning model:** the differentiable S0 model, no noise, starts from the frozen calibration β₀ and the S0 default gains θ₀.
- **Tuning scenarios:** A1, A8, CIRCLE, FIG8. **Test scenarios:** A4, OVAL (never used in the loop).
- **One round** (spec):
  1. Fly θ_l once in frsim on each tuning scenario (one noise seed per flight; the round number sets the seed).
  2. Fit β (θ_l fixed, J_sysId, full state).
  3. Tune θ (β fixed, J_task, 3-D position).
  4. The next round flies θ_{l+1}.

  The update starts from θ_l (spec). Rounds l = 0 … 5. θ_best = best real cost among the 6 flown gain sets.
- **Adam:** K/2 = 50 epochs per inner loop, stop rule per decision 4, Adam state reset at the start of each inner loop (state this as our choice; the paper does not say).

### B1. Learning rate (the paper does not state it)

- A short pilot on P1 only: 3 learning rates in the log-parametrisation, e.g. 0.01 / 0.03 / 0.1, the same value for θ and β, 2 rounds each.
- Pick the one with the lowest real cost after round 2, freeze it, and use it for everything below and later for all controllers.
- Report the pilot table. This is a desk choice on the shifted simulator, stated as such in the spec.

### B2. Full loop

Run P1 and P2, 6 rounds each, with the frozen learning rate.

### B3. Comparisons (paper ablations, same flight budget)

- **(a) Default gains:** θ₀, no tuning.
- **(b) Tune only, no fit** ("DiffTune with model rollout"): θ tuned in the unfitted model β₀, one update per round, the same epochs.
- **(c) SysId then tune:** fit β once on 6 flights of θ₀ (the same flight count as the loop), then tune θ once.
- **(d) DiffCoTune** (B2).

**Final evaluation** of each θ_best in the "real drone":
- 5 noise seeds on every tuning and test scenario;
- 3-D RMS and z error in cm, crashes.

### B4. Report per plant (P1, P2)

1. **β over the rounds:** each of the 4 values vs its true shifted value (plot), and the final error.
2. **Real cost per round** (the learning curve) for (b), (d), and the line for (a) and (c).
3. **Final table:** method × scenario (tuning and test): 3-D RMS / z error mean over 5 seeds, crashes.
4. **θ over the rounds** (plot), and θ_best vs θ₀.
5. **Inner loops:** epochs used, which stop fired (converged / rose), and whether the normalised stop rule (decision 4) behaves sensibly. Recommend keep or change.
6. **Adam on weak parameters:** the epoch-by-epoch trace of motor lag and plume scale. Flag steps that go against the finite-difference sign at that point (check a few epochs).
7. **Wall time per round** (fit + tune, 4 scenarios in parallel on CPU). Target ≤ 15 min.

### Pass criteria (P1; report P2 the same way, as information)

- β moves toward the true values (the thrust-law gain within 5 % after round 5; report the others).
- The real 3-D RMS of DiffCoTune's θ_best is lower than (a) default on the tuning scenarios **and** on the test scenarios (mean over 5 seeds).
- (d) is not worse than (b) and (c). If it is, report it; do not change the method.
- No crash of θ_best in the final evaluation.
- Wall time per round ≤ 15 min.

If a criterion fails, find and report the cause. Do not change the method, the costs or the gates yourself.

## Report back (short)

1. Part A:
   - the new scenario numbers;
   - forward check pass yes / no;
   - combined gradient gate table, pass yes / no;
   - equivalence 22 / 22 yes / no.
2. B1 pilot table and the chosen learning rate.
3. B4 items 1–7 for P1 and P2 (plots in `experiments/diffcotune/dryrun/`, tables in the report).
4. Pass criteria: each yes / no.
5. Spec / plan changes this stage suggests (also written into `docs/diffcotune_spec.md`, marked "Stage 3").

## Rules

- **Resources:** `nice -n 19`, at most 16 CPU processes, one GPU job at a time, load ≤ 22 and MemAvailable ≥ 3 GB before each start.
- **Edits:** only in `experiments/diffcotune/` (new `dryrun/` folder for scripts, logs, plots), in `simulator/` for the Part A fixes (every new option defaults to today's behaviour), and in `docs/diffcotune_spec.md` / `docs/diffcotune_inventory.md` (marked "Stage 3"). Nothing else in `experiments/tuning/`, `out/`, the old docs, `docs/papers/`, the meeting docs or the thesis.
- Firmware trees and `~/Desktop/crazyswarm2`: read only. No firmware rebuild.
- No branch switch, checkout, stash, commit or push. No Co-Authored-By. Nothing flashed or flown. No reboot; do not touch the NVIDIA driver.
- No `pkill`, `killall`, or kill of a PID found by pattern; only exact PIDs of your own processes.
- Never invent numbers or rules; write "not stated" and list it as open.
- Time stamps only from the real `date` output. Method names in full in docs; FBL is just "FBL"; say "test error", never "study score".
