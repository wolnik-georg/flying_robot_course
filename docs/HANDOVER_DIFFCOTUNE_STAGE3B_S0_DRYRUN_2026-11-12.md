# DiffCoTune, Stage 3b for S0: fix the stop rule and the simulator-fit states, then rerun the dry run (2026-11-12)

Paste everything below the line into the tuning (main) agent.

---

Your Stage 3 work was checked against the files:
- **Report:** `report_stage3.md` matches your text, and the result JSONs exist.
- **Part A:** passed (ramps, oval, forward check, combined gradient gate, 22 / 22 equivalence).
- **Diagnostics (`diag_fit.log`):** confirmed.
  - D2 (clean flights, no stop rule, 100 epochs) recovers β.
  - D3 (noisy flights) misses downwash.
- **Old `experiments/tuning/`:** untouched.

The dry run found two faults in the setup, not in the code:
1. **The normalised stop rule (decision 4 of Stage 3)** stopped every β fit after 2–5 epochs, so β never moved and later rounds tuned in the wrong model. This rule was our choice, and it was wrong.
2. **Body rates made up 94 % of J_sysId under noise.** A noise-free model cannot match gyro noise, so that part is a floor. It hides position and velocity, which carry β.

## User decisions (12 Nov), now in plan section 5d

1. **Stop rule:** no early stop.
   - Each inner loop (fit and tune) runs the full 50 epochs and returns the iterate with the lowest model cost (best iterate, not last).
   - This replaces the normalised rule.
   - **Spec deviation 8 (stated):** the paper stops at |ΔJ| < 1e-3 or a rise > 1e-3 and returns the last iterate. K = 100 is kept.
2. **J_sysId states:** drop the body rates and use position, velocity and attitude, unweighted (same attitude error form as now).
   - Step B1 below decides between pos + vel + att and pos + vel.
   - **Spec deviation 9 (stated):** the paper uses all observable states.
3. **Learning rate:** redo the pilot after 1 and 2. The 0.01 choice was made with the broken stop rule.
4. **Update start:** stays θ_l (paper, Alg. 1). Check it again in the rerun.
5. **A4:** stays a test scenario. If the tuned gains are still worse than the default on A4 after the fixes, report it as a finding; do not add scenarios yourself.
6. **Wall time:** the target becomes about 20 min per round (fit + tune), because the fit now always runs 50 epochs.

## Do (S0 only, no flights)

Same setup as Stage 3 unless stated:
- plants P1 and P2;
- noise on in the "real drone", off in the model;
- tuning scenarios A1, A8, CIRCLE, FIG8; test scenarios A4, OVAL;
- rounds 0 … 5;
- keep-best among the 6 flown gain sets;
- Adam reset at the start of each inner loop.

Add a switch for the stop rule and for the J_sysId state set in `loop_lib.py` / `run_dryrun.py`. The default stays as in Stage 3, so the old results can be reproduced. Write new results to a new folder `experiments/diffcotune/dryrun/stage3b/`; do not overwrite the Stage 3 results.

### B1. Identifiability check (before any loop)

Use the D3 setup from `diag_fit.py`: P1, noisy frsim flights of θ₀ on the 4 tuning scenarios, fit only, 50 epochs, best iterate, 2 noise seeds.
- **Variants:** pos + vel + att, and pos + vel.
- **Learning rates:** 0.01 / 0.03 / 0.1.
- **Report:**
  - fitted β (all 4, as factors) against the truth;
  - the error per β;
  - the share of each state part in J_sysId at the start and at the end.
- **Pick** the state set that recovers β best: thrust-law gain first, then downwash, then the rest. If neither recovers downwash within 10 %, say so; do not change anything else.

### B2. Learning-rate pilot (redo)

As in Stage 3 B1: P1, the 3 learning rates, 2 rounds each, with the new stop rule and the state set from B1. Pick the one with the lowest real cost after round 2, then freeze it.

### B3. Full loop and comparisons

Rerun P1 and P2, with comparisons (a) default, (b) tune only, (c) sysId then tune, (d) DiffCoTune, under the same flight budget and the same final evaluation (5 noise seeds, every tuning and test scenario, 3-D RMS and z error in cm, crashes).

**(c) in Stage 3:** P1 gave 63.75 cm. Find out why (the β it fitted, the gains it tuned, which scenario blew up) and report it.

### B4. Report per plant

Same items 1–7 as Stage 3, plus:
- the Stage 3 vs Stage 3b numbers side by side, for β error, the learning curve and the final table;
- for (d): in which epoch the best iterate came, per inner loop;
- whether the real cost still rises after round 1 (item 4 of the decisions).

### Pass criteria

Same as Stage 3, with these changes:
- **P1:**
  - β moves toward the truth: the thrust-law gain within 5 % after round 5; report the others;
  - θ_best of (d) beats (a) default on the tuning **and** test scenarios (mean over 5 seeds);
  - (d) is not worse than (b) and (c); if it is, report it and do not change the method;
  - no crash of θ_best;
  - wall time about 20 min per round or less.
- **P2:** report the same items, as information.

If a criterion fails, find and report the cause. Do not change the method, the costs or the gates yourself.

## Report back (short)

1. B1 table and the chosen state set.
2. B2 pilot table and the chosen learning rate.
3. B4 items for P1 and P2: plots in `dryrun/stage3b/`, tables in `dryrun/stage3b/report_stage3b.md`.
4. Why (c) failed in Stage 3.
5. Pass criteria, each yes / no.
6. Spec changes (deviations 8 and 9, the learning rate, the state set) written into `docs/diffcotune_spec.md`, marked "Stage 3b".

## Rules

- **Resources:**
  - `nice -n 19`;
  - at most 16 CPU processes;
  - one GPU job at a time;
  - check load ≤ 22 and MemAvailable ≥ 3 GB before each start.
- **Edits only in:**
  - `experiments/diffcotune/`, with new results in `dryrun/stage3b/`;
  - `docs/diffcotune_spec.md` and `docs/diffcotune_inventory.md`, marked "Stage 3b".
- **Do not touch:**
  - `simulator/` (no changes needed this stage; if one is needed, stop and ask);
  - `experiments/tuning/`, `out/`, the old docs, `docs/papers/`, the meeting docs, the thesis.
- **Firmware trees and `~/Desktop/crazyswarm2`:** read only. No firmware rebuild.
- **Git and hardware:**
  - no branch switch, checkout, stash, commit or push;
  - no Co-Authored-By;
  - nothing flashed or flown;
  - no reboot; do not touch the NVIDIA driver.
- **Stopping processes:** no `pkill`, no `killall`, no kill of a PID found by pattern. Only exact PIDs you started yourself, checked with `ps -o pid,ppid,args -p <PID>`.
- **Facts:** never invent numbers or rules; write "not stated" and list it as open.
- **Wording:**
  - time stamps only from the real `date` output;
  - method names in full in docs;
  - FBL is just "FBL";
  - say "test error", never "study score".
