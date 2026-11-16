# DiffCoTune, Stage 4d for S0: fix the flight-space check, turn the circle, write in the user's protocol decisions (desk only, 2026-11-13)

Paste everything below the line into the tuning (main) agent.

---

Your Stage 4c report was checked against the files. Nothing in crazyswarm2, `flying_drone_stack/` or `simulator/` changed, and the new files exist. **One error is safety relevant.**

## The error: the pad windows of CIRCLE and OVAL

`fly_file.py` (docstring line 7 and `check()`) assumes that the file's origin is the hover point. That is not how a relative start works:
- `startTrajectory(..., relative=True)` is the crazyflie_py default (`crazyflie_py/crazyflie_py/crazyflie.py:417–419`).
- The firmware then shifts the trajectory so that its **first point** sits at the current position: `shift = start_from − traj_eval(t_begin).pos` (`~/Desktop/crazyflie-firmware/src/modules/src/planner.c:335–348`).

So the path's extent counts from the first point, not from the file origin. The circle and oval files start at the edge (x = +0.75 / +0.655 m), so the paths about the hover point are:

| File | Path about the hover point | Pad x window | Pad y window | cf5 pad (0.0, 0.3) |
|---|---|---|---|---|
| CIRCLE | x −1.5…0, y ±0.75 | [0.50, 1.00] | [−1.25, 1.25] | outside (reaches x = −1.5) |
| FIG8 (turned) | x ±0.46, y ±0.92 | [−0.54, 0.54] | [−1.08, 1.08] | ok |
| OVAL | x −1.31…0, y ±1.8 | [0.31, 1.00] | [−0.20, 0.20] | outside |

Flight space x ±1.0, y ±2.0, z 0–1.70 m. Your check passed CIRCLE, so it would have let an unsafe flight through.

This also means your open point "the circle and oval files start with a +0.75 / +0.655 m setpoint step" does not exist. The firmware shift removes the step. Remove that point.

## User decisions (13 Nov)

1. **Applying the gains:** option A, merge into `crazyflies.yaml` each round, then read back on a fresh connection (`lab/apply_gains.py merge` / `readback`).
2. **Flight files:** the `lab/fly_file.py` wrapper, after the check is fixed. Nothing is copied into crazyswarm2.
3. **Flights per scenario:** 1 per tuning scenario per round (as in the dry runs).
4. **Order in a round:** A1, A8 (both drones), then CIRCLE, FIG8 (`cf_second` disabled).
5. **CIRCLE:** turned by 90° so that it fits at the current pad, as for the figure 8. The frsim reference stays unchanged; the ingest maps the frame.
6. **OVAL:** dropped from the **lab** test set. Lab test cells are A4 cell 1 and A4 cell 2. The dry-run results and the simulation test set stay as they are.
7. **Still open, the user's call:** the number of test flights, and the battery rule (flights per pack, repeats).

## Do (desk only)

### 1. Fix the check

- In `fly_file.py`, compute the path extent relative to the **first point** of the trajectory (evaluate the polynomials, subtract the position at t = 0), then the pad window against `FLIGHT_SPACE`. Fix the docstring (cite `planner.c:335–348`).
- Also check the `--height` (z window) and the margins in each axis.
- **Test:** reproduce the table above for the current files (CIRCLE, FIG8, OVAL); the windows must match to 1 cm.
- Check the same way every other place that states a flight volume or pad window: `flightfiles/checks.json`, `make_flightfiles.py`, `cmd_check.py`, the protocol draft and its figure `lab/figs/flightfiles_volume.png`, `report_stage4a`/`4c` text. Correct each one, and say which were wrong.
- For A1, A8 and the A4 cells (`run_formation`): state whether the formation trajectories start relative or absolute (read the code), and check the flight volume of each cell at the current pads on the same basis.

### 2. Turn the circle by 90°

- Write `flightfiles/circle_frsim_rot90_tuning_mode1.csv`, turned so that the path fits the current cf5 pad (0.0, 0.3) with the **largest** margin. Of the two turns (+90° or −90°), pick the one with the larger smallest margin and state both.
- **Checks** (as in Stage 4a): sampled path against the frsim reference after the turn, max difference ≤ 1 mm; peak speed and acceleration unchanged; number of pieces ≤ 31; pad windows by the fixed check; smallest horizontal distance to the `cf_second` pad (parked on the floor while disabled).
- **Ingest:** the frame fit must find the turn, as for the figure 8. Add a test like `lab/test_frame_fig8.py` for the circle (angle, residual).
- Keep the old circle file, marked "not for the lab: does not fit at the current pad".

### 3. Write in the decisions

- **Protocol draft:**
  - write decisions 1–6 as decided (with "user, 13 Nov");
  - remove the setpoint-step point;
  - lab test cells A4 ×2, OVAL dropped;
  - flight card per round: A1, A8, CIRCLE (turned), FIG8 (turned), each with its command line, `rotate_deg` for `flights.json` and the pad check;
  - keep the number of test flights and the battery rule as open.
- **Lab time table:** 4 rounds × 4 tuning flights, with the desk time per round. Test flights are added once the user decides their number.
- `fly_file.py`: drop OVAL from the lab cells, or refuse it with a clear message.

## Report back (short): `dryrun/stage4d/report_stage4d.md`

1. The fixed check: the table for the current files, matching the one above yes / no; the places that were wrong and are now corrected.
2. A1, A8, A4: relative or absolute start, and their volume check.
3. Turned circle: which turn, margins, reference difference, frame test.
4. The protocol draft changes.
5. What is still open for the user.

Also update `lab/README.md`, and `docs/diffcotune_spec.md` and `docs/diffcotune_inventory.md` marked "Stage 4d".

## Rules

- **Resources:** desk work, little CPU. If a frsim check needs a pool: one pool at a time, `nice -n 19`, via `dryrun/stage3c/job_runner.py` (load ≤ 22 and MemAvailable ≥ 3 GB before and one minute after a start), `TORCHINDUCTOR_COMPILE_THREADS=1`, 2 torch threads per worker. Whole stage within 1.5 h.
- **Shells:** no command that waits on standard input (no bare `cat >`); use heredocs or the edit tools.
- **Python:** `/usr/bin/python3` (3.10) for frsim and the model; `~/.pyenv/versions/flying_robots/bin/python` for plots and log analysis; no pip installs.
- **Edits only in:**
  - `experiments/diffcotune/` (with `dryrun/stage4d/`, `lab/`, `flightfiles/`);
  - `docs/diffcotune_lab_protocol_S0_DRAFT.md`;
  - `docs/diffcotune_spec.md` and `docs/diffcotune_inventory.md`, marked "Stage 4d".
- **Do not touch:**
  - `simulator/`, `flying_drone_stack/`, `~/Desktop/crazyswarm2`, `~/Desktop/crazyflie-firmware`, `~/Desktop/fbl_v2_firmware`: read only; read-only git commands only; no firmware rebuild; nothing copied into crazyswarm2;
  - `experiments/tuning/`, `out/`, the old docs, `docs/papers/`, the meeting docs, the thesis, `docs/comparative_study_protocol.md`;
  - Stage 3b–4b results.
- **Data:** complete uSD logs only, never radio CSVs.
- **Git and hardware:**
  - no branch switch, checkout, stash, commit or push;
  - no Co-Authored-By;
  - nothing flashed or flown; no connection to a drone or radio;
  - no reboot; do not touch the NVIDIA driver.
- **Stopping processes:** no `pkill`, `killall` or kill of a PID found by pattern. Only exact PIDs you started yourself, checked with `ps -o pid,ppid,args -p <PID>`.
- **Facts:** never invent numbers or rules; write "not stated" and list it as open.
- **Wording:**
  - time stamps only from the real `date` output;
  - method names in full in docs;
  - FBL is just "FBL";
  - say "test error", never "study score".
