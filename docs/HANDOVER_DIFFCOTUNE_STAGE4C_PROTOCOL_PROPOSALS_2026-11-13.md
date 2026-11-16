# DiffCoTune, Stage 4c for S0: proposals for the open lab-protocol points (desk only, 2026-11-13)

Paste everything below the line into the tuning (main) agent.

---

Your Stage 4b report was checked against the files:
- **Dry test round 1:** θ₂ identical to `loop_P1.json`, gain-limit hits too; real cost 9.713 vs 9.711 cm; keep-best round 1; yaml round trip 8e-17. Confirmed.
- **Strict run:** same θ₂. Confirmed.
- **One pool at a time:** 6.3 GB free one minute after the start. Confirmed.
- **A1 tilt and abort check:** `lab/results/tilt_abort_check.json` matches your numbers.

One leftover: your shell PID 2684347 (child `cat`, PID 2684632) hangs on standard input since about 15:31. The user will stop it; do not start such a shell again (no `cat >` without input).

## User decisions (13 Nov), now in plan §5h and the protocol draft

1. **Lab rounds:** 4 (l = 0…3); `round.py --n-rounds 4`. Keep the code default as it is; the protocol names the flag.
2. **Abort rule** "|roll| or |pitch| > 25° for 0.5 s" is read as 0.5 s continuous.
3. **J_sysId** keeps position + velocity + attitude on A1 too.
4. **P2 scan:** skipped.

## Task: proposals for the open protocol points (desk only)

The user decides each point. You write **proposals** with sources, not rules. Read only; no changes outside `experiments/diffcotune/` and the protocol draft.

### P1. Command lines and `rotate_deg` (open point 2)

- For each tuning and test cell (A1, A8, CIRCLE, FIG8, A4 cell 1 and 2, OVAL): the exact command the lab would type, from the scripts the lab uses (`run_formation`, `simple_flight.py`, flags such as `--height`, `--auto-center`, `--hold`, `--duration`, `--period`, `--kt`), read from `~/Desktop/crazyswarm2` and `flying_drone_stack/` (read only).
- Each command must reproduce the frsim cell used in the tuning (height, dz, speed, duration, start position). Show the check: the commanded reference from the script against the frsim reference, max difference.
- `rotate_deg` per cell: what the lab script does to the frame, and the value the ingest needs (`flights.json`).
- If a cell cannot be flown with an existing script and flag, say so; do not change the scripts.

### P2. How the gain file is applied (open point 3)

- Read how crazyswarm2 sets firmware parameters (`crazyflies.yaml`, `firmware_params`, branches `study_S*`, runtime parameter setting) and how earlier lab sessions applied gains (lab docs in `docs/lab_sessions/`, the FBL priming script and its memory note: the controller init wipes yaml values on a fresh boot).
- Give 2–3 ways (e.g. merge into `crazyflies.yaml` on a branch per round; set at run time with a small script; …), each with: steps in the lab, time per round, risk of a wrong gain set, and how the readback is done.
- Recommend one, with the reason.
- If a script is needed, write it **only** in `experiments/diffcotune/lab/` (e.g. `apply_gains.py` that sets the parameters from `round_<l>/gains.yaml` and reads them back from the firmware, not from the cflib cache), not tested on hardware, and say so.

### P3. File names for the new flight files (open point 5)

- How `simple_flight.py` finds trajectory files (`figure8|circle|oval|hover` with `--kt`). Propose the least invasive way to fly `experiments/diffcotune/flightfiles/*.csv` (e.g. a file-path flag that already exists, a copy with the expected name at lab time, or a wrapper in `experiments/diffcotune/lab/`).
- Nothing is copied into crazyswarm2. If a crazyswarm2 change is the only way, describe it as a proposal for the user.

### P4. Remaining points (1, 4, 6): options with sources only

- **Flights per scenario and scenario order (1):** what the sim loop assumes (1 per scenario per round), what `docs/diffcotune_lab_plan_2026-11-02.md` says (4), and the lab time each gives for 4 rounds (use the throughput in `docs/comparative_study_protocol.md` §4 and the 19–20 min desk time per round). Table, no choice.
- **Gain readback (4):** follows from P2.
- **Battery rule for repeats (6):** existing rules only, with source; list what is not stated.

### P5. Lab time for one S0 loop

A table for 4 rounds: flights (incl. the expected repeats), flying time, desk time between rounds, total. Use only numbers from the repository; mark estimates.

## Report back (short)

Add a section "Proposals for the open points (Stage 4c)" to `docs/diffcotune_lab_protocol_S0_DRAFT.md` (plots first, little text, each item with its source), plus `dryrun/stage4c/report_stage4c.md`:
1. P1 table: cell → command, `rotate_deg`, reference check.
2. P2 options and your recommendation (and the script, if written).
3. P3 proposal.
4. P4 tables.
5. P5 lab-time table.
6. What is still open after this.

## Rules

- **Resources:** desk work, little CPU. If a frsim check needs a pool: one pool at a time, `nice -n 19`, via `dryrun/stage3c/job_runner.py` (load ≤ 22 and MemAvailable ≥ 3 GB before and one minute after a start), `TORCHINDUCTOR_COMPILE_THREADS=1`, 2 torch threads per worker. Whole stage within 2 h.
- **Python:** `/usr/bin/python3` (3.10) for frsim and the model; `~/.pyenv/versions/flying_robots/bin/python` for plots and log analysis; no pip installs.
- **Edits only in:**
  - `experiments/diffcotune/` (with `dryrun/stage4c/`, `lab/`);
  - `docs/diffcotune_lab_protocol_S0_DRAFT.md` (the new proposals section; do not change the decided items);
  - `docs/diffcotune_spec.md` and `docs/diffcotune_inventory.md`, marked "Stage 4c".
- **Do not touch:**
  - `simulator/`, `flying_drone_stack/`, `~/Desktop/crazyswarm2`, `~/Desktop/crazyflie-firmware`, `~/Desktop/fbl_v2_firmware`: read only; read-only git commands only; no firmware rebuild;
  - `experiments/tuning/`, `out/`, the old docs, `docs/papers/`, the meeting docs, the thesis, `docs/comparative_study_protocol.md`;
  - Stage 3b–4b results.
- **Data:** complete uSD logs only, never radio CSVs.
- **Git and hardware:**
  - no branch switch, checkout, stash, commit or push;
  - no Co-Authored-By;
  - nothing flashed or flown; no connection to a drone or radio;
  - no reboot; do not touch the NVIDIA driver.
- **Stopping processes:** no `pkill`, `killall` or kill of a PID found by pattern. Only exact PIDs you started yourself, checked with `ps -o pid,ppid,args -p <PID>`.
- **Facts:** never invent numbers or rules; write "not stated" and list it as open. Proposals are marked as proposals.
- **Wording:**
  - time stamps only from the real `date` output;
  - method names in full in docs;
  - FBL is just "FBL";
  - say "test error", never "study score".
