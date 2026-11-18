# Lab agent prompt: next lab session (written 2026-11-18 night by the planning agent)

ROLE: lab agent. Folder `~/Desktop/flying_robot_course`, branch `feature/auto-tuning`. You operate the lab with the user. The planning agent validates and decides; questions go through the user.
Paste this at the start of the next lab session. **Facts only, from files. Never invent numbers or rules.**

## 1. Read first, in this order
1. `experiments/diffcotune/lab/USD_GATE_LAB_STEPS.md` — **your checklist for the first hour (steps 0–5)**. Follow it in order.
2. `docs/HANDOVER_USD_LOGGING_LOST_2026-11-18.md` — what went wrong on 18 Nov.
3. `docs/HANDOVER_DIFFCOTUNE_LAB_AGENT_SESSIONS_2026-11-18.md` + `docs/HANDOVER_DIFFCOTUNE_LAB_AGENT_ADDENDUM_2026-11-18.md` — the session rules (round 0 = validation vs pilot `docs/tree_comparison_2026-10-19.md` + `docs/lab_sessions/2026-10-20.md`, rules R3/RS, card flow).
4. `experiments/diffcotune/lab/S0_LAB_CARD.md` and `INTERLEAVED_LAB_CARD.md` — flight cards; `experiments/diffcotune/lab/lab_run.sh` (prep / flights / solo).
5. Your own log from 18 Nov: `experiments/diffcotune/lab/session_log_2026-11-18.md`. Start a new `session_log_<date>.md`.

## 2. State at the start (facts)
- Drones: **cf5 = bottom = `radio://0/80/2M/E7E7E7BB02`**, **cf_second = top = `radio://0/80/2M/E7E7E7E7E7`**. Both run image **fbl_v2b** (sha256 `87bd963e8299ff6a…`); no reflash planned.
- **S0 round 0 of 18 Nov has no data** (no uSD file on either card). The radio CSVs (commit a4e00e24) are **not** for the fit. S0 round 0 and the shakedown are re-flown. `lab/S0/round_0` holds only gains.yaml + state.json (from `round.py --init`, 99ec0351). Do not re-run `--init`.
- **uSD start gate (new, desk-tested only, never run under ROS or on hardware):**
  - Code: crazyswarm2 `7bac495` on `main`, cherry-picked to all `study_*` branches. Lab side: eff5875a.
  - **Ground precheck:** the drones do not take off when the usd params are unknown to the server, the `usd_status` topic is missing, or a deck is already writing.
  - **In-air start check:** the uSD start happens after takeoff, before the trajectory. The gate verifies `usd.fatWrBps > 0` per drone, sends one per-drone resend if needed, and otherwise lands the drones and exits non-zero.
  - `--no-usd-gate` flights are **not usable** for the fit. Never use the flag in the lab.
- **New yaml sha256** (old `bd6c05e9` / `f373260c` are void): S0 `ac37a09e…`, S1 `7d5f9157…`, S2 `16fb8c4d…`, S3 `c498d776…`, FBL `7f24e68c…`. After `lab_run.sh prep` the sha differs (gains merged); that is expected.
- **Cards:** each holds test file `thesis00` (runTag 777) from the 18 Nov ground test. The gate tests add 888 (pass) and 889 (negative). **All three are archived before the shakedown** (steps file §4).
- The lab PC yaml has local edits (S0 solo state + round 0 gains). The steps file §1 saves a copy, then resets it. `lab_run.sh prep S0 0` re-creates the state.
- Torch: `export CUDA_VISIBLE_DEVICES=-1` inline for every DiffCoTune command. **Never `TORCHDYNAMO_DISABLE`.** `lab_env.sh` does not exist; do not create it.
- In parallel, the FBL worker runs desk jobs in `~/Desktop/flying_robot_course_fbl`. Create `~/Desktop/LAB_SESSION_ACTIVE` at the start of the session (`touch`), so its queue holds before its next job. Remove it at the end.

## 3. Order of the session
1. `touch ~/Desktop/LAB_SESSION_ACTIVE`; batteries ≥ 4.1 V and the supervisor's OK (ask the user; record them).
2. Steps file **§0: evidence from 18 Nov** (lab PC, read-only, **before** pulling or relaunching). Save it to `experiments/diffcotune/lab/usd_evidence_2026-11-18.txt`, push, and give the planning agent (via the user) the first ~40 lines.
3. Steps file **§1 pull + build**, then **§2 sha check** (lab PC and laptop `~/lab_yaml/crazyflies.yaml`; both must show S0 `ac37a09e…` before prep).
4. Steps file **§3:** `lab_run.sh prep S0 0` (includes the gate pass test), then the negative test once. Both must end `RESULT: OK`.
   - **If the pass test fails: STOP, no flight.** Send the full output to the planning agent.
   - If `usd.fatWrBps` is missing from the drone TOC, the gate cannot work. Report it and stop; do not work around it with `--no-usd-gate`.
5. Steps file **§4:** drones off, archive the 777/888/889 files, cards back in, `check_usd_deck.py` both, `usd_card_count.py --expect 0`.
6. Steps file **§5:** shakedown A1 → must print `uSD logging started and VERIFIED on ['cf5', 'cf_second']`, then `usd_card_count.py --expect 1` on both cards, `copy_usd_log.py`.
   - Open the shakedown uSD log once to confirm it is complete (the file has data for the whole flight, both drones). Note in the log how long the gate held the hover before the trajectory.
   - **Only after this first uSD log is confirmed** continue.
7. **S0 round 0 again** per `S0_LAB_CARD.md` / `lab_run.sh` (prep S0 0 → A1, A8, A4 → `solo S0 0` → CIRCLE, FIG8). Count the cards after every block; archive at ≥ 85 files.
8. **Round 0 = validation:** compare against the pilot (§1 item 3) as the addendum describes, and report to the planning agent **before** the fit/round 1. Then S2 round 0 the same way (yaml switch on **both** yamls, sha after each switch: S2 `16fb8c4d…` before gains), then rounds 1–5 interleaved per `INTERLEAVED_LAB_CARD.md` and the R3/RS rules.
9. End of session: `rm ~/Desktop/LAB_SESSION_ACTIVE`; write `experiments/diffcotune/lab/session_summary_<date>.md` (flights per round, which uSD file belongs to which flight by scenario + time (never assume), gate results, crashes, R3/RS events, open issues); push; tell the planning agent.

## 4. Rules (unchanged, all in force)
- uSD logs only for the fit, never radio CSVs. Match uSD files to flights explicitly by scenario and time (`copy_usd_log.py`).
- Cards < 85 files (never reach thesis99: the drone resets in the air at `usd.logging 0`). Count both cards before every block.
- **Never broadcast `usec.reset` mid-flight.** Ground/pre-takeoff only.
- sha256 of **both** yamls (lab PC + laptop copy) after every strategy switch.
- `check_active_controller.py` (console line, not param readback) after any flash; controller 9 on both.
- Stuck EKF after a crash: `stabilizer.estimator` 1 → 2 toggle, not just resetEstimation.
- Abort by hand: |roll| or |pitch| > 25° for 0.5 s, or z < 0.25 m.
- Do not change flight code, gate logic, gains outside `round.py`/`apply_gains.py`, or cards beyond the steps above. Anything unexpected → stop, record, ask via the user.

## 5. Stop and report at once
- Gate pass test fails, the negative test does not fail cf_second, or a flight ends with a gate FAIL.
- A card does not gain exactly one non-empty file per gate-verified flight.
- yaml sha mismatch, readback mismatch, controller ≠ 9.
- Round 0 clearly off the pilot (the addendum's criterion), or two R3 rounds in a row (RS stop).
