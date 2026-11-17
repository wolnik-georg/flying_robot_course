# S0 lab card (DiffCoTune Split-Alternate, frozen lab variant "ours", 6 rounds of 5 tuning flights) — Stage 5 A3, 13 Nov 2026; updated Stage 8b, 15 Nov 2026 (evening)

Everything below is copied from the protocol draft (`docs/diffcotune_lab_protocol_S0_DRAFT.md`, "PD") and the existing rules; no new rules. "CSP" = `docs/comparative_study_protocol.md`. Items marked *open* are not stated anywhere.

## Before the block

| # | Do | Source |
|---|---|---|
| 1 | Count the uSD files on BOTH cards; **Stage 8b: empty both cards before the session, archive once in the middle (after round 2 of both controllers), never ≥ 85 files; on the first flight check that it is one file per flight (the plan of 42 files per controller and card assumes it)** (the card must never reach 100 files) | PD "One round" step 1; `docs/lab_sessions/2026-10-26.md` item 5; memory `feedback_usd_card_file_counter_100_2026-10-19` |
| 2 | `flying_drone_stack/tools/check_usd_deck.py` on both drones | PD step 3; CSP §4 |
| 3 | After a flash only: `flying_drone_stack/tools/check_active_controller.py` (console line; a parameter read-back is a cache) | PD step 5; memory `feedback_verify_controller_slot_after_flash_2026-10-13` |
| 4 | Pad check at the lab PC: `python3 experiments/diffcotune/lab/fly_file.py --cell CIRCLE --check --pad <x> <y> --yaml <lab crazyflies.yaml>`, same for `FIG8` (default pad cf5 (0.0, 0.3)); accepted: FIG8 passes 0.32 m (horizontal) from the parked cf_second, 1 m above it | PD "Decided by the user (13 Nov, after Stage 4d)" item 5 |
| 5 | `python3 experiments/diffcotune/lab/preflight.py --controller S0 --yaml <lab crazyflies.yaml>` (must print PREFLIGHT PASSED; checks the paper-faithful settings, 6 rounds, 5 flights, the NS2 switches and weights hash; use `/usr/bin/python3` of the desk PC or the lab PC's python with torch, scipy, yaml) | Stage 5 A1 |
| 6 | Fresh batteries: rest ≥ 4.1 V; swap if either drone < 4.0 V | CSP §4 (PD "Decided" item 2) |
| 7 | Round 0 initialised: `python3 lab/round.py --init --controller S0 --state-dir lab/S0` (gains = defaults) | `lab/round.py` doc |

## Per round l = 0 … 5 (gains of the round loaded first; Stage 8: 6 rounds, the gains of round 5 are the last update, flown and judged, no fit or tuning after it)

| # | Do | Source |
|---|---|---|
| 1 | `python3 lab/apply_gains.py merge --fragment lab/S0/round_<l>/gains.yaml --yaml <crazyflies.yaml>` → put the `.new` file in place → `lab_check_strategy.py` → launch CS2 | PD P2 option A |
| 2 | Firmware read-back of the 15 gains on a fresh connection (CS2 stopped): `python3 lab/apply_gains.py readback --uri <cf5 uri> --fragment lab/S0/round_<l>/gains.yaml`; relaunch; again after every battery swap | PD P2, "Decided" item 4 |
| 3 | Flight 1, A1: `ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.375 --hold 15.0 --height 0.5 --auto-center --yes` (cf5 + cf_second) | PD "Flight card per round" |
| 4 | Flight 2, A8: `… --scenario A8 --dz 0.375 --duration 8.0 --passes 2 --height 0.5 --auto-center --yes` | PD flight card |
| 4a | Flight 3 (Stage 8, new), A4 cell 1: `… --scenario A4 --dz 0.4 --speed 0.35 --offset 0.125 --motion line --rotate 90 --height 0.5 --auto-center --yes` (cf5 + cf_second, before the `--solo` switch; this is a tuning flight now, in-sample; the held-out test cell is A4 cell 2) | Stage 8, user decision 15 Nov |
| 5 | (after flights 1–3) `set_tree_strategy.py S0 --solo` (cf_second `enabled: false`), `lab_check_strategy.py`, relaunch CS2; gains of the round still on cf5 (`apply_gains.py readback`) | PD flight card |
| 6 | Flight 4, CIRCLE: `python3 experiments/diffcotune/lab/fly_file.py --cell CIRCLE` | PD flight card, P3 |
| 7 | Flight 5, FIG8: `python3 experiments/diffcotune/lab/fly_file.py --cell FIG8` | PD flight card, P3 |
| 8 | Merge the uSD files of the round (merged files only, never radio CSVs): `flying_drone_stack/tools/merge_usd_logs.py` | PD step 7 |
| 9 | `flights.json` in the round's log folder (template: `python3 lab/flights_template.py --out <folder>/flights.json`; fill in file names; `rotate_deg` A4 −90, CIRCLE −90 and FIG8 −90, A1 / A8 none; attempt 1, 2, 3 for repeats; 5 tuning flights) | PD flight card, P1 |
| 9a | **Rule R3 (Stage 8d, user decision 16 Nov; `round.py` version marker `R3-2026-11-16`, checked by `preflight.py` 6b)**: if a scenario has 3 failed attempts (crashed, tilted or floor flight; attempts 1, 2, 3 in `flights.json`) fly that scenario ONE more time with θ_best (the gains of the keep-best round K: `lab/<C>/round_<K>/gains.yaml`; `round.py` prints the exact scenario and file if it is missing and stops) and add it to `flights.json` by hand with `"role": "fallback"`, `"attempt": 4` and the usual fields (`session.py manifest` does not write this entry: not stated). The fallback flight is NOT used for the fit, the real cost or θ_best; the β fit uses the usable flights of the other scenarios; the set θ_l is not eligible for keep-best. Round 0 (no θ_best yet): no fallback flight. Flight count: +1 flight per R3 case (and the 2 repeats before it). | user decision 16 Nov (Stage 8d) |
| 10 | `python3 lab/round.py --controller S0 --round <l> --logs <folder>` (Stage 8b: the default method is the frozen "ours" (`--method ours`, stated here on purpose; `--method paper` is not used in the lab), `--n-rounds` default 6; the desk step ends with the safety preview, see the next row) | PD step 8 |
| 10a | **Safety preview (Stage 8b, proposed)**: the desk step prints `PREVIEW PASSED` or `PREVIEW FAILED: <scenario, seed, reason>` for the new gain set (frsim, the round's fitted β, 2 noise seeds, protocol abort rules; also in `lab/<C>/round_<l+1>/PREVIEW.txt`). It never changes the gains: `gains.yaml` is written either way. On PASSED fly the set. **On FAILED do not fly it without asking the user; default action: fly the best flown set again** (the round number is printed). The preview takes about 1 min inside the desk step (`--no-preview` skips it) | Stage 8b, `lab/preview.py` |
| 11 | Next gains and report appear in `lab/S0/round_<l+1>/` and `round_<l>/report.md`; the last round writes `gains_theta_best.yaml` | PD step 9 |
| 12 | Test flights (Stage 8: A4 cell 2 is the held-out test cell; A4 cell 1 is in-sample): A4 cell 2 `--scenario A4 --dz 0.4 --speed 0.35 --offset 0.075 --motion line --rotate 90 --height 0.5 --auto-center --yes`, 3 flights with the default gains (round 0) and 3 with θ_best (after round 5); A4 cell 1 (`--offset 0.125`) 3 + 3 the same way, reported as in-sample; `rotate_deg` −90, role `test` | PD flight card (lab test cells), "Decided" item 1 |

## Rules

| Rule | Source |
|---|---|
| Abort at \|roll\| or \|pitch\| > 25° for 0.5 s, or z < 0.25 m; "for 0.5 s" is read as 0.5 s continuous (user, 13 Nov) | CSP §4; PD "Crash and floor handling", item 10 |
| Stop after 2 crashes; after a crash toggle `stabilizer.estimator` 1 → 2; human pilot ready | CSP §4; PD "Crash and floor handling" |
| A crashed, tilted or floor flight is repeated and not used for the fit; at most 2 repeats (attempt ≤ 3) | PD "Crash and floor handling" (implemented in `round.py`) |
| 3 failed attempts on a scenario: rule R3 (item 9a): one fallback flight with θ_best, not used for the fit; the set is not eligible for keep-best; eligible sets only count for θ_best; if no set is eligible after round 5, θ_best = default gains (θ_0). **Keep-best eligibility = consequence of R3, user to confirm** | user decision 16 Nov (Stage 8d) |
| Battery: fresh packs, rest ≥ 4.1 V, swap if either drone < 4.0 V, repeats included; flights per pack and a voltage limit for a repeat: *not stated* (open) | CSP §4; PD "Decided" item 2, P4 |
| Never use radio CSVs for the fit (complete uSD logs only) | PD step 7; memory `feedback_analysis_usd_only_2026-10-19` |
