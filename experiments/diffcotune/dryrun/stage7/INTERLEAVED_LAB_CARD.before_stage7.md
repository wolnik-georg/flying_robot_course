# Interleaved lab card (two controllers, one session) — Stage 6, 15 Nov 2026 — **every item below: Stage 6, proposed, not decided**

Derived from `S0_LAB_CARD.md` (decided items stay as they are; this card only adds how two controllers share one session). Nothing here was flown or run with hardware; the tools were tested with fake uSD files (`lab/test_session_mock.py`, result in `dryrun/stage6/report_stage6.md`). Pairs (proposal): session 1 = S0 + S2, session 2 = S1 + S3 (S3 is not lab-ready on the desk side, see `dryrun/stage5c_rdfix/report_rdfix.md`). "Not stated" = not in the repository and not measured.

## Before the session (Stage 6, proposed)

| # | Do | Source |
|---|---|---|
| 1 | All items 1–7 of `S0_LAB_CARD.md` "Before the block", for BOTH controllers (preflight per controller: `preflight.py --controller S0`, `--controller S2`) | S0 lab card |
| 2 | uSD files: both cards below 85 at the start; the plan adds 32 files per controller and card (20 tuning + 12 test, assumption 1 file per flight and card, not stated), i.e. 64 for a pair: `python3 lab/session.py cards --start-count <n> --pair S0 S2` | session.py; S0 lab card rule (≥ 85 archive) |
| 3 | State dirs: `round.py --init --controller S0 --state-dir lab/S0` and `--controller S2 --state-dir lab/S2` | round.py |
| 4 | Decide the strategy-switch overhead assumption for planning: not measured (3 / 5 / 10 min in `results/session_timeline.md`) | not stated |

## Per round (Stage 6, proposed)

| # | Do | Source |
|---|---|---|
| 1 | `python3 lab/session.py status --pair S0 S2 --loaded <C>` prints what to fly next, e.g. "fly S2 round 1: A1, A8, then --solo CIRCLE, FIG8", and "S0 round 1 desk ready at hh:mm" | session.py |
| 2 | Switch to the controller to be flown (only if another controller is loaded): `python3 lab/session.py switch --to <C> --fragment lab/<C>/round_<l>/gains.yaml` prints the sequence: stop CS2 → `set_tree_strategy.py <C>` → `apply_gains.py merge` (put `.new` in place) → `lab_check_strategy.py` → launch CS2 → stop → `apply_gains.py readback` (fresh connection) → relaunch | S0 lab card per-round items 1, 2; session.py |
| 3 | S2 / S3 beyond S0: `rnn.en` 1 on cf5; cf_second enabled for A1 / A8; for CIRCLE / FIG8 `switch --solo` (cf_second `enabled: false`) as in the S0 card item 5 — the extra relaunch per round is the same as for S0 | session.py; `set_tree_strategy.py` |
| 4 | Flights 1–4 as in the S0 card (A1, A8, `--solo` switch, CIRCLE, FIG8); NS2 weights are part of the flashed image (`study_tree_v2.bin`): not checked by these tools (not stated) | S0 lab card |
| 5 | Merge uSD files (merged files only), then the manifest: `python3 lab/session.py manifest --controller <C> --round <l> --inbox <folder> --out <folder>/flights.json --root lab --assign A1=<file> A8=<file> CIRCLE=<file> FIG8=<file>` (explicit scenario-to-file assignment; never by arrival time; counts files on the card) | session.py; memory rule "never assume a uSD log and a radio CSV are the same flight" |
| 6 | Start the desk step in the background: `python3 lab/session.py desk --controller <C> --round <l> --logs <folder> --root lab` (calls `round.py`; the pool and the compile run while the other controller is flown). Run two desk steps at once only if the PC fits (measured: S0 + S2 together 24.8 / 30.7 min against 19.4 / 26.0 alone, MemAvailable minimum 9.6 GB, load 7.2) | session.py; Stage 6 step 2 |
| 7 | Fly the other controller's round (switch first); return when "<C> round <l+1> desk ready" is printed; if the wait is short, fly test flights of the loaded controller (A4 cells, default-gain set only) | session.py status; protocol S0 test flights |
| 8 | After round 4 of both: tests with θ_best (6 flights per controller) after the last desk step | S0 lab card item 12 |

## Rules (unchanged from the S0 card)
Abort, crash, repeat and battery rules of `S0_LAB_CARD.md` apply unchanged to every controller of the session; the two-controller session adds none. Flights per battery pack and packs per hour: not stated. Which PC runs the loop in the lab: not stated.

## Open decisions (Stage 6, proposed)
Switch overhead (not measured); whether S0 + S2 first and S1 + S3 later; the scenario set (`dryrun/stage6/scenario_decision.md`); which PC runs `round.py` in the lab; whether two desk steps at once are acceptable on that PC (the desk PC measurement does not say).
