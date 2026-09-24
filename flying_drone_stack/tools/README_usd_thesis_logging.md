# uSD logging for thesis flights

Radio logging cannot carry the thesis dataset for more than one drone. The `all:` block in
`crazyflies.yaml` streams 8 custom topics at 100 Hz ≈ **600 packets/s per drone**, and a single
Crazyradio tops out around 1000 packets/s *shared*. Two drones at full rate will drop packets,
and dropped packets in a residual-force dataset are silently corrupt training data.

uSD logging solves this properly: each drone writes its own file, onboard, at **500 Hz**
(synchronous with the stabilizer loop), independent of radio and independent of how many drones
are flying.

## Install

Requires the **Micro SD card deck** on each drone.

```bash
# 1. format FAT32, copy the config to the card root, named exactly config.txt
sudo mkfs.vfat -F 32 -n THESIS /dev/<sd>1
cp flying_drone_stack/tools/usd_thesis_config.txt /media/<sd>/config.txt

# Diagnostic variant (2026-09-24, mocap/EKF investigation): swaps the currently-unused
# rnn.pred_*/rnn.clamped slots (rnn.en=0, no trained weights on any drone right now, so this
# costs nothing) for locSrv.x/y/z + locSrvZ.tick -- the RAW external pose the firmware receives
# via send_extpose, and how stale it is, logged independently of stateEstimate.* (the EKF's own
# fused output). The standing config only ever showed the fused result, so a bad-pose incident
# could never be told apart from a bad-fusion incident after the fact. Use this one instead of
# usd_thesis_config.txt for any session investigating position/attitude anomalies:
#   cp flying_drone_stack/tools/usd_mocap_diagnostic_config.txt /media/<sd>/config.txt
# Swap back to usd_thesis_config.txt before resuming normal C.1 collection.

# 2. card into the deck, power-cycle the drone, then confirm the deck came up.
#    Safe to run as many times as you like -- reads two read-only status params
#    (usd.bcUSD, usd.canLog) and never touches usd.logging, so it cannot contaminate
#    a recording the way the pre-2026-09-15 version of this check did.
python3 flying_drone_stack/tools/check_usd_deck.py radio://0/80/2M/<uri>
```

## Recording

`usd.logging` is toggled around the trajectory by `flight.py`, `formation_flight.py` and
`run_formation.py` (`setParam("usd.logging", 1)` before, `0` after), so with the card installed
you get data with no script changes. All three ALSO broadcast `usec.reset` — but **on the
ground, before takeoff**, not next to the logging start.

> **Never move `usec.reset` back to just before `usd.logging=1`.** That is where it originally
> sat (2026-09-14 → 2026-09-15) and it crashed *every* flight, including a pure A1 hover. The
> high-level commander's whole time base is that same clock
> (`crtp_commander_high_level.c`: `float t = usecTimestamp() / 1e6;`) and the planner stores
> `t_begin` from it at takeoff. Zeroing it mid-flight makes `piecewise_eval` compute
> `t - t_begin` ≈ *minus* several hundred seconds, evaluate a degree-7 polynomial far outside
> its domain, and hand the controller a garbage setpoint — motors cut or the vehicle slams
> over within one control tick. On the ground the planner is IDLE, so the reset is safe there.
> The uSD logs still share an origin, which is the only thing that actually mattered.

**One file per logging session, and the file is only finished on a clean stop.** Verified in
`usddeck.c`'s `usdWriteTask` (2026-09-15):

- `usd.logging` 0 → 1: scan for the first `thesisNN` that does **not** exist, `f_open(...
  FA_CREATE_ALWAYS)`, write the header. So every start makes a **new** file and the counter
  never reuses a number.
- `usd.logging` 1 → 0: drain the ring buffer, write the CRC, **`f_close`**. The file's size
  only lands in the FAT directory entry at this point.

Two consequences that matter in practice:

1. **A 0-byte `thesisNN` is a session that started but never cleanly stopped** — the file was
   created, then the drone lost power (or the card was pulled, or the flight crashed and the
   script never reached `usd.logging = 0`) before `f_close` ran. It is not a card fault and not
   a deck fault. 2026-09-15 produced several of these, all from crashed flights.
2. **The counter is not a timeline.** It increments per session, including sessions that
   produced nothing, and it does not reset on reformat. Higher number = later session *on that
   card*, and nothing more — in particular it says nothing about which drone flew it, since
   cards get moved between vehicles.

An earlier version of this file claimed the opposite (that sessions append into one file per
power cycle). That was wrong; the source says otherwise.

Because the reset now fires pre-takeoff, the onboard clock already reads roughly
takeoff + climb + converge + upload (**~10–12 s**) by the time the scenario itself starts. That
is the expected `find_flight_window.py` lag for a healthy post-2026-09-15 flight — *not* zero.
A lag of hundreds of seconds means the `usec.reset` broadcast never reached that drone (check
the flight's terminal output for its `WARN`) or that the recording predates the fix.

## Copying off the card

```bash
python3 flying_drone_stack/tools/copy_usd_log.py /media/<mount> <drone_name>
```

**Never `cp` a uSD log by hand.** Two mistakes are easy to make and this script exists
specifically to prevent them:

1. **The card's own log-file name (`thesis00`, `thesis01`, ...) tells you nothing** — the
   counter increments on every logging session, including ones that produced an empty file
   (2026-09-15: both cards' `thesis00` was 0 bytes from an earlier session; the real flight was
   `thesis01`). The script finds the largest *non-empty* file, not the highest-numbered one.
2. **The drone's own log timestamp (`usecTimestamp()`) is µs since that drone's power-on, not a
   calendar time** — it cannot name the file, and it cannot be used to match one drone's file to
   another's or to a radio CSV. The only trustworthy real-world timestamp is the *copying
   machine's* wall clock at the moment of copy, which is exactly what this script uses
   (`{drone}_{YYYY-MM-DD_HH-MM-SS}.bin` in `experiments/logs/usd_raw/`, verified byte-identical
   via sha256 after the copy).

**Never assume a radio CSV that happens to arrive via `git pull` around the same time is the
match for a uSD log.** 2026-09-15: exactly this assumption led to comparing a uSD log against a
radio CSV from the *previous night's* session, purely because both landed in the same pull.
uSD logs and radio CSVs are never merged automatically — matching one to the other is a
deliberate step, done by scenario name and approximate flight time, confirmed with whoever was
in the lab, never inferred from file arrival order.

## Matching uSD to radio after a lab day (2026-09-19 workflow)

Full protocol and flight catalog: **`docs/28_USD_Radio_Matching_and_Session_Analysis.md`**.

**Quick recipe:**

1. Archive both cards to `experiments/logs/usd_raw/<date>_THESIS{1,2}/` with `copy_usd_log.py`
   (or a verified bulk copy — never hand-rename in the archive).
2. Note in the lab session **which physical drone wore THESIS1 vs THESIS2** (cards swap —
   the label is not “always bottom/top”) and **`thesisNN` per flight** (cheapest match key
   when scenario params repeat). After copy, assign **bottom/top from merge role RMS**, not
   from the card name (`docs/28`).
3. Merge with **`merge_usd_logs.py --meta experiments/logs/A8_<stamp>.meta.json --roles bottom top`**
   on symlinks named `cf5_...` / `cf_second_...` so merged columns match radio meta names.
4. Refuse merge if either drone reports **RMS &gt; 15 cm** (wrong file or role).
5. Run the session suite (example):  
   `python3 experiments/analysis/run_a8_2026_09_19_suite.py`  
   → packages under `experiments/logs/a8_<date>_successful/`, plots + metrics, no raw edits.

Optional helper: `experiments/analysis/match_usd_to_radio.py` (radio correlation scores only —
same A8 params make trajectory correlation ambiguous; counter order + merge RMS wins).

## Decoding, and finding the actual flight inside the file

```bash
~/.pyenv/versions/flying_robots/bin/python flying_drone_stack/tools/decode_usd_log.py <file>

# find (and optionally extract) the real flight window, verified against the commanded
# trajectory rather than guessed from file position -- see the script's own docstring
python3 flying_drone_stack/tools/find_flight_window.py <usd_log.bin> bottom \
    experiments/logs/<scenario>_<date>_<time>.meta.json \
    --extract experiments/logs/usd_raw/<scenario>_<drone>_<date>_<time>_flight.csv
```

`find_flight_window.py` rebuilds **any** formation-library scenario from the flight's
`.meta.json` via `formations/scenarios.py` (generalized 2026-09-15). Pass the correct
`bottom` / `top` / … role for that scenario.

## Right after landing, before you touch anything else

Write down, in the lab session doc, one line per flight: which card went into which drone, and
which scenario/take it was. This is the cheapest possible fix for "which file is which" — cheaper
than reconstructing it afterward from file sizes and correlation, which is what 2026-09-15's
whole uSD investigation had to do because this wasn't recorded at the time.

## What it logs and why

| Group | Why |
|---|---|
| `stateEstimate.{x,y,z,vx,vy,vz}` | own state — and, differenced across drones, the relative state that is the NN input |
| **`indi.a_res_{x,y,z}`** | **the residual, f_res/m — the thesis measurement.** Zero unless an RPM source is present |
| `indi.{tau,alp}_*` | INDI internals, for diagnosing the controller |
| `indi.e_r_{x,y,z}`, `indi.e_r_norm` | Geometric attitude error `e_R` used in the torque law. Present under every `ctrl_mode` (0-3), not only INDI (added 2026-09-08) |
| `usd.runTag` | Session tag (unix s) set by host before `usd.logging=1`; 48th channel, uses the last free slot (prior config had 47 variables). Primary pairing key post-flash (`docs/39`). |
| `stabilizer.{roll,pitch,yaw}`, `gyro.*`, `acc.*` | attitude and raw IMU |
| `ctrltarget.*` | commanded position → tracking error |
| `motor.m*` | control effort (PWM), one of the protocol's comparison metrics |
| `motor.m*_rpm` | DShot ESC telemetry on the uSD file — **same source** as `indi_gains.rpm_source=1` uses for `a_res`; logged for offline delay/cross-check, not a second control path |
| `rpm.m*` | optical RPM deck — logged **alongside** DShot; control still uses DShot only |

48 variables at 500 Hz (cap = `MAX_USD_LOG_VARIABLES_PER_EVENT`: dual RPM, full `indi.e_r_*`, and `usd.runTag` as of 2026-09-21 evening).
Cap is **48** (`MAX_USD_LOG_VARIABLES_PER_EVENT` in local `usddeck.c`, was 40). **Reflash both study drones** after raising the cap, then copy this config to both SD cards.
If the card cannot keep up (check for gaps after the first flight), drop
`motor.m*` first, then `acc.*` — `indi.a_res_*` and `stateEstimate.*` are the ones the thesis
cannot do without.

## Radio logging alongside

Keep radio logging on for live monitoring and safety, but **reduce the rates for multi-drone
flights** — see the `MULTI-DRONE` note in `crazyflies.yaml`. Radio is then for watching the flight;
uSD is the dataset.
