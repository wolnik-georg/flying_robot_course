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
# 1. copy the config to the card root, named exactly config.txt
cp flying_drone_stack/tools/usd_thesis_config.txt /media/<sd>/config.txt

# 2. card into the deck, power-cycle the drone, then confirm the deck came up
~/.pyenv/versions/flying_robots/bin/python flying_drone_stack/tools/check_usd_deck.py
```

## Recording

`usd.logging` is already toggled around the trajectory by both `flight.py` and
`formation_flight.py` (`setParam("usd.logging", 1)` before, `0` after), so with the card
installed you get a file per flight with no script changes.

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

## Decoding

```bash
~/.pyenv/versions/flying_robots/bin/python flying_drone_stack/tools/decode_usd_log.py <file>
```

## What it logs and why

| Group | Why |
|---|---|
| `stateEstimate.{x,y,z,vx,vy,vz}` | own state — and, differenced across drones, the relative state that is the NN input |
| **`indi.a_res_{x,y,z}`** | **the residual, f_res/m — the thesis measurement.** Zero unless an RPM source is present |
| `indi.{tau,alp}_*` | INDI internals, for diagnosing the controller |
| `indi.e_r_{x,y,z}`, `indi.e_r_norm` | Geometric attitude error `e_R` used in the torque law. Present under every `ctrl_mode` (0-3), not only INDI (added 2026-09-08) |
| `stabilizer.{roll,pitch,yaw}`, `gyro.*`, `acc.*` | attitude and raw IMU |
| `ctrltarget.*` | commanded position → tracking error |
| `motor.m*` | control effort, one of the protocol's comparison metrics |

39 variables at 500 Hz (`rnn.pred_*` and `rnn.clamped` added 2026-08-23; `indi.e_r_*` added 2026-09-08 -- see docs/13_Residual_Learning.md; the limit is 40, raised from the stock 20 by a local `usddeck.c` change). One slot of headroom left. If the card cannot keep up (check for gaps after the first flight), drop
`motor.m*` first, then `acc.*` — `indi.a_res_*` and `stateEstimate.*` are the ones the thesis
cannot do without.

## Radio logging alongside

Keep radio logging on for live monitoring and safety, but **reduce the rates for multi-drone
flights** — see the `MULTI-DRONE` note in `crazyflies.yaml`. Radio is then for watching the flight;
uSD is the dataset.
