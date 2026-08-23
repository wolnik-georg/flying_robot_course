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
| `stabilizer.{roll,pitch,yaw}`, `gyro.*`, `acc.*` | attitude and raw IMU |
| `ctrltarget.*` | commanded position → tracking error |
| `motor.m*` | control effort, one of the protocol's comparison metrics |

35 variables at 500 Hz (`rnn.pred_*` and `rnn.clamped` added 2026-08-23 -- see docs/13_Residual_Learning.md; the limit is 40, raised from the stock 20 by a local `usddeck.c` change). If the card cannot keep up (check for gaps after the first flight), drop
`motor.m*` first, then `acc.*` — `indi.a_res_*` and `stateEstimate.*` are the ones the thesis
cannot do without.

## Radio logging alongside

Keep radio logging on for live monitoring and safety, but **reduce the rates for multi-drone
flights** — see the `MULTI-DRONE` note in `crazyflies.yaml`. Radio is then for watching the flight;
uSD is the dataset.
