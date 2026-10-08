# Lab session pack — NS2 sign test (after 2026-10-05)

**Goal:** flip the network term sign on **cf5** (`indi_gains.res_sign: -1`, `ctrl_mode=0` ⇒ only the NN term inverts). Confirm A8 crossing dips move **below** the ~−5.9 cm network-off baseline, then collect A8 `rnn.en=0` baselines and **A1 `rnn.en=0`** before any A1 network flight.

**Synced with:** [`next_flight_card.html`](next_flight_card.html) §2 · [`lab_sessions/2026-10-05.md`](lab_sessions/2026-10-05.md) · [`post_flight_check.md`](post_flight_check.md)

**Lab PC:** ROS 2 **Jazzy**, workspace `~/georg/ros2_ws`, **`backend:=cpp` only** (never `cflib`).

---

## 0 — Preflight (every session)

**Batteries:** both packs **fresh**, rest **≥ 4.1 V** on each drone (swap if either **< 4.0 V**).

**Repos + build (lab PC):**

```bash
REPO=~/georg/flying_robot_course
WS=~/georg/ros2_ws
cd "$REPO" && git pull
cd "$WS/src/crazyswarm2" && git pull   # yaml lives here; Claude updates cf5 — you pull only

cd "$WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select crazyflie_interfaces crazyflie_py crazyflie crazyflie_examples
source "$WS/install/setup.bash"
```

**Launch (terminal 1):**

```bash
source ~/georg/ros2_ws/install/setup.bash
ros2 launch crazyflie launch.py backend:=cpp
```

**Sanity (terminal 2, before arming):**

```bash
source ~/georg/ros2_ws/install/setup.bash
ros2 topic echo /cf5/pose --once
```

Expect a real position (not silent / NaN). No `Assert failed`, no `rate is off`.

**uSD decks (each drone, motors off):**

```bash
REPO=~/georg/flying_robot_course
python3 "$REPO/flying_drone_stack/tools/check_usd_deck.py" radio://0/80/2M/E7E7E7BB02
python3 "$REPO/flying_drone_stack/tools/check_usd_deck.py" radio://0/80/2M/E7E7E7E7E9
```

(`check_usd_deck.py` takes a **radio URI**, not `--help` — verified on desk.)

---

## 1 — Session config (yaml on cf5)

**Claude pushes** the crazyswarm2 yaml change on the lab PC repo — **not you**. After `git pull`, **cf5** should include:

```yaml
# under robots.cf5.firmware_params (exact nesting as in crazyflies.yaml):
indi_gains:
  ctrl_mode: 0
  res_sign: -1    # NEW for this session — flips NN term only under geometric ctrl_mode
rnn:
  en: 1           # flights 1–3 (sign test); set en: 0 for block 2 (A8 baseline)
```

**Confirm before arming:** `run_formation` prints the resolved param block at startup — check **`res_sign: -1`**, **`rnn.en`**, and **`ctrl_mode: 0`** for cf5 match the flight block you intend.

---

## 2 — Pose bag + flights (order = flight card §2)

**Start pose bag once per flight block** (keep recording across the 3× A8 sign flights if you prefer one bag; otherwise one bag per flight):

```bash
source ~/georg/ros2_ws/install/setup.bash
ros2 bag record -o ns2_pose_a8_sign1 /poses /cf5/pose /cf_second/pose
# Ctrl+C after the block; rename/move under experiments/logs/rosbags/ when back at the laptop
```

### Block 1 — A8, network on, `res_sign=-1` × **3**

```bash
source ~/georg/ros2_ws/install/setup.bash
ros2 run crazyflie_examples run_formation \
  --scenario A8 --dz 0.5 --height 0.5 --passes 4 --auto-center --yes
```

Pass hint: clean radio + uSD; crossing dip (cf5 z error) **clearly shallower than (a smaller dip, not a deeper one) ~−5.9 cm** network-off baseline.

### Block 2 — A8, `rnn.en=0` × **2**

Set **`rnn.en: 0`** for cf5 in yaml (Claude or you after pull), re-check printed config, same command as block 1.

### Block 3 — A1, `rnn.en=0` baseline × **1** (then stop if gate met)

```bash
ros2 run crazyflie_examples run_formation \
  --scenario A1 --dz 0.5 --height 0.5 --hold 15 --auto-center --yes
```

(Flags match `A1_2026-10-05_18-55-34.meta.json` from 2026-10-05.)

**Abort rules:** |roll| or |pitch| **> 25°** for **0.5 s**, or **z < 0.25 m** → land/stop scenario. **Stop after 2 crashes.** After a crash: toggle `stabilizer.estimator` **1 → 2** before next flight.

---

## 3 — After each flight (laptop)

1. **Pull radio CSVs** into `experiments/logs/` (git).
2. **Battery check:** min `vbat` in both CSVs (loaded samples > 2 V); cf_second **≥ 3.2 V** preferred after 10-05 collapse.
3. **uSD — cf_second card FIRST**, then cf5. The cards swap drones between sessions (10-05: `THESIS1` mount = cf_second at first, then `THESIS2` = cf5), so **identify the drone from the log, not the label**: `ctrltarget_z` max is **1.00 for cf_second (top)** and **0.50 for cf5 (bottom)** at `--height 0.5 --dz 0.5`. Log files sit **directly in the card root** (`thesis00`, `thesis01`, …), one per flight, in flight order; a missing or 0-byte file means no usable log.

```bash
REPO=~/Desktop/flying_robot_course
MOUNT=$(ls -d /media/georg/THESIS* | head -1)   # the card that is inserted now
ls -la "$MOUNT"/thesis*                          # sizes ~2.7 MB = full flight, 0 B = lost
~/.pyenv/versions/flying_robots/bin/python - <<PY
import sys,glob,numpy as np
sys.path.insert(0,"$REPO/flying_drone_stack/tools")
import decode_usd_log as d
for f in sorted(glob.glob("$MOUNT/thesis*")):
    r=d.load(f); print(f.split("/")[-1],"run_tag",int(r["run_tag"][0]),"ctrltarget_z max %.2f"%np.nanmax(r["ctrltarget_z"]))
PY
# match each run_tag to experiments/logs/<SCN>_<DATE>_<HH-MM-SS>.meta.json -> usd_run_tag, then copy with explicit names:
DST=$REPO/experiments/logs/usd_raw
SRC=thesis00; DRONE=cf_second; SCN=A8; DATE=2026-10-06; STAMP=19-17-04      # adjust per file
cp "$MOUNT/$SRC" "$DST/${DRONE}_${SCN}_${SRC}_${DATE}_${STAMP}.bin" && cmp "$MOUNT/$SRC" "$DST/${DRONE}_${SCN}_${SRC}_${DATE}_${STAMP}.bin" && echo ok
# after ALL files of this card are copied and verified: archive on the card
mkdir -p "$MOUNT/_archive_${DATE}" && mv "$MOUNT"/thesis0* "$MOUNT/_archive_${DATE}/" && sync
```

Repeat with the other card (`DRONE=cf5`). A **0-byte** `thesisNN` after a crash is consistent with power loss (cf_second, 10-05).

4. **Merge** (when both bins exist):

```bash
python3 "$REPO/flying_drone_stack/tools/merge_usd_logs.py" \
  "$REPO/experiments/logs/usd_raw/cf5_${SCN}_${THESIS}_${DATE}_${STAMP}.bin" \
  "$REPO/experiments/logs/usd_raw/cf_second_${SCN}_${THESIS}_${DATE}_${STAMP}.bin" \
  -o "$REPO/experiments/logs/merged_${SCN}_${DATE}_${STAMP}.csv" \
  --meta "$REPO/experiments/logs/${SCN}_${DATE}_${STAMP}.meta.json" \
  --roles bottom top
```

(`merge_usd_logs.py --help` verified on desk.)

5. **Post-flight check:**

```bash
~/.pyenv/versions/flying_robots/bin/pip install mcap mcap-ros2-support   # once per env
~/.pyenv/versions/flying_robots/bin/python \
  "$REPO/experiments/analysis/post_flight_check.py" \
  --date "$DATE" \
  --md "$REPO/experiments/analysis/out/post_flight_check_${DATE}.md" \
  --json "$REPO/experiments/analysis/out/post_flight_check_${DATE}.json"
```

---

## 4b — AFTER the NS2 gate only: INDI "Omar + Iz" block (added 2026-10-08, preserved plan)
Do NOT start this block before the NS2 sign test is done (pack §2 Blocks 1–3). Plan and rationale: `docs/65_Omar_Z_Offset_Plan.md`, `docs/66_Omar_Z_Integral_Results.md`. Firmware must contain the `kpos_iz` change: flash the build from the laptop tree (`make DRONE=bl` / the usual flash recipe), check with `ros2 param get` that `ctrlOot5.kpos_iz` (Rust) or `ctrlOmarIndi.Kpos_Iz` (C) exists.
- **Which controller first:** Omar **Rust** (`controller: 10`) — it reads RPM through `rpm_get_all()` (DShot, `rpm_source: 1`, spike filter active). Omar **C** (`controller: 9`) only if C is chosen later: it reads the *optical deck* `rpm.m1..4` directly (needs `deck.bcRpm`, no filter, deck dropped 2 of 4 motors in earlier 2-drone flights).
- **yaml on cf5 (exact nesting as in crazyflies.yaml):** `controller: 10`, `ctrlOot5: {indi: 3, kpos_iz: <value>}` (for C: `controller: 9`, `ctrlOmarIndi: {indi: 3, Kpos_Iz: <value>}`); `indi_gains.rpm_source: 1`; cf_second unchanged (geometric). `kpos_iz` values in this order: **0 → 1.0 → 1.5**. Keep the `res_sign` line out of this block (irrelevant for controller 10).
- **Flights (fresh batteries, pose bag, uSD tags as in §2/§3):**
  1. Single-drone hover 1.0 m, 20 s, `kpos_iz = 0` (Omar exact: expect ≈ +16 cm), then `1.0`, then `1.5` (stop if |roll/pitch| > 25° for 0.5 s or z < 0.25 m).
  2. A8 ×2 with the best value (expected from SIL: mean z within ±1–3 cm; **crossing dips of Omar remain ≈ 15–25 cm below its own level — record them, this is the variant-decision evidence**).
  3. A1 ×2 with the same value.
- **Pass:** hover/A8/A1 mean z error within ±3 cm, no new oscillation (gyro RMS not above the 10-02 Omar level), no windup at takeoff/landing (watch the first 3 s and the descent).
- **Record for Claude:** run tags, `kpos_iz` per flight, vbat min, crossing dips relative to the steady level and absolute.

## 4 — Close-out (send back to Claude)

- Pose bag path(s) under `experiments/logs/rosbags/`
- All `{A8,A1}_{cf5,cf_second}_<date>_*.csv` + matching `*.meta.json`
- Copied uSD bins (`cf5_*`, `cf_second_*`) and merged CSVs if made
- **`post_flight_check` JSON/MD** for the session date
- Note yaml commit hash / `run_formation` printed config for cf5 (`res_sign`, `rnn.en`)
- Any crashes: which drone, time, pose bag yes/no, min `vbat`

---

## Commands verified on desk (2026-10-05)

| Item | Verification |
|------|----------------|
| `flying_drone_stack/tools/merge_usd_logs.py` | `python3 … --help` (exit 0) |
| `flying_drone_stack/tools/check_usd_deck.py` | exists; expects radio URI |
| `experiments/analysis/post_flight_check.py` | `--date 2026-10-05` acceptance run |
| `crazyflie_examples/run_formation.py` | `--scenario`, `--dz`, `--height`, `--hold`, `--passes`, `--auto-center`, `--yes` in `build_parser()` |
| `ros2 bag record … /poses /cf5/pose /cf_second/pose` | standard Jazzy CLI |
| `ros2 topic echo /cf5/pose --once` | standard Jazzy CLI |
| `ros2 launch crazyflie launch.py backend:=cpp` | per project flight card |
