# Lab session pack — NS2 first (2026-10-06)

**SUPERSEDED by [`lab_session_pack_signtest.md`](lab_session_pack_signtest.md) (2026-10-05 session).**

**Goal:** answer one question — does **100 Hz** network eval (`rnn.div=10`) stop **cf5** crashes?

**Synced with:** [`next_steps_checklist.md`](next_steps_checklist.md) · [`meetings/2026-10-05.md`](meetings/2026-10-05.md) (§ Next steps)

**Detail / commands:** [`next_flight_card.html`](next_flight_card.html) · [`ns2_next_lab_protocol.md`](ns2_next_lab_protocol.md) · [`lab_bench_cheatsheet_2026-10-03.md`](lab_bench_cheatsheet_2026-10-03.md)

---

## Immediate lab steps (summary)

**Before leaving:** both `.bin` files + Crazyradio + charged batteries; **fsck THESIS cards** + `config.txt` = thesis config.

1. Lab PC: pull repos → `colcon build` (Jazzy) → `lab_preflight` → launch → `/cf5/pose`.
2. Bench **A** default → **B** RNN + `read_rnn_timing.py` (CS2 stopped) → **C** tracker flip. **If B fails → stop** (no NS2 flights).
3. **Flight 1:** A8, `rnn.en=0`, **100 Hz bin** — clean **twice** before step 4.
4. **If flight 1 clean twice:** `rnn.en=1` on **A1 or A3** (100 Hz bin).
5. **If flight 1 crashes:** **`ki_z` 16 vs 0** on **default** bin (separate network load from integral).

---

## Copy-paste commands

**Paths:** lab PC uses `REPO=~/georg/flying_robot_course`, `WS=~/georg/ros2_ws`. Laptop flash uses `LAPTOP_REPO=~/Desktop/flying_robot_course` (adjust if different).

### Desk verify (before leaving — laptop or desk)

```bash
~/Desktop/flying_robot_course/flying_drone_stack/tools/lab_session_pack_verify.sh
```

### 0 — Lab PC environment

```bash
REPO=~/georg/flying_robot_course
WS=~/georg/ros2_ws

cd "$REPO" && git pull
cd "$WS/src/crazyswarm2" && git pull

cd "$WS"
source /opt/ros/jazzy/setup.bash   # lab PC = ROS 2 Jazzy (docs/lab_sessions/2026-10-03.md)
colcon build --symlink-install --packages-select crazyflie_interfaces crazyflie_py crazyflie crazyflie_examples
source "$WS/install/setup.bash"

python3 "$REPO/flying_drone_stack/tools/lab_preflight.py" \
  --yaml "$WS/src/crazyswarm2/crazyflie/config/crazyflies.yaml" \
  --expect-geometric-cf5

ros2 launch crazyflie launch.py
# Other terminal:
ros2 topic echo /cf5/pose --once
```

### uSD cards (home or lab — before first flight)

**Use `usd_thesis_config.txt`** (logs `rnn.pred_*`, `rnn.clamped`, 500 Hz thesis set). Do **not** leave Omar-only config on the cards.

```bash
REPO=~/Desktop/flying_robot_course   # laptop path; use ~/georg/... on lab PC for repo only
CFG="$REPO/flying_drone_stack/tools/usd_thesis_config.txt"

# THESIS1 (adjust device if needed)
sudo umount /media/$USER/THESIS1 2>/dev/null
sudo fsck.vfat -a /dev/sda1          # fix device name: lsblk

sudo mount /dev/sda1 /media/$USER/THESIS1
cp "$CFG" /media/$USER/THESIS1/config.txt
sync
sudo umount /media/$USER/THESIS1

# Repeat for THESIS2 / second card (device name will differ)
```

**After cards are in drones (CS2 stopped):**

```bash
REPO=~/georg/flying_robot_course
python3 "$REPO/flying_drone_stack/tools/check_usd_deck.py" radio://0/80/2M/E7E7E7BB02
python3 "$REPO/flying_drone_stack/tools/check_usd_deck.py" radio://0/80/2M/E7E7E7E7E9
```

**Yaml for NS2 flights (lab PC, after `git pull`):** `cf5`: `controller=6`, `ctrl_mode=0`, `ki_z=16`, **`rnn.en=0`** until Checklist G. Confirm with `lab_preflight.py` and read **`run_formation` printed config** before arming.

### A — Flash default (laptop + Crazyradio; stop CS2 first)

```bash
LAPTOP_REPO=~/Desktop/flying_robot_course
sha256sum "$LAPTOP_REPO/flying_drone_stack/firmware_app/build_artifacts/cf21bl_default.bin"
# expect e766f0a43128bc1bfbc1a0f1f0783c213da8cc99477745e418dafef0664fc817

cfloader flash "$LAPTOP_REPO/flying_drone_stack/firmware_app/build_artifacts/cf21bl_default.bin" \
  stm32-fw -w radio://0/80/2M/E7E7E7BB02
# power-cycle cf5; then launch on lab PC and echo /cf5/pose
```

### B — Flash RNN 100 Hz + timing (laptop; CS2 stopped for timing script)

```bash
LAPTOP_REPO=~/Desktop/flying_robot_course
sha256sum "$LAPTOP_REPO/flying_drone_stack/firmware_app/build_artifacts/cf21bl_rnn_100hz.bin"
# expect f7711848ac17063c7ddc31564935b22e38ffac704ab64a0b714e5110438ea2ea

cfloader flash "$LAPTOP_REPO/flying_drone_stack/firmware_app/build_artifacts/cf21bl_rnn_100hz.bin" \
  stm32-fw -w radio://0/80/2M/E7E7E7BB02

python3 "$LAPTOP_REPO/flying_drone_stack/tools/read_rnn_timing.py" \
  radio://0/80/2M/E7E7E7BB02 --seconds 10
```

### C — Tracker flip (motors off, mocap running)

```bash
WS=~/georg/ros2_ws
source "$WS/install/setup.bash"
ros2 topic echo /poses
# flip cf5 by hand ~0.5 m from cf_second — note if cf5 track jumps to cf_second
```

### Flight 1 — A8 predict-only (`rnn.en=0` in yaml; stay on RNN bin from B)

```bash
WS=~/georg/ros2_ws
source "$WS/install/setup.bash"
ros2 launch crazyflie launch.py
# other terminal:
ros2 run crazyflie_examples run_formation \
  --scenario A8 --dz 0.5 --height 0.5 --passes 4 --auto-center --yes
```

### Flight 1b — repeat A8 (same as above)

Second clean A8 with `rnn.en=0` before enabling compensation.

### Flight 2 — Checklist G (`rnn.en=1`, A1 or A3)

Only if **two** clean A8 predict-only flights. Stay on **RNN 100 Hz** bin. Set `rnn.en=1` in yaml, pull/param-update, **`lab_preflight`**, then e.g.:

```bash
ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.5 --height 0.5 --auto-center --yes
```

### Flight 3 — only if A8 **crashed** (not if gate passed)

Re-flash **default** bin (step A). Yaml-only **`ki_z` 16 vs 0** — see [`lab_bench_cheatsheet_2026-10-03.md`](lab_bench_cheatsheet_2026-10-03.md) §D.

### After every flight (laptop, cards in reader)

```bash
LAPTOP_REPO=~/Desktop/flying_robot_course
MOUNT=/media/georg/THESIS1   # or THESIS2
DEST="$LAPTOP_REPO/experiments/logs/usd_raw"
python3 "$LAPTOP_REPO/flying_drone_stack/tools/copy_usd_log.py" "$MOUNT" card_a --tag A8 --dest "$DEST"
# confirm new thesisNN folder covers full flight duration on card before unmount
```

---

## Status on this desk machine (2026-10-05)

| Check | Result |
|--------|--------|
| `test_residual_nn.py` | **31/31 PASS** |
| Stack 8× patch in `crazyflie-firmware` | **applied** (`STABILIZER_TASK_STACKSIZE` 8×) |
| `build_artifacts/cf21bl_default.bin` | **402512 B**, sha256 `e766f0a4…` |
| `build_artifacts/cf21bl_rnn_100hz.bin` | **484392 B**, sha256 `f7711848…` |
| `lab_preflight.py` on local `crazyflies.yaml` | **PASS** (`cf5`: ctrl 6, mode 0, `ki_z=16`, `rnn.en=0`) |
| CS2 `crazyflie_server` binary | present under `~/Desktop/crazyswarm2/build/` |

Re-run before you leave: `flying_drone_stack/tools/lab_session_pack_verify.sh`

---

## Tonight / before lab (desk + laptop)

1. **Git pull both repos** on lab PC *and* laptop; commit or stash anything you still need on the laptop.
2. **Do not use `make cload` alone** for NS2 — it can rebuild **default** over the RNN artifact. Flash from **`build_artifacts/*.bin`** only (`cfloader`).
3. Copy to lab bag: Crazyradio, **both `.bin` files** (or know they live on laptop at paths above), charged batteries, THESIS uSD cards.
4. **Lab PC:** `colcon build` from `~/georg/ros2_ws` (see cheat sheet §0).
5. **Cards:** finish THESIS1 on-card archive if still pending; `fsck.vfat`; `config.txt` → `usd_thesis_config.txt`; run `check_usd_deck.py` per drone when radios are available.
6. **Yaml:** after pull, run `lab_preflight.py --expect-geometric-cf5` and read **`run_formation` printed config** before first flight (10-03 failure: stale yaml).

Optional before NS2 flights (INDI track — **after** NS2 gate passes): apply [`lab_prep_log_filter_params.patch`](lab_prep_log_filter_params.patch) on lab firmware tree only when you start INDI A1 work.

---

## Lab order (do not reorder)

| # | Step | Build on cf5 | Pass |
|---|------|--------------|------|
| 0 | Launch `cpp`, pose, no assert/rate warnings | either | environment OK |
| A | Bench, motors off: **default** bin | default | `/cf5/pose` OK |
| B | **RNN 100 Hz** bin + `read_rnn_timing.py` (CS2 stopped) | RNN | `rnn.us_max` ≪ 1000 µs |
| C | Tracker flip test (floor, ~0.5 m apart) | RNN | note if pose jumps cf5→cf_second |
| 1 | **A8 ×2**, `rnn.en=0`, yaml pulled | RNN | no crash; `rnn_pred_z` varies; x/y=0 OK; cf5 ≠ cf_second |
| 2 | **`rnn.en=1`**, A1/A3 (Checklist G) | RNN | only if two clean A8 runs |
| 3 | **`ki_z` 16 vs 0** (if A8 crashed) | default | separates integral from network load |

**Red flags (stop, no retry same config):** constant `rnn_pred_z` only (x/y=0 is **normal**); cf5 pose locks to cf_second **before** a flip; assert; `rate is off`; gyro chaos on cf5 while cf_second calm.

**After each flight:** new `thesisNN` on card covers flight; merge with `copy_usd_log.py` + matching radio CSV.

---

## What to log in the lab notebook (minimal)

- Flash: which bin, sha256 prefix, time.
- Bench B: `rnn.us_max`, `rnn.us_avg`, `rnn.div` param readback.
- Flight 1: max tilt, crash Y/N, `rnn_pred_z` min/max/mean, whether cf5≠cf_second position in uSD/ROS.
- Any assert / Kalman warning → full console log snippet.

---

## Explicitly **after lab** (do not block the session)

| Item | Why wait |
|------|----------|
| Finish **`docs/62_NS2_Closed_Loop_SIL.md`** + full `ns2_closed_loop_sil_run.py` matrix | SIL baseline not calibrated; lab answers the hardware question first |
| INDI **bench latency** + A1 package flights (`docs/58`/`61`) | separate track; needs log patch + supervisor INDI choice |
| **`ki_z` geometric patch A/B** in flight | only meaningful after NS2 + identity clean (Topic 3) |
| Retrain / smaller network (D8) | only if bench B fails |
| Thesis Ch.6–9 NS2 numbers | after Checklist G or clear fail |
| Supervisor decisions (Pure INDI variant, scenario set) | blocks comparison flights, not NS2 gate |

---

## One-line reminders

- **`cpp` only** for 2-drone NS2 (peer broadcast).
- **`rnn.en=0`** in yaml until flight step 3.
- Full INDI: **`ki_z=0`** always; never **`ki_z=16` + `ctrl_mode=3`**.
- Power-cycle after assert; EKF toggle 1→2 after crash if pose stuck.
