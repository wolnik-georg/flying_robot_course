# Lab bench cheat sheet (2026-10-03)

Single operator document for the next lab session.

```bash
# Lab PC (ROS 2 Jazzy, CS2 build)
REPO=~/georg/flying_robot_course          # e.g. /home/flyingrobots/georg/flying_robot_course
WS=~/georg/ros2_ws                        # crazyswarm2: $WS/src/crazyswarm2

# Laptop (Crazyradio + cfloader; flash artifacts live here — NOT on lab PC git tree)
LAPTOP_REPO=~/Desktop/flying_robot_course # adjust if your laptop path differs
```

**Firmware per step:** **A** = default bin · **B / B2 / C** = RNN 100 Hz bin (`cf21bl_rnn_100hz.bin`) · **D** = default again (re-flash before D).

**Decision tree:** **0 / A / B** fail → stop flying. **C** (RNN build) fail → no Checklist G. **D** (default build, ki_z A/B) fail → keep `rnn.en=0`. **E** only if **0→A→B→B2→C→D** pass.

---

## 0 — Pre-flight (environment)

```bash
REPO=~/georg/flying_robot_course
WS=~/georg/ros2_ws

cd "$REPO" && git pull
cd "$WS/src/crazyswarm2" && git pull

cd "$WS"
colcon build --symlink-install --packages-select crazyflie_interfaces crazyflie_py crazyflie crazyflie_examples
source "$WS/install/setup.bash"

ros2 launch crazyflie launch.py
# PASS: log contains CS2_CONNECT_PARAM_PACE_V1
# FAIL: Assert failed, or STAB:/ESTKALMAN rate is off on cf5 → power-cycle, send launch log
```

**Send back:** Step 0 PASS/FAIL + last 30 lines of launch log if FAIL.

**THESIS card (if read-only):**

```bash
sudo umount /media/$USER/THESIS1 2>/dev/null; sudo fsck.vfat -a /dev/sda1
```

**uSD deck (CS2 stopped):**

```bash
REPO=~/georg/flying_robot_course
python3 "$REPO/flying_drone_stack/tools/check_usd_deck.py" radio://0/80/2M/E7E7E7BB02
```

**Yaml (read-only):**

```bash
REPO=~/georg/flying_robot_course
WS=~/georg/ros2_ws
python3 "$REPO/flying_drone_stack/tools/lab_preflight.py" \
  --yaml "$WS/src/crazyswarm2/crazyflie/config/crazyflies.yaml" \
  --expect-geometric-cf5
```

---

## A — Default firmware on cf5 (flash on **laptop**)

`build_artifacts/*.bin` is **gitignored** on the lab PC — flash from the **laptop** (Crazyradio + `cfloader`).

```bash
LAPTOP_REPO=~/Desktop/flying_robot_course
cd "$LAPTOP_REPO/flying_drone_stack/firmware_app"
sha256sum build_artifacts/cf21bl_default.bin
# EXPECT: e766f0a43128bc1bfbc1a0f1f0783c213da8cc99477745e418dafef0664fc817

# Stop CS2 / any radio app first; power-cycle cf5 after flash
cfloader flash "$LAPTOP_REPO/flying_drone_stack/firmware_app/build_artifacts/cf21bl_default.bin" \
  stm32-fw -w radio://0/80/2M/E7E7E7BB02
```

**Verify on lab PC (after flash + power-cycle):**

```bash
WS=~/georg/ros2_ws
source "$WS/install/setup.bash"
ros2 launch crazyflie launch.py
ros2 topic echo /cf5/pose --once
# PASS: non-zero pose; no Assert failed; no STAB/ESTKALMAN rate-off on cf5
```

**Send back:** sha256 line, one `/cf5/pose`, PASS/FAIL.

---

## B — 100 Hz RNN + timing (flash on **laptop**)

```bash
LAPTOP_REPO=~/Desktop/flying_robot_course
cd "$LAPTOP_REPO/flying_drone_stack/firmware_app"
sha256sum build_artifacts/cf21bl_rnn_100hz.bin
# EXPECT: f7711848ac17063c7ddc31564935b22e38ffac704ab64a0b714e5110438ea2ea

# Stop CS2; power-cycle after flash
cfloader flash "$LAPTOP_REPO/flying_drone_stack/firmware_app/build_artifacts/cf21bl_rnn_100hz.bin" \
  stm32-fw -w radio://0/80/2M/E7E7E7BB02

python3 "$LAPTOP_REPO/flying_drone_stack/tools/read_rnn_timing.py" \
  radio://0/80/2M/E7E7E7BB02 --seconds 10
# Script exit 0 = PASS; non-zero = FAIL (see messages)
```

**Bench B works disarmed:** RNN still runs; `rnn.us_*` should update on the bench.

**PASS:** exit **0**; peak &lt; 300 µs ideal; **never** pass if all `us_last` are 0.  
**FAIL:** exit **1** (no samples / all zero / peak &gt; `--fail-above-us`); exit **2** (no `rnn.us_last` in log TOC — wrong firmware).

Optional: param `rnn.rst=1` to clear peak between runs.

**Lab PC check after B:**

```bash
WS=~/georg/ros2_ws
source "$WS/install/setup.bash"
ros2 launch crazyflie launch.py
ros2 topic echo /cf5/pose --once
# PASS: pose OK; no STAB/ESTKALMAN rate-off on cf5
```

**Send back:** full `read_rnn_timing.py` stdout + exit code.

Offline check (no radio): `python3 "$LAPTOP_REPO/flying_drone_stack/tools/read_rnn_timing.py" --mock-self-test-capture`

---

## B2 — Mocap tracker bench (motors OFF)

```bash
WS=~/georg/ros2_ws
source "$WS/install/setup.bash"
ros2 topic echo /poses
# Flip cf5 by hand — PASS if cf5 track does NOT jump onto cf_second
```

---

## C — A8 predict-only on **RNN build** (`rnn.en=0`)

cf5 should still be on **`cf21bl_rnn_100hz.bin`** from step B. Pull yaml; confirm `run_formation` prints per-drone config.

```bash
WS=~/georg/ros2_ws
source "$WS/install/setup.bash"
ros2 launch crazyflie launch.py
# other terminal:
# Match 10-03 A8 meta (experiments/logs/A8_2026-10-03_13-15-00.meta.json): dz=0.5, height=0.5, passes=4.
# Do NOT drop --height 0.5 — default is 1.0 m (run_formation.py), top would be ~1.5 m with dz=0.5.
ros2 run crazyflie_examples run_formation \
  --scenario A8 --dz 0.5 --height 0.5 --passes 4 --auto-center --yes
```

### C.1–C.3 — uSD copy, ID, merge (**LAPTOP only**)

Cards mount on the **laptop** (`/media/georg/THESIS1`, `/media/georg/THESIS2`). `decode_usd_log.py` needs `cfusdlog` at `~/Desktop/crazyflie-firmware/tools/usdlog` — **not** on the lab PC. Use **system `python3`** (same as `merge_usd_logs.py` / host tests). Pull meta/CSV paths via `git pull` in `$LAPTOP_REPO` after the lab session.

```bash
LAPTOP_REPO=~/Desktop/flying_robot_course
PYTHON=python3
cd "$LAPTOP_REPO" && git pull

MOUNT1=/media/georg/THESIS1
MOUNT2=/media/georg/THESIS2
DEST="$LAPTOP_REPO/experiments/logs/usd_raw"

$PYTHON "$LAPTOP_REPO/flying_drone_stack/tools/copy_usd_log.py" "$MOUNT1" card_a --tag A8 --dest "$DEST"
$PYTHON "$LAPTOP_REPO/flying_drone_stack/tools/copy_usd_log.py" "$MOUNT2" card_b --tag A8 --dest "$DEST"
```

**Primary ID — `max(ctrltarget_z)`:**

```bash
LAPTOP_REPO=~/Desktop/flying_robot_course
PYTHON=python3
DEST="$LAPTOP_REPO/experiments/logs/usd_raw"
$PYTHON -c "
import sys
sys.path.insert(0, '${LAPTOP_REPO}/flying_drone_stack/tools')
from decode_usd_log import load
import numpy as np
for p in sys.argv[1:]:
 d=load(p); mz=float(np.nanmax(d['ctrltarget_z']))
 print(p.split('/')[-1], 'max(ctrltarget_z)=', round(mz,2), 'm ->', 'cf5' if mz < 0.75 else 'cf_second')
" "$DEST"/card_a_A8_*.bin "$DEST"/card_b_A8_*.bin
```

| max(`ctrltarget_z`) | Drone |
|---|---|
| ≈ **0.5 m** | **cf5** (bottom) |
| ≈ **1.0 m** | **cf_second** (top) |

**Secondary (2) — `find_flight_window.py` RMS** (sanity only; **crashed cf5** often shows **bottom RMS ≈ 90 cm** — that is **not** a card swap error):

```bash
LAPTOP_REPO=~/Desktop/flying_robot_course
PYTHON=python3
DEST="$LAPTOP_REPO/experiments/logs/usd_raw"
META="$LAPTOP_REPO/experiments/logs/A8_<YYYY-MM-DD>_<HH-MM-SS>.meta.json"
for BIN in "$DEST"/card_a_A8_*.bin "$DEST"/card_b_A8_*.bin; do
  echo "=== $(basename "$BIN") ==="
  $PYTHON "$LAPTOP_REPO/flying_drone_stack/tools/find_flight_window.py" "$BIN" bottom "$META"
  $PYTHON "$LAPTOP_REPO/flying_drone_stack/tools/find_flight_window.py" "$BIN" top "$META"
done
```

If merge **refuses** with bottom RMS ≫ 15 cm after correct ctrltarget_z ID, treat as **crash/tracker** (expected on 10-03), not mis-pairing.

**Symlinks for merge** (after ID):

```bash
LAPTOP_REPO=~/Desktop/flying_robot_course
WORKDIR="$LAPTOP_REPO/experiments/logs/usd_raw/_merge_staging"
mkdir -p "$WORKDIR"
ln -sf "$(readlink -f /path/to/cf5_file.bin)" "$WORKDIR/cf5_A8_thesisNN.bin"
ln -sf "$(readlink -f /path/to/cf_second_file.bin)" "$WORKDIR/cf_second_A8_thesisNN.bin"
```

**Merge:**

```bash
LAPTOP_REPO=~/Desktop/flying_robot_course
PYTHON=python3
META="$LAPTOP_REPO/experiments/logs/A8_<YYYY-MM-DD>_<HH-MM-SS>.meta.json"
WORKDIR="$LAPTOP_REPO/experiments/logs/usd_raw/_merge_staging"

$PYTHON "$LAPTOP_REPO/flying_drone_stack/tools/merge_usd_logs.py" \
  "$WORKDIR/cf5_A8_thesisNN.bin" \
  "$WORKDIR/cf_second_A8_thesisNN.bin" \
  --meta "$META" \
  --roles bottom top \
  -o "$LAPTOP_REPO/experiments/logs/merged_A8_<stamp>.csv"
```

**PASS:** merge completes; `rnn_pred_*` non-zero; cf5 position ≠ cf_second.  
**FAIL:** `[merge] refusing to merge…` with bad **bottom** RMS after correct ctrltarget_z ID → crash flight; do not “fix” pairing.

**Send back:** merge stdout + exit code.

---

## D — `ki_z` A/B on **default firmware** (after C)

**C used the RNN bin. D requires the default bin again.**

```bash
LAPTOP_REPO=~/Desktop/flying_robot_course
cd "$LAPTOP_REPO/flying_drone_stack/firmware_app"
sha256sum build_artifacts/cf21bl_default.bin
# EXPECT: e766f0a43128bc1bfbc1a0f1f0783c213da8cc99477745e418dafef0664fc817

# Re-flash default (CS2 stopped; power-cycle)
cfloader flash "$LAPTOP_REPO/flying_drone_stack/firmware_app/build_artifacts/cf21bl_default.bin" \
  stm32-fw -w radio://0/80/2M/E7E7E7BB02
```

Compare **ki_z=16** (yaml default) vs **ki_z=0** (patch), **same default firmware**, **≥2–3 attempts per setting**.

**Attempt order (alternate to reduce confounding):** **16 → 0 → 16 → 0** (fresh batteries before each pair if possible).

CS2 reads `firmware_params` **only at launch** — after every `git apply` or `git checkout`, **Ctrl-C** the running launch and start again:

```bash
REPO=~/georg/flying_robot_course
WS=~/georg/ros2_ws
cd "$WS/src/crazyswarm2"
git apply "$REPO/docs/lab_prep_geometric_kiz0_test.patch"
source "$WS/install/setup.bash"
ros2 launch crazyflie launch.py
# Confirm run_formation prints cf5 … ki_z=0.0 before flying this leg
# After attempts: git checkout -- crazyflie/config/crazyflies.yaml
# Restart launch again before ki_z=16 attempts; confirm ki_z=16.0 in run_formation line
```

Fly **A1 with the flags of the original geometric tumbles** (`experiments/logs/A1_2026-10-01_18-38-29.meta.json` … `19-18-18.meta.json`: `cf5` controller 6, `ctrl_mode` 0, **`dz=0.3`, `hold=10`, default `--height 1.0`**). Do **not** reuse the 10-02 A1 flags (`dz 0.5, height 0.5, hold 15` — `A1_2026-10-02_18-36-18.meta.json` was `cf5` **controller 9 = Omar C**, a different controller, and does not reproduce the tumble condition). Note: the `--height 0.5` rule applies to **A8** (top drone at height+dz); for A1 at `dz 0.3` the default height 1.0 was flown safely on 10-01 (top drone 1.3 m).

```bash
# launch already running from block above; second terminal:
ros2 run crazyflie_examples run_formation \
  --scenario A1 --dz 0.3 --hold 10 --auto-center --yes
```


**PASS (each ki_z setting, ≥2–3 attempts):** no |roll| or |pitch| &gt;20° within **3 s of liftoff** (radio CSV or `/cf5/attitude`).

**Send back:** ki_z setting, PASS/FAIL, max tilt in first 3 s.

---

## E — Checklist G (only if A–D pass)

Re-flash **`cf21bl_rnn_100hz.bin`** if G needs onboard RNN (same laptop flash block as step B).

```bash
REPO=~/georg/flying_robot_course
WS=~/georg/ros2_ws
python3 "$REPO/flying_drone_stack/tools/lab_preflight.py" \
  --yaml "$WS/src/crazyswarm2/crazyflie/config/crazyflies.yaml" --allow-rnn-en
# Set rnn.en=1; A1 or A3; merge uSD again
```

---

## Quick reference

| Artifact (laptop path) | SHA256 |
|---|---|
| `…/build_artifacts/cf21bl_default.bin` | `e766f0a4…` (402512 B; peer-resync fix 2026-10-03) |
| `…/build_artifacts/cf21bl_rnn_100hz.bin` | `f7711848…` (484392 B) |

| Step | Firmware on cf5 |
|---|---|
| A | default |
| B, B2, C | RNN 100 Hz |
| D | **default (re-flash)** |
| E | RNN 100 Hz (re-flash if needed) |
