# NS2 / Checklist G — next lab protocol (desk draft, 2026-10-02)

Use after pulling both repos. **Do not fly A8 or enable `rnn.en=1` until Step 4 passes.**

## 0. Firmware feature (read first)

On **CF21BL**, only two builds can run a real onboard network:

| Build | `make` | RAM upload (`residual_nn`) | Flash weights (`residual_nn_flash`) |
|--------|--------|----------------------------|-------------------------------------|
| **Default** | `make DRONE=bl` | **Does not link** (RAM overflow ~41 KB) | N |
| **Flash RNN** | `make DRONE=bl cload-rnn-flash` (from `flying_drone_stack/firmware_app/`; override `RNN_WEIGHTS_NPZ=…` if needed) | N/A | **Yes** — `g_rnn_ready=1` from boot; preds from embedded weights |
| **SIL / host** | `--features residual_nn` on x86_64 | Yes (host only) | N |

**What is on `cf5` today cannot be proven from git alone.** Desk indicators:

- **`build/cf21bl.bin` size (approx.):** default **~402 KB** flash text; **flash-RNN ~485 KB** (Oct 2 desk rebuild with 2026-10-01 npz).
- **Last saved `build/cf2.bin` in this tree:** **393340 B, 2025-09-21** — likely **pre–flash-RNN** default, **not** proof of lab drone.

**At next connect / reflash, record:**

```bash
cd ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app
ls -la build/cf21bl.bin   # after your make
# Optional: note flash map line from make output (text: ~391k default vs ~474k flash-RNN)
make DRONE=bl cload-rnn-flash CLOAD_ARGS='-w radio://0/80/2M/E7E7E7BB02'
# Save cload stdout (flash size / timestamp) in the lab log.
```

If the drone still has **default** firmware: **`rnn.ready=1` after CRTP upload is misleading** — `cf_rnn_finish_upload()` is a stub; **`rnn_pred_*` stay 0**. Fix: **reflash flash-RNN** (command above) before any G attempt.

## 1. Environment — CS2 launch (`backend=cpp`)

**Why not `cflib`:** this lab’s session docs (`2026-09-28_alt_indi_shakedown.md`, etc.) always used
`backend:=cflib`, which sends **per-drone** `send_extpos` only — **no packed multi-ID broadcast**,
so firmware **`peer_localization` never sees other drones**. NS2 needs the C++ path
(`sendExternalPositions` in `crazyflie_server.cpp` → `posesChanged`).

**Desk readiness (2026-10-02, this machine):**

| Check | Status |
|--------|--------|
| C++ node built | **Yes** — `~/Desktop/crazyswarm2/build/crazyflie/crazyflie_server` (~19 MB, last `colcon build` OK) |
| Installed symlink | **Yes** — `install/crazyflie/lib/crazyflie/crazyflie_server` |
| Shared libs | **Yes** — `ldd` shows no missing `.so` |
| Node starts | **Yes** — binary runs, prints connect-param pacing banner |
| `motion_capture_tracking` | **Yes** — `ros2 pkg list` (external to crazyswarm2 workspace; required when `mocap:=True`) |
| Extra yaml for `cpp` | **No** — same `crazyflies.yaml` / `motion_capture.yaml`; `broadcaster_` is created from each robot’s `uri` via `broadcastUriFromUnicastUri` (e.g. `cf5` + `cf_second` on channel **80** → one `radiobroadcast://*/80/2M` shared by both) |
| Launch flags vs lab habit | Lab used **`backend:=cflib` only**; everything else was launch defaults (`mocap:=True`, `teleop:=True`, `gui:=True`). **`cpp` uses the same parameter bundle** — only the server executable changes (`launch.py` lines 77–92). |

**Not desk-verifiable without lab hardware:** OptiTrack → `poses` → broadcast actually received on both
radios, and peer IDs matching `address & 0xFF`. That is Step 3 uSD / behaviour, not a build step.

**Exact launch (terminal 1 — match 2026-09-28 habit, swap backend only):**

```bash
cd ~/Desktop/crazyswarm2
source install/setup.bash
ros2 launch crazyflie launch.py backend:=cpp
```

(`backend` defaults to **`cpp`** in `launch.py` anyway; **`backend:=cpp` is explicit so nobody
accidentally copies an old `cflib` one-liner from session notes.)

Optional (same as defaults): `mocap:=True teleop:=True gui:=True rviz:=False`.

**If the C++ server was never built on the lab PC:** on that machine once:

```bash
cd ~/Desktop/crazyswarm2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select crazyflie_interfaces crazyflie
source install/setup.bash
```

Requires ROS Humble, `libusb`, and build deps from the `crazyflie` `package.xml` (same as any CS2
hardware session). No drone or mocap needed for the build.

**Switching `cpp` ↔ `cflib` in one lab session:** stop the whole launch (**Ctrl-C**), change
`backend:=…`, relaunch. Drones do not retain “backend mode”; only one `crazyflie_server` may run.
**NS2 / any 2-drone peer test → `cpp`.** Non-NS2 flights that worked under **`cflib`** (Omar
controllers, INDI compare) should still work under **`cpp`** — same HL services and
`firmware_logging` topics; `docs/FLIGHT_CARD_VALIDATION.md` already uses default launch (cpp). Use
**`cflib` only if you deliberately want the old path** (no peer broadcast).

**Other environment (unchanged):**

- uSD **`config.txt`** = **`usd_thesis_config.txt`** (not Omar-only variant).
- **`rnn.en=0`** in yaml until Step 6.

## 2. Upload (if using RAM-upload build on a platform that links — not CF21BL)

Skip weight upload if using **`residual_nn_flash`** (weights baked in); still verify **`rnn.ready=1`** on connect.

For upload path (host/SIL or future platform):

```bash
cd ~/Desktop/crazyswarm2
ros2 run crazyflie_examples upload_residual_weights -- \
  --weights "$HOME/Desktop/flying_robot_course/experiments/analysis/out/c2_e2e_2026-10-01/full_bank_c1_complete.npz" \
  --cf cf5
```

Require script line: **`verified rnn.ready=1, rnn.n=19297`**.  
Note: **`rnn.n` alone is not proof** weights live in Rust — treat Step 4 as ground truth.

## 3. Short hover gate (mandatory)

- Single drone or minimal command: **~3–5 s hover** at ~1 m (e.g. `simple_flight` / short `run_formation` A1 takeoff segment), **`rnn.en=0`**.
- Merge uSD immediately.

**Pass criteria:**

- **`rnn_pred_x/y/z` not all zero** (any sample with `|rnn_pred_z| > 1e-3` m/s² is enough for a first gate).
- Optional: add **`rnn.ready`** to uSD config (see below) — should be **1** for the whole log.

**Fail:** do **not** fly A8 for NS2, do **not** set `rnn.en=1`. Reflash / rebuild / diagnose on desk.

## 4. Only after Step 3 passes

- Predict-only scenario (e.g. **A3** or short **A8**) with **`rnn.en=0`**, merge, check preds vs `a_res_*`.
- Then **Checklist G:** **`rnn.en=1`**, **A1 or A3**, merge again.

## 5. uSD config addition (proposed)

`tools/usd_thesis_config.txt` already logs **`rnn.pred_*`** and **`rnn.clamped`**. It does **not** log **`rnn.ready`**.

Add before merge/decode mapping (and extend `decode_usd_log.py` `RENAME` if used):

```
rnn.ready
```

Re-count variables vs **40-variable** uSD limit before flashing cards.

## 6. Peer data (quality, not all-zero gate)

With **`backend=cpp`**, mocap → **`poses`** → broadcast packed extpos → `peer_localization`.  
There is **no** onboard log of peer slots today; merged **`rel.*`** is post-hoc only.

If preds are non-zero but flat at crossings, debug peer IDs / broadcast before trusting interaction metrics.
