# NS2 / Checklist G — next lab protocol (desk draft, 2026-10-02)

**2026-10-06:** Session order and **copy-paste commands** are consolidated in
[`lab_session_pack_2026-10-06.md`](lab_session_pack_2026-10-06.md) (synced with
[`next_steps_checklist.md`](next_steps_checklist.md)). This file remains the pass/fail rationale and build notes.

Use after pulling both repos. **Do not fly A8 or enable `rnn.en=1` until Step 4 passes.**

## STATUS 2026-10-05 (evening) — read this first (supersedes the 10-03 status below)

Bench A/B/C **passed**; the 100 Hz build works (`lab_sessions/2026-10-05.md`). A8 `rnn.en=0` clean; `rnn.en=1` flights crashed only through
**tracker pose swaps** (identical markers) and **`cf_second` battery collapse (2.5 V)**; with fresh batteries 3 of 3 A8 flights were clean (poses and
batteries verified in rosbags/radio/uSD). Network vs measured residual: corr 0.85–0.92. **Open: sign.** Crossing dip −10.9 cm with the network
(`res_sign=+1`, 16 crossings) vs −5.9 cm without (8 crossings). Next: A8 `res_sign=-1` (cf5 yaml, `ctrl_mode=0` ⇒ only the network term flips),
then `rnn.en=0` repeats, then an A1 `rnn.en=0` baseline. Rules: fresh batteries (rest ≥ 4.1 V), pose bag every session.

**NS2 "ready for the comparative study" gate (set 2026-10-05):** (1) A8 with `res_sign=-1`: crossing dip clearly shallower than (a smaller dip, not a deeper one) the network-off baseline (−5.9 cm), clean flights; (2) A1 with the network on and `-1` stable, after an A1 `rnn.en=0` baseline shows whether cf5's ±40° oscillation is the geometric controller or NS2; (3) no over-compensation (dip not overshooting upward; else a gain factor on the network term); (4) scenario set decided with the supervisor (checked so far: A8, A1 only); (5) operating rules for every comparison flight: fresh batteries (rest ≥ 4.1 V), pose bag, `vbat` min checked. Network is z-only by design (as in the reference); 100 Hz vs faster not compared (meeting question).

## STATUS 2026-10-03 — HISTORICAL (superseded by the 2026-10-05 status above; kept for the bench rationale. Commands below use `humble` / `cflib` from that day — current commands: `lab_session_pack_signtest.md`)

First hardware attempt did **not** validate (`lab_sessions/2026-10-03.md`). `cf5` crashed on both A8 flights; its
onboard position was identical to `cf_second`'s; the flash-RNN build evaluates the network on **every 1 kHz tick**
(reference: ≈550 µs per network ⇒ ≈100 Hz max). **Do not fly NS2 again until the bench steps below pass.**

**Revised lab order (bench before flight):**

| Step | What | Pass | If it fails |
|---|---|---|---|
| 0 | Charged batteries; pull both repos on lab PC; build from `~/georg/ros2_ws` root; card THESIS1 fsck | pacing marker `CS2_CONNECT_PARAM_PACE_V1` in launch | fix environment first |
| A | Flash **default** build on `cf5` (`make DRONE=bl`, ≈402 KB), `cpp` launch | `/cf5/pose` publishes, no `rate is off`/Kalman warnings, no assert | not firmware → hardware/radio/mocap; stop, send log |
| B | Flash **100 Hz + timing** build (artifact below), `read_rnn_timing.py` with CS2 stopped | `rnn.us_max` ≪ 1000 µs, avg stable; no stabilizer/Kalman rate warnings; `/cf5/pose` OK | network too heavy even at 100 Hz → contingencies (smaller net / off-board) |
| C | A8 predict-only, `rnn.en=0`, `cf5` geometric (`ctrl_mode=0`, `ki_z=16`), **yaml pulled** | `rnn_pred_x/y/z` non-zero, **`cf5` position ≠ `cf_second` position** | mix-up persists on a clean build → `cpp` ID path implicated, stop |
| D | Checklist G (`rnn.en=1`, A1/A3) | non-zero preds, stable flight | — |

Build and flash are separate; plain `make cload` can rebuild the default over the RNN build.
Copy each `.bin` to `build_artifacts/` so one build cannot overwrite the other unnoticed:
```bash
cd ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app
make DRONE=bl
cp build/cf21bl.bin build_artifacts/cf21bl_default.bin    # Flash ≈402512 B (sha256 e766f0a4…)

make DRONE=bl all-rnn-flash
cp build/cf21bl.bin build_artifacts/cf21bl_rnn_100hz.bin  # Flash ≈484392 B, rnn.div=10 default (sha256 f7711848…)

# cf5 (do not run until bench step chosen):
cfloader flash build_artifacts/cf21bl_default.bin stm32-fw -w radio://0/80/2M/E7E7E7BB02
cfloader flash build_artifacts/cf21bl_rnn_100hz.bin stm32-fw -w radio://0/80/2M/E7E7E7BB02

# Timing (add rnn.us_last, rnn.us_max, rnn.us_avg to crazyflies.yaml firmware_logging first):
python3 ~/Desktop/flying_robot_course/flying_drone_stack/tools/read_rnn_timing.py \
  radio://0/80/2M/E7E7E7BB02 --seconds 10
# Pass: us_max well under 1000 µs; us_avg ≪ 1000; optional rnn.rst=1 to clear peak between runs
```
The stabilizer-stack patch (`flying_drone_stack/firmware_patches/stabilizer_stack_8x.patch`) must be applied to
`~/Desktop/crazyflie-firmware` (`git apply`) before any RNN build; the default build does not need it but is not harmed by it.

**Extra bench test (no flight, motors OFF), added 2026-10-03:** both drones on the floor ~0.5 m apart, `ros2 topic echo /poses`, flip `cf5` by hand → does the `cf5` pose jump onto `cf_second`? (Tests the hypothesis that the late `cf5`↔`cf_second` position lock is the mocap tracker re-assigning a flipped, identical-marker body — a consequence of the crash, not its cause.)

**Gate caveat from 10-03:** `rnn_pred_z` was non-zero, but `rnn_pred_x/y` were exactly 0 and `rnn_pred_z` ≈ constant —
the gate on z alone is **not** sufficient evidence the neighbour path works. Architecture is **Z-only**
by design ([`52_NS2_Reference_Comparison.md`](52_NS2_Reference_Comparison.md)); constant `rnn_pred_z` can be
**ground φ_G** (~−4 m/s² plausible) even when neighbours are wrong — require **variation with peer motion**
on A8 predict-only.

**Desk G5 (2026-10-03, corrected):** CS2 yaml has no `g_rnn_div`; use `run_ns2_div_sim.py` (**subprocess-isolated**,
optional **100 Hz peer packets** via `oot_set_peer` wrapper). Custom A8-like pass, `np` plant (no downwash).
**Realistic 100 Hz peers, predict-only:** div1 vs div10 mean RMS **~0.001 m/s²**, fraction |Δ|>0.1 **~0%**.
**1 kHz SIL peer stamping** inflates diff (noisy 1 ms differencing, clamp spikes). Appendix I's 0.93 m/s² figure
was **State carry-over + wrong peer rate** — retracted in Validation 9 / Appendix J.

## 0. Firmware feature (read first)

On **CF21BL**, only two builds can run a real onboard network:

| Build | `make` | RAM upload (`residual_nn`) | Flash weights (`residual_nn_flash`) |
|--------|--------|----------------------------|-------------------------------------|
| **Default** | `make DRONE=bl` | **Does not link** (RAM overflow ~41 KB) | N |
| **Flash RNN** | `make DRONE=bl all-rnn-flash` then copy to `build_artifacts/`; flash with `cfloader` (do **not** use plain `make cload` — it can rebuild default firmware over the RNN artifact) | N/A | **Yes** — `g_rnn_ready=1` from boot; preds from embedded weights |
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

**2026-10-03 hardware finding:** flash-RNN build ran the NN every tick from boot (`g_rnn_ready=1`) and overflowed the 1.8 KB stabilizer stack (eval ~610 B + phi_forward ~585 B), giving a boot `radiolink.c:171` assert and `cf5` pose stuck at 0. Fix: `STABILIZER_TASK_STACKSIZE` 3x -> 8x in `crazyflie-firmware/src/config/config.h`, saved as `flying_drone_stack/firmware_patches/stabilizer_stack_8x.patch` (re-apply with `git apply` after any firmware tree reset). Pose confirmed working after flashing.

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

Re-count variables vs **56-variable** uSD cap (`MAX_USD_LOG_VARIABLES_PER_EVENT` in patched `usddeck.c`) before flashing cards.

## 6. Peer data (quality, not all-zero gate)

With **`backend=cpp`**, mocap → **`poses`** → broadcast packed extpos → `peer_localization`.  
There is **no** onboard log of peer slots today; merged **`rel.*`** is post-hoc only.

If preds are non-zero but flat at crossings, debug peer IDs / broadcast before trusting interaction metrics.
