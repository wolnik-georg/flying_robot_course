# Next steps checklist

Simple lab + desk list (as of **2026-10-03**, after the NS2 first hardware attempt — see
[`lab_sessions/2026-10-03.md`](lab_sessions/2026-10-03.md)). Update when items move. Grouped by topic, then desk vs lab.

**One-paragraph state:** NS2 did **not** validate. `cf5` crashed on both A8 flights; `cf_second` was clean.
The flash-RNN firmware evaluates the network every **1 kHz** tick from boot (reference ≈ 550 µs/network ⇒ only feasible
at ≈ 100 Hz) → stack overflow (patched, 8×) and probable MCU overload (unproven). Separately, `cf5`'s position estimate locked onto `cf_second`'s late in both flights — **a consequence of the flip** (tracker re-assignment, hypothesis), not the cause; why `cf5` tumbled first is the real open question. `cf5` is currently flashed with the flash-RNN + stack-8×
build and is unstable; the default build is the known-good fallback. `cpp` is the only backend (`cflib` abandoned).

---

## TOPIC 1 — NS2 / Strategy 2 (the blocker)

### Desk (Cursor NS2 block — see [`lab_sessions/2026-10-03.md`](lab_sessions/2026-10-03.md) Appendices A–D)
- [x] **D1 — 100 Hz network evaluation** (`rnn.div`, hold).
- [x] **D2 — timing log** (`rnn.us_*`, `read_rnn_timing.py`).
- [x] **D3 — static scratch buffers**.
- [x] **D4 — host regression** (`test_residual_nn.py` **28/28**; div sweep via `rnn_div_sweep_host.py`).
- [x] **D5 — builds** (`build_artifacts/cf21bl_default.bin`, `cf21bl_rnn_100hz.bin`).
- [x] **D6 — `docs/13`** controller call rate 1 kHz + 100 Hz hold documented.
- [x] **D7 — identity scan** (`check_estimator_identity.py`, `lock_on_scan.py`).
- [ ] **D8 — contingencies sketched** (only if 100 Hz still doesn't fit): smaller retrained network (C.2 retrain), or off-board evaluation.

### Lab — next session (bench BEFORE flight; criteria in `ns2_next_lab_protocol.md` §0)
1. [ ] **Charge/swap batteries; fix card THESIS1** (`umount` + `fsck.vfat -a`), finish its on-card archive/reset.
2. [ ] **Bench A:** flash **default** build on `cf5` → `cpp` launch → `/cf5/pose` publishes, no rate warnings/assert.
3. [ ] **Bench B:** flash **100 Hz + timing** build → read timing log; max must be well below 1 ms; warnings gone.
4. [ ] **A8 predict-only** (`rnn.en=0`, **pull yaml first**) → merge → `rnn_pred_x/y/z` non-zero **and** `cf5` position ≠ `cf_second` position.
5. [ ] **Checklist G** (`rnn.en=1`, A1/A3) only if 4 passes.

---

## TOPIC 2 — `cf5` position "mix-up" (re-interpreted 2026-10-03: consequence of the flip, not the cause)

- Same-clock radio data: `cf5` has its **own** correct position until it is tumbling; only ≈15.5–16 s (both flights) does its estimate jump onto `cf_second`'s. Hypothesis: mocap rigid-body tracker (ICP, identical 4-marker `cf21_active` layouts) re-assigns the flipped/occluded body to the neighbour. `cpp` ID-path theory demoted.
- [ ] **Bench (no flight, motors off):** `ros2 topic echo /poses`, both drones on the floor ~0.5 m apart, flip `cf5` by hand → does the `cf5` pose jump onto `cf_second`? Confirms/refutes the tracker hypothesis.
- [ ] **Desk:** for every archived 2-drone flight find the lock-on time vs tilt events (does it always follow a flip?); review `motion_capture.yaml` marker/rigid-body config. If confirmed: distinct marker layouts / tracker params before more close-crossing flights.

---

## TOPIC 3 — liftoff tumble / `ki_z` (shelved, not closed)

- [ ] Geometric `ki_z` A/B (`docs/lab_prep_geometric_kiz0_test.patch`) — **only after Topics 1–2 are clean**, otherwise a wrong pose / overloaded MCU contaminates it.
- Fact: full INDI + `ki_z=16` → tumble (confirmed 10-02, direct A/B). Fact: `cf5` geometric + `ki_z=16` tumbled on 10-03 **but with the mix-up present**, so inconclusive. Earlier claim "geometric `ki_z=16` re-flown OK on `cf5`" was **wrong** (only `cf_second`).
- [ ] Optional: liftoff uSD diagnostic (`docs/lab_usd_liftoff_logging_diagnostic.patch`, `FORMATION_USD_AT_TAKEOFF=1`).

---

## TOPIC 4 — thesis tracks independent of NS2

- [ ] **Supervisor:** which "Pure INDI" is Strategy 1 — `controller=6/ctrl_mode=3` vs Omar C/Rust (10-02 3-way data; ours tightest; Omar worse on A8). In `docs/meetings/2026-10-03.md`.
- [ ] **Writing:** Ch.6–9 skeletons exist; content waits on C.4 data (needs Strategy 2).
- [ ] **C.3/C.4:** modes 0/1/2 on `controller=6`; with/without residual — after Checklist G.
- [ ] Open INDI items (lower priority): full-INDI A1 attitude oscillation; Omar's A8-only 15–25 cm z offset; hardware z-overshoot vs SIL.

---

## Before you fly (every session)

- [ ] **Charged batteries in both drones** (10-03: `cf5` sagged to 3.0 V, `cf_second` ended 3.65 V).
- [ ] Pull latest **both repos on the lab PC** (`flying_robot_course` + `~/georg/ros2_ws/src/crazyswarm2`) — **flight 1 on 10-03 ran a stale yaml**.
- [ ] Build **from the workspace root** `~/georg/ros2_ws` (`colcon build --symlink-install --packages-select crazyflie_interfaces crazyflie_py crazyflie crazyflie_examples`); never inside `src/crazyswarm2`.
- [ ] Launch: `ros2 launch crazyflie launch.py` (`cpp`, default); confirm `CS2_CONNECT_PARAM_PACE_V1` in the output.
- [ ] `ros2 topic echo /cf5/pose --once` shows a real position **before** any flight; `run_formation` prints the per-drone config — check `ctrl_mode`/`ki_z`/`rnn.en`.
- [ ] Both drones booted cleanly (no `Assert failed`, no `rate is off`); power-cycle after any assert.
- [ ] uSD `config.txt` = `usd_thesis_config.txt`; `check_usd_deck.py` on each drone; card not read-only.
- [ ] `cf5` yaml: geometric `ctrl_mode=0` + `ki_z=16`, or full INDI `ctrl_mode=3` + **`ki_z=0`** — never `ki_z=16` with `ctrl_mode=3`.

---

## Don’t bother right now

- [x] `backend:=cflib` (needs `transforms3d`, then `link_statistics` mismatch) — abandoned.
- [x] `tools/read_stored_radio_address.py` — inconclusive; address mismatch unlikely (URI connects).
- [x] More `kv_z` / `ki_z` SIL sweeps; RPM/DShot (`docs/43` closed); A6/C4 at extreme settings; stock Bitcraze INDI / NA-INDI for the thesis compare.

---

## Already done (don’t redo)

- [x] C.1 active scenarios + retrain gate passed (R² 0.944).
- [x] Upload script fixes (readback OK 10-02) — irrelevant for the flash build (no upload).
- [x] 3-way Pure INDI comparison (Omar C 9, Omar Rust 10, ours 6/3) on A8 + A1.
- [x] Full-INDI liftoff tumble: `ki_z=16` × mode 3; fix `ki_z=0`, 5/5 clean (10-02).
- [x] NS2 desk blockers: `cload-rnn-flash`, `backend=cpp`, ghost-neighbour `id==0`; stack patch 8× (`firmware_patches/`).
- [x] Reference research: `aerorobotics/neural-swarm` public (NS1+NS2, `hardware/nn-export`); firmware submodule private.
- [x] uSD cards archived (PC + on-card) and reset 10-03 (card THESIS1 on-card step pending, read-only).
