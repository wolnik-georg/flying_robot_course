# Next steps checklist

Simple lab + desk list (as of **2026-10-06**, meeting [`meetings/2026-10-05.md`](meetings/2026-10-05.md), after the NS2 first hardware attempt and the 2026-10-05 session —
[`lab_sessions/2026-10-03.md`](lab_sessions/2026-10-03.md)). **Lab session plan + copy-paste commands:**
[`lab_session_pack_2026-10-06.md`](lab_session_pack_2026-10-06.md). Update when items move.

**One-paragraph state (2026-10-05, lab session closed — `lab_sessions/2026-10-05.md`):** the 100 Hz network works and predicts the residual well
(corr 0.85–0.92, A1 stack −1.61 vs measured −1.60 m/s²). The earlier crashes were **tracker pose swaps** (identical marker patterns; trigger unknown)
and **`cf_second` battery collapse** (2.5 V → fell on `cf5` in A1) — not the network; with fresh batteries 3 of 3 A8 flights were clean. **Open:** the
network term is added with `res_sign=+1` (reinforcing): crossing dips −10.9 cm with the network vs −5.9 cm without. Next: `res_sign=-1`. `cpp` is the only backend.

---

**Decisions 2026-10-06:** tracker pose swaps are treated as battery-related for now (if a swap recurs: check batteries first, keep the pose bag running); the `FSCK*.REC` files are dropped; no HTML twin for the lab docs.

## LAB SEQUENCE (preserved 2026-10-08) — one place to read what happens in the lab
1. **NS2 sign test (pack `lab_session_pack_signtest.md` §2):** fresh batteries (rest ≥ 4.1 V), pull crazyswarm2 + check printed cf5 config, pose bag; A8 `res_sign=-1` ×3 (abort ready on the first; SIL expects ≈ −2.3 cm vs −5.9 cm off, pass = ≥ 1 cm shallower), A8 `rnn.en=0` ×2 (Claude pushes it on request), A1 `rnn.en=0` ×1; A1 network-on `-1` only if A8 passes and the A1 baseline is clean (SIL unstable there). Then Claude pushes `res_sign: 1`, you run `post_flight_check.py` + `ns2_signtest_analysis.py`.
2. **INDI "Omar + Iz" (pack §4b, only after 1):** Omar Rust (controller 10) hover `kpos_iz` 0 → 1.0 → 1.5, A8 ×2, A1 ×2; Omar C only if C is chosen.
3. **Later lab items:** bench command→thrust latency, logging patch (`f_d`, applied gains, rpm source — also settles the 1.14 thrust factor and the A8 ours-vs-Omar offset), A1 yaml flights for our oscillation.
4. **RPM filter on the next INDI flight:** active for ours (ctrl_mode 3) and Omar Rust via `rpm_get_all()` (abs cap 28 000 RPM, 10 000 RPM jump, hold-last-good; 0xFFFF sentinel → 0); NOT active for Omar C (reads optical deck `rpm.m1..4` directly).

## WHAT'S NEXT — by area (2026-10-05, from the meeting doc [`meetings/2026-10-05.md`](meetings/2026-10-05.md))

Flight-day steps: [`next_flight_card.html`](next_flight_card.html) · commands: [`lab_session_pack_2026-10-06.md`](lab_session_pack_2026-10-06.md).

**NS2 "ready for the comparative study" gate (set 2026-10-05):** (1) A8 with `res_sign=-1`: crossing dip clearly shallower than (a smaller dip, not a deeper one) the network-off baseline (−5.9 cm), clean flights; (2) A1 with the network on and `-1` stable, after an A1 `rnn.en=0` baseline shows whether cf5's ±40° oscillation is the geometric controller or NS2; (3) no over-compensation (dip not overshooting upward; else a gain factor on the network term); (4) scenario set decided with the supervisor (checked so far: A8, A1 only); (5) operating rules for every comparison flight: fresh batteries (rest ≥ 4.1 V), pose bag, `vbat` min checked. Network is z-only by design (as in the reference); 100 Hz vs faster not compared (meeting question).

**Lab (next session) — NS2 sign test** (commands: `next_flight_card.html`)
- [x] Bench A–C, A8 `rnn.en=0` (2 clean), A8 `rnn.en=1` (crashes explained: tracker swap / battery), 3 clean A8 with fresh batteries — **done 2026-10-05**
- [ ] Fresh batteries (rest ≥ 4.1 V), pose bag recording, `cf_second` card first.
- [ ] **A8, network on, `indi_gains.res_sign=-1` (cf5 yaml), 3 flights** — pass: dip clearly shallower than (a smaller dip, not a deeper one) −5.9 cm.
- [ ] **A8 `rnn.en=0`, 2 more flights** (baseline stats).
- [ ] **A1 `rnn.en=0` baseline** (cf5 ±40° oscillation seen with the network on).
- [ ] Only if swaps recur with healthy batteries: firmware jump gate (reject >30 cm single-sample jump, count rejections).

**After lab or in parallel (do not block NS2 gate)**
- [x] **NS2 closed-loop SIL — CLOSED 2026-10-08 (`docs/62`), VALIDATED for A8 crossing dips:** mirror-symmetric bank force + one torque scalar (c = 0.0032 m², fitted on network-off only): off −5.84 cm (hw −5.9), `res_sign=+1` test −9.89 ± 0.05 cm (hw −10.9 ± 1.5, range −12.2…−9.9), all crossings equal, roll ±22° (hw 21–28°). Prediction `res_sign=−1`: ≈ −2.3 cm (−1.6…−3.8 over ±20 % c), stable 5/5 — ~3.5 cm shallower than off. Not validated: A1 with the network on (SIL tilt 38–48°, hw also crashed there) — keep A1 network-off last. Tilt p99 13° vs 22.9° known mismatch.
- [ ] INDI docs 53–61 / `docs/56` — desk final unless lab contradicts; SIL: fix shared-gain top-drone artefact before trusting 2-drone results.
- [ ] Supervisor: Pure INDI variant + scenario set; FBL email; writing (INDI/RPM now, NS2 after lab).

**Lab, after the NS2 test — INDI oscillation (investigation closed for now; details `docs/56`)**
- [ ] Bench: measure command → thrust latency (and gyro-to-controller latency); the one number that pins down the delay budget.
- [ ] Apply the logging patch on the lab PC (`docs/lab_prep_log_filter_params.patch`) so flights record `res_sign`, `filt_dt_us`, `notch_en`, `filt_prewarp`, rpm source.
- [ ] A1 flights (yaml only, no reflash, abort at |roll/pitch| > 25° for 0.5 s or z < 0.25 m): baseline → **package first** (`res_sign=-1`, `kr=483`, `kw=76`, KP 40/30, KV 8/10, `ki_z=0`) → sign only (`res_sign=-1` at kr 2400 diverged on 2026-09-09). Exact blocks in `docs/58` and `docs/61`.

**Decisions (you + supervisor)**
- [ ] Which INDI variant is the thesis "Pure INDI": Ours / Omar C / Omar Rust — **depends on whether the constant z offset of Omar C/Rust can be fixed** (decided 2026-10-08); Omar's controllers stay as close to Omar's version as possible.
- [x] **Scenarios (decided 2026-10-08): keep ALL prepared scenarios for 2-drone flights, no reductions; A1 stays.**
- [x] **100 Hz for the network is enough for now (decided 2026-10-08).**

**Desk (parallel OK)**
- [ ] INDI investigation docs 53–61 — reopen only if bench latency or A1 flights contradict.
- [x] **INDI input replay — CLOSED 2026-10-07 (docs/64, final status banner):** replay valid where the comparison is valid. The earlier A1 "failure" was a reference error (96 % of A1 samples have a saturated motor, gyro_x std 253 deg/s); on unsaturated A1 samples the replay matches (corr 0.93, −4 %), A8 corr 0.79–0.82 / −10 % (within thrust-mapping uncertainty, regression R² 0.97). Omar C ≡ Omar Rust (~1e-7 / 1e-9). Ours-vs-Omar = same-input output difference only (A8: 0.08 N RMS, 0.05 N after matching mass; A1 diverges, no attribution). Remaining unknown (A8 level offset origin) needs in-flight logging of f_d/applied gains (lab item). Variant decision rests on `docs/56`.
- [ ] **RPM filter in the real control loop — validate (added 2026-10-08).** State: a live filter ALREADY exists (`rpm_get_all()` in `traj_iface.c:667`, since 2026-09-29): DShot > 28 000 RPM rejected, > 10 000 RPM jump per tick rejected, hold-last-good; real flight RPM max is 22–24 k (A8 clean flights 21.3–22.0 k, A1 24.0 k), so a 28 k or 30 k cap makes no practical difference — keep it simple, keep 28 k. In the 10-05 clean A8 logs only 26–42 samples per flight (0.05–0.08 %) are invalid and all are the 0xFFFF sentinel (no 28–59 k bursts in these flights). **Coverage (checked 2026-10-08): the filter acts for our INDI and Omar Rust (both call `rpm_get_all()`); Omar C reads the optical deck directly, unfiltered, and needs `deck.bcRpm`.** Gaps to close: (a) the sentinel 0xFFFF is mapped to **0**, not hold-last-good — check what one motor = 0 for a tick does in INDI (the `ENABLE_RPM_GUARD` is off) and make it hold-last-good if it hurts; (b) confirm from build/flash dates that the 10-02/10-05 firmware contained the filter; (c) geometric `ctrl_mode=0` (NS2 flights) never uses RPM for control, so those flights neither test nor need it; (d) host unit test for sentinel/spike/slew; (e) one short INDI hover on the lab PC after the NS2 gate to confirm nothing changed in flight. The post-process 2000-RPM rule (needs both sources) cannot run in the loop.
- [x] **Omar C/Rust z offset — DESK PART DONE 2026-10-08 (`docs/65`, `docs/66`): cause = commanded-vs-delivered thrust DC mismatch (~1.14), opt-in `kpos_iz`/`Kpos_Iz` implemented in Rust+C (default off bit-identical, ARM build OK), SIL recommends 1.5 (1.0 cautious); lab test needed (INDI hover → A8 → A1) and note: crossing dips of Omar (−11…−26 cm below own level on hardware) are NOT removed by the integral.** Original plan text: (plan (decided 2026-10-08: keep Omar's code as close to the original as possible; a z integral is acceptable only if nothing else breaks):** (1) opt-in z-only integral parameter in Rust (controller 10), default 0 = bit-identical to Omar (existing tests 7/7 and C≡Rust must stay unchanged); anti-windup (`KPOS_I_LIMIT`), reset on takeoff/landing, like our `ki_z`; (2) same parameter in Omar C (controller 9, crazyflie-firmware, patch preserved) and re-check C≡Rust with the integral on; (3) SIL: solo hover, A8, A1, figure-8, small `ki_z` grid, watch for ringing (docs/41 §20 saw ringing at 3–4× in the IMU-bias proxy); (4) replay at `ki_z=0` unchanged; (5) lab: hover → A8 → A1. In the thesis the variant is labelled "Omar + Iz" next to "Omar exact". **Full plan: `docs/65_Omar_Z_Offset_Plan.md`; Cursor prompt: `docs/cursor_prompt_omar_z_integral_2026-10-08.md`.** Original sweep note: Omar Rust has `KPOS_I = 0` (no position integral), `kp = 7`, `MASS = 0.0427` (ours 0.041). Candidates: missing integral, low gain amplifying a thrust-constant error, mass/`MOTORRPM2FORCE`. Sweep in SIL/replay (integral on, mass 0.0427 → 0.041, force constant) and see which removes the +19…+22 cm on A8; whether we may change Omar's code for the comparison is a supervisor question.
- [ ] Our A1 oscillation: new fact (docs/64 addendum 2) — in the 10-02 A1 flight 96 % of samples had a motor at PWM saturation and gyro_x std 253 deg/s; use it in the oscillation analysis (limit cycle at saturation).
- [ ] Commit + push SIL scripts, `docs/62`, `docs/64`, this checklist (waiting for your OK).
- [ ] Your meeting doc: update the "Neural Swarm2" section (SIL result, replay result, lab plan).
- [ ] D8 contingencies only if bench **B** fails; retrain only if data say so.

**Email / writing (parallel OK)**
- [x] FBL-controller follow-up — done by the professor (2026-10-08), waiting for the response.
- [ ] Ch.6–9: INDI, Z, RPM now; NS2 + comparison after lab / decisions.

---

## TOPIC 1 — NS2 / Strategy 2 (the blocker)

### Desk (Cursor NS2 block — see [`lab_sessions/2026-10-03.md`](lab_sessions/2026-10-03.md) Appendices A–D)
- [x] **D1 — 100 Hz network evaluation** (`rnn.div`, hold).
- [x] **D2 — timing log** (`rnn.us_*`, `read_rnn_timing.py`).
- [x] **D3 — static scratch buffers**.
- [x] **D4 — host regression** (`test_residual_nn.py` **31/31** incl. peer-resync; div sweep via `run_ns2_div_sim.py`).
- [x] **D4b — G5 live 2-drone SIL div sweep** (`experiments/sim_validation/run_ns2_div_sim.py` → `ns2_div_sil_results.json`).
- [x] **D9 — NS2 reference matrix** (`docs/52_NS2_Reference_Comparison.md`; clone @ `48b1851…` outside repo).
- [x] **D5 — builds** (`build_artifacts/cf21bl_default.bin`, `cf21bl_rnn_100hz.bin`).
- [x] **D6 — `docs/13`** controller call rate 1 kHz + 100 Hz hold documented.
- [x] **D7 — identity scan** (`check_estimator_identity.py`, `lock_on_scan.py`).
- [ ] **D8 — contingencies sketched** (only if 100 Hz still doesn't fit): smaller retrained network (C.2 retrain), or off-board evaluation.
1. [x] **Charge/swap batteries; fix card THESIS1** — done 2026-10-05 (cards fixed; fresh batteries are now a per-session rule, see the flight card).
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

- [ ] **Supervisor:** which "Pure INDI" is Strategy 1 — `controller=6/ctrl_mode=3` vs Omar C/Rust (10-02 3-way data: ours best on A8, worst on A1; Omar C/Rust ~20 cm high on A8). In [`meetings/2026-10-05.md`](meetings/2026-10-05.md).
- [ ] **Supervisor:** which scenarios go into the comparative study (influences the INDI choice).
- [ ] **FBL controller:** follow up the email (~2 weeks old; Strategy 3 blocked without it).
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
