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
- [ ] **NS2 closed-loop SIL — MUST WORK:** reproduce the hardware crossing dips (off −5.9 cm; on, `res_sign=+1` −10.9 cm; one calibrated scalar from the off cohort only), then predict `res_sign=-1`. Prompt `docs/cursor_prompt_ns2_closed_loop_sil_2026-10-06.md`; writes `docs/62_NS2_Closed_Loop_SIL.md`.
- [ ] INDI docs 53–61 / `docs/56` — desk final unless lab contradicts; SIL: fix shared-gain top-drone artefact before trusting 2-drone results.
- [ ] Supervisor: Pure INDI variant + scenario set; FBL email; writing (INDI/RPM now, NS2 after lab).

**Lab, after the NS2 test — INDI oscillation (investigation closed for now; details `docs/56`)**
- [ ] Bench: measure command → thrust latency (and gyro-to-controller latency); the one number that pins down the delay budget.
- [ ] Apply the logging patch on the lab PC (`docs/lab_prep_log_filter_params.patch`) so flights record `res_sign`, `filt_dt_us`, `notch_en`, `filt_prewarp`, rpm source.
- [ ] A1 flights (yaml only, no reflash, abort at |roll/pitch| > 25° for 0.5 s or z < 0.25 m): baseline → **package first** (`res_sign=-1`, `kr=483`, `kw=76`, KP 40/30, KV 8/10, `ki_z=0`) → sign only (`res_sign=-1` at kr 2400 diverged on 2026-09-09). Exact blocks in `docs/58` and `docs/61`.

**Decisions (you + supervisor)**
- [ ] Which INDI variant is the thesis "Pure INDI": Ours / Omar C / Omar Rust.
- [ ] Which scenarios go into the comparative study (temporary downwash favours ours, constant downwash favours Omar's) — decide together with the INDI variant.
- [ ] Is 100 Hz enough for the network, or try higher?

**Desk (parallel OK)**
- [ ] INDI investigation docs 53–61 — reopen only if bench latency or A1 flights contradict.
- [ ] **INDI input replay across ours / Omar C / Omar Rust — MUST WORK (not parked):** on-policy check fails today (A8 −10%, A1 +50% vs the flown command), so something concrete is wrong. Prompt `docs/cursor_prompt_indi_replay_root_cause_2026-10-06.md` (ladder: input path → geometric on-policy → full-INDI on-policy → only then ours-vs-Omar). Only Omar C ≡ Rust is validated so far (`docs/64`).
- [ ] After INDI decision: Omar C/Rust z offset; optional second look at our oscillation.
- [ ] D8 contingencies only if bench **B** fails; retrain only if data say so.

**Email / writing (parallel OK)**
- [ ] FBL-controller follow-up (~2 weeks).
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
