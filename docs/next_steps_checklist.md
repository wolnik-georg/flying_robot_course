# Next steps checklist

Simple lab + desk list (as of **2026-10-02**, post–lab + NS2/Part A desk close-out). Update when items move.

---

## Next lab session — do these first (3–5)

1. **Reflash cf5 flash-RNN + apply ghost-neighbour fix** — `make DRONE=bl cload-rnn-flash` (`firmware_app/Makefile`); commit/apply `traj_iface.c` `peer_get_all` **skip `id==0`** (desk fix, may still be uncommitted). Cards: **`usd_thesis_config.txt`** on both decks (not Omar-only config).
2. **Launch CS2 with peers** — `source ~/Desktop/crazyswarm2/install/setup.bash` then `ros2 launch crazyflie launch.py backend:=cpp` (see `docs/ns2_next_lab_protocol.md`). **Not `cflib`** — lab habit used cflib; cpp is prepared, not yet flown for NS2.
3. **NS2 hover gate (mandatory before `rnn.en=1`)** — short hover or A1 segment, **`rnn.en=0`**, merge uSD; need **`|rnn_pred_z| > 1e-3`** somewhere. Fail → stop; do not enable compensation.
4. **Checklist G** — if gate passes: predict-only flight, then **`rnn.en=1`** on A1 or A3, merge again with non-zero preds.
5. **Geometric A1 `ki_z` A/B** (if time) — apply `docs/lab_prep_geometric_kiz0_test.patch`, test whether **`ki_z=16`** explains original ctrl_mode=0 liftoff tumbles (separate from full-INDI tumble, already fixed with **`ki_z=0`** on mode 3).

**Optional same session:** Part A liftoff diagnostic — `docs/lab_usd_liftoff_logging_diagnostic.patch` + `FORMATION_USD_AT_TAKEOFF=1` (captures ramp; not required for NS2 gate).

---

## Next 5 (priority overview)

1. **NS2 end-to-end hardware gate** — three desk blockers addressed (flash build, `backend=cpp`, ghost peer slot); **one flight proves all** (`docs/ns2_next_lab_protocol.md`).
2. **Restore / keep uSD thesis config** before NS2 and C.1.
3. **Geometric A1 + `ki_z=0` A/B** — rule in or out same tumble mechanism as full INDI (`ki_z=16`).
4. **Supervisor: Strategy 1 (“Pure INDI”)** — Oct 2 evidence: Omar C/Rust (9/10) vs ours (6/3); ours tightest; Omar worse on **A8 crossing**, not simple hover; stock Bitcraze / NA-INDI out of scope.
5. **Checklist G** — still blocked until merged uSD shows **non-zero `rnn_pred_*`**.

---

## Before you fly

- [ ] Pull latest **both repos** (`flying_robot_course` + `crazyswarm2`).
- [ ] Weights baked in flash build: **`full_bank_c1_complete.npz`** (2026-10-01) unless `RNN_WEIGHTS_NPZ=…` override.
- [ ] uSD **`config.txt`** = **`usd_thesis_config.txt`**.
- [ ] **cf5 yaml:** full INDI (`ctrl_mode=3`) → **`ki_z=0`** override; plain geometric → **`ki_z=16`** OK (re-flown Oct 2).
- [ ] CS2 **`backend:=cpp`** for NS2 / any flight needing onboard neighbours.

---

## Desk (when not in lab)

- [x] **NS2 desk prep (2026-10-02):** `make cload-rnn-flash`; `backend=cpp` build/launch doc in `docs/ns2_next_lab_protocol.md`; ghost slot fix in `traj_iface.c` (review/commit before flash).
- [x] **Part A:** no existing log covers liftoff ramp; HL takeoff is smooth 7th-order ramp in code; diagnostic patch ready (`docs/lab_usd_liftoff_logging_diagnostic.patch`).
- [ ] **Part B:** Omar-gains liftoff SIL — optional; hardware comparisons partly substitute.
- [x] **Skip:** more **kv_z / ki_z** sweeps (SIL exhausted).
- [x] **Skip:** RPM / DShot (`docs/43` closed).

---

## Lab — must do (blocks Strategy 2 / G)

- [x] Upload script verify — **2026-10-02:** `rnn.ready=1`, `rnn.n=19297` (not proof of onboard eval on default firmware).
- [ ] **Reflash flash-RNN** on cf5 (and peer-fix firmware if not in image yet).
- [ ] Merge uSD — **`rnn_pred_*` not all zero** (failed Oct 2 on default build + cflib).
- [ ] Fly **A1 or A3** with **`rnn.en=1`** after gate passes.
- [ ] If that passes → **Checklist G done**.

---

## Lab — should do (data quality)

- [ ] **Crash-surviving logs** (0-byte uSD on tumble) before more aggressive A1 retries.
- [ ] **Full INDI A1 attitude oscillation** (~7–9° std vs ~2° on A8) — open, separate from tumble.

---

## Lab — after G passes

- [ ] **C.3:** Modes **0 / 1 / 2** on **controller=6**.
- [ ] **C.4:** With vs without residual compare flights.

---

## Ask supervisor (data-backed)

- [ ] **Strategy 1 identity** — thesis **`controller=6` / `ctrl_mode=3`** vs **Omar c=9/10**? Oct 2 3-way flights are the pack.

---

## Don’t bother right now

- [x] Re-fly **A6/C4** at old extreme settings.
- [x] **Stock Bitcraze INDI** / **NA-INDI** for thesis compare — out of scope.
- [ ] **Controller=10** deep tuning unless supervisor picks Omar Rust as Strategy 1.
- [ ] **Full INDI + `ki_z=16`** on cf5 — known liftoff tumble (use **`ki_z=0`**).

---

## Already done (don’t redo)

- [x] C.1 active scenarios + **retrain gate passed**.
- [x] Upload script fix; Oct 2 readback OK.
- [x] **Full INDI liftoff tumble** — **`ki_z=16` × mode 3**; fix **`ki_z=0`**, 5/5 clean (Oct 2).
- [x] **Z-integral under plain geometric** — **`ki_z=16` still OK** (Oct 2, both drones).
- [x] **Pure INDI comparison** — Omar C (9), Omar Rust (10), ours (6/3) on **A8 + A1**.
- [x] **RPM dual-source / DShot spikes** — closed (`docs/43`).
- [x] **kv_z=7** — SIL says leave it.
- [x] Z-overshoot **does not** explain **geometric** crashes (Oct 1).

---

## Still open

- [ ] **Geometric A1/A6 tumbles** (~0.7 s) — **suspect `ki_z=16`**; direct A/B not flown yet.
- [ ] **NS2 preds on hardware** — desk path complete; **needs gate flight** (flash + cpp + peer fix).
- [ ] **Omar A8-only ~15–25 cm z offset** (C and Rust) — why scenario-specific.
- [ ] **Full INDI A1 oscillation** vs A8 — same config.
- [ ] **Hardware Z-overshoot vs SIL** — no log at liftoff; code says smooth HL ramp; optional diagnostic flight.
- [ ] **Checklist G** — until non-zero preds on flash-RNN + cpp.
