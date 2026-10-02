# Next steps checklist

Simple lab + desk list (as of 2026-10-02). Update this file when items move.

---

## Before you fly

- [ ] Pull latest **both repos** (`flying_robot_course` + `crazyswarm2`, upload script fix).
- [ ] Weights file ready (26-flight retrain **`full_bank_c1_complete.npz`** or path from manifest).

---

## Desk (do these first if not done)

- [ ] **Part A:** Check clean A1 uSD — does **commanded height/setpoint** jump during takeoff? (uncropped log)
- [ ] **Part B:** Omar gains in liftoff SIL — read result, decide if anything changes
- [ ] **Skip:** more **kv_z / ki_z** sweeps (done)
- [ ] **Skip:** RPM deck vs DShot spike study on old C.1 set (done — `docs/43`)

---

## Lab — must do (blocks thesis compare)

- [ ] Upload weights with **fixed script**
- [ ] Script says **`rnn.ready=1`** AND **`rnn.n ≈ 19297`**
- [ ] Fly **A1 or A3** (not same-day training scenario) with **`rnn.en=1`**
- [ ] Merge uSD — **`rnn_pred_x/y/z` not all zero**
- [ ] If that passes → **Checklist G done** for Strategy 2

---

## Lab — should do (data quality)

- [ ] Plan **logs that survive crashes** (0-byte uSD today) before chasing tumble with more A1 tries

---

## Lab — after G passes

- [ ] **C.3:** Fly all three compared modes on **controller=6** — **0 / 1 / 2** — clean on frozen library
- [ ] **C.4:** Start real **with vs without** residual compare flights

---

## Ask supervisor (not blocking today’s upload)

- [ ] **Pure INDI** for thesis = **`ctrl_mode=3` on c=6** or **Omar’s separate controller?**

---

## Don’t bother right now

- [ ] Re-fly **A6/C4** at old extreme settings (dropped)
- [ ] More **A1 tumble** flights until upload + crash logging are fixed
- [ ] **Controller=10** Omar tuning (parked)

---

## Already done (don’t redo)

- [x] C.1 data for active scenarios + **retrain gate passed**
- [x] Upload script bug fixed (NaN + fake success)
- [x] Z-overshoot **does not** explain crashes
- [x] Liftoff tumble **correlate search** (no root cause; mocap occlusion not in logs)
- [x] **kv_z=7, ki_z=16** — SIL says leave them
- [x] **RPM dual-source / DShot spikes** — desk closed on C.1 logs

---

## Still open (OK to live with for now)

- [ ] **Why** liftoff tumble sometimes (~0.7 s after leave ground)
- [ ] **Why** SIL overshoot tiny vs ~15 cm on hardware (Part A/B may help)
- [ ] First **G attempt** — preds were zero (invalid); **must repeat**
