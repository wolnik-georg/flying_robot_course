# 29 — INDI baseline desk plan (Strategy 0 vs 1)

**Purpose.** Before quoting C.4 numbers, lock **which INDI implementations** count as Strategy 1
and **how** they are flown on the same scenarios as geometric (Strategy 0). Desk-only planning;
execution is lab.

**Living comparison table:** [`24_Downwash_Compensation_Comparison.md`](24_Downwash_Compensation_Comparison.md)

---

## Rows to fill (minimum thesis set)

| ID | Implementation | Firmware | Status (21 Sep 2026) | Next lab action |
|---|---|---|---|---|
| **S0** | Our geometric | `c=6`, `ctrl_mode=0` | Flying on **cf5**; uSD rows in docs/24 § cf5 | C.1 collection + C.4 repeats |
| **S1** | Our full INDI | `c=6`, `ctrl_mode=3` | 2-drone A8 on cf5; ~tie with geo on z (n=2) | Same scenarios as S0, 3–5 flights each |
| **S1b** | Stock Bitcraze INDI | `c=3` | One flight 19 Sep — unstable (n=1, gains not tuned) | Hover gate, then A8 if clean; document gains |
| **S1c** | Briesewitz INDI | `c=7` | Sim clean post `state.acc` fix; **not flown** | Single-drone hover → A8 ladder (`naindi_reference_build_notes.md`) |
| **S4** | NA-INDI hybrid | `c=8` | Sim clean; **not flown** | After supervisor NA-INDI data strategy; same ladder as c=7 |
| **S2** | Geometric + NS2 | `c=6`, `rnn.en=1` | Train path verified; onboard RAM gated | C.1 → C.2 → C.3 |

**External reference (optional row):** Omar / established INDI stack — coordinate with supervisor;
same scenario params as table protocol.

---

## Protocol (must match S0)

- Study drone: **cf5** bottom; **cf_second** stock Lee top (`controller: 5`).
- **uSD on both**; reported metrics from merged uSD @ 500 Hz (`docs/27` policy).
- Scenario params from flight `.meta.json`; prefer **`ctrltarget.*`** for tracking error.
- **3–5 repeat flights** per controller per scenario for C.4 claims (`docs/27` P3).

---

## Desk deliverables (no hardware)

- [ ] Keep **docs/24** updated after each new row (do not mix cf231-era radio-window rows with cf5 uSD rows without labelling).
- [ ] One-page gain/config snapshot per S1 variant actually flown (from log meta / yaml).
- [ ] Note **disturbance pairing**: report `a_res_z` peak/RMS alongside tracking error when comparing runs.

---

## What not to do

- Do not treat **cf231 18 Sep** 103 mm vs 36 mm as the cf5 story — different vehicle and metric window.
- Do not block **C.1** on finishing S1b/S1c; collection stays **geometric on cf5** (`docs/25`).
