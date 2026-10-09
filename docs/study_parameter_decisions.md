# Study parameter decisions (frozen 2026-10-08) — read before changing any of these

> **DECISION 2026-10-09 (user): the only Omar INDI in the study is "Omar C + z integral 1.5" (controller 9, `ctrlOmarIndi.Kpos_Iz 1.5`) — the new default Omar INDI.** Omar Rust (controller 10) is dropped (was slightly worse, more oscillation, same code as C) and the variants without the integral ("exact") are no longer considered (offset unfixed). Existing Rust/exact data stays in the repo as history only. Candidates for "Pure INDI" in the study: ours vs Omar C + Iz 1.5.


## 1. `indi_gains.res_sign` (sign of the residual-force term)
| `ctrl_mode` | controller | value | why |
|---|---|---|---|
| **0** (geometric + NS2 network, Strategy 2) | 6 | **−1** | Hardware-verified 2026-10-08 (`docs/68`): A8 crossing dip −3.4 cm (−1) vs −5.9 (network off) vs −11.0 (+1, 10-05); A1 stable. Derivation (lib.rs ~L1899): desired acceleration must carry **−a_res** so the controller compensates the unmodelled force; +1 (the frozen INDI-branch default) **reinforced** the downwash. |
| **3** (our full INDI) | 6 | **+1** (default) | `res_sign=−1` diverged in full INDI on 2026-09-09 (lagged a_res in the attitude path). Never set −1 with `ctrl_mode=3`. |
| Omar C / Omar Rust | 9 / 10 | irrelevant | own code; ignore `res_sign`. |
The yaml keeps `res_sign: -1` for cf5 because cf5's default is `ctrl_mode: 0` — if cf5 is ever run with `ctrl_mode: 3` (our INDI), set `res_sign: 1` in the same edit.

## 2. Omar z integral (opt-in, firmware default 0 = Omar exact)
- **Study value for BOTH: 1.5** — Rust `ctrlOot5.kpos_iz: 1.5` (controller 10), C `ctrlOmarIndi.Kpos_Iz: 1.5` (controller 9).
- Evidence (A8, steady z error): Rust 0 → +21.9 (10-02, older: +19); 1.0 +6.3; **1.5 +1.5**; 2.0 +2.1 cm. C 0 → +23.2 (older: +22); 1.0 +2.2; **1.5 +2.2**; 2.0 +1.5 cm (corrected 2026-10-09 with the robust crossing detector, `docs/71`) (`docs/69`, `docs/70`). 1.5 is inside the flat region for both; 2.0 gains nothing (Rust: more PWM-ceiling samples).
- The two ports' integral gains are not 1:1 comparable (C closed at 1.0, Rust needs 1.5) — reason open on the desk.
- Needs firmware `cf21bl_omar_iz_sentinel.bin` (sha256 c24de1f1…953e, flash only cf5). The 10-03 `cf21bl_default.bin` / `cf21bl_rnn_100hz.bin` do NOT contain the parameter (yaml push of it would fail at connect). Yaml snippets to re-enable:
```yaml
# Omar Rust:  stabilizer.controller: 10 ; ctrlOot5: {indi: 3, kpos_iz: 1.5} ; indi_gains.rpm_source: 1
# Omar C:     stabilizer.controller: 9  ; ctrlOmarIndi: {indi: 3, Kpos_Iz: 1.5} ; needs deck.bcRpm == 1
```

**In the comparative study Omar always runs WITH the integral (1.5 for both ports); the exact variants (`kpos_iz`/`Kpos_Iz` 0) are only a reference to show the effect of the integral (same-day exact baselines 2026-10-09: Rust +13.3 cm, see `docs/lab_sessions/2026-10-09.md`).** The size of the exact-variant offset varies with day/battery (+13…+22 cm); with the integral it was +1.5…+2.2 cm on 10-08.

## 3. Firmware artifacts
| file | content | use |
|---|---|---|
| `cf21bl_rnn_100hz.bin` (10-03) | NS2 100 Hz network, no kpos_iz | NS2 flights (flash to cf5) |
| `cf21bl_omar_iz_sentinel.bin` (10-08) | no network, `kpos_iz`, `RPM_FILTER_HOLD_SENTINEL=1` | Omar variants |
(`.bin` files are gitignored; they exist on the laptop only. A unified build with network + kpos_iz is not built yet.)

## 4. Standing rules (unchanged)
Fresh batteries (rest ≥ 4.1 V), pose bag, `check_usd_deck.py`, cf_second card first, never touch `res_sign` and `rnn.en` mid-session without writing it into the lab session doc.
