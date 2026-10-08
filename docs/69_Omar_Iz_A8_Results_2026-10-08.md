# Omar Rust + z integral (`ctrlOot5.kpos_iz`) — A8 hardware results (2026-10-08)

> **Correction 2026-10-09:** recomputed with the robust crossing detector (`a8_crossings.py`); the first version mislocated crossings in 3 of these 6 flights. Steady z error 1.5 → **+1.5 cm** (was +1.0), 2.0 → **+2.1** (was +1.2); dips 1.5 → **−7.6** (was −6.6), 2.0 → **−9.9** (was −8.6). Conclusion unchanged. Full comparison: `docs/71`.

**Setup:** cf5 `controller 10` (Omar Rust, `indi 3`, `rpm_source 1`), firmware `cf21bl_omar_iz_sentinel.bin` (sha256 c24de1f1…953e; `kpos_iz` present, `RPM_FILTER_HOLD_SENTINEL=1`), cf_second geometric (unflashed), A8 `--dz 0.5 --height 0.5 --passes 4`. crazyswarm2 yaml: `kpos_iz` 1.0 (deead9c) → 1.5 (527aed4) → 2.0 (a468dac). **The meta does not record `kpos_iz`; values are assigned from flight order (2 flights each) — confirm.**

| Flights | `kpos_iz` | steady z err (excl. crossings) | crossing dip abs | dip rel. to steady | z sd | gyro_x sd | roll p99 | cmd thrust (mean) | samples with a motor at PWM ceiling |
|---|---|---|---|---|---|---|---|---|---|
| 18-17-33, 18-19-07 | 1.0 | **+6.3 cm** (+6.5/+6.1) | −15.9 | −22.2 | 2.7 | 67 | 21° | 0.34 N | 3.2/4.3 % |
| 18-23-52, 18-25-23 | **1.5** | **+1.5 cm** (+1.8/+1.2) | **−7.6** | **−9.1** | 1.7 | 60 | 16° | 0.40 N | 0.2/0.3 % |
| 18-28-04, 18-29-35 | 2.0 | +2.1 cm (+1.9/+2.4) | −9.9 | −12.1 | 1.7 | 58 | 16° | 0.40 N | 1.1/2.0 % |
Reference: Omar exact (`kpos_iz=0`) on A8, 10-02: mean z error +19…+22 cm, dips 11–26 cm below its own level. Ours (geometric, 10-05): ±2 cm, dip −6 cm.

- **The integral closes the z offset:** +19…+22 → +6.3 (1.0) → +1.5 (1.5) → +2.1 cm (2.0). 1.5 is the knee; 2.0 gives nothing more (and slightly more PWM-ceiling samples, 2 % vs 0.3 %).
- **Unexpected: the crossing dips also shrink** (rel. −22 → −9 at 1.5). The SIL predicted they would NOT be removed (docs/66). Mechanism not established (n = 2 per value, value assignment inferred, no `kpos_iz=0` flight in this session); a plausible reading is that the commanded thrust sits ~19 % below weight at 1.0 (0.34 N) and ~5 % below at ≥ 1.5 (0.40 N), so the dip depended on the same thrust deficit.
- No ringing at 2.0 (z sd 2.6 cm, gyro_x sd 58 vs 67), takeoff overshoot falls 13–15 cm (1.0) → 5–7 cm (≥ 1.5).
- Recommendation for the study: **Omar + Iz, `kpos_iz = 1.5`** (needs A1 confirmation; a same-session `kpos_iz=0` flight would pin the dip comparison).
- Open: A1 with 1.5; solo hover ladder results (not reported yet); `kpos_iz=0` baseline.
- Data: uSD cf5 6/6 + cf_second 6/6 (`usd_raw/*_A8_thesis0N_2026-10-08_18-*`), merged `merged_A8_2026-10-08_18-*.csv` (clock agreement 40–60 ms), script `experiments/analysis/omar_iz_a8_2026_10_08.py`, summary `out/omar_iz_2026-10-08/a8_summary.json`. Rosbags not pushed yet.
