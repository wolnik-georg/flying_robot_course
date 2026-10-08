# Omar C (controller 9) + `Kpos_Iz` — A8 hardware ladder and comparison with Omar Rust (2026-10-08)

> **Correction 2026-10-09:** recomputed with the robust crossing detector (`a8_crossings.py`). Flight 18-40-18 does NOT have two positive dips — that was a mislocated crossing. Corrected: 1.0 → steady **+2.2 cm**, dip **−9.1** (was +1.6 / −5.8). C is flat at +1.5…+2.2 cm from 1.0 to 2.0; dips −6…−10 cm. Full comparison: `docs/71`.

**Setup:** cf5 `controller 9`, `ctrlOmarIndi.indi 3`, same flashed binary as docs/69 (`cf21bl_omar_iz_sentinel.bin`), deck RPM (`deck.bcRpm 1`), cf_second geometric. crazyswarm2 yaml 9df9e47 (1.0) → 2b34825 (1.5) → b87c4e3 (2.0). Flights in order, 2 per value: 1.0 = 18-40-18, 18-42-05; 1.5 = 18-45-11, 18-47-48; 2.0 = 18-49-37, 18-51-15 (value from flight order; meta lacks it). A8 `--dz 0.5 --height 0.5 --passes 4`.

| `Kpos_Iz` | steady z err | dip abs | dip rel. | z sd | gyro_x sd | roll p99 | PWM-ceiling samples | cf5 vbat min |
|---|---|---|---|---|---|---|---|---|
| 1.0 | +2.2 cm (+2.1/+2.4) | −9.1 | −11.3 | 1.4 | 52 | 14° | 3.1 / 4.1 % | 3.31 / 3.27 |
| 1.5 | +2.2 (+2.4/+2.0) | −9.9 | −12.1 | 1.6 | 58 | 14° | 8.3 / 0.1 % | 3.21 / 3.71 |
| 2.0 | +1.5 (+1.1/+1.8) | −6.1 | −7.5 | 1.3 | 60 | 17° | 0.3 / 0.4 % | 3.57 / 3.54 |
Reference: Omar C exact on A8 (10-02): +22 cm, dips 11–26 cm below its level.

## Findings
- **C is already closed at 1.0** (+2.2 cm) and flat from 1.0 to 2.0; Rust needed 1.5 (+6.3 at 1.0). The gain is therefore NOT 1:1 comparable between the ports — to be understood on the desk (clamp / error scaling / mass: Rust `MASS` 0.0427).
- **Dips ≈ −6…−10 cm (abs) at all values**, no monotone trend (per-crossing scatter ±3 cm, n = 8 per value; (the two positive "dips" of 18-40-18 in the first version were a mislocated crossing, corrected)). Confounded by battery: the 1.5 flight at 3.21 V has the deepest dips and 8 % ceiling samples; after the pack swap (3.71 V) 0.1 %. PWM-ceiling samples track battery state, not `Kpos_Iz`.
- **Deck RPM dropouts did not occur** (all four deck motors non-zero in every airborne sample of all 6 flights); the unfiltered deck path was fine here.
- No ringing at 2.0 (z sd 1.3 cm, lowest of the three).

## Rust vs C with the integral (A8, best values)
| | steady z err | dip abs | z sd |
|---|---|---|---|
| Omar Rust, `kpos_iz` 1.5 | +1.5 cm | −7.6 | 1.7 |
| Omar C, `Kpos_Iz` 1.0 | +2.2 | −9.1 | 1.4 |
| Omar C, `Kpos_Iz` 2.0 | +1.5 | −6.1 | 1.3 |
→ essentially equal on A8 (consistent with C ≡ Rust in the replay). No basis yet to call either "best"; the discriminator is A1 (10-02: C +2…+4 cm vs Rust −11 cm without the integral) — A1 with the integral for BOTH is still open.

Data: uSD cf5 6/6 + cf_second 6/6 (`usd_raw/*_A8_thesis0N_2026-10-08_18-4*/18-5*`), merged `merged_A8_2026-10-08_18-{40-18,42-05,45-11,47-48,49-37,51-15}.csv` (clock agreement 40–60 ms), `experiments/analysis/omar_c_iz_a8_2026_10_08.py`, `out/omar_c_iz_2026-10-08/a8_summary.json`.
