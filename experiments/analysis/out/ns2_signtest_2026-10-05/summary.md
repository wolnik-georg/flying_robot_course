# NS2 sign-test analysis — 2026-10-05

Pass: the crossing dip (cf5 z error, a negative number) is clearly SHALLOWER than the network-off baseline of about −5.9 cm (mean at least MIN_IMPROVEMENT_CM less negative), ranges not overlapping, no over-compensation. Current network-on, res_sign=+1: −10.9 cm (DEEPER = worse).

**Pass/fail (en1 vs en0):** FAIL

## Cohort statistics (crossing dip min e_z, ±1 s)

| Cohort | n | mean [cm] | std | min | max |
|--------|---|-----------|-----|-----|-----|
| en0 | 8 | -5.90 | 0.64 | -7.19 | -5.36 |
| en1 | 16 | -10.96 | 0.82 | -12.20 | -9.90 |

## vs 2026-10-05 reference
- Reference rnn.en=0 mean: **-5.9 cm** (8 crossings)
- Reference rnn.en=1 res_sign=+1 mean: **-10.9 cm** (16 crossings)

- Welch p (this session en0 vs en1): 3.2310564270304506e-12
- Ranges overlap: False

