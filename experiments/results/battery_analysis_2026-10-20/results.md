# Battery vs tracking error (pilot flights 2026-10-19/20)

Battery = mean loaded voltage of the bottom drone while airborne. Error: A8 mean crossing dip per flight [cm, negative = low], A1 mean z error [cm], A4 z RMS [cm] (A4: the offset effect removed within each strategy by subtracting its offset-cell mean, strategy mean kept). Slope = cm per 0.1 V (positive slope for dip/z error = higher battery gives a smaller negative error; for RMS positive = worse). Bootstrap 95 % interval over flights.

## A8
| strategy | n | battery range [V] | r | Spearman | slope [cm / 0.1 V] | 95 % interval |
|---|---|---|---|---|---|---|
| S0 | 4 | 3.60–3.82 | -0.61 | -0.20 | -0.24 | [-0.53, +17.55] |
| S1 | 4 | 3.34–3.65 | +0.94 | +0.40 | +1.89 | [-9.25, +2.95] |
| S2 | 4 | 3.54–3.61 | -0.51 | +0.00 | -1.02 | [-2.86, +60.25] |
| S3 | 5 | 3.43–3.72 | +0.16 | +0.10 | +0.12 | [-0.51, +1.60] |
| **INDI on (S1+S3), strategy means removed** | 9 | | +0.69 | | +1.06 | [-0.18, +1.98] |
| **INDI off (S0+S2), strategy means removed** | 8 | | -0.42 | | -0.34 | [-1.45, +0.26] |

Mean battery per strategy (is the comparison confounded?): S0 3.74 V, S1 3.54 V, S2 3.57 V, S3 3.55 V

Strategy means raw -> adjusted to the common battery 3.60 V (common slope per INDI group): S0 -12.5 -> -12.1, S1 -8.4 -> -7.9, S2 -5.6 -> -5.7, S3 -3.8 -> -3.3

## A1
| strategy | n | battery range [V] | r | Spearman | slope [cm / 0.1 V] | 95 % interval |
|---|---|---|---|---|---|---|
| S0 | 6 | 3.28–3.69 | -0.10 | +0.14 | -0.14 | [-1.35, +3.66] |
| S1 | 5 | 3.33–3.71 | +0.87 | +1.00 | +1.18 | [+0.38, +2.21] |
| S2 | 7 | 3.34–3.77 | +0.14 | +0.18 | +0.13 | [-1.21, +0.75] |
| S3 | 5 | 3.34–3.51 | +0.89 | +0.70 | +2.40 | [-0.12, +3.56] |
| **INDI on (S1+S3), strategy means removed** | 10 | | +0.83 | | +1.42 | [+0.57, +2.38] |
| **INDI off (S0+S2), strategy means removed** | 13 | | +0.02 | | +0.03 | [-0.61, +0.68] |

Mean battery per strategy (is the comparison confounded?): S0 3.55 V, S1 3.51 V, S2 3.59 V, S3 3.44 V

Strategy means raw -> adjusted to the common battery 3.53 V (common slope per INDI group): S0 -17.7 -> -17.7, S1 -1.8 -> -1.6, S2 -8.9 -> -8.9, S3 +2.7 -> +4.0

## A4
| strategy | n | battery range [V] | r | Spearman | slope [cm / 0.1 V] | 95 % interval |
|---|---|---|---|---|---|---|
| S0 | 9 | 3.31–3.71 | +0.01 | -0.13 | +0.01 | [-0.54, +0.43] |
| S1 | 9 | 3.58–3.72 | +0.02 | +0.03 | +0.01 | [-0.40, +0.54] |
| S2 | 9 | 3.45–3.75 | +0.08 | +0.28 | +0.10 | [-1.01, +0.64] |
| S3 | 9 | 3.04–3.63 | -0.68 | -0.40 | -0.29 | [-0.44, +0.30] |
| **INDI on (S1+S3), strategy means removed** | 18 | | -0.57 | | -0.26 | [-0.33, +0.13] |
| **INDI off (S0+S2), strategy means removed** | 18 | | +0.04 | | +0.04 | [-0.39, +0.36] |

Mean battery per strategy (is the comparison confounded?): S0 3.51 V, S1 3.65 V, S2 3.61 V, S3 3.43 V

Strategy means raw -> adjusted to the common battery 3.55 V (common slope per INDI group): S0 +11.3 -> +11.3, S1 +3.4 -> +3.6, S2 +6.3 -> +6.3, S3 +3.2 -> +2.9

