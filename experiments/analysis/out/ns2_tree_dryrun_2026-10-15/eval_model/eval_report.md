# Residual model evaluation (P5)

**Weights:** `/home/georg/Desktop/flying_robot_course/experiments/analysis/out/ns2_tree_dryrun_2026-10-15/full_bank_c1_complete.npz`
**Logs:** 13 merged file(s)

## Combined (all samples)

- n = 131410
- RMSE = 0.1957 m/s²
- R² = 0.9601
- Baseline predict-zero RMSE = 1.1259 m/s² (82.6% reduction)
- Baseline predict-mean RMSE = 0.9794 m/s²

## Validation blocks (contiguous split, seed=0)

- val RMSE = 0.1996 m/s², R² = 0.9609

## Per flight (same weights, no retrain)

| file | n | RMSE | R² | vs zero |
|---|---:|---:|---:|---:|
| `A1_2026-10-15_17-37-45_merged_usd.csv` | 7053 | 0.2807 | 0.896 | 81.4% |
| `A1_2026-10-15_17-48-27_merged_usd.csv` | 6955 | 0.2740 | 0.172 | 90.6% |
| `A1_2026-10-15_17-57-57_merged_usd.csv` | 7053 | 0.3076 | 0.155 | 89.3% |
| `A1_2026-10-15_18-18-32_merged_usd.csv` | 7053 | 0.2485 | 0.915 | 81.9% |
| `A8_2026-10-15_17-31-28_merged_usd.csv` | 11501 | 0.1344 | 0.887 | 68.6% |
| `A8_2026-10-15_17-35-53_merged_usd.csv` | 11501 | 0.1270 | 0.884 | 68.8% |
| `A8_2026-10-15_17-43-15_merged_usd.csv` | 11454 | 0.1728 | 0.828 | 61.1% |
| `A8_2026-10-15_17-44-43_merged_usd.csv` | 11466 | 0.1704 | 0.840 | 62.2% |
| `A8_2026-10-15_17-46-44_merged_usd.csv` | 11454 | 0.1945 | 0.790 | 56.7% |
| `A8_2026-10-15_17-54-37_merged_usd.csv` | 11462 | 0.1847 | 0.822 | 61.0% |
| `A8_2026-10-15_17-56-18_merged_usd.csv` | 11454 | 0.2121 | 0.797 | 57.9% |
| `A8_2026-10-15_18-14-47_merged_usd.csv` | 11502 | 0.1301 | 0.888 | 69.2% |
| `A8_2026-10-15_18-16-57_merged_usd.csv` | 11502 | 0.1481 | 0.864 | 65.5% |

## Binned |error| on validation blocks

- dy 0.00-0.05 m: n=4146, RMSE=0.3088
- dy 0.05-0.10 m: n=1086, RMSE=0.3893
- dy 0.10-0.15 m: n=679, RMSE=0.3688
- dy 0.15-0.20 m: n=360, RMSE=0.3596
- dy 0.20-0.25 m: n=13, RMSE=0.1028
- dz 0.23-0.47 m: n=502, RMSE=0.2978
- dz 0.47-0.71 m: n=4872, RMSE=0.3325
- dz 0.71-0.96 m: n=910, RMSE=0.3571
