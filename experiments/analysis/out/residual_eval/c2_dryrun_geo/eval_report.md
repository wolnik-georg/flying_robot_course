# Residual model evaluation (P5)

**Weights:** `flying_drone_stack/tools/residual/weights/c2_dryrun_2026-09-19_geo.npz`
**Logs:** 1 merged file(s)

## Combined (all samples)

- n = 4540
- RMSE = 0.3628 m/s²
- R² = 0.2294
- Baseline predict-zero RMSE = 0.4838 m/s² (25.0% reduction)
- Baseline predict-mean RMSE = 0.4133 m/s²

## Validation blocks (contiguous split, seed=0)

- val RMSE = 0.1049 m/s², R² = 0.1585

## Per flight (same weights, no retrain)

| file | n | RMSE | R² | vs zero |
|---|---:|---:|---:|---:|
| `A8_2026-09-19_15-04-41_merged_usd.csv` | 4540 | 0.3628 | 0.229 | 25.0% |

## Binned |error| on validation blocks

