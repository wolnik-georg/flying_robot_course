# NS2 retrain chain on the tree — dry run on the 2026-10-15 validation flights (desk, 2026-10-15)

Purpose: prove merge → label adapter → dataset → train → leave-one-flight-out → eval end to end on tree logs before the 66 training flights exist. **Not for flashing** (13 flights of mixed strategies S0–S3, same session, same two scenarios).

## 1. Generic bank merge (new) — `experiments/analysis/merge_training_bank.py`
```bash
python3 experiments/analysis/merge_training_bank.py --date 2026-10-15 --out experiments/logs/bank_<name> [--after HH-MM --before HH-MM --scenarios A8 A1 --only-tag-contains <tag> --rpm deck]
```
Pairs cf5 / cf_second by the `usd.runTag` decoded from the log (checked against `usd_run_tag` of every `*.meta.json` of the date), merges with `merge_usd_logs.py --meta --roles bottom top --omar-labels deck`, writes one folder per flight and `training_manifest.json` in the format `run_c2_lab_retrain.py` reads. Unmatched flights are listed, never merged. Result on 2026-10-15: 13 flights merged (23 s), 3 skipped = exactly the battery / lost-file outliers (A8 18:09, 18:11, 18:12). Merged CSVs are not committed (large).

## 2. Retrain chain dry run — `run_c2_lab_retrain.py --manifest …/training_manifest_dryrun.json --new-scenarios A8 A1` (4 min 8 s)
Stage A: 13 / 13 usable, 22 958 – 22 994 rows per A8 flight, ≈ 14 000 per A1 (`a_res_z` from the offline Omar labels; baseline predict-zero RMS 0.60–0.72 m/s², full-bank fit −76…−78 %). Export fold check max |trained − exported| 5–7e-7 m/s². Leave-one-flight-out R² / reduction vs zero:
| Held-out | R² | % vs zero | | Held-out | R² | % vs zero |
|---|---|---|---|---|---|---|
| A8 S0 17:31 | 0.859 | 69.0 | | A1 S0 17:37 | 0.877 | 74.2 |
| A8 S0 17:35 | 0.840 | 67.6 | | A1 S1 17:48 | **0.252** | 66.5 |
| A8 S1 17:43 | 0.769 | 59.0 | | A1 S3 17:57 | 0.739 | 67.7 |
| A8 S1 17:44 | 0.803 | 62.4 | | A1 S2 18:18 | 0.878 | 73.6 |
| A8 S1 17:46 | 0.712 | 54.2 | | | | |
| A8 S3 17:54 | 0.741 | 57.9 | | | | |
| A8 S3 17:56 | 0.738 | 56.5 | | | | |
| A8 S2 18:14 | 0.790 | 62.6 | | | | |
| A8 S2 18:16 | 0.817 | 63.7 | | | | |
The script's hard-coded gate (R² ≥ 0.35 and ≥ 50 % vs zero) prints NO-GO because of one fold (A1 S1 17:48, R² 0.25 although 66.5 % vs zero: large RMSE 0.75 m/s², i.e. a large-residual flight, not a failure to predict). **Read as: the chain works; the numbers are optimistic (same session, near-duplicate A8 flights, each held-out flight still has siblings in training) and say nothing about the retrained network. The acceptance rule is still yours to define (open item 4 of `lab_sessions/next_session_plan.md`).** The built-in thresholds are the old v1 ones.

## 3. NS2 clamp in the validation flights (`rnn_clamped`, raw uSD of cf5, motors on)
| Condition | Result |
|---|---|
| bottom vehicle airborne (z ≥ 0.3 m), 13 merged flights | clamped 0.00 % of samples in every strategy; |pred_z| p95 1.4–1.9 m/s², max 3.4–3.8 m/s² |
| low altitude with motors on (z < 0.3 m: S2 A1 takeoff, solo hovers/figure-8) | 0 % clamped, pred_z −3.5 … 0 m/s² (ground-effect term, negative) |
| top vehicle (always S0, neighbour BELOW, NS2 not used) | ≤ 0.06 % clamped, spikes to 8.0 (network input outside its training situation; unused) |
| on the ground, motors off (post-flight readback) | 8.0, clamped (z ≈ 0 ground input); never active in flight |
So the +8 m/s² clamp seen in the post-flight readback is a resting-on-the-floor artefact, not an in-flight event; no clamp effect in any S2/S3 flight. Note for the retrain: the label range in flight is within ±4 m/s², the ±8 clamp is not reached.
