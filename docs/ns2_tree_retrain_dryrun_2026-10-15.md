# NS2 retrain chain on the tree — dry run on the 2026-10-15 validation flights (desk, 2026-10-15)

Purpose: prove merge → label adapter → dataset → train → leave-one-flight-out → eval end to end on tree logs before the 66 training flights exist. **Not for flashing** (13 flights of mixed strategies S0–S3, same session, same two scenarios).

## 1. Generic bank merge (new) — `experiments/analysis/merge_training_bank.py`
```bash
python3 experiments/analysis/merge_training_bank.py --date 2026-10-15 --out experiments/logs/bank_<name> [--after HH-MM --before HH-MM --scenarios A8 A1 --only-tag-contains <tag> --rpm deck]
```
Pairs cf5 / cf_second by the `usd.runTag` decoded from the log (checked against `usd_run_tag` of every `*.meta.json` of the date), merges with `merge_usd_logs.py --meta --roles bottom top --omar-labels deck`, writes one folder per flight and `training_manifest.json` in the format `run_c2_lab_retrain.py` reads. Unmatched flights are listed, never merged. Result on 2026-10-15: 13 flights merged (23 s), 3 skipped = exactly the battery / lost-file outliers (A8 18:09, 18:11, 18:12). Merged CSVs are not committed (large).

## 2. Retrain chain dry run on the 13 tree flights (`run_c2_lab_retrain.py`, 2 min 37 s) — rerun after the label fix of §5
Rows per flight 11 454 – 11 502 (A8), ≈ 7 050 (A1): bottom vehicle only. Leave-one-flight-out, new network vs v1 (v1 never saw these flights, so this comparison is fair):
| Held-out | R² new | % vs zero | RMSE new | RMSE v1 | |
|---|---|---|---|---|---|
| A8 ×9 (S0–S3) | 0.75 – 0.83 | 52 – 62 | 0.16 – 0.23 | 0.28 – 0.52 | new better in 9 / 9 |
| A1 S0 17:37 | 0.81 | 74.8 | 0.38 | 0.57 | better |
| A1 S2 18:18 | 0.79 | 71.4 | 0.39 | 0.54 | better |
| A1 S1 17:48 | **−7.3** | 70.1 | 0.87 | 0.64 | worse |
| A1 S3 17:57 | **−7.4** | 66.2 | 0.97 | 0.39 | worse |
The two A1 folds fail the R² rule only because the label of those flights is nearly flat (R² is measured against the flight's own variance, 70 % better than predicting zero is still true); the new network has a bias on them that v1 does not. Read as: the chain works and the new network beats v1 on all A8 flights; A1 under INDI is the weak spot of a 13-flight, one-session bank.

## 4. Old v1 bank relabelled with the tree definition (`relabel_v1_bank.py`, 26 / 26 flights)
- Raw uSD pairs resolved by `usd.runTag`, roles by file name (cf5 = bottom; the cards swap between the drones, a card-based assignment aligned each drone against the other's trajectory and gave 40–70 cm RMS); pre-tag flights (09-21 ×4) and the two 09-23 A1 repeats via the pairing notes / the `.usd` copies next to the v1 merge; 10-01 flights via `merge_training_bank.py`. Merge RMS: 5–11 cm (A1 close-stack 17–22 cm, expected downwash).
- Label agreement with the old logged label (best lag 0): r = 0.88 – 0.98 (A1 09-28 17:47 0.77), RMS difference 0.12 – 0.60 against a label std of 0.4 – 0.97 m/s².
- v1 scored against the relabelled labels (bottom rows, in-sample): R² 0.66 – 0.94, so the two definitions are consistent.
- LOO on the relabelled bank (26 folds, 8 min): 22 / 26 OK (R² 0.59 – 0.96, 54 – 82 % vs zero); hard fails A1 09-28 17:47 (R² −0.33, 26 % vs zero) and A6 10-01 (R² −0.21, 12 %; A6 is out of the active scope since 10-01); the rule marks the new network worse than v1 on 11 / 26 folds, **but v1 was trained on exactly these flights (in-sample), so this column favours v1 and is not a fair comparison.**
- **Consequence for gate (b):** compare new vs v1 only on flights v1 has never seen (the new bank). On the old flights v1's numbers are an upper bound, not a competitor.

## 5. Label convention fix (important)
The relabel produces a label for both drones. v1 was trained on bottom-ego samples only (the top vehicle's `a_res` was zero in the old logs and `dataset.build` drops it). With the top labels kept, v1 scored R² −1.4 … −5.9 (it predicts downwash for a vehicle that has the neighbour below). The top vehicle's `a_res_*` columns are therefore set to exactly 0 in every bank (`experiments/analysis/bank_labels.py`, called by `merge_training_bank.py` and `relabel_v1_bank.py`; marker comment `# top_a_res_zeroed`), as before. If three-vehicle data should teach the network "neighbour below → ~0", that is a deliberate later choice.

## 3. NS2 clamp in the validation flights (`rnn_clamped`, raw uSD of cf5, motors on)
| Condition | Result |
|---|---|
| bottom vehicle airborne (z ≥ 0.3 m), 13 merged flights | clamped 0.00 % of samples in every strategy; |pred_z| p95 1.4–1.9 m/s², max 3.4–3.8 m/s² |
| low altitude with motors on (z < 0.3 m: S2 A1 takeoff, solo hovers/figure-8) | 0 % clamped, pred_z −3.5 … 0 m/s² (ground-effect term, negative) |
| top vehicle (always S0, neighbour BELOW, NS2 not used) | ≤ 0.06 % clamped, spikes to 8.0 (network input outside its training situation; unused) |
| on the ground, motors off (post-flight readback) | 8.0, clamped (z ≈ 0 ground input); never active in flight |
So the +8 m/s² clamp seen in the post-flight readback is a resting-on-the-floor artefact, not an in-flight event; no clamp effect in any S2/S3 flight. Note for the retrain: the label range in flight is within ±4 m/s², the ±8 clamp is not reached.
