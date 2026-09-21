# A8 — 19 Sep 2026 — analysis packages

**Raw uSD is never copied or modified.** Each flight folder holds:

- `package_manifest.json` — match, status, paths to archive files
- Symlinks `cf5_A8_*` / `cf_second_A8_*` → `../usd_raw/2026-09-19_THESIS{1,2}/thesisNN`
- Copies of radio CSVs + `A8_<stamp>.meta.json`
- `A8_<stamp>_merged_usd.csv` — 500 Hz, `merge_usd_logs.py --meta --roles`
- `analysis/` — metrics, dashboard PNG, interaction PNG

Session index: `session_manifest.json`. Full desk pipeline:
`python3 experiments/analysis/run_a8_2026_09_19_suite.py`.

**Flight table (configs, thesisNN, every plot path):**
[`docs/28_USD_Radio_Matching_and_Session_Analysis.md`](../../../docs/28_USD_Radio_Matching_and_Session_Analysis.md).
