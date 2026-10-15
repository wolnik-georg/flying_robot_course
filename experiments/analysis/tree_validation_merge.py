#!/usr/bin/env python3
"""Merge the paired cf5 / cf_second uSD logs of the 2026-10-15 tree validation (reproduces experiments/logs/merged_tree_2026-10-15/).
Pairs are matched by file label (cf5_<label>_thesisNN.bin <-> cf_second_<label>_thesisNN.bin in experiments/logs/usd_raw) and the
radio sidecar of the same minute; alignment is each vehicle against its own commanded trajectory (merge_usd_logs --meta --roles).
  python3 experiments/analysis/tree_validation_merge.py"""
import glob, re, subprocess
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]; USD = ROOT / "experiments/logs/usd_raw"; OUT = ROOT / "experiments/logs/merged_tree_2026-10-15"
OUT.mkdir(parents=True, exist_ok=True)
for f in sorted(USD.glob("cf_second_tree_*_thesis*.bin")):
    label = re.search(r"cf_second_(tree_.*)_thesis\d+\.bin", f.name).group(1)
    c5 = sorted(USD.glob(f"cf5_{label}_thesis*.bin"))[0]
    h, m = re.search(r"_(\d{2})(\d{2})(?:_|$)", label).groups()
    meta = sorted((ROOT / "experiments/logs").glob(f"A[18]_2026-10-15_{h}-{m}-*.meta.json"))[0]
    cmd = ["python3", str(ROOT / "flying_drone_stack/tools/merge_usd_logs.py"), str(c5), str(f), "--meta", str(meta), "--roles", "bottom", "top",
           "-o", str(OUT / f"merged_{label}.csv")] + (["--tag-quality-only"] if ("FAILED" in label or "_A1_" in label) else [])
    r = subprocess.run(cmd, capture_output=True, text=True)
    print(label, "rc", r.returncode, [l.strip() for l in (r.stdout + r.stderr).split("\n") if "RMS" in l][:2])
