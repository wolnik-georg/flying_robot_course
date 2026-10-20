#!/usr/bin/env python3
"""Did the FBL branch (Strategy 3) really run in a uSD log recorded with the FBL config (usd_fbl_config.txt, --fbl of restore_study_sd_config.py)?
Written 2026-10-20 on the desk, NOT yet run on a real FBL log.  FBL ran if ctrlFbl.steps counts up while airborne and ctrlFbl.active is 1; tripped = the tilt latch fell back to Omar.
  python3 experiments/analysis/fbl_log_check.py <log.bin> [...]"""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "flying_drone_stack/tools"))
import decode_usd_log as D

for path in sys.argv[1:]:
    d = D.load(path)
    if "fbl_steps" not in d: print(f"{Path(path).name}: no ctrlFbl.* channels (not recorded with the FBL config)"); continue
    air = np.asarray(d["z"]) > 0.30; st = d["fbl_steps"]; act = d["fbl_active"]
    n = int(air.sum()); ran = st[-1] - st[0] if len(st) else 0
    print(f"{Path(path).name}: airborne {n / 500:.1f} s | steps {int(st[0])} -> {int(st[-1])} (+{int(ran)}) | active while airborne {100 * act[air].mean() if n else 0:.0f} % | tripped max {int(d['fbl_tripped'].max())} | faults {int(d['fbl_faults'].max())}"
          f" | => {'FBL RAN' if ran > 0 and n and act[air].mean() > 0.5 else 'FBL DID NOT RUN (or barely)'}")
