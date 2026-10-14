#!/usr/bin/env python3
"""Check of the offline NS2 label reconstruction (omar_labels.py) against the labels Omar's controller logged itself.
Flight: Omar C A8, 2026-10-13 (uSD with the ctrlOmarIndi.a_*_f channels, indi 3). Passes if the reconstructed a_res_z has
correlation > 0.99 and RMS error < 0.05 m/s^2 (logged std 0.43), and x/y correlate > 0.95.   python3 test_omar_labels.py"""
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent))
import decode_usd_log  # noqa: E402
from omar_labels import reconstruct_a_res  # noqa: E402

BIN = ROOT / "experiments/logs/usd_raw/cf5_A8_omarC_1_thesis04_2026-10-13_18-07-46.bin"
if not BIN.is_file():
    print("SKIP: reference flight not on this machine"); sys.exit(0)
d = decode_usd_log.load(str(BIN))
a_res, _, _, valid = reconstruct_a_res(d, "deck")
lg = np.stack([d["omar_a_imu_fx"] - d["omar_a_rpm_fx"], d["omar_a_imu_fy"] - d["omar_a_rpm_fy"], d["omar_a_imu_fz"] - d["omar_a_rpm_fz"]], axis=1)
active = valid & (np.abs(np.stack([d["omar_a_rpm_fx"], d["omar_a_rpm_fy"], d["omar_a_rpm_fz"]], axis=1)).sum(axis=1) > 0)
res = []
ok = True
for ax, nm in enumerate("xyz"):
    e = a_res[active, ax] - lg[active, ax]
    c = float(np.corrcoef(a_res[active, ax], lg[active, ax])[0, 1])
    rms = float(np.sqrt(np.mean(e ** 2)))
    good = (c > 0.99 and rms < 0.05) if nm == "z" else c > 0.95
    ok &= good
    print(f"  {'PASS' if good else 'FAIL'}  a_res_{nm}: corr {c:.4f}  RMS {rms:.4f} m/s^2  (logged std {lg[active, ax].std():.3f})")
print("RESULT:", "ALL PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
