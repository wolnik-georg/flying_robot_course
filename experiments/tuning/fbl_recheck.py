#!/usr/bin/env python3
"""Re-check FBL gain sets on the plant of record (plant_core.py: quadratic motor law, thrust cap optional) - tuning cells and study cells, nominal and feedforward x0.4."""
import json, sys
from multiprocessing import Pool
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent)); import plant_core as P
base = dict(kp_xy=14.0, kv_xy=9.5, ki_xy=12.0, kp_z=11.25, kv_z=5.4, ki_z=12.0)
best = [json.loads(l) for l in open(Path(__file__).parent / "out/fbl_cmaes_s0.jsonl")]; best = min(best, key=lambda d: d["score"])["gains"]
SETS = {"preset 1": base, "Step B candidate A": dict(base, kp_xy=28.0, kv_xy=9.5, ki_xy=48.0), "CMA-ES seed 0 best (old plant)": best}
STUDY = [("A1", dict(dz=0.45, hold=15.0)), ("A4", dict(dz=0.4, offset=0.075, axis='y', motion='line', speed=0.35)), ("A4", dict(dz=0.4, offset=0.125, axis='y', motion='line', speed=0.35)), ("A8", None)]
if __name__ == "__main__":
    cfgs = []; meta = []
    for cells, cname in ((P.TUNING_CELLS, "tuning"), (STUDY, "study")):
        for sn, g in SETS.items():
            for v, vn in ((dict(), "nominal"), (dict(ffs=0.4), "ff0.4"), (dict(md=40), "delay40")):
                for sc, kw in cells:
                    c = P.plant(dict(v, J="fbl")); c.update(scen=sc, sc=kw, gains=g, seed=0, nm=f"rc_{cname}_{sn[:6]}_{vn}_{sc}_{kw and kw.get('dz')}_{kw and kw.get('offset')}", tag="rc"); cfgs.append(c); meta.append((cname, sn, vn, sc))
    (P.OUT / "stock_inner_traces").mkdir(parents=True, exist_ok=True)
    with Pool(8) as pool: res = pool.map(P.run, cfgs, chunksize=1)
    agg = {}
    for m, r in zip(meta, res): agg.setdefault(m[:3], []).append(100.0 if not r["stable"] else float(np.sqrt(r["x_rms_cm"] ** 2 + r["y_rms_cm"] ** 2 + r["z_rms_cm"] ** 2)))
    for (cname, sn, vn), v in agg.items(): print("%-6s %-32s %-8s 3-D RMS mean %.2f cm  (cells: %s)" % (cname, sn, vn, np.mean(v), " ".join("%.1f" % x for x in v)), flush=True)
