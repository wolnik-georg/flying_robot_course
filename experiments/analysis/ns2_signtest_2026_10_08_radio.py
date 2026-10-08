#!/usr/bin/env python3
"""NS2 sign test 2026-10-08: A8 crossing dips of cf5 (res_sign=-1, rnn.en=1) from radio CSVs
(+ uSD cross-check on 17-18-40). Same dip definition as ns2_2026_10_05_crossing_dip.py:
per crossing, min(z - z_sp) in +-1 s around the |y| minimum. z_sp = 0.5 m (radio has no setpoint)."""
import sys, glob, json
from pathlib import Path
import numpy as np, pandas as pd
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "experiments/analysis"))
import ns2_2026_10_05_crossing_dip as ns2
from a8_crossings import zero_crossings  # robust detector (2026-10-09 audit)

STAMPS = ["17-16-27", "17-18-40", "17-22-13", "17-24-54"]
res = {}
for st in STAMPS:
    f = REPO / f"experiments/logs/A8_cf5_2026-10-08_{st}.csv"
    d = pd.read_csv(f, comment="#")
    t = d.time_s.values - d.time_s.values[0]
    z, y = d.pos_z.values, d.pos_y.values
    # scenario window: flight above 0.3 m
    up = z > 0.3
    vb = d.vbat.values
    rows = []
    for tc in zero_crossings(t[up], y[up]):
        m = (t >= tc - 1) & (t <= tc + 1)
        e = (z[m] - 0.5) * 100
        rows.append(dict(t_cross=round(tc, 1), dip_cm=round(float(e.min()), 2), n=int(m.sum())))
    res[st] = dict(rows=rows, vbat_min_loaded=float(vb[vb > 2].min()), dur=float(t[-1]),
                   steady_mean_z_err_cm=float((z[up & (np.abs(y) > 0.3)].mean() - 0.5) * 100))
usd = REPO / "experiments/logs/usd_raw/cf5_A8_thesis01_2026-10-08_17-18-40.bin"
import decode_usd_log  # noqa
sys.path.insert(0, str(REPO / "flying_drone_stack/tools"))
u = ns2.crossing_dips_usd(usd)
res["usd_17-18-40"] = [dict(t=round(r["t_cross_s"], 1), dip_cm=round(r["dip_cm"], 2), pred_z=round(r["pred_z_mean"], 2), a_res_z=round(r["a_res_z_mean"], 2)) for r in u]
print(json.dumps(res, indent=1))
for k in STAMPS:
    r = res[k]["rows"]; print(k, "n", len(r), "mean %.2f" % np.mean([x["dip_cm"] for x in r]) if r else "", "vbat", round(res[k]["vbat_min_loaded"], 2), "dur", round(res[k]["dur"], 1))
json.dump(res, open(REPO / "experiments/analysis/out/ns2_signtest_2026-10-08/radio_dips.json", "w"), indent=1)
