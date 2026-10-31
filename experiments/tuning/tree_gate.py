#!/usr/bin/env python3
"""P4 gate (plan rev 3 section 5): S0-S3 with default gains in the plant against the flown default-gain data of the unseen block (2026-10-23, experiments/results/unseen_block_2026-10-23/cells.csv).
Metrics as in unseen_block_analysis.flight_row: A1 mean z error (2-15 s), A4 z RMS (from 1 s), A8 crossing dip (minimum within +-win_s of a crossing minus the steady median).
  CFFIRMWARE_DIR=~/Desktop/crazyflie-firmware/build python3 experiments/tuning/tree_gate.py [--procs 8] [--tag name] [key=value plant options]"""
import argparse, json, sys
from multiprocessing import Pool
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import plant_core as P

STRAT = {"S0": dict(indi=0, ns2=0), "S1": dict(indi=3, ns2=0), "S2": dict(indi=0, ns2=1), "S3": dict(indi=3, ns2=1)}
CELLS = {"A1 dz0.45": ("A1", dict(dz=0.45, hold=15.0)), "A4 dz0.4 off0.075 v0.35": ("A4", dict(dz=0.4, offset=0.075, axis='y', motion='line', speed=0.35)),
         "A4 dz0.4 off0.125 v0.35": ("A4", dict(dz=0.4, offset=0.125, axis='y', motion='line', speed=0.35)), "A8 dz0.4 dur6.25": ("A8", None)}


def metric(scen, r, dur=6.25):
    try: return _metric(scen, r, dur)
    except ValueError: return float("nan")


def _metric(scen, r, dur=6.25):
    ts = np.array(r["ts"]); ez = np.array(r["ez"])
    if len(ts) == 0: return float("nan")
    if scen == "A1": return 100 * float(ez[(ts >= 2) & (ts <= 15)].mean())
    if scen == "A4": return 100 * float(np.sqrt((ez[(ts >= 1) & (ts <= r["T_sc"] + 3)] ** 2).mean()))
    win = 0.75 * dur / 4.375; cr = r["cross"]
    m = np.zeros(len(ts), bool)
    for c in cr: m |= np.abs(ts - c) < win
    steady = ~m & (ts > 3) & (ts < r["T_sc"]); base = float(np.median(ez[steady])) if steady.any() else 0.0
    return 100 * float(np.mean([ez[np.abs(ts - c) < win].min() - base for c in cr])) if cr else float("nan")


def hardware():
    import csv
    out = {}
    for row in csv.DictReader(open(Path(__file__).parents[1] / "results/unseen_block_2026-10-23/cells.csv")): out[(row["cell"], row["strategy"])] = (float(row["mean"]), float(row["std"]), int(row["n"]))
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--procs", type=int, default=8); ap.add_argument("--tag", default="nominal"); ap.add_argument("--seeds", type=int, default=3); ap.add_argument("--cal", default=None); ap.add_argument("opts", nargs="*"); a = ap.parse_args()
    extra = {k: (float(v) if v.replace(".", "", 1).replace("-", "", 1).isdigit() else v) for k, v in (o.split("=") for o in a.opts)}
    pa = ps = 1.0
    if a.cal:
        best = json.load(open(a.cal))["best"]; pa, ps = best.pop("pa"), best.pop("ps"); extra.update(best); extra["J"] = "omar"; print("calibrated plant:", best, "plume scale", pa, ps)
    cfgs = []; meta = []
    for st, so in STRAT.items():
        for cell, (scen, kw) in CELLS.items():
            for sd in range(a.seeds):
                c = P.plant(dict(extra)); c["plume"] = (c["plume"][0] * pa, c["plume"][1] * ps, c["plume"][2]); c.update(scen=scen, sc=kw, gains={}, seed=sd, tree=True, nm=f"{a.tag}_{st}_{cell.replace(' ', '')}_{sd}", tag="gate", **so); cfgs.append(c); meta.append((st, cell, scen))
    with Pool(a.procs) as pool: res = pool.map(P.run, cfgs, chunksize=1)
    hw = hardware(); rows = {}
    for (st, cell, scen), r in zip(meta, res): rows.setdefault((cell, st), []).append((metric(scen, r), r["stable"], r["tilt_max_deg"]))
    print("plant option:", a.tag, extra); print("%-26s %-3s | plant mean (sd) | hardware mean (sd, n) | diff | within max(1.5 cm, 30 %%)" % ("cell", "S"))
    ok_all = True; table = {}
    for cell in CELLS:
        for st in STRAT:
            v = rows[(cell, st)]; pm = float(np.nanmean([x[0] for x in v])) if not all(np.isnan(x[0]) for x in v) else float('nan'); ps = float(np.nanstd([x[0] for x in v])) if not np.isnan(pm) else float('nan'); h = hw[(cell, st)]; tol = max(1.5, 0.3 * abs(h[0])); ok = (not np.isnan(pm)) and abs(pm - h[0]) <= tol; ok_all &= ok
            table[f"{cell}|{st}"] = dict(plant=pm, plant_sd=ps, hw=h[0], hw_sd=h[1], n=h[2], ok=bool(ok), stable=all(x[1] for x in v), tilt=max(x[2] for x in v))
            print("%-26s %-3s | %7.1f (%.1f)    | %7.1f (%.1f, %d)    | %+6.1f | %s%s" % (cell, st, pm, ps, h[0], h[1], h[2], pm - h[0], "yes" if ok else "NO", "" if all(x[1] for x in v) else "  UNSTABLE"))
    for cell in CELLS:        # sign of pairwise differences
        pr = [(s1, s2) for i, s1 in enumerate(STRAT) for s2 in list(STRAT)[i + 1:]]
        agree = sum(np.sign(table[f"{cell}|{s1}"]["plant"] - table[f"{cell}|{s2}"]["plant"]) == np.sign(table[f"{cell}|{s1}"]["hw"] - table[f"{cell}|{s2}"]["hw"]) for s1, s2 in pr); print("%-26s pairwise order agrees: %d of %d" % (cell, agree, len(pr)))
    json.dump(table, open(Path(__file__).parent / f"out/tree_gate_{a.tag}.json", "w"), indent=1)
