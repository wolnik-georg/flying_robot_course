#!/usr/bin/env python3
"""Tree validation 2026-10-15: per-flight and per-strategy numbers from the merged two-vehicle uSD logs.
  python3 experiments/analysis/tree_validation_analysis.py [--dir experiments/logs/merged_tree_2026-10-15] [--md out.md]
Bottom = cf5 (carries the strategy), top = cf_second (always S0). A8: crossing = relative y passes zero; dip = lowest bottom z error
within +-0.8 s of the crossing relative to the steady offset outside +-1.2 s windows; separation error = (z_top - z_bottom) - dz_cmd at
the crossings. A1: bottom z error, separation error, attitude excursions over the hold."""
import argparse, glob, re, sys
from pathlib import Path
import numpy as np

def load(path):
    lines = [l for l in open(path) if not l.startswith("#")]
    hdr = lines[0].strip().split(",")
    a = np.loadtxt(lines[1:], delimiter=",", ndmin=2)
    cols = {h: a[:, i] for i, h in enumerate(hdr)}
    pre = {}
    for h in hdr:
        if "." in h:
            p, n = h.split(".", 1); pre.setdefault(p, set()).add(n)
    b = next(p for p in pre if p.startswith("cf5_")); t = next(p for p in pre if p.startswith("cf_second_"))
    return cols, b, t

def metrics(path, dz_cmd=0.5):
    c, b, t = load(path); T = c["t"]
    g = lambda p, n: c[f"{p}.{n}"]
    zb, zt = g(b, "z"), g(t, "z"); tgt = g(b, "ctrltarget_z")
    air = (zb > 0.3) & (T > 0)
    sel = air & (np.abs(tgt - np.median(tgt[air])) < 0.2)
    ez = (zb - tgt) * 100
    out = {"flight": Path(path).stem.replace("merged_", ""), "air_s": air.sum() / 500.0}
    out["z_err_mean"], out["z_err_std"] = float(ez[sel].mean()), float(ez[sel].std())
    out["roll_max"] = float(np.abs(g(b, "roll_deg")[air]).max()); out["pitch_max"] = float(np.abs(g(b, "pitch_deg")[air]).max())
    out["xy_rms"] = float(np.sqrt(np.mean(((g(b, "x") - g(b, "ctrltarget_x")) ** 2 + (g(b, "y") - g(b, "ctrltarget_y")) ** 2)[sel])) * 100)
    sep_dz = (zt - zb - dz_cmd) * 100
    out["sep_dz_mean"], out["sep_dz_std"] = float(sep_dz[sel].mean()), float(sep_dz[sel].std())
    out["top_z_std"] = float((zt - g(t, "ctrltarget_z"))[sel].std() * 100)
    if "A8" in out["flight"]:
        ry = g(b, "y") - g(t, "y"); s = np.sign(ry); idx = np.where((s[:-1] * s[1:] < 0) & air[:-1])[0]
        mask = sel.copy()
        for i in idx: mask &= ~((T > T[i] - 1.2) & (T < T[i] + 1.2))
        off = float(np.median(ez[mask])); dips, peaks, seps = [], [], []
        for i in idx:
            w = (T > T[i] - 0.8) & (T < T[i] + 0.8)
            if w.sum() < 100: continue
            dips.append(ez[w].min() - off); peaks.append(ez[w].max() - off)
            j = np.argmin(np.abs(T[w] - T[i])); seps.append(sep_dz[w][j])
        out.update(n_cross=len(dips), offset=off, dip_mean=float(np.mean(dips)), dip_min=float(np.min(dips)),
                   peak_mean=float(np.mean(peaks)), peak_max=float(np.max(peaks)), sep_err_cross=float(np.mean(seps)))
    return out

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--dir", default="experiments/logs/merged_tree_2026-10-15"); ap.add_argument("--md")
    a = ap.parse_args(); R = []
    for f in sorted(glob.glob(f"{a.dir}/merged_*.csv")):
        try: R.append(metrics(f))
        except Exception as e: print("skip", f, e, file=sys.stderr)
    strat = lambda n: re.search(r"tree_(S\d|unknownstrategy)", n).group(1)
    print("flight                     air  zerr mean/std  xy_rms  roll/pitch max  sep_dz mean/std | A8: n  dip mean/min  upward peak mean/max  sep err @cross")
    for r in R:
        s = f"{r['flight']:34s} {r['air_s']:4.1f} {r['z_err_mean']:+6.1f}/{r['z_err_std']:4.1f}  {r['xy_rms']:5.1f}  {r['roll_max']:5.1f}/{r['pitch_max']:5.1f}  {r['sep_dz_mean']:+6.1f}/{r['sep_dz_std']:4.1f}"
        if "dip_mean" in r: s += f" | {r['n_cross']} {r['dip_mean']:+6.1f}/{r['dip_min']:+6.1f}  {r['peak_mean']:+5.1f}/{r['peak_max']:+5.1f}  {r['sep_err_cross']:+6.1f}"
        print(s)
    print("\nper strategy A8 (mean over flights, crossings pooled):")
    for sname in ("S0", "S1", "S2", "S3"):
        rs = [r for r in R if strat(r["flight"]) == sname and "dip_mean" in r]
        if rs:
            print(f"  {sname}: flights {len(rs)}  dip {np.mean([r['dip_mean'] for r in rs]):+.1f} cm (spread of flight means {np.std([r['dip_mean'] for r in rs]):.1f})  "
                  f"worst {min(r['dip_min'] for r in rs):+.1f}  upward peak {np.mean([r['peak_mean'] for r in rs]):+.1f} (max {max(r['peak_max'] for r in rs):+.1f})  "
                  f"offset {np.mean([r['offset'] for r in rs]):+.1f}  z std {np.mean([r['z_err_std'] for r in rs]):.1f}  sep err at crossing {np.mean([r['sep_err_cross'] for r in rs]):+.1f} cm")
