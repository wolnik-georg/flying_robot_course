#!/usr/bin/env python3
"""Motor-output response at DShot-sentinel instants vs random airborne instants (RPM-filter check).

usage: rpm_sentinel_motor_response.py [--label NAME] file.bin [file.bin ...]
Per file and pooled: number of raw 0xFFFF sentinels (motor_m*_rpm >= 65000), excursion of the summed motor PWM
within 5 ticks after the instant relative to the median of the surrounding ticks, and the same at 5x as many random
airborne instants. ratio = median(sentinel) / median(random); ~1 means the sentinel has no detectable effect.
"""
import argparse, sys
from pathlib import Path
import numpy as np
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "flying_drone_stack/tools"))
import decode_usd_log as d

def excursions(path, rng):
    r = d.load(str(path)); up = r["z"] > 0.3
    pw = np.sum([r[f"motor_m{i}"] for i in range(1, 5)], axis=0).astype(float)
    def exc(k):
        base = np.median(np.r_[pw[k-10:k-1], pw[k+5:k+14]]); return float(np.max(np.abs(pw[k:k+5] - base)) / max(base, 1))
    ks = [k for i in range(1, 5) for k in np.where(up & (r[f"motor_m{i}_rpm"] >= 65000))[0] if 15 < k < len(pw) - 15]
    cand = np.where(up)[0]; cand = cand[(cand > 15) & (cand < len(pw) - 15)]
    rnd = rng.choice(cand, size=min(len(cand), max(len(ks), 1) * 5), replace=False)
    return [exc(k) for k in ks], [exc(k) for k in rnd], int(up.sum()) * 4

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--label", default=""); ap.add_argument("files", nargs="+")
    a = ap.parse_args(); rng = np.random.default_rng(0); S = []; B = []; N = 0
    for f in a.files:
        s, b, n = excursions(f, rng); S += s; B += b; N += n
        print(f"{Path(f).name:60s} sentinels {len(s):3d}  median exc {np.median(s) if s else float('nan'):.3f}")
    if S:
        print(f"{a.label or 'pooled'}: sentinels {len(S)} = {100*len(S)/N:.3f} % of airborne motor samples | sentinel median {np.median(S):.3f} p90 {np.percentile(S,90):.3f} | random median {np.median(B):.3f} p90 {np.percentile(B,90):.3f} | ratio {np.median(S)/np.median(B):.2f}")
    else:
        print("no sentinels found")
main()
