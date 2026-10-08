#!/usr/bin/env python3
"""Multi-seed network-off / network-on runs for the torque-plant calibration (see torque_fit.py)."""
import sys, json
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from ns2_closed_loop_sil_torque_fit import run, summarize  # noqa: E402

def go(cs, seeds, rnn_en, res_sign=1):
    jobs = [(c, s) for c in cs for s in seeds]
    with ThreadPoolExecutor(4) as ex:
        res = list(ex.map(lambda j: run(j[0], seed=j[1], rnn_en=rnn_en, res_sign=res_sign), jobs))
    out = {}
    for (c, s), r in zip(jobs, res):
        out.setdefault(c, []).append(r if "error" in r else summarize(r))
    return out

if __name__ == "__main__":
    rnn = int(sys.argv[1]); rs = int(sys.argv[2]); seeds = list(range(int(sys.argv[3]))); cs = [float(x) for x in sys.argv[4:]]
    out = go(cs, seeds, bool(rnn), rs)
    for c, rows in out.items():
        for s, r in enumerate(rows):
            if "error" in r: print(c, s, "ERR", str(r["error"])[-120:]); continue
            print(f"c={c} seed{s}: dip {r['dip']:.2f}  per-crossing {np.round(r['dips'].get('dips_cm',[]),1).tolist() if isinstance(r['dips'],dict) else ''}  roll {np.round(r['roll_peaks'],1).tolist()} p99 {r['tilt_p99']:.1f}")
        ok=[r['dip'] for r in rows if 'error' not in r]
        print(f"  => c={c} mean dip {np.mean(ok):.2f} +/- {np.std(ok):.2f} (n={len(ok)})")
