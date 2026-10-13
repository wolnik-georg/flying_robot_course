#!/usr/bin/env python3
"""Why does ours diverge in the single-drone closed-loop SIL (airborne hover start, 44 ms motor lag), with the frozen parameters and with Omar's values?
One variable at a time on top of 'ours with Omar's values'. Prints time of divergence (tilt > 30 deg or |z error| > 0.3 m) and max tilt.
  python3 indi_ours_sil_sweep.py [--noise] [--seconds 10] [--jobs 10]"""
import argparse, json, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import indi_variants_sil_check as V   # noqa: E402

OM = dict(V.OMAR_IG); OMP = dict(V.OMAR_PG, ki_z=0.0)
LADDER = [("L0 frozen", {}, {}),
          ("L1 + mass, kt", dict(mass=0.0427, kt1=V.OM_KT, kt2=V.OM_KT, kt3=V.OM_KT, kt4=V.OM_KT), {}),
          ("L3 + position gains 7/4", dict(mass=0.0427, kt1=V.OM_KT, kt2=V.OM_KT, kt3=V.OM_KT, kt4=V.OM_KT), dict(kp_xy=7.0, kp_z=7.0, kv_xy=4.0, kv_z=4.0, ki_z=0.0)),
          ("L4 + residual sign -1", dict(mass=0.0427, kt1=V.OM_KT, kt2=V.OM_KT, kt3=V.OM_KT, kt4=V.OM_KT, res_sign=-1), dict(kp_xy=7.0, kp_z=7.0, kv_xy=4.0, kv_z=4.0, ki_z=0.0)),
          ("L5 + attitude gains", dict(mass=0.0427, kt1=V.OM_KT, kt2=V.OM_KT, kt3=V.OM_KT, kt4=V.OM_KT, res_sign=-1, kr=421.3, kw=69.2, kr_z=273.4, kw_z=68.3), dict(kp_xy=7.0, kp_z=7.0, kv_xy=4.0, kv_z=4.0, ki_z=0.0)),
          ("L6 + j_scale 0.693", dict(mass=0.0427, kt1=V.OM_KT, kt2=V.OM_KT, kt3=V.OM_KT, kt4=V.OM_KT, res_sign=-1, kr=421.3, kw=69.2, kr_z=273.4, kw_z=68.3, j_scale=0.693), dict(kp_xy=7.0, kp_z=7.0, kv_xy=4.0, kv_z=4.0, ki_z=0.0)),
          ("frozen + residual sign -1 only", dict(res_sign=-1), {}),
          ("frozen + position gains 7/4 only", {}, dict(kp_xy=7.0, kp_z=7.0, kv_xy=4.0, kv_z=4.0, ki_z=0.0)),
          ("frozen + filters 60 Hz only", dict(fc_bw=60.0, res_fc=60.0), {}),
          ("frozen + filters 100 Hz only", dict(fc_bw=100.0, res_fc=100.0), {})]
CASES = {
    "frozen (as flown)": ({}, {}),
    "Omar's values (all)": (OM, OMP),
    "Omar's values, res_sign +1": ({**OM, "res_sign": 1}, OMP),
    "Omar's values, j_scale 1 and k_R 292 / k_w 48": ({**OM, "j_scale": 1.0, "kr": 292.3, "kw": 48.0, "kr_z": 247.3, "kw_z": 61.8}, OMP),
    "Omar's values, filters 60 Hz": ({**OM, "fc_bw": 60.0, "res_fc": 60.0}, OMP),
    "Omar's values, filter order 1": ({**OM, "filt_order": 1}, OMP),
    "Omar's values, no motor lag in the controller (act_tau 0.044)": ({**OM, "act_tau": 0.044}, OMP),
    "frozen, fc 30 Hz": ({"fc_bw": 30.0, "res_fc": 30.0}, {}),
    "frozen, k_R 421 / k_w 69": ({"kr": 421.3, "kw": 69.2, "kr_z": 273.4, "kw_z": 68.3}, {}),
}
for _n, _i, _p in LADDER:
    CASES[_n] = (_i, _p)


def child(name, noise, seconds):
    import numpy as np
    import tk_indi_sil_run as T
    ig, pg = CASES[name]
    sets = V.ours_sets(ig, pg)
    res, (t, p, s, q) = T.run("hover", seconds, sets=sets, controller="oot", noise=noise, seed=1, verbose=False, airborne=True)
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
    bad = np.where((tilt > 30) | (np.abs(p[:, 2] - s[:, 2]) > 0.3))[0]
    print(json.dumps(dict(case=name, diverged_at_s=float(t[bad[0]]) if len(bad) else None, max_tilt_deg=float(tilt.max()), tilt_rms_last3s=float(np.sqrt((tilt[t > t[-1] - 3] ** 2).mean())))))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--noise", action="store_true"); ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--jobs", type=int, default=10); ap.add_argument("--child"); a = ap.parse_args()
    if a.child:
        child(a.child, a.noise, a.seconds); sys.exit(0)

    def one(n):
        r = subprocess.run([sys.executable, str(Path(__file__)), "--child", n, "--seconds", str(a.seconds)] + (["--noise"] if a.noise else []), cwd=str(HERE), capture_output=True, text=True)
        line = [l for l in r.stdout.splitlines() if l.startswith("{")]
        return json.loads(line[-1]) if line else dict(case=n, error=r.stderr.strip()[-200:])
    with ThreadPoolExecutor(a.jobs) as ex:
        for r in ex.map(one, CASES):
            print(r, flush=True)
