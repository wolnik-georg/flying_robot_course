#!/usr/bin/env python3
"""Fit the single torque-lever scalar c [m^2] of the downwash plant on the network-OFF cohort only.

Targets (hardware network off, cf5, merged_A8_rnn0_2026-10-05_17-39-27 / 17-41-09):
crossing dip -5.9 +/- 0.8 cm, signed roll peak 21-28 deg within 0.6 s after each crossing,
roll sign alternating (+,-,+,-).
"""
from __future__ import annotations
import json, sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from ns2_closed_loop_sil_calibrate import a8_cfg  # noqa: E402
from ns2_closed_loop_sil_metrics import find_crossing_times  # noqa: E402
from ns2_closed_loop_sil_run import run_one, mean_dip  # noqa: E402

OUT = Path(__file__).resolve().parent / "out" / "ns2_closed_loop_sil"


def signed_roll_deg(q):
    w, x, y, z = q.T
    return np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))


def roll_peaks(r):
    lg = r["logs"]
    t = np.asarray(lg["t"]); pos = np.asarray(lg["pos_bot"]); q = np.asarray(lg["quat_bot"])
    roll = signed_roll_deg(q)
    peaks = []
    for tc in find_crossing_times(t, pos[:, 1], n_expect=4)[:4]:
        m = (t >= tc) & (t <= tc + 0.6)
        i = np.argmax(np.abs(roll[m])); peaks.append(float(roll[m][i]))
    tilt_all = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
    return peaks, float(np.percentile(tilt_all[t > 6], 99))


def run(c, seed=0, noise=True, rnn_en=False, scale=1.0, force=True, res_sign=1, sym=None):
    import os
    sym = bool(int(os.environ.get('SYM', '0'))) if sym is None else sym
    cfg = {**a8_cfg(scale, seed=seed, meas_noise=noise, rnn_en=rnn_en, downwash_plant="bank"),
           "torque_c": c, "res_sign": res_sign, "plant_symmetrize": sym,
           "label": f"torque_c{c}_s{scale}_seed{seed}_rnn{int(rnn_en)}_rs{res_sign}" + ("_sym" if sym else "")}
    return run_one(cfg, force=force)


def summarize(r):
    peaks, p99 = roll_peaks(r)
    return dict(dip=mean_dip(r), dips=r.get("crossing_dips", {}), roll_peaks=peaks, tilt_p99=p99)


if __name__ == "__main__":
    cs = [float(x) for x in sys.argv[1:]]
    with ThreadPoolExecutor(4) as ex:
        res = list(ex.map(lambda c: run(c), cs))
    for c, r in zip(cs, res):
        if "error" in r: print(c, "ERR", r["error"][-300:]); continue
        s = summarize(r); print(f"c={c}: dips {np.round(r.get('crossing_dips',{}).get('dip_cm_each',[]),1).tolist()} dip {s['dip']:.2f} cm  roll peaks {np.round(s['roll_peaks'],1)}  tilt p99 {s['tilt_p99']:.1f}")
