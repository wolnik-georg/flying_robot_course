#!/usr/bin/env python3
"""Scenario bank for gradient tuning with downwash: for every setting of the NS2 training grid (none of them a study cell) the bottom drone's setpoints (pos, vel, acc, jerk) and the top
drone's pos / vel at 1 ms, built exactly like plant_core.run (takeoff ramp, hold, scenario, landing ramp, ground). Also writes the plume / downwash constants of the plant of record.
Needs the firmware bindings: CFFIRMWARE_DIR=~/Desktop/crazyflie-firmware/build python3 diff/build_bank.py"""
import itertools, json, sys
from pathlib import Path
import numpy as np
here = Path(__file__).parent.parent; sys.path.insert(0, str(here))
import plant_core as P, core
C = P.C; anchor = np.array([0, 0, 0.5]); T_TO, T_HOLD, T_LAND, T_GND = 3.0, 1.5, 3.0, 1.0

def ramp(u_, T_):
    s_ = 10 * u_ ** 3 - 15 * u_ ** 4 + 6 * u_ ** 5; ds = (30 * u_ ** 2 - 60 * u_ ** 3 + 30 * u_ ** 4) / T_
    dds = (60 * u_ - 180 * u_ ** 2 + 120 * u_ ** 3) / T_ ** 2 if 0 < u_ < 1 else 0.0; ddds = (60 - 360 * u_ + 360 * u_ ** 2) / T_ ** 3 if 0 < u_ < 1 else 0.0
    return s_, ds, dds, ddds

def instance(sc):
    T_sc = min(r.curve.duration for r in sc.robots); Ttot = T_TO + T_HOLD + T_sc + T_LAND + T_GND; n = int(Ttot / 1e-3)
    p_start = C.kin(sc, 0, 0.0, anchor)[0]; p_end = C.kin(sc, 0, T_sc, anchor)[0]; out = np.zeros((n, 18), np.float32)
    for i in range(n):
        t = i * 1e-3; ts = t - T_TO - T_HOLD; tp, tv = C.kin(sc, 1, max(ts, 0.0), anchor)[:2]
        if ts < 0:
            s_, ds, dds, ddds = ramp(min(max(t / T_TO, 0.0), 1.0), T_TO); h = p_start[2]
            pr = np.array([p_start[0], p_start[1], h * s_]); vr = np.array([0, 0, h * ds]); ar = np.array([0, 0, h * dds]); jr = np.array([0, 0, h * ddds])
        elif ts <= T_sc: pr, vr, ar, jr, _ = C.kin(sc, 0, ts, anchor)
        else:
            s_, ds, dds, ddds = ramp(min(max((ts - T_sc) / T_LAND, 0.0), 1.0), T_LAND); h = p_end[2]
            pr = np.array([p_end[0], p_end[1], h * (1 - s_)]); vr = np.array([0, 0, -h * ds]); ar = np.array([0, 0, -h * dds]); jr = np.array([0, 0, -h * ddds])
        out[i] = np.r_[pr, vr, ar, jr, tp, tv]
    return out, Ttot

BANKS = {
    'A1': [dict(dz=dz, hold=12.0) for dz in (0.375, 0.5, 0.625, 0.75)],
    'A4': [dict(dz=dz, offset=o, axis='y', motion='line', speed=s) for dz, o, s in itertools.product((0.375, 0.5, 0.75), (0.05, 0.1, 0.15), (0.2, 0.3, 0.4, 0.5))],
    'A8': [dict(dz=dz, span=1.0, duration=1.875 / s, settle=2.0, passes=2) for dz, s in itertools.product((0.375, 0.5, 0.75), (0.2, 0.3, 0.4, 0.5))]}
for name, kws in BANKS.items():
    arrs, lens = [], []
    for kw in kws:
        a, T_tot = instance(getattr(C.FSC, name)(**kw)); arrs.append(a); lens.append(a.shape[0])
    N = max(lens); bank = np.zeros((len(arrs), N, 18), np.float32)
    for k, a in enumerate(arrs): bank[k, :len(a)] = a; bank[k, len(a):] = a[-1]
    np.savez(here / f'out/bank_{name}.npz', bank=bank, n_valid=np.array(lens), settings=np.array([json.dumps(k) for k in kws]))
    print(name, bank.shape, 'max length %.1f s' % (N / 1000), flush=True)
A0, s0, n0 = json.load(open(here / 'out/plume_fit.json'))['A'], json.load(open(here / 'out/plume_fit.json'))['sigma'], json.load(open(here / 'out/plume_fit.json'))['n']
json.dump(dict(dws=core.CAL['dws'], plume=[A0 * core.CAL['pa'], s0 * core.CAL['ps'], n0], fmax=core.CAL['fmax'], gn=core.CAL['gn'], lag=core.CAL['lag']), open(here / 'out/dw_params.json', 'w'), indent=1)
print('dw params', open(here / 'out/dw_params.json').read())
