#!/usr/bin/env python3
"""Plume samples from all two-drone uSD flights: relative position bottom - top and the disturbance the bottom drone's motors had to cancel, 0.5 s means every 0.25 s.
  torque need (x, y, z) = -(torque the rpm produced) [N m]   (valid when the angular acceleration averages out; windows with large rate activity are flagged)
  fz_dist = weight - rpm thrust [N]                             (valid when the vertical acceleration averages out)
  ~/.pyenv/versions/flying_robots/bin/python experiments/fbl/sysid/collect_plume_samples.py"""
import json, glob, warnings
from pathlib import Path
import numpy as np, pandas as pd
warnings.filterwarnings('ignore')
REPO = Path(__file__).resolve().parents[3]; LOGS = REPO / 'experiments/logs'
ARM = 0.70710678 * 0.05; T2T = 0.00569278844371417; KF = 3.91194027330774e-08; M = 0.0427; G = 9.81
rows = []; seen = set(); skipped = []
for man in sorted(LOGS.glob('bank_*/training_manifest.json')):
    for fl in json.load(open(man)).get('flights', []):
        path = REPO / fl['path']; key = path.name
        if key in seen or not path.exists(): continue
        seen.add(key)
        try:
            hdr = pd.read_csv(path, comment='#', nrows=0).columns
            p5 = sorted({c.split('.')[0] for c in hdr if c.startswith('cf5')}); ps = sorted({c.split('.')[0] for c in hdr if c.startswith('cf_second')})
            if not p5 or not ps: skipped.append((key, 'no pair')); continue
            need = ['t'] + [p5[0] + '.' + v for v in ('x', 'y', 'z', 'vz', 'rpm_m1', 'rpm_m2', 'rpm_m3', 'rpm_m4', 'gyro_x', 'gyro_y')] + [ps[0] + '.' + v for v in ('x', 'y', 'z')]
            if not all(c in hdr for c in need): skipped.append((key, 'missing cols')); continue
        except Exception as e:
            skipped.append((key, str(e)[:40])); continue
        b = pd.read_csv(path, comment='#', usecols=need)[need]; b.columns = ['t', 'x', 'y', 'z', 'vz', 'r1', 'r2', 'r3', 'r4', 'gx', 'gy', 'tx_', 'ty_', 'tz_']
        b = b[(b.z > 0.2) & (b.r1 > 1000)].reset_index(drop=True)
        if len(b) < 1000: continue
        t = b.t.values; dt = np.median(np.diff(t)); w = max(int(0.5 / dt), 5)
        F = [KF * (b['r%d' % i].values * 2 * np.pi / 60) ** 2 for i in (1, 2, 3, 4)]
        tx = ARM * (-F[0] - F[1] + F[2] + F[3]); ty = ARM * (-F[0] + F[1] + F[2] - F[3]); tz = T2T * (-F[0] + F[1] - F[2] + F[3]); T = sum(F)
        az = np.gradient(pd.Series(b.vz.values).rolling(w, center=True, min_periods=1).mean().values, dt)
        gxs = pd.Series(np.radians(b.gx.values)).rolling(w, center=True, min_periods=1).mean().values; gys = pd.Series(np.radians(b.gy.values)).rolling(w, center=True, min_periods=1).mean().values
        ax_ = np.gradient(gxs, dt); ay_ = np.gradient(gys, dt)
        topvx = np.gradient(pd.Series(b.tx_.values).rolling(w, center=True, min_periods=1).mean().values, dt); topvy = np.gradient(pd.Series(b.ty_.values).rolling(w, center=True, min_periods=1).mean().values, dt)
        mean = lambda a: pd.Series(a).rolling(w, center=True, min_periods=1).mean().values
        for i in range(w, len(b) - w, int(0.25 / dt)):
            rows.append((fl['scenario'], key[:-len('_merged_usd.csv')], t[i], b.x[i] - b.tx_[i], b.y[i] - b.ty_[i], b.z[i] - b.tz_[i], topvx[i], topvy[i], -mean(tx)[i], -mean(ty)[i], -mean(tz)[i], M * G - mean(T)[i], az[i], ax_[i], ay_[i]))
S = pd.DataFrame(rows, columns=['scen', 'flight', 't', 'dx', 'dy', 'dz', 'tvx', 'tvy', 'tau_x', 'tau_y', 'tau_z', 'fz_dist', 'az', 'alpha_x', 'alpha_y'])
out = Path(__file__).parent / 'out'; out.mkdir(exist_ok=True); S.to_csv(out / 'plume_samples.csv', index=False)
print(len(S), 'samples from', S.flight.nunique(), 'flights;', S.scen.value_counts().to_dict(), '; skipped', len(skipped)); print(skipped[:5])
