#!/usr/bin/env python3
"""Closed-loop equivalence: plant_core.run (firmware bindings, tree S0, noise off, no downwash) against diff/s0_plant_torch.rollout on the same setpoints.
System python3 + CFFIRMWARE_DIR=~/Desktop/crazyflie-firmware/build."""
import json, os, sys
from pathlib import Path
import numpy as np, torch
sys.path.insert(0, str(Path(__file__).parent.parent)); sys.path.insert(0, str(Path(__file__).parent))
import plant_core as P
import s0_torch as T, s0_plant_torch as PT
C = P.C
class Z:        # noise-free rng stub for plant_core.run
    def normal(self, loc=0, scale=1, size=None): return np.zeros(size if size is not None else ())
    def uniform(self, lo=0, hi=1, size=None): return lo + 0 * np.zeros(size if size is not None else ())
np.random.default_rng = lambda *a, **k: Z()
SCEN = os.environ.get('SCEN', 'A8'); sc = P.scenario(SCEN)
gains = dict(Kpos_P=(6.6, 6.6, 4.9), Kpos_D=(2.7, 2.7, 4.0), Kpos_Iz=4.2, KR=(.0066, .0066, .0069), Komega=(.0008, .0008, .0015), KI=(.118, .118, .118), Kpos_I_limit=2.0)
tg = {'Kpos_P.x': 6.6, 'Kpos_P.y': 6.6, 'Kpos_P.z': 4.9, 'Kpos_D.x': 2.7, 'Kpos_D.y': 2.7, 'Kpos_D.z': 4.0, 'Kpos_I.z': 4.2, 'KR.x': .0066, 'KR.y': .0066, 'KR.z': .0069, 'Komega.x': .0008, 'Komega.y': .0008, 'Komega.z': .0015, 'KI.x': .118, 'KI.y': .118, 'KI.z': .118}
MD = int(os.environ.get('MD', 0)); LAG = float(os.environ.get('LAG', 30))
GJ = os.environ.get('GJ')
if GJ:       # S0_PARAMS-style gains from a diff_*.json
    gg = json.load(open(GJ))['gains']
    gains = dict(Kpos_P=(gg['Pxy'], gg['Pxy'], gg['Pz']), Kpos_D=(gg['Dxy'], gg['Dxy'], gg['Dz']), Kpos_Iz=gg['Iz'], KR=(gg['KRxy'], gg['KRxy'], gg['KRz']), Komega=(gg['Kwxy'], gg['Kwxy'], gg['Kwz']), KI=(gg['KIxy'], gg['KIxy'], .03), Kpos_I_limit=2.0)
    tg = {'Kpos_P.x': gg['Pxy'], 'Kpos_P.y': gg['Pxy'], 'Kpos_P.z': gg['Pz'], 'Kpos_D.x': gg['Dxy'], 'Kpos_D.y': gg['Dxy'], 'Kpos_D.z': gg['Dz'], 'Kpos_I.z': gg['Iz'], 'KR.x': gg['KRxy'], 'KR.y': gg['KRxy'], 'KR.z': gg['KRz'], 'Komega.x': gg['Kwxy'], 'Komega.y': gg['Kwxy'], 'Komega.z': gg['Kwz'], 'KI.x': gg['KIxy'], 'KI.y': gg['KIxy']}
cfg = dict(P.plant(dict(J='omar', md=MD, plume=None, gn=0.0, fmax=0.2758, dws=0.0, rd=0, lag=LAG, tag='equiv')), scen=SCEN, tree=1, indi=0, ns2=0, tree_gains=tg, seed=0, gains={})
cfg['plume'] = None
res = P.run(cfg)
name = f"{SCEN}_i{cfg['inner']}_p{cfg['preset']}_{cfg['dist']}_lag{LAG}_{cfg['J']}_md{MD}_adNone_k0_{cfg.get('nm','')}_equiv_scr.npy"
rec = np.load(P.OUT / 'stock_inner_traces' / name)
# setpoint arrays at 1 ms exactly as plant_core builds them
anchor = np.array([0, 0, 0.5]); T_TO, T_HOLD, T_LAND, T_GND = 3.0, 1.5, 3.0, 1.0; T_sc = min(r.curve.duration for r in sc.robots); Ttot = T_TO + T_HOLD + T_sc + T_LAND + T_GND
n = int(Ttot / 1e-3); p_start = C.kin(sc, 0, 0.0, anchor)[0]; sp = {k: np.zeros((n, 3)) for k in ('pos', 'vel', 'acc', 'jerk')}
def ramp(u_, T_):
    s_ = 10 * u_ ** 3 - 15 * u_ ** 4 + 6 * u_ ** 5; ds = (30 * u_ ** 2 - 60 * u_ ** 3 + 30 * u_ ** 4) / T_
    dds = (60 * u_ - 180 * u_ ** 2 + 120 * u_ ** 3) / T_ ** 2 if 0 < u_ < 1 else 0.0; ddds = (60 - 360 * u_ + 360 * u_ ** 2) / T_ ** 3 if 0 < u_ < 1 else 0.0
    return s_, ds, dds, ddds
p_end = C.kin(sc, 0, T_sc, anchor)[0]
for i in range(n):
    t = i * 1e-3; ts = t - T_TO - T_HOLD
    if ts < 0:
        s_, ds, dds, ddds = ramp(min(max(t / T_TO, 0.0), 1.0), T_TO); h = p_start[2]
        pr = np.array([p_start[0], p_start[1], h * s_]); vr = np.array([0, 0, h * ds]); ar = np.array([0, 0, h * dds]); jr = np.array([0, 0, h * ddds])
    elif ts <= T_sc: pr, vr, ar, jr, _ = C.kin(sc, 0, ts, anchor)
    else:
        s_, ds, dds, ddds = ramp(min(max((ts - T_sc) / T_LAND, 0.0), 1.0), T_LAND); h = p_end[2]
        pr = np.array([p_end[0], p_end[1], h * (1 - s_)]); vr = np.array([0, 0, -h * ds]); ar = np.array([0, 0, -h * dds]); jr = np.array([0, 0, -h * ddds])
    sp['pos'][i], sp['vel'][i], sp['acc'][i], sp['jerk'][i] = pr, vr, ar, jr
spt = {k: torch.tensor(v, dtype=torch.float64)[:, None, :] for k, v in sp.items()}
g = {k: torch.tensor(v, dtype=torch.float64)[None] if np.ndim(v) else torch.tensor(float(v), dtype=torch.float64) for k, v in gains.items()}
out, fin = PT.rollout(spt, g, dict(lag=LAG, fmax=0.2758, quad=True, kt=1.0, md=MD))
xt = out[:, 0, :].numpy(); xr = rec[:len(xt), 8:11] if rec.shape[1] > 10 else None
m = min(len(xt), len(rec)); d = np.abs(xt[:m] - rec[:m, 8:11]); tt = rec[:m, 0]
print('samples', m, ' max |dx| %.2e m   rms |dx| %.2e m   (final-window RMS error: firmware %.3f cm, torch %.3f cm)' % (d.max(), np.sqrt((d ** 2).mean()),
      100 * np.sqrt(np.mean((rec[:m, 1:4] ** 2))), 100 * np.sqrt(np.mean((xt[:m] - sp['pos'][::10][:m]) ** 2))))
print('worst at t=%.2f s' % tt[d.max(1).argmax()], ' tilt max torch %.1f deg, firmware %.1f deg' % (float(fin['tilt_max'][0]), res['tilt_max_deg']))
