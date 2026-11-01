#!/usr/bin/env python3
"""Equivalence with downwash (NS2 force truth + plume torque + top drone): plant_core.run (noise off) vs diff/s0_plant_torch.rollout_blocks on a bank instance.
CFFIRMWARE_DIR=~/Desktop/crazyflie-firmware/build python3 diff/check_dw_equiv.py   (env SCEN=A8|A4|A1, IDX=bank index)"""
import json, os, sys
from pathlib import Path
import numpy as np, torch
sys.path.insert(0, str(Path(__file__).parent.parent)); sys.path.insert(0, str(Path(__file__).parent))
import plant_core as P, core
import s0_torch as T, s0_plant_torch as PT, ns2_torch as NS2
class Z:
    def normal(self, loc=0, scale=1, size=None): return np.zeros(size if size is not None else ())
    def uniform(self, lo=0, hi=1, size=None): return lo + 0 * np.zeros(size if size is not None else ())
np.random.default_rng = lambda *a, **k: Z()
SCEN = os.environ.get('SCEN', 'A8'); IDX = int(os.environ.get('IDX', 0)); LAG = float(os.environ.get('LAG', 30)); MD = int(os.environ.get('MD', 0))
bk = np.load(Path(__file__).parent.parent / f'out/bank_{SCEN}.npz'); sett = json.loads(str(bk['settings'][IDX])); n = int(bk['n_valid'][IDX]); arr = bk['bank'][IDX, :n]
gg = {'Pxy': 7., 'Pz': 7., 'Dxy': 4., 'Dz': 4., 'Iz': 1.5, 'KRxy': .007, 'KRz': .008, 'Kwxy': .00115, 'Kwz': .002, 'KIxy': .03}
tg = {'Kpos_P.x': 7., 'Kpos_P.y': 7., 'Kpos_P.z': 7., 'Kpos_D.x': 4., 'Kpos_D.y': 4., 'Kpos_D.z': 4., 'Kpos_I.z': 1.5, 'KR.x': .007, 'KR.y': .007, 'KR.z': .008, 'Komega.x': .00115, 'Komega.y': .00115, 'Komega.z': .002, 'KI.x': .03, 'KI.y': .03}
cfg = dict(core.base_plant(dict(gn=0.0, lag=LAG, md=MD, rd=0, tag='dwequiv', J='omar')), scen=SCEN, sc=sett, tree=1, indi=0, ns2=0, tree_gains=tg, seed=0, gains={})
res = P.run(cfg)
name = f"{SCEN}_i{cfg['inner']}_p{cfg['preset']}_{cfg['dist']}_lag{LAG}_{cfg['J']}_md{MD}_adNone_k0__dwequiv_scr.npy"
rec = np.load(P.OUT / 'stock_inner_traces' / name)
dev = 'cpu'; dtype = torch.float64; B = 1
sp = {k: torch.tensor(arr[:, a:a + 3], dtype=dtype)[:, None, :] for k, a in (('pos', 0), ('vel', 3), ('acc', 6), ('jerk', 9), ('tp', 12), ('tv', 15))}
dwj = json.load(open(Path(__file__).parent.parent / 'out/dw_params.json'))
net = NS2.load('/home/georg/Desktop/flying_robot_course/flying_drone_stack/tools/residual/weights/ns2_v2.npz', dtype, dev)
plant = dict(lag=LAG, fmax=dwj['fmax'], kt=1.0, md=MD, dw=dict(net=net, dws=dwj['dws'], plume=tuple(torch.tensor(v, dtype=dtype) for v in dwj['plume'])))
g = {k: torch.tensor(v, dtype=dtype)[None] if np.ndim(v) else torch.tensor(float(v), dtype=dtype) for k, v in dict(Kpos_P=(7., 7., 7.), Kpos_D=(4., 4., 4.), Kpos_Iz=1.5, KR=(.007, .007, .008), Komega=(.00115, .00115, .002), KI=(.03, .03, .03), Kpos_I_limit=2.0).items()}
nz = dict(pos=torch.zeros(n, 1, 3, dtype=dtype), vel=torch.zeros(n, 1, 3, dtype=dtype), gyro=torch.zeros(n // 2 + 1, 1, 3, dtype=dtype))
out, fin = PT.rollout_blocks(sp, g, plant, dtype=dtype, device=dev, noise=nz, compile=False)
xt = out[:, 0, :].numpy(); m = min(len(xt), len(rec)); d = np.abs(xt[:m] - rec[:m, 8:11])
print('%s %s: samples %d  max |dx| %.2e m  rms |dx| %.2e m  (RMS error over the run: firmware %.2f cm, torch %.2f cm)' % (SCEN, sett, m, d.max(), np.sqrt((d ** 2).mean()), 100 * np.sqrt(np.mean(rec[:m, 1:4] ** 2)), 100 * np.sqrt(np.mean((xt[:m] - arr[::10, 0:3][:m]) ** 2))))
print('tilt max torch %.1f deg, firmware %.1f deg' % (float(fin['tilt_max'][0]), res['tilt_max_deg']))
