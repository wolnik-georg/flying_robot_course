#!/usr/bin/env python3
"""Fit a plume model to the samples of collect_plume_samples.py: per-rotor lift loss dF_k = -A (0.4/dz)^n exp(-rho_k^2 / (2 s^2)), rho_k = horizontal distance of rotor k to the top drone's axis;
predicts fz_dist = -sum dF_k ... (sign: fz_dist = weight - thrust = extra thrust needed < 0), torque need = -(sum r_k x dF_k). Robust least squares on quasi-steady windows.
  ~/.pyenv/versions/flying_robots/bin/python experiments/fbl/sysid/fit_plume_torque.py"""
import numpy as np, pandas as pd, json
from scipy.optimize import least_squares
S = pd.read_csv('experiments/fbl/sysid/out/plume_samples.csv') if False else pd.read_csv(__import__('pathlib').Path(__file__).parent / 'out/plume_samples.csv')
ARM = 0.70710678 * 0.05; R = np.array([[ARM, -ARM], [-ARM, -ARM], [-ARM, ARM], [ARM, ARM]])
S['h'] = -S.dz; q = S[(S.az.abs() < 0.3) & (S.alpha_x.abs() < 6) & (S.alpha_y.abs() < 6) & (S.h > 0.15) & (S.h < 0.8) & (np.hypot(S.dx, S.dy) < 0.6)].reset_index(drop=True)
print(len(q), 'quasi-steady samples,', q.flight.nunique(), 'flights')
def predict(p, d):
    A, s, n, tp_ = p; dF = np.zeros((len(d), 4))
    for k in range(4):      # plume axis trails the moving top drone by tp_ seconds
        rho2 = (d.dx.values + R[k, 0] + tp_ * d.tvx.values) ** 2 + (d.dy.values + R[k, 1] + tp_ * d.tvy.values) ** 2
        dF[:, k] = -A * (0.4 / d.h.values) ** n * np.exp(-rho2 / (2 * s ** 2))
    fz = dF.sum(1); tx = (dF * R[:, 1]).sum(1); ty = -(dF * R[:, 0]).sum(1)
    return fz, tx, ty      # fz_dist = weight - thrust = sum of the rotor lift changes (< 0); tau_x = sum y_k dF_k; tau_y = -sum x_k dF_k (the disturbance, = -(rpm torque))
def resid(p, d, wz=0.02, wt=2e-4):
    fz, tx, ty = predict(p, d)
    return np.r_[(d.fz_dist.values - fz) / wz, (d.tau_x.values - tx) / wt, (d.tau_y.values - ty) / wt]
best = None
for A0 in (0.02, 0.05, 0.1):
    for s0 in (0.05, 0.1, 0.2):
        r = least_squares(resid, [A0, s0, 1.5, 0.0], args=(q,), loss='soft_l1', bounds=([0.0, 0.01, 0.0, -1.0], [1.0, 1.0, 5.0, 2.0]))
        if best is None or r.cost < best.cost: best = r
A, s, n, tpl = best.x; print('plume fit: A %.4f N per rotor at dz 0.4, sigma %.3f m, n %.2f, trail %.2f s' % (A, s, n, tpl))
fz, tx, ty = predict(best.x, q)
for name, meas, pred in (('fz_dist [N]', q.fz_dist.values, fz), ('tau_x [uNm]', q.tau_x.values * 1e6, tx * 1e6), ('tau_y [uNm]', q.tau_y.values * 1e6, ty * 1e6)):
    c = np.corrcoef(meas, pred)[0, 1]; k = (pred @ meas) / (pred @ pred); print('%-12s corr %.2f  slope meas/pred %.2f  rms meas %.0f  rms resid %.0f' % (name, c, k, np.sqrt((meas ** 2).mean()), np.sqrt(((meas - pred) ** 2).mean())))
# held-out check: fit on half of the flights, test on the other half
fl = np.array(q.flight.unique(), dtype=object); rng = np.random.default_rng(0); rng.shuffle(fl); tr = q[q.flight.isin(fl[:len(fl) // 2])]; te = q[~q.flight.isin(fl[:len(fl) // 2])]
r2 = least_squares(resid, best.x, args=(tr,), loss='soft_l1', bounds=([0.0, 0.01, 0.0, -1.0], [1.0, 1.0, 5.0, 2.0])); fz, tx, ty = predict(r2.x, te)
print('half/half: params', r2.x.round(3), ' test corr fz %.2f tau_x %.2f tau_y %.2f' % tuple(np.corrcoef(m, p)[0, 1] for m, p in ((te.fz_dist.values, fz), (te.tau_x.values, tx), (te.tau_y.values, ty))))
json.dump(dict(A=A, sigma=s, n=n, trail=tpl, n_samples=len(q), n_flights=int(q.flight.nunique())), open(__import__('pathlib').Path(__file__).parent / 'out/plume_fit.json', 'w'))
