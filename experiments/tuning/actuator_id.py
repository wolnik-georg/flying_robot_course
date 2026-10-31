#!/usr/bin/env python3
"""Motor command -> measured rpm (bottom drone cf5, DShot rpm_m*) from the real flights of all controllers, per flight and motor.
Model: F_meas = g * LPF_tau( (cmd/65535)^2 delayed by d ), F_meas = k*(rpm*2pi/60)^2. Grid over (tau, d), g by least squares; reports tau, d, g, R2.
Use:  python3 experiments/tuning/actuator_id.py   (pyenv or system python with numpy/pandas)  -> out/actuator_id.csv, summary on stdout."""
import glob, re, sys
import numpy as np, pandas as pd
from pathlib import Path
LOGS = Path(__file__).resolve().parents[1] / 'logs'
DT = 0.002
TAUS = [4, 8, 12, 16, 20, 25, 30, 35, 40, 45, 50, 60, 70, 85, 100]; DLYS = list(range(0, 31, 2))      # ms; delay in samples of 2 ms
STRAT = [('bank_unseen_S0', 'S0'), ('bank_unseen_S1', 'S1'), ('bank_unseen_S3S2', 'S2'), ('bank_unseen_S3', 'S3'), ('bank_fbl', 'S4'), ('bank_pilot', 'pilot'), ('bank_ns2train', 'ns2train'), ('bank_a1clean', 'a1clean')]


def lpf(x, a):
    y = np.empty_like(x); s = x[0]
    for i in range(len(x)): s += a * (x[i] - s); y[i] = s
    return y


def fit_motor(cmd, F):
    best = None; u = (cmd / 65535.0) ** 2
    for tau in TAUS:
        a = DT / (tau * 1e-3 + DT); y = lpf(u, a)
        for d in DLYS:
            k = d // 2; yy = y[:len(y) - k] if k else y; ff = F[k:]
            x = yy - yy.mean(); z = ff - ff.mean(); sxx = x @ x
            if sxx <= 0: continue
            g = (x @ z) / sxx; r2 = 1 - ((z - g * x) ** 2).sum() / (z @ z)
            if best is None or r2 > best[0]: best = (r2, tau, d, g)
    return best


rows = []
for f in sorted(glob.glob(str(LOGS / 'bank_*' / '*' / '*_merged_usd.csv'))):
    bank = Path(f).parents[1].name; lab = next((l for p, l in STRAT if bank.startswith(p)), None)
    if lab is None: continue
    df = pd.read_csv(f, comment='#'); pre = next((c.split('.')[0] for c in df.columns if c.startswith('cf5') and c.endswith('.rpm_m1')), None)
    if pre is None or f'{pre}.motor_m1' not in df.columns: continue
    z = df[[c for c in df.columns if c.endswith('.z') and c.startswith(pre)] or [f'{pre}.z']].iloc[:, 0].values if f'{pre}.z' in df.columns else None
    t = df['t'].values; ok = np.ones(len(t), bool) if z is None else (z > 0.2)
    if ok.sum() < 3000: continue
    i0, i1 = np.where(ok)[0][[0, -1]]; sl = slice(i0, i1)
    if np.median(np.diff(t[sl])) > 0.0025: continue
    for m in (1, 2, 3, 4):
        cmd = df[f'{pre}.motor_m{m}'].values[sl].astype(float); w = df[f'{pre}.rpm_m{m}'].values[sl].astype(float)
        good = (w > 3000) & (w < 40000)
        if good.mean() < 0.98: continue
        F = 3.91194027330774e-08 * (w * 2 * np.pi / 60) ** 2
        r2, tau, d, g = fit_motor(cmd, F); rows.append(dict(strategy=lab, bank=bank, flight=Path(f).parent.name, motor=m, r2=r2, tau_ms=tau, delay_ms=d, gain=g))
    print(lab, Path(f).parent.name, 'done', flush=True)
R = pd.DataFrame(rows); R.to_csv(Path(__file__).parent / 'out/actuator_id.csv', index=False)
print(R.groupby('strategy')[['tau_ms', 'delay_ms', 'gain', 'r2']].agg(['median', 'count']).round(3).to_string())
print('overall median tau %.1f ms, delay %.1f ms, r2 %.2f' % (R.tau_ms.median(), R.delay_ms.median(), R.r2.median()))
