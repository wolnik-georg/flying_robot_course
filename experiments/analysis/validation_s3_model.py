#!/usr/bin/env python3
"""Validation of the S3 versions (2026-10-18), part 3: a vertical-axis model of the controller, driven by the MEASURED
push and the LOGGED NS2 prediction of the A8 crossings, to reconcile the hardware dips of S0, S1, S2 and S3 and to
predict Version B.

Model (500 Hz, z only, the controller's equations):
  plant      z'' = f/m - g + s_p * p(t)               p = crossing-mean measured residual (offline, 30 Hz-filtered,
                                                      the controller's definition), s_p = push scale
  motors     f follows gamma * T_cmd through a first-order lag tau_m plus a pure delay d_m
  INDI       a_rpm = f(t - d_rpm)/m - g ; a_imu = z''(t) ; both through 2nd-order Butterworth 30 Hz (500 Hz)
             a_indi = f30(a_rpm) - f30(a_imu)          (switch A); Version B adds f30(a_nn)
  NS2        a_nn = s_n * pred(t + lead)               crossing-mean logged prediction, zero-order hold at 100 Hz
  position   a_d = g + Kp e + Kd e' + Ki int(e) - a_nn (NS2 on);  e = z_ref - z;  Kp 7, Kd 4, Ki 1.5, |int e| <= 2
  command    T_cmd = m (a_d + a_indi)
The deconvolution caveat: p is measured with the 30 Hz filter; the true push is slightly sharper. Scales s_p, s_n and the
motor parameters are fitted to the four measured mean dips and peaks; the fit is coarse (grid search), the point is
which mechanism can reproduce all four at once, not exact numbers.
Reads out/validation_s3/events.npz. Writes out/validation_s3/model_fit.csv and prints the A/B predictions.
"""
import itertools, os
import numpy as np
import pandas as pd
from scipy.signal import butter

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
OUT = os.path.join(ROOT, 'experiments', 'analysis', 'out', 'validation_s3')
G, M, DT = 9.81, 0.0427, 0.002
KP, KD, KI, ILIM = 7.0, 4.0, 1.5, 2.0
B, A = butter(2, 30.0, fs=500.0)
MEAS = {'S0': (-15.70, -0.48), 'S1': (-10.62, -0.80), 'S2': (-5.53, 0.59), 'S3': (-1.87, 1.54)}  # dip, peak (cm), crossings pooled


class Bw:
    def __init__(self):
        self.x = [0.0, 0.0]; self.y = [0.0, 0.0]
    def __call__(self, u):
        y = B[0]*u + B[1]*self.x[0] + B[2]*self.x[1] - A[1]*self.y[0] - A[2]*self.y[1]
        self.x = [u, self.x[0]]; self.y = [y, self.y[0]]
        return y


def simulate(p_t, n_t, indi, ns2, mode_b, tau_m=0.03, d_m=0.0, d_rpm=0.0, gamma=1.0, noise=0.0, seed=0, d_imu=0.0, eta=1.0):
    """p_t, n_t: push and prediction on the 500 Hz grid (m/s^2). Returns z error [cm] array."""
    rng = np.random.default_rng(seed)
    n = len(p_t); z = v = ie = 0.0
    f = M*G; f_hist = [f]*(int(round(max(d_m, d_rpm)/DT))+2)
    T_hist = [M*G/gamma]*(int(round(d_m/DT))+2)
    fr, fi, fn = Bw(), Bw(), Bw()
    # warm the filters at the hover state
    for _ in range(200):
        fr(0.0); fi(0.0); fn(0.0)
    acc_hist = [0.0]*(int(round(d_imu/DT))+2); out = np.empty(n); hold = 0.0
    for k in range(n):
        e = -z; ie = np.clip(ie + e*DT, -ILIM, ILIM)
        if k % 5 == 0:                       # NS2 evaluated every 10th 1 kHz tick = every 5th controller step
            hold = n_t[k]
        a_d = G + KP*e + KD*(-v) + KI*ie - (hold if ns2 else 0.0)
        a_indi = 0.0
        if indi:
            f_meas = f_hist[-1 - int(round(d_rpm/DT))]
            a_rpm = f_meas/M - G
            a_imu = acc_hist[-1 - int(round(d_imu/DT))] + noise*rng.standard_normal()
            a_indi = eta*(fr(a_rpm) - fi(a_imu))
            an = fn(hold if ns2 else 0.0)
            if mode_b and ns2:
                a_indi += an
        T = max(M*(a_d + a_indi), 0.0)
        T_hist.append(T); T_hist.pop(0)
        T_del = T_hist[-1 - int(round(d_m/DT))]
        f += (gamma*T_del - f) * (DT/tau_m if tau_m > 0 else 1.0)
        f_hist.append(f); f_hist.pop(0)
        acc = f/M - G + p_t[k]
        v += acc*DT; z += v*DT; acc_hist.append(acc); acc_hist.pop(0)
        out[k] = -z*100.0 * -1   # z error = z - z_ref (cm)
    return out


def dip_peak(ez, w):
    seg = ez[w]
    return seg.min(), seg.max()


def main():
    ev = np.load(os.path.join(OUT, 'events.npz'))
    W = ev['W']; t = np.arange(-3.0, 3.0, DT)
    # push profile: crossing mean of the measured residual, all strategies pooled (C1/C2: the push does not depend on
    # the strategy); baseline removed; prediction: pooled crossing mean of the logged prediction
    p = np.mean([ev[f'{s}_ares'] for s in ('S0', 'S1', 'S2', 'S3')], 0)
    pn = np.mean([ev[f'{s}_pred'] for s in ('S0', 'S1', 'S2', 'S3')], 0)
    p = p - np.median(p[:15]); pn = pn - np.median(pn[:15])
    P = np.interp(t, W, p, left=0, right=0); N = np.interp(t, W, pn, left=0, right=0)
    w = np.abs(t) <= 0.8
    fam = {
        'F1 mechanistic (RPM/IMU delays, motor lag)': dict(s_p=[1.0, 1.25, 1.5, 1.75], tau_m=[0.03, 0.06], d_m=[0.0, 0.02],
                                                            d_rpm=[0.0, 0.04, 0.08, 0.12, 0.16], d_imu=[0.0, 0.03], s_n=[0.75, 1.0, 1.25], eta=[1.0]),
        'F2 phenomenological (INDI effectiveness eta)': dict(s_p=[1.0, 1.25, 1.5, 1.75], tau_m=[0.03, 0.06], d_m=[0.0, 0.02],
                                                              d_rpm=[0.0, 0.04], d_imu=[0.0], s_n=[0.75, 1.0, 1.25], eta=[0.2, 0.35, 0.5, 0.7]),
    }
    best_by = {}
    for fname, grid in fam.items():
        rows = []
        for vals in itertools.product(*grid.values()):
            kw = dict(zip(grid.keys(), vals)); s_p, s_n = kw.pop('s_p'), kw.pop('s_n')
            res = {s: dip_peak(simulate(s_p*P, s_n*N, indi, ns2, False, **kw), w)
                   for s, (indi, ns2) in {'S0': (0, 0), 'S1': (1, 0), 'S2': (0, 1), 'S3': (1, 1)}.items()}
            err = sum((res[s][0]-MEAS[s][0])**2 + 0.5*(res[s][1]-MEAS[s][1])**2 for s in MEAS)
            rows.append(dict(family=fname, s_p=s_p, s_n=s_n, **kw, err=err,
                             **{f'{s}_dip': res[s][0] for s in res}, **{f'{s}_peak': res[s][1] for s in res}))
        df = pd.DataFrame(rows).sort_values('err'); best_by[fname] = df
        df.to_csv(os.path.join(OUT, 'model_fit_' + fname.split()[0] + '.csv'), index=False)
        pd.set_option('display.width', 250)
        print('\n' + fname + ' -- measured (dip, peak cm):', MEAS)
        print(df.head(5).drop(columns=['family']).round(3).to_string(index=False))
        b = df.iloc[0]
        kw = dict(tau_m=b.tau_m, d_m=b.d_m, d_rpm=b.d_rpm, d_imu=b.d_imu, eta=b.eta)
        Pp, Nn = b.s_p*P, b.s_n*N
        print('best fit, all variants (dip / peak cm):')
        for lab, (indi, ns2, mb, nsc) in {'S0': (0, 0, 0, 1), 'S1': (1, 0, 0, 1), 'S2': (0, 1, 0, 1), 'S3 A': (1, 1, 0, 1), 'S3 B': (1, 1, 1, 1),
                                          'S3 A, NS2 x0': (1, 1, 0, 0), 'S3 B, NS2 x0': (1, 1, 1, 0),
                                          'S3 A, NS2 x-1': (1, 1, 0, -1), 'S3 B, NS2 x-1': (1, 1, 1, -1),
                                          'S3 A, NS2 x1.3': (1, 1, 0, 1.3), 'S3 B, NS2 x1.3': (1, 1, 1, 1.3)}.items():
            dp = dip_peak(simulate(Pp, nsc*Nn, indi, ns2, mb, **kw), w)
            print(f'  {lab:15s} {dp[0]:+6.2f} / {dp[1]:+5.2f}')
    b = best_by[list(fam)[0]].iloc[0]
    kw = dict(tau_m=b.tau_m, d_m=b.d_m, d_rpm=b.d_rpm, d_imu=b.d_imu, eta=b.eta)
    Pp, Nn = b.s_p*P, b.s_n*N
    # ideal-INDI reference (C1 as stated): no motor lag, no delay
    print('\nideal INDI (tau_m -> 0.002 s, no delays), same push/prediction:')
    for lab, (indi, ns2, mb) in {'S1': (1, 0, 0), 'S3 A': (1, 1, 0), 'S3 B': (1, 1, 1)}.items():
        dp = dip_peak(simulate(Pp, Nn, indi, ns2, mb, tau_m=0.002), w)
        print(f'  {lab:6s} {dp[0]:+6.2f} / {dp[1]:+5.2f}')
    # quasi-static check (A1-like): constant push of the A1 size, held 10 s
    t2 = np.arange(0, 12, DT); Pc = np.where(t2 > 1, -2.9, 0.0); Nc = np.where(t2 > 1, -2.35, 0.0)
    print('\nA1-like step (push -2.9, prediction -2.35 from t = 1 s): z error at 1.5 / 3 / 11 s [cm], max up [cm]')
    for lab, (indi, ns2, mb) in {'S1': (1, 0, 0), 'S3 A': (1, 1, 0), 'S3 B': (1, 1, 1)}.items():
        ez = simulate(Pc, Nc, indi, ns2, mb, **kw)
        print(f'  {lab:6s} {ez[int(1.5/DT)]:+6.2f} {ez[int(3/DT)]:+6.2f} {ez[int(11/DT)]:+6.2f}  max {ez.max():+6.2f}')


if __name__ == '__main__':
    main()
