#!/usr/bin/env python3
"""Stage 8 step 1b (S0, plant P1, true beta, theta_0, 5 tuning scenarios): why the body rates break J_sysId.
 A  noise-free truth against the noise-free model:   J_sysId parts p, v, q, w of (frsim flight without sensor noise) against (model rollout at the true beta)  = the model-mismatch floor
 B  noisy truth against the noise-free model:        the same for two noisy frsim flights (noise seeds 1, 2)                                                  = what a fit to noisy data cannot remove
 C  two noise seeds against each other:              parts of (noisy flight 1) against (noisy flight 2), no model                                            = the noise floor of the target itself
 and for A1, A8, A4: the cross-correlation of the body rates of the two noisy flights (lag 0 and best lag within +-1 s), the dominant frequency of the rate, and the RMS body rate of the noise-free truth against the noisy flights.
Units: p m^2, v (m/s)^2, q (unit quaternion difference)^2, w (rad/s)^2; mean over the sysId window (0 < t <= scenario end) as costs.j_sysid.  Usage (job_runner): python3 floors.py -> results/floors.json"""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent))
import loop_lib as L  # noqa
import ingest  # noqa
def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)
def parts_traces(t1, t2, scen_s):
    a = ingest.from_trace(t1); b = ingest.on_grid(ingest.from_trace(t2), a.t); m = (a.t > 0) & (a.t <= scen_s)
    dq = a.q - np.sign((a.q * b.q).sum(-1, keepdims=True) + (((a.q * b.q).sum(-1, keepdims=True)) == 0)) * b.q
    f = lambda d: float((d[m] ** 2).sum(-1).mean())
    return dict(p=f(a.p - b.p), v=f(a.v - b.v), q=f(dq), w=f(a.w - b.w))
def xcorr(a, b, fs, maxlag_s=1.0):
    a = a - a.mean(); b = b - b.mean(); n = len(a); out = {}
    c0 = float((a * b).sum() / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-300)); best = (c0, 0.0)
    for lag in range(-int(maxlag_s * fs), int(maxlag_s * fs) + 1):
        x, y = (a[lag:], b[:n - lag]) if lag >= 0 else (a[:n + lag], b[-lag:])
        c = float((x * y).sum() / (np.linalg.norm(x) * np.linalg.norm(y) + 1e-300))
        if abs(c) > abs(best[0]): best = (c, lag / fs)
    return dict(lag0=c0, best=best[0], best_lag_s=best[1])
if __name__ == '__main__':
    plant = L.PLANTS['P1']; gains = L.gains_of(np.zeros(10)); x_true = np.r_[np.zeros(10), L.PSI_TRUE]
    pool = L.Pool(len(L.TUNING), states=('p', 'v', 'q', 'w'))
    try:
        jobs_clean = [(s, c, gains, plant, 0) for s, c in L.TUNING]
        clean = pool.pool.map(L.worker_flyclean, jobs_clean, chunksize=1)
        noisy = [pool.pool.map(L.worker_fly, [(s, c, gains, plant, seed) for s, c in L.TUNING], chunksize=1) for seed in (1, 2)]
        out = dict(scenarios=[s for s, _ in L.TUNING], A={}, B={}, C={}, rates={})
        for i, (s, c) in enumerate(L.TUNING):
            ref = L.R.build(s, c); sc = ref.scenario_s; F0 = clean[i][0]; N1 = noisy[0][i][0]; N2 = noisy[1][i][0]
            pa = pool.pool.map(L.worker_parts, [(s, c, x_true, F0), (s, c, x_true, N1), (s, c, x_true, N2)], chunksize=1)
            out['A'][s] = pa[0][1]; out['B'][s] = {'seed1': pa[1][1], 'seed2': pa[2][1]}; out['C'][s] = parts_traces(N1, N2, sc)
            a = ingest.from_trace(N1); b = ingest.from_trace(N2); f = ingest.from_trace(F0); m = (a.t > 0) & (a.t <= sc); fs = 1.0 / float(np.median(np.diff(a.t)))
            wn = a.w[m]; wn2 = b.w[m]; w0 = np.interp(a.t[m], f.t, f.w[:, 0])[:, None] if False else np.stack([np.interp(a.t[m], f.t, f.w[:, j]) for j in range(3)], -1)
            rr = {}
            for j, nm in enumerate('xyz'):
                fr_ = np.fft.rfftfreq(len(wn), 1 / fs); P = np.abs(np.fft.rfft(wn[:, j] - wn[:, j].mean())) ** 2; k = int(np.argmax(P[1:])) + 1
                rr[nm] = dict(rms_noisy1=float(np.sqrt((wn[:, j] ** 2).mean())), rms_noisy2=float(np.sqrt((wn2[:, j] ** 2).mean())), rms_noisefree=float(np.sqrt((w0[:, j] ** 2).mean())), dominant_hz_noisy1=float(fr_[k]),
                              xcorr_noisy1_noisy2=xcorr(wn[:, j], wn2[:, j], fs), xcorr_noisy1_noisefree=xcorr(wn[:, j], w0[:, j], fs))
            out['rates'][s] = dict(fs=fs, axes=rr, ref_ms_v=float((ref.vel ** 2).sum(-1).mean()))
            say(s, 'A', {k: round(v, 5) for k, v in out['A'][s].items()}, 'B1', {k: round(v, 5) for k, v in out['B'][s]['seed1'].items()}, 'C', {k: round(v, 5) for k, v in out['C'][s].items()})
        json.dump(out, open(HERE / 'results' / 'floors.json', 'w'), indent=1); say('done')
    finally:
        pool.close()
