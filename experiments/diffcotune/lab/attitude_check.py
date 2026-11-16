#!/usr/bin/env python3
"""Stage 4b item 5.2: (a) attitude part of J_sysId on the 23 Oct flights: roll / pitch of the log (est_q of the bottom vehicle, ingest trace) against the S0 model (beta_0, default gains) over the scored window:
correlation, best lag (|lag| <= 0.5 s), dominant frequency, rms; (b) real tilt per flight over the scored window only (bottom and top vehicle, logged roll_deg / pitch_deg) and the abort rule of
docs/comparative_study_protocol.md S4 (|roll| or |pitch| > 25 deg for 0.5 s: longest continuous run in the scored window, and whether the rule would have fired). Output lab/results/attitude_check.json. Desk job, one process."""
import csv, json, os, sys
from pathlib import Path
import numpy as np
os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
HERE = Path(__file__).resolve().parent; ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import torch
torch.set_num_threads(2)
import usd_ingest as U, ingest, s0_model as S, test_ingest_oct23 as T  # noqa
import pandas as pd


def rp(q):
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]       # quaternion order of the traces and of the model is (x, y, z, w)
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))); pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    return roll, pitch


def best_lag(a, b, dt, maxlag_s=0.5):
    a = a - a.mean(); b = b - b.mean(); n = int(maxlag_s / dt); best = (-9, 0)
    for k in range(-n, n + 1, 2):
        x, y = (a[k:], b[:len(b) - k]) if k >= 0 else (a[:k], b[-k:])
        c = float(np.corrcoef(x, y)[0, 1]); best = max(best, (c, k * dt))
    return best


def dom_freq(a, dt):
    a = a - a.mean(); f = np.fft.rfftfreq(len(a), dt); P = np.abs(np.fft.rfft(a * np.hanning(len(a)))) ** 2; P[f < 0.3] = 0
    return float(f[np.argmax(P)])


def longest_run(mask, dt):
    best = cur = 0
    for m in mask:
        cur = cur + 1 if m else 0; best = max(best, cur)
    return best * dt


MODE = sys.argv[sys.argv.index('--top') + 1] if '--top' in sys.argv else 'auto'     # as round.py: auto = logged top path for A1 / A8

if __name__ == '__main__':
    rows = [r for r in csv.DictReader(open(T.PF)) if r['bank'] == 'bank_unseen_S0_2026-10-23']; out = []
    for r in rows:
        sc = r['scenario']; cell = T.cell_of(sc, r); f = T.BANK / r['folder'] / (r['folder'] + '_merged_usd.csv')
        g = U.ingest_flight(f, sc, cell, top_from_log=(MODE == 'logged' or (MODE == 'auto' and sc in ('A1', 'A8')))); ref = g['ref']; tr = g['trace']
        with torch.no_grad():
            o = S.rollout(ref, np.zeros((1, 10)), np.zeros((1, 4)), compile=False)
        tm = np.asarray(o['t']); dt = float(np.median(np.diff(tm))); m = (tm > 0) & (tm <= ref.scenario_s)
        qm = o['q'][:, 0, :].numpy(); tt = tr[:, 0]
        ql = np.stack([np.interp(tm, tt, tr[:, 7 + j]) for j in range(4)], 1)
        rl, pl = rp(ql); rm, pm = rp(qm); res = dict(top_mode=MODE, folder=r['folder'], scenario=sc, cell=cell, frame_theta_deg=g['frame']['theta_deg'])
        tl_ = np.degrees(np.arccos(np.clip(1 - 2 * (ql[:, 0] ** 2 + ql[:, 1] ** 2), -1, 1))); tm_ = np.degrees(np.arccos(np.clip(1 - 2 * (qm[:, 0] ** 2 + qm[:, 1] ** 2), -1, 1)))
        res['tilt_angle_rms_deg'] = dict(real=float(np.sqrt((tl_[m] ** 2).mean())), model=float(np.sqrt((tm_[m] ** 2).mean())), real_max=float(tl_[m].max()), model_max=float(tm_[m].max()))
        for name, a, b in (('roll', rl, rm), ('pitch', pl, pm)):
            c, lag = best_lag(a[m], b[m], dt)
            res[name] = dict(rms_real_deg=float(np.sqrt(((a[m]) ** 2).mean())), rms_model_deg=float(np.sqrt(((b[m]) ** 2).mean())), corr_zero_lag=float(np.corrcoef(a[m], b[m])[0, 1]) if a[m].std() > 1e-6 and b[m].std() > 1e-6 else None,
                             best_corr=c, best_lag_s=lag, f_dom_real_hz=dom_freq(a[m], dt), f_dom_model_hz=dom_freq(b[m], dt) if b[m].std() > 1e-6 else None)
        df = pd.read_csv(f, comment='#'); t = df['t'].values; sm = (t >= 2.0) & (t <= ref.scenario_s) if False else (t > 0) & (t <= ref.scenario_s)
        tdt = float(np.median(np.diff(t)))
        for veh in ('cf5_unseen', 'cf_second_unseen'):
            ro = df[veh + '.roll_deg'].values; pi = df[veh + '.pitch_deg'].values; tilt = np.maximum(abs(ro), abs(pi))
            res[veh] = dict(scored_window_s=[float(t[sm][0]), float(t[sm][-1])], max_abs_roll_deg=float(abs(ro[sm]).max()), max_abs_pitch_deg=float(abs(pi[sm]).max()), p99_tilt_deg=float(np.percentile(tilt[sm], 99)),
                            frac_over_25=float((tilt[sm] > 25).mean()), longest_run_over_25_s=longest_run(tilt[sm] > 25, tdt), rule_fires_0p5s=bool(longest_run(tilt[sm] > 25, tdt) >= 0.5),
                            first_time_rule_fires_s=None, whole_flight_longest_run_over_25_s=longest_run(tilt > 25, tdt), max_abs_gyro_dps=float(max(abs(df[veh + '.gyro_%s' % a].values[sm]).max() for a in 'xy')))
            if res[veh]['rule_fires_0p5s']:
                run = 0
                for i in np.where(sm)[0]:
                    run = run + 1 if tilt[i] > 25 else 0
                    if run * tdt >= 0.5: res[veh]['first_time_rule_fires_s'] = float(t[i]); break
        out.append(res)
        print(r['folder'], 'roll', {k: (round(v, 3) if isinstance(v, float) else v) for k, v in res['roll'].items()}, '\n   pitch', {k: (round(v, 3) if isinstance(v, float) else v) for k, v in res['pitch'].items()},
              '\n   bottom', {k: (round(v, 3) if isinstance(v, float) else v) for k, v in res['cf5_unseen'].items()}, '\n   top', {k: (round(v, 3) if isinstance(v, float) else v) for k, v in res['cf_second_unseen'].items()}, flush=True)
    json.dump(out, open(HERE / 'results' / ('attitude_check_%s.json' % MODE), 'w'), indent=1)
