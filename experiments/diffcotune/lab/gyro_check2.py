#!/usr/bin/env python3
"""Stage 4b item 2, second part (after gyro_check.py): (i) logged gyro against the S0 model body rate directly: correlation per axis, best lag (+-100 ms), dominant frequency of gyro_x
(logged, model), and J_w if the model signal is shifted by the best lag; (ii) model side: S0 model body rate against frsim (state trace, noise off, default gains, nominal plant, 1 ms ticks ->
the model's 10 ms output grid) for A8 and A1: sign / axis by correlation and max difference, and the derived-from-quaternion test on the model quaternion.
Output lab/results/gyro_check2.json."""
import csv, json, os, sys
from pathlib import Path
import numpy as np
os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / 'dryrun'))
import torch
torch.set_num_threads(2)
import usd_ingest as U, ingest, costs as C, s0_model as S, reference as R  # noqa
import test_ingest_oct23 as T, gyro_check as G  # noqa
from frsim import config  # noqa
from frsim.episode import EpisodeConfig, run_episode  # noqa
import loop_lib as L  # noqa


def dom_freq(a, dt):
    a = a - a.mean(); f = np.fft.rfftfreq(len(a), dt); P = np.abs(np.fft.rfft(a * np.hanning(len(a)))) ** 2; P[f < 0.3] = 0
    return float(f[P.argmax()])


if __name__ == '__main__':
    rows = [r for r in csv.DictReader(open(T.PF)) if r['bank'] == 'bank_unseen_S0_2026-10-23']; out = dict(vs_model=[], model_vs_frsim=[])
    for r in rows:
        sc = r['scenario']; cell = T.cell_of(sc, r); f = T.BANK / r['folder'] / (r['folder'] + '_merged_usd.csv')
        g = U.ingest_flight(f, sc, cell, top_from_log=sc in ('A1', 'A8')); ref = g['ref']; tr = g['trace']
        with torch.no_grad():
            o = S.rollout(ref, np.zeros((1, 10)), np.zeros((1, 4)), compile=False)
        tgt = ingest.on_grid(ingest.from_trace(tr), o['t']); wm = o['w'][:, 0, :].numpy(); wl = tgt.w; tm = np.asarray(o['t']); dt = float(np.median(np.diff(tm)))
        wn = np.asarray(C.window(o['t'], ref.scenario_s, 'sysid'), bool); res = dict(folder=r['folder'], scenario=sc)
        res['corr_axes'] = [float(np.corrcoef(wl[wn, j], wm[wn, j])[0, 1]) for j in range(3)]
        best = (np.inf, 0)
        for k in range(-10, 11):                       # +-100 ms in 10 ms steps
            a = np.roll(wm, k, axis=0); e = float(((a - wl)[wn] ** 2).sum(-1).mean()); best = min(best, (e, k))
        res['Jw'] = float(((wm - wl)[wn] ** 2).sum(-1).mean()); res['Jw_best_shift'] = best[0]; res['best_shift_ms'] = 10 * best[1]
        res['dom_freq_hz_logged_gyro_x'] = dom_freq(wl[wn, 0], dt); res['dom_freq_hz_model_wx'] = dom_freq(wm[wn, 0], dt)
        res['dom_freq_hz_logged_gyro_y'] = dom_freq(wl[wn, 1], dt); res['dom_freq_hz_model_wy'] = dom_freq(wm[wn, 1], dt)
        out['vs_model'].append(res)
        print('%-24s corr %s | Jw %.2f, best shift %d ms -> %.2f | dom freq gx %.2f / model %.2f Hz, gy %.2f / %.2f' % (r['folder'], np.round(res['corr_axes'], 2).tolist(), res['Jw'], res['best_shift_ms'], res['Jw_best_shift'],
              res['dom_freq_hz_logged_gyro_x'], res['dom_freq_hz_model_wx'], res['dom_freq_hz_logged_gyro_y'], res['dom_freq_hz_model_wy']), flush=True)
    # model side: S0 model against frsim, noise off, default gains, nominal plant
    for sc, cell in (('A8', study_cell) for study_cell in [None]):
        pass
    from frsim.scenarios import study_cells
    for sc in ('A8', 'A1'):
        cell = study_cells(sc)[0]; ref = R.build(sc, cell)
        r_ = run_episode(EpisodeConfig('S0', sc, cell, L.gains_of(np.zeros(10)), 'nominal', 0, plant=config.plant_with({}), noise=False, record_state=True, record_every=10))
        trf = r_.state_trace
        with torch.no_grad():
            o = S.rollout(ref, np.zeros((1, 10)), np.zeros((1, 4)), compile=False)
        tm = np.asarray(o['t']); wm = o['w'][:, 0, :].numpy(); qm = o['q'][:, 0, :].numpy()
        wf = np.stack([np.interp(tm, trf[:, 0], trf[:, 11 + j]) for j in range(3)], 1)
        m = (tm > 0) & (tm <= ref.scenario_s); dt = float(np.median(np.diff(tm)))
        d = G.omega_from_q(qm, dt, k=2)
        res = dict(scenario=sc, rms_model=float(np.sqrt((wm[m] ** 2).sum(-1).mean())), rms_frsim=float(np.sqrt((wf[m] ** 2).sum(-1).mean())), max_abs_diff_rad_s=float(np.abs(wm - wf)[m].max()),
                   corr_axes=[float(np.corrcoef(wm[m, j], wf[m, j])[0, 1]) for j in range(3)], corr_model_w_vs_model_q_derivative=[float(np.corrcoef(wm[m, j], d[m, j])[0, 1]) for j in range(3)],
                   slope_model_w_on_q_derivative=[float((wm[m, j] * d[m, j]).sum() / (d[m, j] ** 2).sum()) for j in range(3)])
        out['model_vs_frsim'].append(res); print('model vs frsim', json.dumps(res), flush=True)
    json.dump(out, open(HERE / 'results' / 'gyro_check2.json', 'w'), indent=1)
