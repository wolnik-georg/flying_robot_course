#!/usr/bin/env python3
"""Stage F2b.2 / F2b.1: read-only statistics of the complete uSD logs (merged_usd.csv of the training banks in the lab checkout, never radio CSVs) of the FBL flights (banks bank_fbl*) and, for the noise comparison,
of S0 A1 hover flights (bank_pilot*, bank_a1clean). Bottom drone = cf5 columns. Window = scenario time in (SCORE_AFTER_S, scenario_s] of the frsim A1 plan (t = 0 is scenario start, '# meta:t_zero=scenario_start').
High-frequency noise estimate of a channel = std(diff)/sqrt(2) in the window (includes real motion, so it is an upper bound; the same estimator is applied to frsim traces by the caller).
  /usr/bin/python3 fbl/real_flights_f2b.py -> fbl/real_flights_f2b.json"""
import glob, json, os, sys
from pathlib import Path
import numpy as np, pandas as pd
HERE = Path(__file__).resolve().parent; LOGS = Path.home() / 'Desktop/flying_robot_course/experiments/logs'
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent.parent.parent / 'simulator'))
from frsim import config
from frsim.flightplan import FlightPlan
from frsim.scenarios import study_cells, Motion
SC = {'A1': 11.0}
def hf(x):  # std of first difference / sqrt 2
    return float(np.std(np.diff(x)) / np.sqrt(2))
def stats(f, scen_s):
    d = pd.read_csv(f, comment='#'); pre = sorted({c.split('.')[0] for c in d.columns if c.startswith('cf5')})[0]
    g = lambda n: d[pre + '.' + n].to_numpy(float); t = d['t'].to_numpy(float)
    w = (t > config.SCORE_AFTER_S) & (t <= scen_s)
    if w.sum() < 100: return dict(n_window=int(w.sum()), note='window too short')
    qx, qy = g('est_qx'), g('est_qy'); tilt = np.degrees(np.arccos(np.clip(1 - 2 * (qx ** 2 + qy ** 2), -1, 1)))
    e = np.stack([g('x') - g('ctrltarget_x'), g('y') - g('ctrltarget_y'), g('z') - g('ctrltarget_z')], 1)
    gy = np.stack([g('gyro_x'), g('gyro_y'), g('gyro_z')], 1)
    out = dict(n_window=int(w.sum()), t_end_s=float(t[-1]), tilt_max_deg=float(tilt[w].max()), tilt_rms_deg=float(np.sqrt(np.mean(tilt[w] ** 2))), pos3d_rms_cm=float(100 * np.sqrt(np.mean(np.sum(e[w] ** 2, 1)))),
               z_rms_cm=float(100 * np.sqrt(np.mean(e[w, 2] ** 2))), gyro_rms_dps=[float(np.sqrt(np.mean(gy[w, i] ** 2))) for i in range(3)], gyro_hf_dps=[hf(gy[w, i]) for i in range(3)],
               pos_hf_mm=[1000 * hf(g(a)[w]) for a in ('x', 'y', 'z')], vel_hf_mm_s=[1000 * hf(g(a)[w]) for a in ('vx', 'vy', 'vz')], locsrv_hf_mm=[1000 * hf(g('locsrv_' + a)[w]) for a in ('x', 'y', 'z')],
               dt_median_s=float(np.median(np.diff(t))))
    for k in ('fbl_active', 'fbl_tripped', 'fbl_faults'):
        if pre + '.' + k in d.columns: out[k + '_max'] = float(np.nanmax(g(k)))
    out['fbl_active_pct'] = float(100 * np.mean(g('fbl_active')[w])) if pre + '.fbl_active' in d.columns else None
    return out
if __name__ == '__main__':
    plan = FlightPlan(Motion(**study_cells('A1')[0])) if False else None
    from frsim.scenarios import study_cells as sc
    import reference as R
    scen_s = R.build('A1', study_cells('A1')[0] if False else None).scenario_s if False else None
    res = []
    banks = sorted(glob.glob(str(LOGS / 'bank_fbl*'))) + sorted(glob.glob(str(LOGS / 'bank_pilot*'))) + [str(LOGS / 'bank_a1clean_2026-10-19')]
    import forward_check_fbl as FC
    scen_s = R.build('A1', FC.CELLS['A1'][1]).scenario_s
    for b in banks:
        m = json.load(open(b + '/training_manifest.json'))
        for fl in m['flights']:
            if fl['scenario'] != 'A1' and 'fbl' not in os.path.basename(b): continue
            csv = glob.glob(os.path.join(os.path.dirname(b), os.path.basename(fl['path']).join(['', ''])))  # unused
            p = LOGS.parent.parent / fl['path']
            if not p.exists(): res.append(dict(bank=os.path.basename(b), folder=fl['folder'], scenario=fl['scenario'], missing=True)); continue
            st = stats(p, scen_s if fl['scenario'] == 'A1' else None) if fl['scenario'] == 'A1' else dict(note='not A1: only listed')
            res.append(dict(bank=os.path.basename(b), folder=fl['folder'], scenario=fl['scenario'], cf5=fl.get('cf5'), **st))
    json.dump(res, open(HERE / 'real_flights_f2b.json', 'w'), indent=1)
    for r in res: print(r['bank'][:28].ljust(28), r['folder'][:22], r['scenario'], {k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items() if k in ('tilt_max_deg', 'tilt_rms_deg', 'pos3d_rms_cm', 'fbl_tripped_max', 'fbl_active_pct', 'note', 'missing')})
