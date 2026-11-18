#!/usr/bin/env python3
"""Stage F2b.1: why FBL (frsim) crashes on A1 with sensor noise. Single process. The simulator copy is NOT edited: sensor groups are switched by scaling config.STATE_NOISE_POSITION_M /
STATE_NOISE_VELOCITY_M_S and plant.gyro_noise_deg_s (the random draws stay, so the draw order is unchanged; std 0 -> exactly zero noise), thrust is logged by wrapping vehicle.wrench.
  python3 fbl/a1_noise_diag.py MODE  (MODE: traces | table | s0)  -> fbl/a1_noise_diag_<mode>.json"""
import json, os, sys, time
from dataclasses import replace
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent))
import numpy as np
import forward_check_fbl as FC
from frsim import config, vehicle
from frsim.episode import EpisodeConfig, run_episode
P0, V0 = config.STATE_NOISE_POSITION_M, config.STATE_NOISE_VELOCITY_M_S
PLANTS = {'calibration': config.CALIBRATED_PLANT, 'P1': config.plant_with(FC.ROWS['P1'])}
_wr = vehicle.wrench; LOG = []
def _wrap(f):
    LOG.append(float(np.sum(f))); return _wr(f)
vehicle.wrench = _wrap  # total motor force (N) per 1 ms tick

def run(ctrl, var, scen, plant, seed, sp, sv, sg, noise=True):
    os.environ['FRSIM_FBL_VARIANT'] = var
    config.STATE_NOISE_POSITION_M, config.STATE_NOISE_VELOCITY_M_S = P0 * sp, V0 * sv
    p = replace(PLANTS[plant], gyro_noise_deg_s=PLANTS[plant].gyro_noise_deg_s * sg)
    LOG.clear(); t0 = time.time()
    r = run_episode(EpisodeConfig(ctrl, scen, FC.CELLS[scen][1], None, 'nominal', seed, plant=p, noise=noise, record_state=True))
    tr = r.state_trace; thr = np.array(LOG)[::10][:len(tr)]
    return r, tr, thr, time.time() - t0

def tilt_of(q):
    x, y, z, w = q.T; return np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))

def summ(r, tr):
    til = tilt_of(tr[:, 7:11]); ts = tr[:, 0]; on = np.nonzero(til > 10)[0]
    return dict(stable=bool(r.stable), crash=r.crash_time_s is not None, crash_time_s=r.crash_time_s, tilt_max_deg=float(r.tilt_max_deg), tilt_rms_deg=float(r.tilt_rms_deg), x_rms_cm=r.x_rms_cm, y_rms_cm=r.y_rms_cm, z_rms_cm=r.z_rms_cm,
                tripped=None, fbl_active_pct=float(r.fbl_active_pct), t_tilt_gt10_s=float(ts[on[0]]) if len(on) else None, completed=bool(r.completed))

if __name__ == '__main__':
    mode = sys.argv[1]; out = {}
    if mode == 'traces':
        for plant in PLANTS:
            for seed in (1000, 1001, None):
                r, tr, thr, dt = run('FBL', 'base', 'A1', plant, seed if seed else 0, 1, 1, 1, noise=seed is not None)
                k = '%s_%s' % (plant, seed if seed else 'nonoise'); out[k] = dict(summary=summ(r, tr), t=tr[:, 0].tolist(), tilt=tilt_of(tr[:, 7:11]).tolist(), w=tr[:, 11:14].tolist(), thrust_N=thr.tolist(),
                                                                                err=(tr[:, 1:4] - tr[:, 14:17]).tolist(), sec=dt); print(k, out[k]['summary'], '%.0f s' % dt, flush=True)
    elif mode == 'table':
        cfgs = [('none', 0, 0, 0)] + [('%s x%g' % (g, l), l if g == 'pos' else 0, l if g == 'vel' else 0, l if g == 'gyro' else 0) for g in ('pos', 'vel', 'gyro') for l in (1, .5, .25)] + [('all x0.5', .5, .5, .5), ('all x0.25', .25, .25, .25), ('all x1', 1, 1, 1)]
        for name, sp, sv, sg in cfgs:
            for plant in PLANTS:
                for seed in (1000, 1001):
                    r, tr, thr, dt = run('FBL', 'base', 'A1', plant, seed, sp, sv, sg); s = summ(r, tr); out['%s|%s|%d' % (name, plant, seed)] = s
                    print('%-10s %-11s %d crash %-5s t_crash %s tilt_max %6.1f rms3d(x,y,z cm) %.2f %.2f %.2f  %.0f s' % (name, plant, seed, s['crash'], s['crash_time_s'], s['tilt_max_deg'], s['x_rms_cm'], s['y_rms_cm'], s['z_rms_cm'], dt), flush=True)
    elif mode == 'levels':  # gyro only, finer levels (x0.17 = about the 2 deg/s of the real logs, real_flights_f2b.json)
        for l in (0.17, 0.75, 0.9):
            for plant in PLANTS:
                for seed in (1000, 1001):
                    r, tr, thr, dt = run('FBL', 'base', 'A1', plant, seed, 0, 0, l); s = summ(r, tr); out['gyro x%g|%s|%d' % (l, plant, seed)] = s
                    print('gyro x%-5g %-11s %d crash %-5s t_crash %s tilt_max %6.1f tilt_rms %5.1f' % (l, plant, seed, s['crash'], s['crash_time_s'], s['tilt_max_deg'], s['tilt_rms_deg']), flush=True)
    elif mode == 's0':
        for plant in PLANTS:
            for seed in (1000, 1001):
                r, tr, thr, dt = run('S0', 'base', 'A1', plant, seed, 1, 1, 1); s = summ(r, tr); out['S0|%s|%d' % (plant, seed)] = s
                print('S0', plant, seed, s, '%.0f s' % dt, flush=True)
    json.dump(out, open(HERE / ('a1_noise_diag_%s.json' % mode), 'w'))
