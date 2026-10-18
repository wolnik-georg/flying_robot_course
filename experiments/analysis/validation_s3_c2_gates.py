#!/usr/bin/env python3
"""Validation of the S3 versions (2026-10-18), part 1: the INDI-measured push inside the NS2 gate (claim C2).

For every tree-validation flight of 2026-10-15 (S0..S3, A1 and A8):
  * rebuild the controller's residual a_res = a_imu_f - a_rpm_f OFFLINE from the 500 Hz uSD log with the
    controller's own definition (deck RPM, MOTORRPM2FORCE, mass 0.0427, sensor acc rotated by the estimated
    attitude minus g, 2nd-order Butterworth 30 Hz at 500 Hz) -- needed for S0/S2, where the controller does not
    compute it; checked against the onboard value on S1/S3;
  * align the uSD log to the merged 100 Hz CSV (which carries the top vehicle) by cross-correlating bottom z;
  * report gate means/medians of the onboard and offline residual and of the NS2 prediction for two gates.
Writes out/validation_s3/c2_gates.csv and per-flight aligned 100 Hz arrays out/validation_s3/aligned_<flight>.npz.

Usage: ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/validation_s3_c2_gates.py
"""
import glob, os, re, sys
import numpy as np
import pandas as pd
from scipy.signal import butter, lfilter

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'flying_drone_stack', 'tools'))
import decode_usd_log  # noqa: E402

RES = os.path.join(ROOT, 'experiments', 'results', 'tree_validation_2026-10-15')
RAW = os.path.join(ROOT, 'experiments', 'logs', 'usd_raw')
OUT = os.path.join(ROOT, 'experiments', 'analysis', 'out', 'validation_s3')
KT = 3.91194027330774e-08     # N per (rad/s)^2, platform_defaults_cf21bl.h
MASS = 0.0427                 # kg, CF_MASS of the build
G = 9.81
B, A = butter(2, 30.0, fs=500.0)   # 30 Hz, 500 Hz (the controller's Butterworth2LowPass is the same design)


def quat_to_R(qx, qy, qz, qw):
    n = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2); qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    R = np.empty((len(qx), 3, 3))
    R[:, 0, 0] = 1-2*(qy*qy+qz*qz); R[:, 0, 1] = 2*(qx*qy-qz*qw); R[:, 0, 2] = 2*(qx*qz+qy*qw)
    R[:, 1, 0] = 2*(qx*qy+qz*qw); R[:, 1, 1] = 1-2*(qx*qx+qz*qz); R[:, 1, 2] = 2*(qy*qz-qx*qw)
    R[:, 2, 0] = 2*(qx*qz-qy*qw); R[:, 2, 1] = 2*(qy*qz+qx*qw); R[:, 2, 2] = 1-2*(qx*qx+qy*qy)
    return R


def offline_residual(u):
    R = quat_to_R(*(np.asarray(u[k], float) for k in ('est_qx', 'est_qy', 'est_qz', 'est_qw')))
    w = [np.asarray(u[f'rpm_m{k}'], float) * 2*np.pi/60 for k in range(1, 5)]
    f_rpm = KT * sum(wi**2 for wi in w)
    a_rpm_z = f_rpm/MASS * R[:, 2, 2] - G
    acc = np.stack([np.asarray(u[k], float) for k in ('acc_x', 'acc_y', 'acc_z')], 1)
    a_imu_z = G*(np.einsum('nj,nj->n', R[:, 2, :], acc) - 1.0)
    return lfilter(B, A, a_imu_z) - lfilter(B, A, a_rpm_z), f_rpm


def align_offset(t_u, z_u, t_c, z_c):
    """Offset s such that uSD time t_u + s matches CSV time t_c (cross-correlation of z on a 100 Hz grid)."""
    best = (np.inf, 0.0)
    zc = z_c - np.mean(z_c)
    for s in np.arange(t_c[0] - t_u[-1], t_c[-1] - t_u[0], 0.01):
        zi = np.interp(t_c, t_u + s, z_u, left=np.nan, right=np.nan)
        ok = ~np.isnan(zi)
        if ok.sum() < 0.8*len(t_c):
            continue
        err = np.mean((zi[ok] - np.mean(zi[ok]) - zc[ok])**2)
        if err < best[0]:
            best = (err, s)
    return best[1], np.sqrt(best[0])


def main():
    os.makedirs(OUT, exist_ok=True)
    rows = []
    for csv in sorted(glob.glob(os.path.join(RES, 'tree_S*_A*_*.csv'))):
        name = os.path.basename(csv)[:-4]
        strat, scen, hhmm = re.match(r'tree_(S\d)_(A\d)_(\d+)', name).groups()
        raw = glob.glob(os.path.join(RAW, f'cf5_tree_{strat}_{scen}_{hhmm}_*.bin'))
        c = pd.read_csv(csv)
        u = decode_usd_log.load(raw[0])
        t_u = np.asarray(u['t'], float)
        ares_off, f_rpm = offline_residual(u)
        s, rms = align_offset(t_u, np.asarray(u['z'], float), c.t.values, c.bottom_z.values)
        on = lambda k: np.interp(c.t.values, t_u + s, np.asarray(u[k], float))
        al = dict(t=c.t.values, ares_off=np.interp(c.t.values, t_u + s, ares_off),
                  ares_on=on('omar_a_imu_fz') - on('omar_a_rpm_fz'), pred=on('rnn_pred_z'),
                  thrust_cmd=on('omar_thrust_si'), f_rpm_off=np.interp(c.t.values, t_u + s, f_rpm),
                  z=c.bottom_z.values, zref=c.bottom_ctrltarget_z.values,
                  dx=(c.top_x-c.bottom_x).values, dy=(c.top_y-c.bottom_y).values, dz=(c.top_z-c.bottom_z).values,
                  vbat=c.bottom_vbat.values)
        np.savez(os.path.join(OUT, f'aligned_{name}.npz'), **al)
        indi_on = strat in ('S1', 'S3')
        # validation of the offline residual against the onboard one (S1/S3)
        if indi_on:
            m = np.abs(al['ares_on']) > 1e-6
            corr = np.corrcoef(al['ares_on'][m], al['ares_off'][m])[0, 1]
            bias = np.mean(al['ares_off'][m] - al['ares_on'][m])
        else:
            corr = bias = np.nan
        gates = {
            'g1 (top above, |dx|,|dy|<0.2)': (al['dz'] > 0) & (np.abs(al['dx']) < 0.2) & (np.abs(al['dy']) < 0.2),
            'g2 (top above 0.3-0.7, |dx|,|dy|<0.1)': (al['dz'] > 0.3) & (al['dz'] < 0.7) & (np.abs(al['dx']) < 0.1) & (np.abs(al['dy']) < 0.1),
        }
        out_gate = np.abs(al['dx']) > 0.5
        for gname, g in gates.items():
            rows.append(dict(flight=name, strategy=strat, scenario=scen, gate=gname, n=int(g.sum()),
                             ares_onboard_mean=np.mean(al['ares_on'][g]) if indi_on else np.nan,
                             ares_onboard_median=np.median(al['ares_on'][g]) if indi_on else np.nan,
                             ares_offline_mean=np.mean(al['ares_off'][g]), ares_offline_median=np.median(al['ares_off'][g]),
                             ares_offline_outside_gate=np.mean(al['ares_off'][out_gate]) if out_gate.any() else np.nan,
                             pred_mean=np.mean(al['pred'][g]), pred_median=np.median(al['pred'][g]),
                             pred_outside_gate=np.mean(al['pred'][out_gate]) if out_gate.any() else np.nan,
                             vbat_median=np.median(al['vbat']), align_rms_m=rms, offline_vs_onboard_corr=corr,
                             offline_minus_onboard_bias=bias))
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(OUT, 'c2_gates.csv'), index=False)
    pd.set_option('display.width', 250); pd.set_option('display.max_columns', 30)
    print(df.round(3).to_string(index=False))


if __name__ == '__main__':
    main()
