#!/usr/bin/env python3
"""Stage 4b item 2: why is the gyro part of J_sysId 1.5-17 rad^2/s^2 on the 23 Oct S0 flights? Tests per flight (10 flights, logged top path for A1 / A8 as in round.py):
 (a) sign and axis: logged gyro (deg/s -> rad/s) against the body rate derived from the logged attitude (est_q: w = 2 Im(conj(q) * dq/dt); and the euler columns roll / pitch / yaw with the
     legacy pitch sign), per axis: correlation and scale (regression slope of gyro on derived), after a 20 ms moving average of both;
 (b) frame: the same after usd_ingest.apply_frame (A4 -90 deg): derived from the turned quaternion against the turned gyro;
 (c) vibration: share of the power above 20 Hz of the logged gyro and of the model's body rate; J_w with both signals low-passed (10 Hz moving average) and the part of J_w above that;
 (d) attitude part: J_q split into tilt and yaw (euler of the logged and of the model quaternion, mean yaw offset);
 (e) model side: S0 model body rate against frsim (state trace, no noise) for one flight, sign and axis by the same derivative test.
Output lab/results/gyro_check.json and .log."""
import csv, json, os, sys
from pathlib import Path
import numpy as np
os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
HERE = Path(__file__).resolve().parent; ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import torch
torch.set_num_threads(2)
import usd_ingest as U, ingest, costs as C, s0_model as S  # noqa
import test_ingest_oct23 as T  # noqa
from frsim.scenarios import study_cells  # noqa

AX = 'xyz'


def omega_from_q(q, dt, k=10):
    """Body rate (n, 3) in rad/s from quaternions (x, y, z, w) at a uniform step dt: w = 2 Im(conj(q) * qdot), qdot by a central difference over +-k samples."""
    q = q.copy()
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0: q[i] = -q[i]
    qd = np.zeros_like(q); qd[k:-k] = (q[2 * k:] - q[:-2 * k]) / (2 * k * dt)
    qc = q * np.array([-1, -1, -1, 1.0])
    p = U._qmul(qc, qd)
    return 2 * p[:, :3]


def ma(a, n):
    ker = np.ones(n) / n
    return np.stack([np.convolve(a[:, j], ker, mode='same') for j in range(a.shape[1])], 1)


def corr_scale(g, d, m):
    out = []
    for j in range(3):
        a, b = g[m, j], d[m, j]; a = a - a.mean(); b = b - b.mean()
        out.append(dict(corr=float((a * b).sum() / np.sqrt((a * a).sum() * (b * b).sum() + 1e-30)), slope=float((a * b).sum() / ((b * b).sum() + 1e-30)), std_gyro=float(a.std()), std_derived=float(b.std())))
    return out


def euler_rates(c, dt, k=10):
    """d(roll, pitch, yaw)/dt in rad/s from the euler columns (legacy pitch sign: body-y pitch angle = -pitch_deg)."""
    e = np.radians(np.stack([c['roll_deg'], -c['pitch_deg'], c['yaw_deg']], 1)); e = np.unwrap(e, axis=0)
    d = np.zeros_like(e); d[k:-k] = (e[2 * k:] - e[:-2 * k]) / (2 * k * dt)
    return d


def psd_share(a, dt, fcut=20.0):
    a = a - a.mean(0); n = len(a); f = np.fft.rfftfreq(n, dt); P = (np.abs(np.fft.rfft(a * np.hanning(n)[:, None], axis=0)) ** 2).sum(1)
    return float(P[f > fcut].sum() / P[1:].sum())


def yaw_tilt(q):
    x, y, z, w = q.T
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)); tilt = np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1))
    return yaw, tilt


if __name__ == '__main__':
    rows = [r for r in csv.DictReader(open(T.PF)) if r['bank'] == 'bank_unseen_S0_2026-10-23']; out = []
    for r in rows:
        sc = r['scenario']; cell = T.cell_of(sc, r); f = T.BANK / r['folder'] / (r['folder'] + '_merged_usd.csv')
        g = U.ingest_flight(f, sc, cell, top_from_log=sc in ('A1', 'A8')); fl = g['flight']; c = fl.cols[fl.bottom]; t = fl.t; dt = float(np.median(np.diff(t)))
        ref = g['ref']; trl, _ = U.to_trace(fl, ref); tr = g['trace']; m = U.window_mask(t, ref.scenario_s)
        gyro_lab = trl[:, 11:14]; dq_lab = omega_from_q(trl[:, 7:11], dt); de_lab = euler_rates(c, dt); n = 10
        row = dict(folder=r['folder'], scenario=sc, frame_theta_deg=g['frame']['theta_deg'])
        row['a_quat_unturned'] = corr_scale(ma(gyro_lab, n), ma(dq_lab, n), m)
        row['a_euler_unturned'] = corr_scale(ma(gyro_lab, n), ma(de_lab, n), m)
        row['b_quat_turned'] = corr_scale(ma(tr[:, 11:14], n), ma(omega_from_q(tr[:, 7:11], dt), n), m)
        with torch.no_grad():
            o = S.rollout(ref, np.zeros((1, 10)), np.zeros((1, 4)), compile=False)
        tgt = ingest.on_grid(ingest.from_trace(tr), o['t']); tot, parts = C.j_sysid(o, tgt, ref.scenario_s, parts=True); parts = {k: float(v[0]) for k, v in parts.items()}
        wm = o['w'][:, 0, :].numpy(); qm = o['q'][:, 0, :].numpy(); tm = np.asarray(o['t']); dtm = float(np.median(np.diff(tm)))
        mm = (tm > 0) & (tm <= ref.scenario_s)
        row['c_share_above_20Hz_logged_gyro'] = psd_share(tr[m, 11:14], dt); row['c_share_above_20Hz_model_w'] = psd_share(wm[mm], dtm)
        # J_w with both low-passed (10 Hz moving average, 0.1 s)  and the remainder
        win = C.window(o['t'], ref.scenario_s, 'sysid') if hasattr(C, 'window') else mm
        wl = tgt.w; k = max(1, int(round(0.1 / dtm))); wls = ma(wl, k); wms = ma(wm, k); wn = np.asarray(win, bool)
        jw = float(((wm - wl)[wn] ** 2).sum(-1).mean()); jw_lp = float(((wms - wls)[wn] ** 2).sum(-1).mean())
        row['c_Jw'] = jw; row['c_Jw_lowpassed_10Hz'] = jw_lp; row['c_Jw_part_above'] = jw - jw_lp
        row['c_rms_rad_s_logged'] = float(np.sqrt((wl[wn] ** 2).sum(-1).mean())); row['c_rms_rad_s_model'] = float(np.sqrt((wm[wn] ** 2).sum(-1).mean()))
        row['c_rms_rad_s_logged_lp'] = float(np.sqrt((wls[wn] ** 2).sum(-1).mean())); row['c_rms_rad_s_model_lp'] = float(np.sqrt((wms[wn] ** 2).sum(-1).mean()))
        # (d) attitude part
        yl, tl = yaw_tilt(tgt.q); ym, tm_ = yaw_tilt(qm)
        row['d_J_parts'] = parts; row['d_yaw_logged_mean_deg'] = float(np.degrees(yl[wn].mean())); row['d_yaw_model_mean_deg'] = float(np.degrees(ym[wn].mean()))
        row['d_yaw_logged_rms_deg'] = float(np.degrees(np.sqrt((yl[wn] ** 2).mean()))); row['d_yaw_model_rms_deg'] = float(np.degrees(np.sqrt((ym[wn] ** 2).mean())))
        row['d_tilt_logged_rms_deg'] = float(np.degrees(np.sqrt((tl[wn] ** 2).mean()))); row['d_tilt_model_rms_deg'] = float(np.degrees(np.sqrt((tm_[wn] ** 2).mean())))
        out.append(row)
        print('%-24s Jw %.2f (lp %.2f) rms w logged %.3f model %.3f (lp %.3f / %.3f) | >20Hz logged %.2f model %.2f | corr quat %s | corr euler %s | turned %s | yaw logged %.1f model %.1f deg, tilt rms %.1f / %.1f' % (
            r['folder'], jw, jw_lp, row['c_rms_rad_s_logged'], row['c_rms_rad_s_model'], row['c_rms_rad_s_logged_lp'], row['c_rms_rad_s_model_lp'], row['c_share_above_20Hz_logged_gyro'], row['c_share_above_20Hz_model_w'],
            [round(a['corr'], 3) for a in row['a_quat_unturned']], [round(a['corr'], 3) for a in row['a_euler_unturned']], [round(a['corr'], 3) for a in row['b_quat_turned']],
            row['d_yaw_logged_mean_deg'], row['d_yaw_model_mean_deg'], row['d_tilt_logged_rms_deg'], row['d_tilt_model_rms_deg']), flush=True)
    json.dump(out, open(HERE / 'results' / 'gyro_check.json', 'w'), indent=1)
