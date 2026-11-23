"""Stage 4a, C1: real uSD ingest for DiffCoTune on S0 (desk work, no flights).

A round folder holds one merged uSD file per flight (`*_merged_usd.csv`, written by flying_drone_stack/tools/merge_usd_logs.py with
`# meta:t_zero=scenario_start`, 500 Hz, both drones, columns `<drone>.<channel>`). Never radio CSVs. For every flight this module gives
  * the quality gate (rows, rate, gaps, NaNs, both drones present for the two-drone scenarios A1 / A4 / A8),
  * the state trace on the frsim state-trace layout (ingest.COLUMNS: time, p, v, q (x y z w), w, reference) so ingest.from_trace / on_grid and the
    costs of costs.py / loop_lib.py apply unchanged,
  * the reference check (logged `ctrltarget` against the model / frsim reference of the same cell),
  * the scored window, the crash / floor flag and the real cost (spec section 4: mean squared 3-D position error over the scored part).

Conventions (read from the repository, see the report for the sources):
  * time: t = 0 at the scenario start (`# meta:t_zero=scenario_start`); takeoff and hold have t < 0.
  * attitude: the logged estimator quaternion `est_q{w,x,y,z}` (scalar first in the file) -> (x, y, z, w) as in the model. Only when it is missing the
    fallback builds a ZYX quaternion from `roll_deg`, `-pitch_deg` (the log carries the legacy pitch sign: stabilizer.pitch = -pitch of the standard
    convention, experiments/analysis/study_metrics.py:99-127) and `yaw_deg` (yaw sign not checked against a flight: fallback not validated).
  * body rates: `gyro_x/y/z` in deg/s -> rad/s.
  * position / velocity: `x y z`, `vx vy vz` (estimator output, m, m/s).
  * bottom vehicle = the controlled and scored one = the drone whose prefix is `cf5` (study_metrics.py:331-342 scores cf5); the other is the top vehicle.
"""
import re
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

import ingest
import reference as R
import costs as COSTS
from frsim import config

BOTTOM_PREFIX = 'cf5'
TWO_DRONE = ('A1', 'A4', 'A8')
GAP_MS = 3.0            # check_usd_log_quality.py: a sample gap is dt > 3 ms ...
GAP_FRACTION = 0.001    # ... and more than 0.1 % of the samples is "the card cannot keep up"
RATE_HZ = 500.0
STATE_COLS = ('x', 'y', 'z', 'vx', 'vy', 'vz', 'gyro_x', 'gyro_y', 'gyro_z', 'ctrltarget_x', 'ctrltarget_y', 'ctrltarget_z')
Q_COLS = ('est_qw', 'est_qx', 'est_qy', 'est_qz')


@dataclass
class Flight:
    path: str
    meta: dict
    t: np.ndarray
    cols: dict          # drone prefix -> {channel: array}
    bottom: str
    top: str


def load_merged(path):
    """Merged uSD CSV -> Flight (comment lines starting with '#' are read as meta)."""
    path = Path(path); meta = {}; header = None; rows = []
    with open(path) as f:
        for ln in f:
            if ln.startswith('#'):
                m = re.match(r'#\s*meta:(\w+)=(.*)', ln.strip())
                if m:
                    meta[m.group(1)] = m.group(2)
                continue
            if header is None:
                header = ln.strip().split(','); continue
            rows.append(ln.rstrip('\n').split(','))
    arr = np.array([[_f(x) for x in r] for r in rows], float)
    cols = {}
    for j, h in enumerate(header):
        if h == 't' or '.' not in h:
            continue
        drone, ch = h.split('.', 1)
        cols.setdefault(drone, {})[ch] = arr[:, j]
    drones = sorted(cols)
    bottom = next((d for d in drones if d.startswith(BOTTOM_PREFIX)), drones[0] if drones else None)
    top = next((d for d in drones if d != bottom and d != 'rel'), None)
    return Flight(str(path), meta, arr[:, header.index('t')], cols, bottom, top)


def _f(x):
    try:
        return float(x)
    except ValueError:
        return np.nan


def quality_gate(fl, scenario):
    """Per-flight gate: rows, rate, gaps (rule of check_usd_log_quality.py), NaNs in the used channels, both drones present for A1 / A4 / A8,
    time axis covers the scenario start (t_zero meta). Returns dict(ok, reasons, ...)."""
    t = fl.t; dt = np.diff(t) * 1000.0; reasons = []
    n = len(t); rate = 1000.0 / float(np.median(dt)) if n > 1 else 0.0
    late = int(np.sum(dt > GAP_MS)); frac = late / max(len(dt), 1)
    if abs(rate - RATE_HZ) > 0.02 * RATE_HZ:
        reasons.append('rate %.1f Hz' % rate)
    if frac > GAP_FRACTION:
        reasons.append('gaps %.3f %% (max %.1f ms)' % (100 * frac, dt.max()))
    if fl.meta.get('t_zero') != 'scenario_start':
        reasons.append('no t_zero=scenario_start meta')
    need = STATE_COLS
    bad = {}
    if fl.bottom is None:
        reasons.append('no drone columns'); nan_bad = {}
    else:
        c = fl.cols[fl.bottom]
        miss = [k for k in need + Q_COLS if k not in c]
        if miss:
            reasons.append('missing columns %s' % miss)
        for k in need + Q_COLS:
            if k in c:
                nn = int(np.isnan(c[k]).sum())
                if nn:
                    bad[k] = nn
        if bad:
            reasons.append('NaNs %s' % bad)
    if scenario in TWO_DRONE and fl.top is None:
        reasons.append('top drone missing')
    return dict(ok=not reasons, reasons=reasons, rows=n, rate_hz=rate, duration_s=float(t[-1] - t[0]), gaps_over_3ms=late, gap_fraction=frac,
                max_gap_ms=float(dt.max()) if len(dt) else 0.0, nan_columns=bad, t_start=float(t[0]), t_end=float(t[-1]), bottom=fl.bottom, top=fl.top)


def _quat(c):
    if all(k in c for k in Q_COLS) and np.nanstd(c['est_qw']) > 0:
        return np.stack([c['est_qx'], c['est_qy'], c['est_qz'], c['est_qw']], 1), 'est_q'
    r = np.radians(c['roll_deg']) / 2; p = np.radians(-c['pitch_deg']) / 2; y = np.radians(c['yaw_deg']) / 2     # legacy pitch sign; ZYX
    q = np.stack([np.sin(r) * np.cos(p) * np.cos(y) - np.cos(r) * np.sin(p) * np.sin(y), np.cos(r) * np.sin(p) * np.cos(y) + np.sin(r) * np.cos(p) * np.sin(y),
                  np.cos(r) * np.cos(p) * np.sin(y) - np.sin(r) * np.sin(p) * np.cos(y), np.cos(r) * np.cos(p) * np.cos(y) + np.sin(r) * np.sin(p) * np.sin(y)], 1)
    return q, 'euler fallback (not validated)'


def to_trace(fl, ref):
    """Flight -> array (n, 23) in the frsim state-trace layout (ingest.COLUMNS) on the log's own 500 Hz time base. The reference columns are the
    logged `ctrltarget` (what the controller was told); the model reference is checked against them in reference_check."""
    c = fl.cols[fl.bottom]; q, src = _quat(c)
    tr = np.zeros((len(fl.t), 23))
    tr[:, 0] = fl.t
    tr[:, 1:4] = np.stack([c['x'], c['y'], c['z']], 1); tr[:, 4:7] = np.stack([c['vx'], c['vy'], c['vz']], 1)
    tr[:, 7:11] = q
    tr[:, 11:14] = np.radians(np.stack([c['gyro_x'], c['gyro_y'], c['gyro_z']], 1))
    tr[:, 14:17] = np.stack([c['ctrltarget_x'], c['ctrltarget_y'], c['ctrltarget_z']], 1)
    tr[:, 17:20] = np.stack([c.get('ctrltarget_vx', np.zeros(len(fl.t))), c.get('ctrltarget_vy', np.zeros(len(fl.t))), c.get('ctrltarget_vz', np.zeros(len(fl.t)))], 1)
    tr[:, 20:23] = np.stack([c.get('ctrltarget_ax', np.zeros(len(fl.t))), c.get('ctrltarget_ay', np.zeros(len(fl.t))), c.get('ctrltarget_az', np.zeros(len(fl.t)))], 1)
    return tr, src


def estimate_frame(fl, ref, rotate_deg=None, lag_s=None):
    """The lab flies the frsim scenarios rotated about the vertical axis (run_formation.py --rotate-deg, simulator scenarios.rotate_scenario) and with its own anchor, so the logged
    `ctrltarget` of the bottom drone is the frsim reference in another frame; and the merged file's t = 0 is not exactly the scenario start (see below). Fit of the logged reference
    xy onto ref.pos over the scored window: model = R(theta) lab + t at scenario time t_log + lag_s. Alternating: (rotation, translation) at the current lag, then the lag
    (search +-200 ms, 2 ms steps, minimum of the max 3-D difference), twice. `rotate_deg` / `lag_s` given: that value is used. Without motion in the scored window (A1 hover)
    theta and the lag are not observable and are 0.
    Source of the lag (Stage 3e): merge_usd_logs.py --meta sets t = 0 by matching the MEASURED position to the commanded trajectory (find_flight_window.find_offset), so the
    closed-loop tracking delay of the drone is absorbed into the time base and the logged command runs ahead of the frsim reference by that delay."""
    c = fl.cols[fl.bottom]; t = fl.t
    lab = np.stack([c['ctrltarget_x'], c['ctrltarget_y'], c['ctrltarget_z']], 1)
    def refpos(tt):
        return np.stack([np.interp(tt, ref.time_s, ref.pos[:, j]) for j in range(3)], 1)
    m = window_mask(t + (lag_s or 0.0), ref.scenario_s)
    moving = np.abs(lab[m, :2] - lab[m, :2].mean(0)).max() >= 0.05
    lag = float(lag_s) if lag_s is not None else 0.0; th = np.radians(rotate_deg) if rotate_deg is not None else 0.0; src = 'given' if rotate_deg is not None else 'not observable (no motion), 0'
    tr = np.zeros(2)
    for it in range(2 if moving else 1):
        m = window_mask(t + lag, ref.scenario_s); a = lab[m, :2]; b = refpos(t[m] + lag)[:, :2]; ac, bc = a - a.mean(0), b - b.mean(0)
        if rotate_deg is None and moving:
            th = float(np.arctan2((ac[:, 0] * bc[:, 1] - ac[:, 1] * bc[:, 0]).sum(), (ac * bc).sum())); src = 'fitted'
        R2 = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]); tr = b.mean(0) - R2 @ a.mean(0)
        if lag_s is None and moving:
            R3 = _rz(th); lt = lab @ R3.T + np.array([tr[0], tr[1], 0.0]); best = (np.inf, 0.0)
            for k in range(-100, 101):
                s_ = k * 0.002; mm = window_mask(t + s_, ref.scenario_s)
                best = min(best, (float(np.abs(lt[mm] - refpos(t[mm] + s_)).max()), s_))
            lag = best[1]
    return dict(theta_deg=float(np.degrees(th)), t_xy=tr.tolist(), lag_s=lag, source=src, lag_source='given' if lag_s is not None else ('fitted from the logged command' if moving else 'not observable, 0'))


def _rz(th):
    return np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1.0]])


def _qmul(a, b):
    """Hamilton product, rows (x, y, z, w)."""
    ax, ay, az, aw = a.T; bx, by, bz, bw = b.T
    return np.stack([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz], 1)


def apply_frame(tr, frame):
    """Lab frame -> model frame for a state trace (n, 23): positions / velocities / reference by R(theta) and t; attitude by conjugation q_m = qz(theta) q qz(-theta)
    (the drone is relabelled by the same yaw turn, so a yaw-0 flight stays yaw-0; assumes the airframe symmetric under that turn, NOT checked); body rates w_m = R(theta) w."""
    th = np.radians(frame['theta_deg']); R3 = _rz(th); out = tr.copy(); t3 = np.array([frame['t_xy'][0], frame['t_xy'][1], 0.0])
    out[:, 0] = tr[:, 0] + frame.get('lag_s', 0.0)
    out[:, 1:4] = tr[:, 1:4] @ R3.T + t3; out[:, 14:17] = tr[:, 14:17] @ R3.T + t3
    for sl in (slice(4, 7), slice(17, 20), slice(20, 23), slice(11, 14)):
        out[:, sl] = tr[:, sl] @ R3.T
    qz = np.array([0, 0, np.sin(th / 2), np.cos(th / 2)]); qzi = np.array([0, 0, -np.sin(th / 2), np.cos(th / 2)])
    n = len(tr)
    out[:, 7:11] = _qmul(_qmul(np.tile(qz, (n, 1)), tr[:, 7:11]), np.tile(qzi, (n, 1)))
    return out


def reference_check(fl, ref, frame=None):
    """Logged ctrltarget against the model / frsim reference of the cell, at the same scenario time (the reference at the log's sample times by
    linear interpolation of ref.pos). Reports the max difference over the log, the max over the scored window, and after removing the constant offset
    (the lab anchor need not be the frsim anchor) and the best lag within +-50 ms (the controller receives the command with a delay)."""
    c = fl.cols[fl.bottom]; t = fl.t + (frame.get('lag_s', 0.0) if frame is not None else 0.0); rt = ref.time_s
    lg = np.stack([c['ctrltarget_x'], c['ctrltarget_y'], c['ctrltarget_z']], 1)
    if frame is not None:
        lg = lg @ _rz(np.radians(frame['theta_deg'])).T + np.array([frame['t_xy'][0], frame['t_xy'][1], 0.0])
    rp = np.stack([np.interp(t, rt, ref.pos[:, j]) for j in range(3)], 1)
    inwin = (t > config.SCORE_AFTER_S) & (t <= ref.scenario_s)
    d = lg - rp
    out = dict(max_abs_m=float(np.abs(d).max()), max_abs_scored_m=float(np.abs(d[inwin]).max()) if inwin.any() else float('nan'))
    off = np.median(d[inwin], 0) if inwin.any() else np.zeros(3)
    out['offset_m'] = off.tolist(); out['max_abs_scored_after_offset_m'] = float(np.abs(d[inwin] - off).max()) if inwin.any() else float('nan')
    best = (np.inf, 0.0)
    for lag_ms in range(-50, 51, 2):
        rpl = np.stack([np.interp(t - lag_ms / 1000.0, rt, ref.pos[:, j]) for j in range(3)], 1)
        e = np.abs(lg - rpl - off)[inwin].max() if inwin.any() else np.inf
        best = min(best, (e, lag_ms))
    out['best_lag_ms'] = best[1]; out['max_abs_scored_after_offset_and_lag_m'] = float(best[0])
    return out


def window_mask(t, scenario_s):
    return (t > config.SCORE_AFTER_S) & (t <= scenario_s)


def flight_flags(tr, ref):
    """Scored window, crash flag (tilt of the body z axis from the quaternion >= the tree crash limit inside the scenario part, as frsim) and floor flag
    (mean z of the scored part < 50 % of the mean reference z, Stage 3d rule)."""
    t = tr[:, 0]; sc = (t > 0) & (t <= ref.scenario_s); m = window_mask(t, ref.scenario_s)
    q = tr[:, 7:11]; tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 0] ** 2 + q[:, 1] ** 2), -1, 1)))
    ref_z = np.interp(t[m], ref.time_s, ref.pos[:, 2]).mean() if m.any() else float('nan')
    return dict(scored_from_s=float(config.SCORE_AFTER_S), scored_to_s=float(ref.scenario_s), n_scored=int(m.sum()), tilt_max_deg=float(tilt[sc].max()) if sc.any() else float('nan'),
                crashed=bool(sc.any() and tilt[sc].max() >= config.CRASH_TILT_DEG['tree']), floor=bool(m.any() and tr[m, 3].mean() < 0.5 * ref_z),
                z_mean_m=float(tr[m, 3].mean()) if m.any() else float('nan'), ref_z_mean_m=float(ref_z))


def real_cost(tr, ref):
    """Spec section 4 / loop_lib.flight_cost: (mean squared 3-D position error, mean squared z error) in m^2 over the scored window, against the
    model reference (the position the flight was told to track), interpolated at the log's sample times."""
    t = tr[:, 0]; m = window_mask(t, ref.scenario_s)
    rp = np.stack([np.interp(t[m], ref.time_s, ref.pos[:, j]) for j in range(3)], 1)
    e = tr[m, 1:4] - rp
    return float((e * e).sum(-1).mean()), float((e[:, 2] ** 2).mean())


def with_logged_top(ref, fl, frame=None):
    """Reference whose top-vehicle path is the logged one (position, velocity from the top drone's `x y z`, `vx vy vz`) at the model's 1 ms ticks;
    held outside the log. For the S0 model's peer input (option; default is the planned top path of frsim)."""
    if fl.top is None:
        return ref
    c = fl.cols[fl.top]; rt = ref.time_s; tl = fl.t + (frame.get('lag_s', 0.0) if frame is not None else 0.0)
    pos = np.stack([np.interp(rt, tl, c[k]) for k in ('x', 'y', 'z')], 1); vel = np.stack([np.interp(rt, tl, c[k]) for k in ('vx', 'vy', 'vz')], 1)
    if frame is not None:
        R3 = _rz(np.radians(frame['theta_deg'])); pos = pos @ R3.T + np.array([frame['t_xy'][0], frame['t_xy'][1], 0.0]); vel = vel @ R3.T
    return replace(ref, top_pos=pos, top_vel=vel)


def ingest_flight(path, scenario, cell, top_from_log=False, rotate_deg=None, lag_s=None, align=True):
    """One merged uSD file -> dict(flight, ref, gate, frame, trace (model frame), trace_source, reference_check, flags, cost_m2, z_cost_m2). The lab frame is mapped onto the
    model frame (estimate_frame / apply_frame) unless align=False; `reference_check` is reported before ('raw') and after the alignment. `ref` carries the logged top path
    (in the model frame) when top_from_log. The real cost is computed in the model frame against the model reference."""
    fl = load_merged(path); ref = R.build(scenario, cell); gate = quality_gate(fl, scenario)
    out = dict(path=str(path), scenario=scenario, cell=cell, gate=gate)
    if fl.bottom is None or any(k not in fl.cols[fl.bottom] for k in STATE_COLS):
        out['usable'] = False; return out
    tr_lab, src = to_trace(fl, ref)
    frame = estimate_frame(fl, ref, rotate_deg, lag_s) if align else dict(theta_deg=0.0, t_xy=[0.0, 0.0], source='off')
    tr = apply_frame(tr_lab, frame) if align else tr_lab
    out.update(flight=fl, frame=frame, ref=with_logged_top(ref, fl, frame if align else None) if top_from_log else ref, trace=tr, trace_source=src,
               reference_check_raw=reference_check(fl, ref), reference_check=reference_check(fl, ref, frame if align else None), flags=flight_flags(tr, ref))
    c, cz = real_cost(tr, ref)
    out.update(cost_m2=c, z_cost_m2=cz, rms3d_cm=100 * c ** 0.5, usable=gate['ok'] and not np.isnan(c))
    if COSTS.TASK_FORM == 'att':          # Protocol v2 (ours2): the cost of the round is ||p - p*||^2 + tr(I - R*^T R)/2; the 3-D position RMS stays the position part
        tc = COSTS.trace_cost(tr, ref, ref.scenario_s)
        out.update(cost_m2=tc['cost'], pos_cost_m2=tc['pos'], att_cost=tc['att'], z_cost_m2=tc['z'], rms3d_cm=100 * tc['pos'] ** 0.5, usable=gate['ok'] and not np.isnan(tc['cost']))
    return out


def ingest_round_folder(folder, manifest):
    """Round folder -> {flight name: ingest_flight dict}. `manifest` = list of dict(file, scenario, cell) (the flight card of the round)."""
    return {m['file']: ingest_flight(Path(folder) / m['file'], m['scenario'], m['cell']) for m in manifest}
