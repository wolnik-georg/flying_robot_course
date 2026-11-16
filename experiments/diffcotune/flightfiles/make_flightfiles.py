#!/usr/bin/env python3
"""Stage 4a C3: Poly4D flight files that reproduce the frsim reference of the solo scenarios exactly (smooth 2 s ramps, 1.8 m oval; simulator/docs/SCENARIOS.md).

Method (a new export, not the Rust exporter): the scenario part of the frsim reference (frsim.scenarios.Motion.kin: position, velocity, acceleration, jerk at scenario time t)
is cut into equal pieces of at most 0.5 s; on each piece the degree-7 polynomial is the Hermite interpolant of position, velocity, acceleration and jerk at both ends (8 conditions,
so the pieces join with continuous derivatives up to the jerk). Format = the CSV of the existing flight files (crazyflie_py Trajectory.loadcsv: duration, x^0..x^7, y^0..y^7, z^0..z^7,
yaw^0..yaw^7 per piece, local time per piece, `%.12g`). Frame = the frame of the existing files: the loop in the frsim reference frame, with z = reference z - height (height 1.0 m is
the `--height` of simple_flight.py), yaw 0. The file starts at the scenario start (the takeoff / hold / landing are those of simple_flight.py, not of frsim).
Checks written to flightfiles/checks.json: sampled position against the frsim reference (1 kHz, max abs difference), peak speed and acceleration of the file, flight volume against
safety.FLIGHT_SPACE with the margins, start and end positions. Nothing is copied to crazyswarm2."""
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import reference as R  # noqa (puts simulator/ on sys.path)
from frsim.scenarios import Motion, study_cells, tuning_cells, safety  # noqa

HEIGHT = 1.0
CELLS = [('CIRCLE', tuning_cells('CIRCLE')[0], 0.0), ('CIRCLE', tuning_cells('CIRCLE')[0], 90.0), ('FIG8', tuning_cells('FIG8')[0], 90.0), ('OVAL', study_cells('OVAL')[0], 0.0)]   # Stage 4d: circle turned by +90 deg added (the unturned circle and the oval do not fit the current pad)
PIECE_MAX_S = 0.5
NAMES = {'CIRCLE': 'circle', 'FIG8': 'figure8', 'OVAL': 'oval'}
ROTATE_DEG = {}      # the figure 8 file is turned by +90 deg (decision 13 Nov): its long axis (+-0.92 m) lies along y (flight space x +-1.0, y +-2.0); the frsim reference is unchanged and the ingest maps the frame (usd_ingest.estimate_frame, as for A4)


def rot(th):
    c, s_ = np.cos(np.radians(th)), np.sin(np.radians(th)); return np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])


def hermite(d, p0, v0, a0, j0, p1, v1, a1, j1):
    """Coefficients c0..c7 of the degree-7 polynomial on [0, d] with the given position / velocity / acceleration / jerk at both ends (per axis arrays)."""
    c = np.zeros((8, len(p0)))
    c[0], c[1], c[2], c[3] = p0, v0, a0 / 2, j0 / 6
    A = np.array([[d ** k, k * d ** (k - 1), k * (k - 1) * d ** (k - 2), k * (k - 1) * (k - 2) * d ** (k - 3)] for k in range(4, 8)]).T
    for ax in range(len(p0)):
        known = c[:4, ax]
        rhs = np.array([p1[ax] - sum(known[k] * d ** k for k in range(4)),
                        v1[ax] - sum(k * known[k] * d ** (k - 1) for k in range(1, 4)),
                        a1[ax] - sum(k * (k - 1) * known[k] * d ** (k - 2) for k in range(2, 4)),
                        j1[ax] - 6 * known[3]])
        c[4:, ax] = np.linalg.solve(A, rhs)
    return c


def evalpoly(c, tau, der=0):
    out = np.zeros((len(tau), c.shape[1]))
    for k in range(der, 8):
        coef = np.prod([k - i for i in range(der)]) if der else 1.0
        out += coef * c[k][None, :] * tau[:, None] ** (k - der)
    return out


def build(scen, cell, rot_deg):
    m = Motion(scen, cell); T = m.duration_s if hasattr(m, 'duration_s') else None
    from frsim.flightplan import FlightPlan
    T = FlightPlan(m).scenario_s
    n = int(np.ceil(T / PIECE_MAX_S)); d = T / n; pieces = []
    off = np.array([0.0, 0.0, HEIGHT]); Rm = rot(rot_deg)
    def kin(t):
        p, v, a, j, _ = m.kin(0, t); return Rm @ (p - off), Rm @ v, Rm @ a, Rm @ j          # z - height is not touched by the turn about z (anchor (0, 0))
    for i in range(n):
        k0 = kin(i * d); k1 = kin((i + 1) * d if i < n - 1 else T)
        pieces.append((d, hermite(d, *k0, *k1)))
    return m, T, pieces


LAB_USE = {'circle_frsim_tuning_mode1.csv': 'NOT FOR THE LAB: does not fit at the current pad (pad x window [0.50, 1.00] m, cf5 x = 0.0); kept as the Stage 4a file',
           'oval_frsim_study_mode1.csv': 'DROPPED from the lab test set (user, 13 Nov); does not fit at the current pad (pad window x [0.31, 1.00], y [-0.20, 0.20] m)'}
PAD_YAML = Path.home() / 'Desktop' / 'crazyswarm2' / 'crazyflie' / 'config' / 'crazyflies.yaml'


def pads():
    """initial_position of cf5 and cf_second from the crazyswarm2 yaml (read only)."""
    import yaml
    r = yaml.safe_load(open(PAD_YAML))['robots']; return np.array(r['cf5']['initial_position'][:2], float), np.array(r['cf_second']['initial_position'][:2], float)


def volume(got, pad, pad2, fs, height=HEIGHT):
    """Stage 4d fix. simple_flight hovers at pad + height, then startTrajectory(relative=True): the firmware shifts the trajectory so that its FIRST point sits at the current position
    (crazyflie-firmware planner.c plan_start_trajectory: shift = start_from - traj_eval(t_begin).pos). The path about the hover point is therefore (file - file(0)), not the file itself.
    Pad window = the pad xy for which the path stays inside FLIGHT_SPACE; margin = smallest distance to the box at the current cf5 pad; clearance = smallest horizontal distance of the path to the cf_second pad."""
    rel = got - got[0]; lo, hi = rel.min(0), rel.max(0)
    win = {ax: [float(fs[ax][0] - lo[i]), float(fs[ax][1] - hi[i])] for i, ax in enumerate('xy')}
    xy = pad[None, :] + rel[:, :2]; z = height + rel[:, 2]
    marg = dict(x=float(min(xy[:, 0].min() - fs['x'][0], fs['x'][1] - xy[:, 0].max())), y=float(min(xy[:, 1].min() - fs['y'][0], fs['y'][1] - xy[:, 1].max())),
                z_floor=float(z.min() - fs['z'][0]), z_ceiling=float(fs['z'][1] - z.max()))
    return dict(path_about_hover_min=lo.tolist(), path_about_hover_max=hi.tolist(), flight_space=fs, pad_window=win, pad_cf5=pad.tolist(), pad_cf_second=pad2.tolist(),
                margin_at_pad_m=marg, min_margin_at_pad_m=float(min(marg['x'], marg['y'])), fits_at_pad=bool(min(marg.values()) > 0),
                min_horizontal_distance_to_cf_second_pad_m=float(np.linalg.norm(xy - pad2[None, :], axis=1).min()),
                note='Stage 4d: path about the hover point = file - file(0) (planner.c:335-348); the Stage 4a margins assumed the file origin as hover point and were wrong')


def write_csv(path, pieces):
    hdr = 'duration,' + ','.join('%s^%d' % (ax, k) for ax in ('x', 'y', 'z', 'yaw') for k in range(8)) + ',\n'
    with open(path, 'w') as f:
        f.write(hdr)
        for d, c in pieces:
            row = [d] + [c[k, 0] for k in range(8)] + [c[k, 1] for k in range(8)] + [c[k, 2] for k in range(8)] + [0.0] * 8
            f.write(','.join('%.12g' % x for x in row) + ',\n')


def read_csv(path):
    rows = [ln.strip().rstrip(',').split(',') for ln in open(path)][1:]
    out = []
    for r in rows:
        v = np.array(r, float); d = v[0]
        out.append((d, np.stack([v[1:9], v[9:17], v[17:25]], 1)))
    return out


def sample(pieces, t, der=0):
    """Position (der 0), velocity or acceleration of the piecewise polynomial at times t (file frame)."""
    edges = np.cumsum([0] + [d for d, _ in pieces]); out = np.zeros((len(t), 3))
    idx = np.clip(np.searchsorted(edges, t, side='right') - 1, 0, len(pieces) - 1)
    for i, (d, c) in enumerate(pieces):
        s = idx == i
        if s.any():
            out[s] = evalpoly(c, t[s] - edges[i], der)
    return out


if __name__ == '__main__':
    checks = {}; pad_cf5, pad_cf2 = pads()
    for scen, cell, rot_deg in CELLS:
        m, T, pieces = build(scen, cell, rot_deg)
        name = '%s_frsim%s_%s_mode1.csv' % (NAMES[scen], '_rot%d' % rot_deg if rot_deg else '', 'tuning' if scen != 'OVAL' else 'study')
        write_csv(HERE / name, pieces)
        pf = read_csv(HERE / name)                                            # check the file as written (12 significant digits), not the in-memory pieces
        t = np.arange(0, int(round(T * 1000)) + 1) / 1000.0; t = t[t <= T]
        Rm = rot(rot_deg); wl = lambda a: a @ Rm.T       # reference rotated into the file frame
        ref = wl(np.stack([m.kin(0, float(x))[0] - np.array([0, 0, HEIGHT]) for x in t])) + np.array([0, 0, HEIGHT]); got = sample(pf, t) + np.array([0, 0, HEIGHT])
        vel = sample(pf, t, 1); acc = sample(pf, t, 2); refv = wl(np.stack([m.kin(0, float(x))[1] for x in t])); refa = wl(np.stack([m.kin(0, float(x))[2] for x in t]))
        fs = safety.FLIGHT_SPACE
        lo = got.min(0); hi = got.max(0)
        checks[name] = dict(scenario=scen, cell=cell, rotate_deg=rot_deg, duration_s=T, pieces=len(pf), max_piece_s=max(d for d, _ in pf),
                            max_abs_position_diff_mm=float(1000 * np.abs(got - ref).max()), max_abs_velocity_diff_mm_s=float(1000 * np.abs(vel - refv).max()),
                            max_abs_acceleration_diff_mm_s2=float(1000 * np.abs(acc - refa).max()),
                            peak_speed_m_s=float(np.linalg.norm(vel, axis=1).max()), ref_peak_speed_m_s=float(np.linalg.norm(refv, axis=1).max()),
                            peak_acc_m_s2=float(np.linalg.norm(acc, axis=1).max()), ref_peak_acc_m_s2=float(np.linalg.norm(refa, axis=1).max()),
                            start_file_frame=sample(pf, np.array([0.0]))[0].tolist(), end_file_frame=sample(pf, np.array([T]))[0].tolist(),
                            start_file_frame_plus_height=got[0].tolist(), end_file_frame_plus_height=got[-1].tolist(),
                            **volume(got, pad_cf5, pad_cf2, fs))
        checks[name]['lab_use'] = LAB_USE.get(name, 'lab')
        c = checks[name]
        print('%-34s T %.3f s, %d pieces | max pos diff %.4f mm | peak v %.3f (ref %.3f) m/s, a %.3f (ref %.3f) m/s2 | start %s end %s | margins %s' % (
            name, T, c['pieces'], c['max_abs_position_diff_mm'], c['peak_speed_m_s'], c['ref_peak_speed_m_s'], c['peak_acc_m_s2'], c['ref_peak_acc_m_s2'],
            np.round(c['start_file_frame_plus_height'], 3).tolist(), np.round(c['end_file_frame_plus_height'], 3).tolist(), {k: round(v, 3) for k, v in c['margin_at_pad_m'].items()}))
    json.dump(checks, open(HERE / 'checks.json', 'w'), indent=1)
