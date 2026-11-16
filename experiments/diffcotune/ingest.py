"""Ingest sketch (Stage 2): one flight trace -> the J_sysId target of the differentiable model.

Stand-in for a uSD log: the input is a frsim full-state trace (frsim.episode.STATE_TRACE_COLUMNS, CSV or array). The
alignment rule is the one the real uSD path needs (merged uSD file, `# meta:t_zero=scenario_start`, 500 Hz):
  * time base = scenario time, t = 0 at the scenario start (takeoff and hold have t < 0);
  * the target rows are put on the model's output grid (10 ms, start-of-tick) by linear interpolation in time
    (quaternions: normalised linear interpolation with sign continuity);
  * cut: only rows inside the cost window enter the cost (takeoff and landing are cut by the window, see costs.py);
  * attitude = quaternion (x, y, z, w), body rates in rad/s (a uSD log has `gyro.*` in deg/s and `stabilizer.pitch` with
    the legacy sign: the Stage 4 reader converts to this convention, not done here).
Not done here (Stage 4): reading uSD files, per-round folder handling, quality gate, crash flag.
"""
import csv
from dataclasses import dataclass

import numpy as np

COLUMNS = ['time_s', 'x_m', 'y_m', 'z_m', 'vx_m_s', 'vy_m_s', 'vz_m_s', 'qx', 'qy', 'qz', 'qw', 'wx_rad_s', 'wy_rad_s', 'wz_rad_s',
           'ref_x_m', 'ref_y_m', 'ref_z_m', 'ref_vx_m_s', 'ref_vy_m_s', 'ref_vz_m_s', 'ref_ax_m_s2', 'ref_ay_m_s2', 'ref_az_m_s2']


@dataclass
class Target:
    t: np.ndarray       # (n,) scenario time [s]
    p: np.ndarray       # (n, 3)
    v: np.ndarray       # (n, 3)
    q: np.ndarray       # (n, 4) x, y, z, w
    w: np.ndarray       # (n, 3) rad/s
    ref_p: np.ndarray   # (n, 3) reference position of the flight (what the controller was told)


def read_csv(path):
    """State-trace CSV (as written by scripts/run_scenario.py) -> array (n, 23)."""
    with open(path) as f:
        rows = list(csv.reader(f))
    assert rows[0] == COLUMNS, 'unexpected columns in %s' % path
    return np.array(rows[1:], float)


def from_trace(trace):
    """Array (n, 23) of a state trace -> Target on the trace's own time base."""
    tr = np.asarray(trace, float)
    q = tr[:, 7:11].copy()
    for k in range(1, len(q)):                       # sign continuity
        if np.dot(q[k], q[k - 1]) < 0:
            q[k] = -q[k]
    return Target(t=tr[:, 0], p=tr[:, 1:4], v=tr[:, 4:7], q=q, w=tr[:, 11:14], ref_p=tr[:, 14:17])


def on_grid(target, t_grid):
    """Target interpolated to the times `t_grid` (scenario time, [s]); rows outside the target's time range are held."""
    ip = lambda a: np.stack([np.interp(t_grid, target.t, a[:, j]) for j in range(a.shape[1])], -1)
    q = ip(target.q)
    q /= np.linalg.norm(q, axis=-1, keepdims=True)
    return Target(t=np.asarray(t_grid), p=ip(target.p), v=ip(target.v), q=q, w=ip(target.w), ref_p=ip(target.ref_p))
