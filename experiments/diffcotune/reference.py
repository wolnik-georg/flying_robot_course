"""Reference motion of a frsim scenario cell on the 1 ms tick grid, built with frsim's own FlightPlan (so the
differentiable model and frsim fly exactly the same reference). Needs only numpy and the frsim package (not the
firmware bindings)."""
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

SIMULATOR = Path(__file__).resolve().parents[2] / 'simulator'
if str(SIMULATOR) not in sys.path:
    sys.path.insert(0, str(SIMULATOR))
from frsim import config  # noqa: E402
from frsim.flightplan import FlightPlan  # noqa: E402
from frsim.scenarios import Motion, study_cells, tuning_cells  # noqa: E402

CACHE = Path(__file__).resolve().parent / 'cache'
DT = config.DT_S
COMMAND_DELAY_S = config.COMMAND_DELAY_S
NO_TOP_Z = -10.0  # solo scenarios: the "top vehicle" is far below, the downwash gate never opens


@dataclass
class Reference:
    scenario: str
    cell: dict
    n_ticks: int
    scenario_s: float            # duration of the scenario part (s)
    has_top: bool
    pos: np.ndarray              # (n, 3) setpoint position per 1 ms tick (bottom vehicle)
    vel: np.ndarray
    acc: np.ndarray
    jerk: np.ndarray
    top_pos: np.ndarray          # (n, 3) top vehicle position (NO_TOP_Z below for solo scenarios)
    top_vel: np.ndarray
    start_xy: np.ndarray

    @property
    def time_s(self):
        """Scenario time of every tick (negative during takeoff and hold)."""
        return np.arange(self.n_ticks) * DT - config.TAKEOFF_S - config.HOLD_S


def _key(scenario, cell):
    return scenario + '_' + '_'.join('%s%s' % (k, cell[k]) for k in sorted(cell))


def build(scenario, cell=None, cache=True):
    """Reference of a scenario cell (default: first study cell). Cached as npz in experiments/diffcotune/cache."""
    cell = dict(study_cells(scenario)[0] if cell is None else cell)
    f = CACHE / (_key(scenario, cell).replace('/', '_') + '.npz')
    motion = Motion(scenario, cell)
    plan = FlightPlan(motion)
    n = int(plan.total_s / DT)
    if cache and f.exists():
        d = np.load(f)
        arrays = {k: d[k] for k in ('pos', 'vel', 'acc', 'jerk', 'top_pos', 'top_vel')}
    else:
        arrays = {k: np.zeros((n, 3)) for k in ('pos', 'vel', 'acc', 'jerk', 'top_pos', 'top_vel')}
        for i in range(n):
            t = i * DT
            arrays['pos'][i], arrays['vel'][i], arrays['acc'][i], arrays['jerk'][i] = plan.setpoint(t)
            if plan.has_top:
                arrays['top_pos'][i], arrays['top_vel'][i] = plan.top_vehicle(t)
            else:
                arrays['top_pos'][i] = (0.0, 0.0, NO_TOP_Z)
        if cache:
            CACHE.mkdir(exist_ok=True)
            np.savez_compressed(f, **arrays)
    return Reference(scenario, cell, n, plan.scenario_s, plan.has_top, start_xy=plan.start_position[:2].copy(), **arrays)


def command_jitter(n_blocks):
    """flag[b] True: at the first millisecond of block b (tick 2b) frsim still applies the command of block b - 3 instead
    of block b - 2, because the command release time k * dt + 0.004 is compared with t = i * dt in float arithmetic
    (the same expressions as frsim.episode)."""
    b = np.arange(n_blocks)
    return (b >= 2) & (((2 * b - 4) * DT + COMMAND_DELAY_S) > (2 * b) * DT)


# ---- Stage 8: the reference STATE of the paper-faithful task cost (attitude and body rates from differential flatness) ----------------------------------------------------------------------
_FLAT = {}


def flat_ref(ref):
    """(q_ref (n, 4) as x, y, z, w; w_ref (n, 3) body rates [rad/s]) of the reference trajectory by differential flatness, with the formulas of the S0 controller (controller_omar_indi.c as copied in s0_torch / s1_model._ctrl_step):
    F = a_ref + g e_z (no feedback, no INDI term), x_b = |y_c x F|, y_b = |F x x_b|, z_b = x_b x y_b with y_c = (0, 1, 0); R_ref = [x_b y_b z_b];
    body rates od = (D2 / B1, D1 / B1, -B3 D1 / (B1 C3)) with D1 = x_b . jerk, D2 = -y_b . jerk, B1 = |F|, B3 = -y_c . z_b, C3 = |y_c x z_b|. Cached per reference object."""
    key = id(ref)
    if key in _FLAT and _FLAT[key][0] is ref:
        return _FLAT[key][1], _FLAT[key][2]
    from scipy.spatial.transform import Rotation
    g = 9.81; F = ref.acc + np.array([0.0, 0.0, g]); yc = np.array([0.0, 1.0, 0.0])
    nrm = lambda a: a / np.linalg.norm(a, axis=-1, keepdims=True)
    xb = nrm(np.cross(yc, F)); yb = nrm(np.cross(F, xb)); zb = np.cross(xb, yb)
    Rm = np.stack([xb, yb, zb], -1); q = Rotation.from_matrix(Rm).as_quat()
    B1 = np.linalg.norm(F, axis=-1); D1 = (xb * ref.jerk).sum(-1); D2 = -(yb * ref.jerk).sum(-1); B3 = -(yc * zb).sum(-1); C3 = np.linalg.norm(np.cross(np.broadcast_to(yc, zb.shape), zb), axis=-1)
    w = np.stack([D2 / B1, D1 / B1, -B3 * D1 / (B1 * C3)], -1)
    _FLAT[key] = (ref, q, w); return q, w
