"""Costs of the DiffCoTune spec (docs/diffcotune_spec.md section 3) on the output of s0_model.rollout.

J_task  = mean over the window of ||p - p*||^2 (3-D position, SI m^2 times `scale`)
J_sysId = mean over the window of ||p-p_s||^2 + ||v-v_s||^2 + ||q - q_s||^2 + ||w-w_s||^2 (unweighted sum of the squared
errors of each part, paper p. 4; units m, m/s, -, rad/s; attitude part = squared norm of the quaternion difference with
sign alignment: PLACEHOLDER for the open choice 6.4 of the spec).
Unit / normalisation (`scale`) and window are open choices of the spec (6.2, 6.6); the defaults here are SI units and the
frsim scoring window (task) / the scenario part (sysId), used by the checks only.
"""
import os
import numpy as np
import torch

import reference  # noqa: F401  (puts the simulator package on sys.path)
from frsim import config


def window(t, scenario_s, kind='task'):
    """Boolean mask of the rows of the cost window: task = scenario time > 2 s (frsim SCORE_AFTER_S) to the scenario end;
    sysid = the whole scenario part (0 < t <= scenario end)."""
    lo = config.SCORE_AFTER_S if kind == 'task' else 0.0
    return (t > lo) & (t <= scenario_s)


# Stage 8: DIFFCOTUNE_METHOD=paper selects the task cost of the paper (arXiv 2505.24068, p. 4: c = ||x_t - x_t*||^2, "for states that evolve in different geometric spaces (e.g. position in R^3 and quaternions in S^3)
# an unweighted sum of the individual cumulative norm squared errors"): ||p - p*||^2 + ||v - v*||^2 + ||q - q*||^2 + ||w - w*||^2 against the reference STATE (position, velocity from the trajectory; attitude and
# body rates by differential flatness, reference.flat_ref), the same four parts as J_sysId. Default (ours): the position-only cost below.
TASK_FORM = 'paper' if os.environ.get('DIFFCOTUNE_METHOD', 'ours') == 'paper' else 'position'


def j_task(out, scenario_s, scale=1.0):
    """(B,) mean squared 3-D position error [m^2 * scale] against the reference of the same ticks (TASK_FORM 'paper': the unweighted sum over p, v, q, w, see above)."""
    if TASK_FORM == 'paper':
        return j_task_paper(out, scenario_s, scale)
    m = torch.as_tensor(window(out['t'], scenario_s, 'task'))
    e = out['x'][m] - out['ref_pos'][m][:, None, :]
    return scale * (e * e).sum(-1).mean(0)


def j_sysid(out, target, scenario_s, scale=1.0, parts=False, states=('p', 'v', 'q', 'w')):
    """(B,) closed-loop state error against a Target (ingest.on_grid) on the model's output grid. `states` = the parts that enter the
    total (Stage 3b switch; default all four as in Stage 3); `parts=True` always returns all four."""
    m = torch.as_tensor(window(out['t'], scenario_s, 'sysid'))
    f = lambda a: torch.as_tensor(a, dtype=out['x'].dtype)[m][:, None, :]
    dp = out['x'][m] - f(target.p); dv = out['v'][m] - f(target.v); dw = out['w'][m] - f(target.w)
    qt = f(target.q); sgn = torch.sign((out['q'][m] * qt).sum(-1, keepdim=True)).detach()
    sgn = torch.where(sgn == 0, torch.ones_like(sgn), sgn)
    dq = out['q'][m] - sgn * qt
    p = {'p': (dp * dp).sum(-1).mean(0), 'v': (dv * dv).sum(-1).mean(0), 'q': (dq * dq).sum(-1).mean(0), 'w': (dw * dw).sum(-1).mean(0)}
    total = scale * sum(p[k] for k in states)
    return (total, p) if parts else total


def j_task_paper(out, scenario_s, scale=1.0, parts=False):
    """Stage 8: (B,) mean over the task window of ||p - p*||^2 + ||v - v*||^2 + ||q - s q*||^2 + ||w - w*||^2 (s = sign alignment of the quaternions, detached); out['ref_q'] = (rows, 7) = q* (x, y, z, w) and w* (rad/s)."""
    m = torch.as_tensor(window(out['t'], scenario_s, 'task'))
    rq = out['ref_q'][m]; qs = rq[:, None, 0:4]; ws = rq[:, None, 4:7]
    dp = out['x'][m] - out['ref_pos'][m][:, None, :]; dv = out['v'][m] - out['ref_vel'][m][:, None, :]; dw = out['w'][m] - ws
    sgn = torch.sign((out['q'][m] * qs).sum(-1, keepdim=True)).detach(); sgn = torch.where(sgn == 0, torch.ones_like(sgn), sgn); dq = out['q'][m] - sgn * qs
    p = {'p': (dp * dp).sum(-1).mean(0), 'v': (dv * dv).sum(-1).mean(0), 'q': (dq * dq).sum(-1).mean(0), 'w': (dw * dw).sum(-1).mean(0)}
    tot = scale * (p['p'] + p['v'] + p['q'] + p['w'])
    return (tot, p) if parts else tot
