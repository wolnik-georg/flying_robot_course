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
# Protocol v2 (user decision 23 Nov): DIFFCOTUNE_METHOD=ours2 = the frozen 'ours' method with the task cost of DiffTune's hardware loss, ||p - p*||^2 + tr(I - R*^T R)/2 (weight 1, per tick, mean over the task window);
# R* = the attitude of the reference by differential flatness (reference.flat_ref, the same q* as TASK_FORM 'paper'). tr(I - R*^T R)/2 = 1 - cos(angle) = 2 (1 - <q, q*>^2) for unit quaternions (sign invariant).
TASK_FORM = {'paper': 'paper', 'ours2': 'att', 'ours21': 'att'}.get(os.environ.get('DIFFCOTUNE_METHOD', 'ours'), 'position')


def j_task(out, scenario_s, scale=1.0):
    """(B,) mean squared 3-D position error [m^2 * scale] against the reference of the same ticks (TASK_FORM 'paper': the unweighted sum over p, v, q, w, see above)."""
    if TASK_FORM == 'paper':
        return j_task_paper(out, scenario_s, scale)
    if TASK_FORM == 'att':
        return j_task_att(out, scenario_s, scale)
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


def att_err(q, qs):
    """tr(I - R*^T R)/2 = 1 - cos(angle between the attitudes) = 2 (1 - <q, q*>^2) for unit quaternions (rows x, y, z, w); sign invariant. Torch or numpy; the quaternions are normalised here."""
    if isinstance(q, torch.Tensor):
        q = q / q.norm(dim=-1, keepdim=True); qs = qs / qs.norm(dim=-1, keepdim=True); return 2.0 * (1.0 - (q * qs).sum(-1) ** 2)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True); qs = qs / np.linalg.norm(qs, axis=-1, keepdims=True); return 2.0 * (1.0 - (q * qs).sum(-1) ** 2)


def j_task_att(out, scenario_s, scale=1.0, parts=False):
    """Protocol v2: (B,) mean over the task window of ||p - p*||^2 + tr(I - R*^T R)/2 (both weight 1); out['ref_q'][:, 0:4] = q* (x, y, z, w). parts=True also returns {'p': ..., 'att': ...}."""
    m = torch.as_tensor(window(out['t'], scenario_s, 'task'))
    qs = out['ref_q'][m][:, None, 0:4]
    dp = out['x'][m] - out['ref_pos'][m][:, None, :]
    p = {'p': (dp * dp).sum(-1).mean(0), 'att': att_err(out['q'][m], qs).mean(0)}
    tot = scale * (p['p'] + p['att'])
    return (tot, p) if parts else tot


def trace_cost(tr, ref, scenario_s, att=None):
    """Cost of a state trace (n, 23, ingest layout; model frame) over the scored window (t > SCORE_AFTER_S, t <= scenario end), against the model reference `ref`:
    returns dict(pos=mean ||p - p*||^2 [m^2], z=mean (z - z*)^2, att=mean tr(I - R*^T R)/2 or None, cost=pos + att (att form) or pos (position form)).
    p* = the reference position interpolated at the trace times (as usd_ingest.real_cost); q* = the flatness attitude of the reference at the nearest reference tick (1 ms)."""
    att = (TASK_FORM == 'att') if att is None else att
    t = tr[:, 0]; m = (t > config.SCORE_AFTER_S) & (t <= scenario_s)
    rp = np.stack([np.interp(t[m], ref.time_s, ref.pos[:, j]) for j in range(3)], 1)
    e = tr[m, 1:4] - rp; pos = float((e * e).sum(-1).mean()); z = float((e[:, 2] ** 2).mean())
    a = None
    if att:
        qref, _ = reference.flat_ref(ref); idx = np.clip(np.searchsorted(ref.time_s, t[m]), 0, len(ref.time_s) - 1)
        i0 = np.clip(idx - 1, 0, len(ref.time_s) - 1); idx = np.where(np.abs(ref.time_s[i0] - t[m]) <= np.abs(ref.time_s[idx] - t[m]), i0, idx)
        a = float(att_err(tr[m, 7:11], qref[idx]).mean())
    return dict(pos=pos, z=z, att=a, cost=pos + (a if a is not None else 0.0))
