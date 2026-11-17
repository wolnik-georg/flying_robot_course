"""DiffCoTune loop for S0 (Stage 3 dry run): "real drone" = frsim with noise on, tuning model = the differentiable S0 model.

One round (docs/diffcotune_spec.md section 2 / 3): fly theta_l once on each tuning scenario in frsim, fit beta (theta_l fixed, J_sysId
on the full state), tune theta (beta fixed, J_task on the 3-D position), the next round flies theta_{l+1}. Adam with the stop rule of the paper
on the cost normalised by its value at the start of the inner loop (decision 4, to be confirmed by this dry run); Adam state reset at the
start of every inner loop (OUR choice, the paper does not say). Everything runs in a pool of 4 spawned worker processes (one tuning scenario
per process; a worker also runs the frsim flights); the main process only does Adam and bookkeeping.
Parameters = log-factors: x = [phi (10 gains, relative to the S0 defaults), psi (4 beta, relative to the frozen calibration)].
"""
import json
import multiprocessing as mp
import os
import sys
import time
from pathlib import Path

import numpy as np

# Stage 3c: one torch-compile helper per process instead of 22 (memory); the setting is inherited by the spawned workers. Numbers are checked in C1.
os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import reference as R  # noqa: E402  (also puts simulator/ on sys.path)
from frsim import config, controllers  # noqa: E402
from frsim.scenarios import study_cells, tuning_cells  # noqa: E402

# Stage 5: the controller of the run (S0 default = all earlier behaviour). Set DIFFCOTUNE_CONTROLLER=S2|S1|S3 in the environment BEFORE the import (spawned workers inherit it).
CTRL = os.environ.get('DIFFCOTUNE_CONTROLLER', 'S0')
MODEL_MODULES = {'S0': 's0_model', 'S2': 's2_model', 'S1': 's1_model', 'S3': 's3_model'}
# Stage 5b: how the gradient of the scalar cost is computed. 'reverse' (default, all earlier results) = torch.autograd backward, compiled block; 'forward' = forward mode (torch.func.jacfwd: one tangent per parameter,
# batched with vmap, eager block); both give the same autodiff gradient. Environment variable DIFFCOTUNE_GRAD_MODE, set BEFORE the import (spawned workers inherit it).
GRAD_MODE = os.environ.get('DIFFCOTUNE_GRAD_MODE', 'reverse')
# Stage 5c: 'mixed' = reverse everywhere except the noise-free beta fit on A1, whose gradient is the exact forward sensitivity computed in the compiled model (fsens.py); same gradient, other differentiation mode per scenario.
# Stage 7: 'mixed2' = 'mixed' plus the noisy task gradient (tuning step) on A1 by the same compiled forward sensitivity (reverse mode on A1 is rounding driven also with noise at the fitted beta of the rdfix runs); A8, CIRCLE, FIG8 stay on reverse.
# Stage 7 (user decision 15 Nov): 'cfwd' = the compiled forward sensitivity (fsens.py) for EVERY gradient, fit and tuning, on every scenario (S0-S3); reverse mode stays as 'task_R' / 'sysid_R' in the checks. Same autodiff gradient, other way of computing it.
assert GRAD_MODE in ('reverse', 'forward', 'mixed', 'mixed2', 'cfwd'), GRAD_MODE
# Stage 8 (user decisions 15 Nov): DIFFCOTUNE_METHOD = ours | paper (default ours: every earlier result stays reproducible). 'paper' = the method as close to Krishna et al. 2025 (arXiv 2505.24068) as the study allows:
# task cost = unweighted full-state ||x - x*||^2 (costs.TASK_FORM), J_sysId = unweighted p + v + q + w, the paper's stop rule on the raw cost (change < 1e-3 or rise > 1e-3, cap K/2 = 50 epochs, the LAST iterate is returned),
# no sensor noise in the tuning step, no +-25 % trust region (gains stay positive: they are exp(phi) times the defaults), plain Adam lr from DIFFCOTUNE_LR (Stage 8 step 1c), 6 rounds. 'ours' = the frozen method of Stages 3-7, now with 6 rounds (decision 8).
METHOD = os.environ.get('DIFFCOTUNE_METHOD', 'ours')
assert METHOD in ('ours', 'paper'), METHOD
METHOD_SETTINGS = {'ours': dict(lr=0.01, best=True, states=('p', 'v', 'q'), gain_limit=0.25, noise=True, paper_stop=False, n_rounds=6),
                   'paper': dict(lr=float(os.environ.get('DIFFCOTUNE_LR', '0.01'))          # Stage 8 step 1c: 0.01 chosen by the probe rule (S0, real error after round 1: 1e-3 18.31, 3e-3 17.67, 1e-2 17.24 cm)
                           , best=False, states=('p', 'v', 'q', 'w'), gain_limit=None, noise=False, paper_stop=True, n_rounds=6)}
N_BETA = 5 if CTRL in ('S1', 'S3') else 4          # S1 / S3 carry the rpm delay as a fifth beta (decision 5b.2 of the plan)
# Stage 5c, RPM delay fixed (user decision, 14 Nov 2026): DIFFCOTUNE_FIX_RD=1 keeps the fifth beta (RPM delay) of S1 / S3 at the calibration (psi = 0, 4.0502 ms) and fits only the other four; the model still has N_BETA = 5 entries. Default off (= all earlier results).
FIX_RD = os.environ.get('DIFFCOTUNE_FIX_RD', '0') == '1' and N_BETA == 5
PSI_FIT_IDX = np.arange(10, 10 + N_BETA - (1 if FIX_RD else 0))             # the beta entries the fit moves


def model():
    import importlib
    return importlib.import_module(MODEL_MODULES[CTRL])


GAIN_NAMES = list(controllers.TREE_GAIN_NAMES)
GAIN_DEFAULT = np.array([controllers.TREE_DEFAULT_GAINS[k] for k in GAIN_NAMES])
ALL_STATES = ('p', 'v', 'q', 'w')          # Stage 3 J_sysId: position, velocity, attitude, body rates
BETA_NAMES = ['thrust_gain', 'motor_lag_ms', 'downwash_scale', 'plume_scale']
SCORE_AFTER_S = config.SCORE_AFTER_S
CRASH_TILT_DEG = config.CRASH_TILT_DEG['tree']

# Stage 7 (user decision 15 Nov): the tuning set has FIVE scenarios: A1, A8, CIRCLE, FIG8 and A4 cell 1 (study cell 0 of A4: dz 0.4, speed 0.35, offset 0.125, line), test = A4 cell 2 (held out; A4 cell 1 is in-sample) and OVAL (frsim only, not flyable at the pad).
# DIFFCOTUNE_TUNING_SET=4 gives the Stage 3-6 set (four scenarios, test A4 cells 1 and 2 and OVAL) for checks that must reproduce earlier results. Default: 5.
TUNING_SET = os.environ.get('DIFFCOTUNE_TUNING_SET', '5')
assert TUNING_SET in ('4', '5'), TUNING_SET
TUNING = [(s, tuning_cells(s)[0]) for s in ('A1', 'A8', 'CIRCLE', 'FIG8')] + ([('A4', study_cells('A4')[0])] if TUNING_SET == '5' else [])
TEST = ([('A4', study_cells('A4')[1]), ('OVAL', study_cells('OVAL')[0])] if TUNING_SET == '5' else [('A4', study_cells('A4')[0]), ('A4', study_cells('A4')[1]), ('OVAL', study_cells('OVAL')[0])])
P1 = dict(overrides={'thrust_gain': 'x0.8', 'motor_lag_ms': 45.0, 'downwash_scale': 'x1.3', 'plume_scale': 'x0.7'}, variant='nominal')
P2 = dict(overrides=P1['overrides'], variant='md10')           # P1 plus the measured state delay of 10 ms, which no beta covers
P1RD = dict(overrides=dict(P1['overrides'], rpm_delay_ms=8.0), variant='nominal')       # Stage 5: P1 plus an rpm delay of 8 ms (plan dry-run value) instead of the calibrated 4.05 ms
PLANTS = {'P1': P1, 'P2': P2, 'P1RD': P1RD}
PSI_TRUE = np.log([0.8, 45.0 / 30.0, 1.3, 0.7])                 # true beta of P1 / P2 as log-factors relative to the calibration
RD_TRUE_MS = 8.0                                                  # Stage 5: true rpm delay of P1RD
if N_BETA == 5:
    PSI_TRUE = np.r_[PSI_TRUE, np.log(RD_TRUE_MS / config.CALIBRATED_PLANT.rpm_delay_ms)]


# ---------------------------------------------------------------------------------------------------------- worker side
def _w_init():
    import torch
    torch.set_num_threads(int(os.environ.get('DIFFCOTUNE_THREADS', '2')))      # Stage 3c C2: threads per worker (default 2 as before)
    sys.path.insert(0, str(HERE.parent))


def worker_rollout(args):
    """(kind, scenario, cell, x (14,), traces) -> (J, gradient (14,)) or (J, None) for the *_fwd kinds. kind: task | sysid (+ _fwd)."""
    kind, scen, cell, x, traces = args[:5]
    force_fwd_mode = kind.endswith('_F')          # Stage 5b: kind 'task_F' / 'sysid_F' = forward-mode gradient whatever GRAD_MODE is (checks and timing); 'task_R' / 'sysid_R' = reverse
    force_rev_mode = kind.endswith('_R')
    force_comp_fwd = kind.endswith('_C')          # Stage 5c: compiled forward sensitivity (fsens.py) for all parameters, any scenario, task / sysid, with or without noise
    if force_fwd_mode or force_rev_mode or force_comp_fwd:
        kind = kind[:-2]
    states = args[5] if len(args) > 5 else ALL_STATES
    noise_seeds = args[6] if len(args) > 6 else None        # Stage 3d (variant A): task cost with frsim's sensor noise, one row per seed
    import torch
    import costs as C
    import ingest
    S = model()
    ref = R.build(scen, cell)
    fwd = kind.endswith('_fwd')
    mixed_a1 = GRAD_MODE in ('mixed', 'mixed2') and kind == 'sysid' and scen == 'A1' and not force_rev_mode and not force_fwd_mode and not fwd
    mixed_task_a1 = GRAD_MODE == 'mixed2' and kind == 'task' and scen == 'A1' and not force_rev_mode and not force_fwd_mode and not fwd          # Stage 7
    cfwd_all = GRAD_MODE == 'cfwd' and not force_rev_mode and not force_fwd_mode and not fwd          # Stage 7
    if (force_comp_fwd or mixed_a1 or mixed_task_a1 or cfwd_all) and not fwd:
        import fsens
        n_all = np.asarray(x).shape[0]
        return fsens.gradient(CTRL, ref, ref.scenario_s, kind, x, range(n_all) if force_comp_fwd else (range(10) if (mixed_task_a1 or (cfwd_all and kind == 'task')) else range(10, n_all)), traces=traces, states=states, noise_seeds=noise_seeds, model=S)
    if (force_fwd_mode or (GRAD_MODE == 'forward' and not force_rev_mode)) and not fwd:
        return _worker_rollout_forward(kind, scen, ref, x, traces, states, noise_seeds, S)
    grad = (not fwd) or bool(noise_seeds)          # the compiled CPU graph of several rows needs autograd on even for the cost alone (Stage 3c: Inductor inference bug)
    z = torch.tensor(np.asarray(x)[None], dtype=torch.float64, requires_grad=grad)
    with torch.set_grad_enabled(grad):
        if kind.startswith('task') and noise_seeds:
            nz = np.stack([S.frsim_noise(sd, ref.n_ticks) for sd in noise_seeds], 1)
            out = S.rollout(ref, z[:, :10].expand(len(noise_seeds), 10), z[:, 10:].expand(len(noise_seeds), z.shape[1] - 10), compile=True, noise=nz)
            Js = [C.j_task(out, ref.scenario_s).mean()]          # mean over the noise seeds (rows)
        elif kind.startswith('task'):
            out = S.rollout(ref, z[:, :10], z[:, 10:], compile=True)
            Js = [C.j_task(out, ref.scenario_s).sum()]
        else:
            out = S.rollout(ref, z[:, :10], z[:, 10:], compile=True)
            Js = [C.j_sysid(out, ingest.on_grid(ingest.from_trace(tr), out['t']), ref.scenario_s, states=states).sum() / len(traces) for tr in traces]
    J = sum(Js)
    if fwd:
        return float(J), None
    for i, Ji in enumerate(Js):         # one backward per trace (same gradient as the sum, one graph at a time)
        Ji.backward(retain_graph=i < len(Js) - 1)
    return float(J), z.grad[0].numpy().copy()


def _worker_rollout_forward(kind, scen, ref, x, traces, states, noise_seeds, S):
    """Stage 5b: the same (J, gradient) as worker_rollout, gradient by forward mode (jacfwd = one jvp tangent per parameter, batched by vmap, eager block)."""
    import torch
    from torch.func import jacfwd
    import costs as C
    import ingest
    nz = np.stack([S.frsim_noise(sd, ref.n_ticks) for sd in noise_seeds], 1) if (kind.startswith('task') and noise_seeds) else None
    tg = None if kind.startswith('task') else [ingest.from_trace(tr) for tr in traces]

    def f(z):
        if nz is not None:
            out = S.rollout(ref, z[:, :10].expand(len(noise_seeds), 10), z[:, 10:].expand(len(noise_seeds), z.shape[1] - 10), compile=False, noise=nz)
            J = C.j_task(out, ref.scenario_s).mean()
        elif kind.startswith('task'):
            out = S.rollout(ref, z[:, :10], z[:, 10:], compile=False)
            J = C.j_task(out, ref.scenario_s).sum()
        else:
            out = S.rollout(ref, z[:, :10], z[:, 10:], compile=False)
            J = sum(C.j_sysid(out, ingest.on_grid(t_, out['t']), ref.scenario_s, states=states).sum() / len(traces) for t_ in tg)
        return J, J.detach()
    z = torch.tensor(np.asarray(x)[None], dtype=torch.float64)
    with torch.no_grad():
        jac, J = jacfwd(f, has_aux=True)(z)
    return float(J), jac.reshape(-1).numpy().copy()


def worker_fly(args):
    """(scenario, cell, gains dict, plant spec, seed) -> (state trace, stable, max tilt deg in the scenario part)."""
    scen, cell, gains, plant, seed = args
    from frsim.episode import EpisodeConfig, run_episode
    p = config.plant_with(plant['overrides'])
    r = run_episode(EpisodeConfig(CTRL, scen, cell, gains, plant['variant'], seed, plant=p, noise=True, record_state=True))
    return r.state_trace, bool(r.stable), float(r.tilt_max_deg)


# ----------------------------------------------------------------------------------------------------------- main side
class Pool:
    def __init__(self, n=None, states=ALL_STATES, noise_seeds=None):
        n = len(TUNING) if n is None else n          # Stage 7: one worker per tuning scenario (5)
        self.states = tuple(states); self.noise_seeds = tuple(noise_seeds) if noise_seeds else None
        self.pool = mp.get_context('spawn').Pool(n, initializer=_w_init)
        self.n = n

    def close(self):
        self.pool.close(); self.pool.join()

    def rollouts(self, kind, x, traces_by_scen, scen_list=TUNING):
        """Mean cost and mean gradient over the scenarios of scen_list (equal weights)."""
        res = self.pool.map(worker_rollout, [(kind, s, c, x, traces_by_scen.get(i) if traces_by_scen else None, self.states, self.noise_seeds) for i, (s, c) in enumerate(scen_list)], chunksize=1)
        J = float(np.mean([r[0] for r in res]))
        g = None if res[0][1] is None else np.mean([r[1] for r in res], axis=0)
        return J, g

    def fly(self, jobs):
        return self.pool.map(worker_fly, jobs, chunksize=1)


def gains_of(phi):
    return {k: float(v * np.exp(p)) for k, v, p in zip(GAIN_NAMES, GAIN_DEFAULT, phi)}


def task_window(t, scenario_s):
    return (t > SCORE_AFTER_S) & (t <= scenario_s)


def flight_cost(trace, scenario_s):
    """(mean squared 3-D position error [m^2], mean squared z error [m^2]) over the task window of a state trace."""
    t = trace[:, 0]; m = task_window(t, scenario_s)
    e = trace[m, 1:4] - trace[m, 14:17]
    return float((e * e).sum(-1).mean()), float((e[:, 2] ** 2).mean())


def floor_failed(trace, scenario_s):
    """Stage 3d floor rule: the mean z of the scored part is < 50 % of the mean reference z (the drone stays on the floor). Reported for every flight; it
    does not trigger a repeat (only the tilt rule does, as before)."""
    t = trace[:, 0]; m = task_window(t, scenario_s)
    return bool(trace[m, 3].mean() < 0.5 * trace[m, 16].mean())


def fly_set(pool, phi, plant, scen_list, seeds, max_tries=3):
    """One flight per scenario with the gains exp(phi) * defaults. A crashed flight (tilt >= 70 deg in the scenario part) is repeated with
    another noise seed and is not used (plan section 4 Stage 4); returns list of dict(trace, cost, z_cost, seed, tries, crashed)."""
    gains = gains_of(phi); out = []
    todo = list(range(len(scen_list))); tries = {i: 0 for i in todo}; final = {}
    while todo:
        jobs = [(scen_list[i][0], scen_list[i][1], gains, plant, seeds[i] + 1000 * tries[i]) for i in todo]
        res = pool.fly(jobs); nxt = []
        for i, (tr, stable, tilt) in zip(todo, res):
            tries[i] += 1
            c, cz = flight_cost(tr, R.build(*scen_list[i]).scenario_s)
            final[i] = dict(trace=tr, cost=c, z_cost=cz, floor=floor_failed(tr, R.build(*scen_list[i]).scenario_s), seed=jobs[todo.index(i)][4], tries=tries[i], crashed=not stable, tilt=tilt)
            if not stable and tries[i] < max_tries:
                nxt.append(i)
        todo = nxt
    return [final[i] for i in range(len(scen_list))]


def round_cost(flights):
    """Real cost of a gain set: mean over the scenarios of the mean squared 3-D error [m^2]; 3-D RMS and z RMS in cm."""
    c = float(np.mean([f['cost'] for f in flights])); cz = float(np.mean([f['z_cost'] for f in flights]))
    return dict(cost=c, rms3d_cm=100 * c ** 0.5, z_rms_cm=100 * cz ** 0.5, crashed=int(sum(f['crashed'] for f in flights)), floor=int(sum(f['floor'] for f in flights)), tries=[f['tries'] for f in flights])


def adam_inner(eval_fn, x0, idx, lr, K2=50, checks=None, check_fn=None, label='', stop=True, best=False, limit=None, paper_stop=False):
    """Adam on the coordinates idx of x (others fixed), at most K2 epochs. eval_fn(x) -> (J, gradient). Stop rule of the paper (p. 4) on the
    cost normalised by its value at the start of the loop: stop when |dJn| < 1e-3 or Jn rose by more than 1e-3; the iterate whose cost
    triggered the stop is returned (the paper returns the last iterate, also after a rise). A non-finite cost stops the loop and returns the
    previous iterate (not stated in the paper). best=True (Stage 3b, spec deviation 8; needs stop=False): K2 updates are always done, the
    final iterate is evaluated as well (K2 + 1 evaluations), the iterate with the lowest cost is returned and `best_epoch` is its index. Adam: beta1 0.9, beta2 0.999, eps 1e-8, state reset at the start of the loop."""
    if paper_stop:
        return _adam_paper(eval_fn, x0, idx, lr, K2, label, limit)
    if best:
        return _adam_best(eval_fn, x0, idx, lr, K2, checks, check_fn, label, limit)
    x = np.array(x0, float); m = np.zeros(len(idx)); v = np.zeros(len(idx)); log = []; J0 = None; Jn_prev = None; x_prev = x.copy(); reason = 'max epochs'
    for k in range(K2):
        t0 = time.time()
        J, g = eval_fn(x)
        if not np.isfinite(J) or not np.all(np.isfinite(g[idx])):
            x = x_prev; reason = 'non-finite'; break
        if J0 is None:
            J0 = J
        Jn = J / J0
        rec = dict(k=k, J=J, Jn=Jn, x=x[idx].tolist(), g=g[idx].tolist(), seconds=time.time() - t0)
        if checks and k in checks and check_fn is not None:
            rec['fd_check'] = check_fn(x, idx)
        log.append(rec)
        if stop and k >= 1:
            if abs(Jn - Jn_prev) < 1e-3:
                reason = 'converged'; break
            if Jn > Jn_prev + 1e-3:
                reason = 'rose'; break
        m = 0.9 * m + 0.1 * g[idx]; v = 0.999 * v + 0.001 * g[idx] ** 2
        mh = m / (1 - 0.9 ** (k + 1)); vh = v / (1 - 0.999 ** (k + 1))
        x_prev = x.copy(); Jn_prev = Jn
        x = x.copy(); x[idx] = x[idx] - lr * mh / (np.sqrt(vh) + 1e-8)
    return x, dict(label=label, epochs=len(log), reason=reason, J0=J0, log=log)


def _jn_best(rf):
    """Stage 4b: normalised cost at the returned iterate ('J_best' / 'J0', 'best_epoch'), as text; the printed 'Jn' of the last epoch is not the cost of the returned iterate when best=True."""
    if rf.get('J_best') is None or not rf.get('J0'):
        return 'n/a (best=False: the last iterate is returned)'
    return '%.3f (epoch %d)' % (rf['J_best'] / rf['J0'], rf['best_epoch'])


def _adam_paper(eval_fn, x0, idx, lr, K2, label, limit=None):
    """Stage 4e (T1): the literal stop rule of the paper (arXiv 2505.24068, p. 4, Alg. 2 and 3) on the RAW cost in SI units (no normalisation):
    stop when |J_k - J_{k-1}| < 1e-3 or J_k > J_{k-1} + 1e-3 (k >= 1); the LAST iterate is returned, i.e. the iterate x_k whose cost J_k triggered the
    stop, also after a rise (the paper returns (beta_k, theta_i)). At most K2 epochs; if no stop fires the iterate after K2 updates is not evaluated
    (the last evaluated iterate x_{K2-1} is returned). Same Adam (beta1 0.9, beta2 0.999, eps 1e-8, reset per loop) and the same projection onto
    +-limit (log half width) as _adam_best. Non-finite cost: previous iterate (not stated in the paper). `epochs` = number of cost evaluations (rollouts)."""
    hits = np.zeros(len(idx), int); lo = np.array(x0, float)[idx] - (limit or 0); hi = np.array(x0, float)[idx] + (limit or 0)
    x = np.array(x0, float); m = np.zeros(len(idx)); v = np.zeros(len(idx)); log = []; J0 = None; J_prev = None; x_prev = x.copy(); reason = 'max epochs'
    for k in range(K2):
        t0 = time.time()
        J, g = eval_fn(x)
        if not np.isfinite(J) or not np.all(np.isfinite(g[idx])):
            x = x_prev; reason = 'non-finite'; break
        if J0 is None:
            J0 = J
        log.append(dict(k=k, J=J, Jn=J / J0, x=x[idx].tolist(), g=g[idx].tolist(), seconds=time.time() - t0))
        if k >= 1:
            if abs(J - J_prev) < 1e-3:
                reason = 'converged |dJ| < 1e-3'; break
            if J > J_prev + 1e-3:
                reason = 'rose dJ > 1e-3'; break
        m = 0.9 * m + 0.1 * g[idx]; v = 0.999 * v + 0.001 * g[idx] ** 2
        mh = m / (1 - 0.9 ** (k + 1)); vh = v / (1 - 0.999 ** (k + 1))
        x_prev = x.copy(); J_prev = J
        x = x.copy(); x[idx] = x[idx] - lr * mh / (np.sqrt(vh) + 1e-8)
        if limit:
            xc = np.clip(x[idx], lo, hi); hits += (xc != x[idx]); x[idx] = xc
    return x, dict(label=label, epochs=len(log), reason=reason, J0=J0, J_end=float(log[-1]['J']) if log else None, best_epoch=None, J_best=None, log=log, limit=limit, hits=hits.tolist(), paper_stop=True)


def _adam_best(eval_fn, x0, idx, lr, K2, checks, check_fn, label, limit=None):
    """limit (Stage 3d, deviation 11): None, or the half width w = log(1.25) of a box around the starting point x0[idx] (the flown gains theta_l); after every
    Adam step the coordinates are projected onto [x0 - w, x0 + w] (projected Adam). `hits` = per coordinate the number of projected steps on which the
    projection changed the value."""
    hits = np.zeros(len(idx), int); lo = np.array(x0, float)[idx] - (limit or 0); hi = np.array(x0, float)[idx] + (limit or 0)
    x = np.array(x0, float); m = np.zeros(len(idx)); v = np.zeros(len(idx)); log = []; J0 = None; reason = 'max epochs'
    bx, bJ, bk = x.copy(), np.inf, -1
    for k in range(K2 + 1):
        t0 = time.time()
        J, g = eval_fn(x)
        if not np.isfinite(J) or not np.all(np.isfinite(g[idx])):
            reason = 'non-finite'; break
        if J0 is None:
            J0 = J
        rec = dict(k=k, J=J, Jn=J / J0, x=x[idx].tolist(), g=g[idx].tolist(), seconds=time.time() - t0)
        if checks and k in checks and check_fn is not None:
            rec['fd_check'] = check_fn(x, idx)
        log.append(rec)
        if J < bJ:
            bx, bJ, bk = x.copy(), J, k
        if k == K2:
            break
        m = 0.9 * m + 0.1 * g[idx]; v = 0.999 * v + 0.001 * g[idx] ** 2
        mh = m / (1 - 0.9 ** (k + 1)); vh = v / (1 - 0.999 ** (k + 1))
        x = x.copy(); x[idx] = x[idx] - lr * mh / (np.sqrt(vh) + 1e-8)
        if limit:
            xc = np.clip(x[idx], lo, hi); hits += (xc != x[idx]); x[idx] = xc
    return bx, dict(label=label, epochs=len(log), reason=reason, J0=J0, best_epoch=bk, J_best=float(bJ), log=log, limit=limit, hits=hits.tolist())


def fd_check(pool, kind, x, idx, traces, h=1e-6, which=(1, 3)):
    """Central finite difference of the combined cost with respect to x[idx[which]] vs the autograd gradient at the same point."""
    J, g = pool.rollouts(kind, x, traces)
    out = []
    for w in which:
        i = idx[w]
        xp = np.array(x, float); xm = xp.copy(); xp[i] += h; xm[i] -= h
        fp, _ = pool.rollouts(kind + '_fwd', xp, traces); fm, _ = pool.rollouts(kind + '_fwd', xm, traces)
        out.append(dict(param=i, autograd=float(g[i]), fd=float((fp - fm) / (2 * h))))
    return out


def run_loop(pool, plant, method, lr, n_rounds=6, extra_final_flight=False, checks_rounds=(), check_epochs=(0, 10, 30), log=print, stop=True, best=False, skip_last_fit=False, gain_limit=None, noise_seeds_of=None, on_round=None, seed_base=0, paper_stop=False, trace_fn=None, stop_after=None):
    """method: 'cotune' (fit beta then tune theta every round) or 'tune_only' (theta tuned in the unfitted model, beta0). Rounds l = 0 ... n_rounds-1
    fly theta_l; the update after the last round is only computed when `extra_final_flight` (pilot: its cost is the criterion) or, for the
    fit, to report beta after the last round. Returns the record dict."""
    x = np.zeros(10 + N_BETA); rec = dict(plant=plant, method=method, lr=lr, stop=stop, best_iterate=best, states=list(pool.states), rounds=[]); keep = None   # keep = keep-best over the flown gain sets (Stage 3d: renamed, it used to shadow the flag `best`)
    phi_idx = np.arange(10); psi_idx = PSI_FIT_IDX
    last = n_rounds + (1 if extra_final_flight else 0)
    for l in range(last):
        t0 = time.time()
        fl = fly_set(pool, x[:10], plant, TUNING, [seed_base + 10 * l + i for i in range(len(TUNING))])      # Stage 3e: seed_base 100 for the new flight seeds (default 0 = Stage 3b to 3d)
        rc = round_cost(fl)
        r = dict(round=l, phi=x[:10].tolist(), psi=x[10:].tolist(), real=rc, seconds_fly=time.time() - t0,
                 flights=[dict(scenario=TUNING[i][0], cost=f['cost'], z_cost=f['z_cost'], floor=f['floor'], crashed=f['crashed'], tries=f['tries'], seed=f['seed']) for i, f in enumerate(fl)])   # Stage 3d: per flight
        if keep is None or rc['cost'] <= keep['cost']:
            keep = dict(cost=rc['cost'], round=l, phi=x[:10].tolist())
        log('[%s %s lr %g] round %d  real 3-D RMS %.2f cm  z %.2f cm  crashes %d  psi err %s' % (plant['variant'], method, lr, l, rc['rms3d_cm'], rc['z_rms_cm'], rc['crashed'], np.round(x[10:] - PSI_TRUE, 3).tolist()))
        traces = {i: [f['trace'] if trace_fn is None else trace_fn(f['trace'], l, i)] for i, f in enumerate(fl)}      # Stage 4e: trace_fn = measured-gyro variants of the fit target (default: the state trace as it is)
        t1 = time.time()
        if method == 'cotune' and l < n_rounds and not (skip_last_fit and l == n_rounds - 1):   # skip_last_fit: theta_6 is never flown, so the fit of the last round is unused (default off)
            fit_fn = lambda xx: pool.rollouts('sysid', xx, traces)
            cf = (lambda xx, ii: fd_check(pool, 'sysid', xx, ii, traces)) if l in checks_rounds else None
            x, rf = adam_inner(fit_fn, x, psi_idx, lr, checks=set(check_epochs) if cf else None, check_fn=cf, label='fit beta, round %d' % l, stop=stop, best=best, paper_stop=paper_stop)
            r['fit'] = rf; r['seconds_fit'] = time.time() - t1
            log('    fit: %d epochs, %s, Jn at the last epoch %.3f, at the best iterate %s, psi err %s (%.0f s)' % (rf['epochs'], rf['reason'], rf['log'][-1]['Jn'], _jn_best(rf), np.round(x[10:] - PSI_TRUE, 3).tolist(), r['seconds_fit']))
        r['psi_after_fit'] = x[10:].tolist()
        t2 = time.time()
        if l < n_rounds - 1 or (extra_final_flight and l < n_rounds):
            pool.noise_seeds = tuple(noise_seeds_of(l)) if noise_seeds_of else None      # Stage 3d variant A: fixed noise seeds of this round's tuning step
            r['noise_seeds'] = pool.noise_seeds
            tune_fn = lambda xx: pool.rollouts('task', xx, None)
            x, rt = adam_inner(tune_fn, x, phi_idx, lr, label='tune theta, round %d' % l, stop=stop, best=best, limit=None if not gain_limit else float(np.log(1 + gain_limit)), paper_stop=paper_stop)
            r['tune'] = rt; r['seconds_tune'] = time.time() - t2
            log('    tune: %d epochs, %s, Jn at the last epoch %.3f, at the best iterate %s (%.0f s)' % (rt['epochs'], rt['reason'], rt['log'][-1]['Jn'], _jn_best(rt), r['seconds_tune']))
        r['seconds_round'] = time.time() - t0
        rec['rounds'].append(r)
        if on_round:
            on_round(rec)          # Stage 3d: partial record saved after every round
        if stop_after is not None and stop_after(rec):      # Stage 4e: early stop of the loop after this round (the record keeps the rounds done)
            break
    rec['best'] = keep
    return rec


def evaluate(pool, phi, plant, seeds=(1000, 1001, 1002, 1003, 1004), scen_list=None):
    """Final evaluation of a gain set on the real drone: 5 noise seeds on every tuning and test cell (no repeat of crashed flights).
    Returns list of dict(scenario, role, seed, rms3d_cm, z_rms_cm, crashed)."""
    from frsim.scenarios import tuning_cells  # noqa: F401
    cells = [(s, c, 'tuning') for s, c in TUNING] + [(s, c, 'test') for s, c in TEST] if scen_list is None else scen_list
    gains = gains_of(phi)
    jobs = [(s, c, gains, plant, seed) for s, c, _ in cells for seed in seeds]
    res = pool.fly(jobs); out = []
    for (s, c, seed_), (tr, stable, tilt) in zip([(s, c, sd) for s, c, _ in cells for sd in seeds], res):
        role = [r for ss, cc, r in cells if ss == s and cc == c][0]
        cst, cz = flight_cost(tr, R.build(s, c).scenario_s)
        out.append(dict(scenario=s, cell=c, role=role, seed=seed_, rms3d_cm=100 * cst ** 0.5, z_rms_cm=100 * cz ** 0.5, crashed=not stable, floor=floor_failed(tr, R.build(s, c).scenario_s), tilt=tilt))
    return out


def jdump(obj, path):
    def conv(o):
        if isinstance(o, np.ndarray):
            return o.tolist()
        if isinstance(o, (np.floating, np.integer)):
            return o.item()
        raise TypeError(type(o))
    json.dump(obj, open(path, 'w'), indent=1, default=conv)


def strip_traces(rec):
    """Copy of a record without large arrays (none are stored in rec; flights' traces stay in memory only)."""
    return rec


def worker_parts(args):
    """(scenario, cell, x (14,), trace) -> J_sysId and its four parts (p, v, q, w) at x (forward only), for the diagnostics."""
    scen, cell, x, tr = args
    import torch
    import costs as C
    import ingest
    S = model()
    ref = R.build(scen, cell)
    z = torch.tensor(np.asarray(x)[None], dtype=torch.float64)
    with torch.no_grad():
        out = S.rollout(ref, z[:, :10], z[:, 10:], compile=True)
        J, parts = C.j_sysid(out, ingest.on_grid(ingest.from_trace(tr), out['t']), ref.scenario_s, parts=True)
    return float(J), {k: float(v) for k, v in parts.items()}


def worker_flyclean(args):
    """Like worker_fly with the sensor noise switched off (diagnostics)."""
    scen, cell, gains, plant, seed = args
    from frsim.episode import EpisodeConfig, run_episode
    p = config.plant_with(plant['overrides'])
    r = run_episode(EpisodeConfig(CTRL, scen, cell, gains, plant['variant'], seed, plant=p, noise=False, record_state=True))
    return r.state_trace, bool(r.stable), float(r.tilt_max_deg)
