#!/usr/bin/env python3
"""Stage 7 step 1 a + b: is the S3 round-0 tuning step a descent step? Point: theta_0 = defaults (phi = 0) and beta after the round-0 fit of the Stage 5c "RPM delay fixed" run
(../stage5c_rdfix/results/loop_<C>.json, rounds[0]['psi_after_fit']); noise seeds of the round-0 tuning step (6000, 6001); the 4 tuning scenarios, J = mean over scenarios as in the loop (Pool.rollouts).
a. descent test: noisy J (and per scenario, and noise-free) along -g/|g| for step lengths 1e-5 ... 1e-2 and for one Adam step (-lr * sign(g), lr 0.01), with the predicted first-order change g . delta.
b. landscape: 1-D noisy cost slices along each of the 10 gains (+-25 %: log +-0.2231, 21 points) and along -g (+-0.05, 21 points), per scenario.
Usage (job_runner, nice 19, controller from DIFFCOTUNE_CONTROLLER, DIFFCOTUNE_GRAD_MODE=mixed, DIFFCOTUNE_FIX_RD=1 for S1 / S3): python3 diag7.py [--skip-b]   -> results/diag7_<C>.json"""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent))
import loop_lib as L  # noqa
def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)
if __name__ == '__main__':
    C = L.CTRL; rec = json.load(open(HERE.parent / 'stage5c_rdfix' / 'results' / ('loop_%s.json' % C))); psi = np.array(rec['rounds'][0]['psi_after_fit']); x0 = np.r_[np.zeros(10), psi]
    NS = (6000, 6001); n = len(L.TUNING); out = dict(controller=C, psi=psi.tolist(), noise_seeds=NS)
    pool = L.Pool(4, noise_seeds=NS)
    def J_of(xs, noise=True, kind='task_fwd'):
        jobs = [(kind, s, c, x, None, pool.states, NS if noise else None) for x in xs for s, c in L.TUNING]
        res = pool.pool.map(L.worker_rollout, jobs, chunksize=1)
        return np.array([r[0] for r in res]).reshape(len(xs), n)               # (points, scenarios)
    try:
        t0 = time.time()
        jobs = [('task', s, c, x0, None, pool.states, NS) for s, c in L.TUNING]; res = pool.pool.map(L.worker_rollout, jobs, chunksize=1)
        gs = np.array([r[1][:10] for r in res]); J0s = np.array([r[0] for r in res]); g = gs.mean(0)                  # noisy: compiled reverse = the loop's gradient
        jobs = [('task_C', s, c, x0, None, pool.states, None) for s, c in L.TUNING]; res = pool.pool.map(L.worker_rollout, jobs, chunksize=1)
        gs_nf = np.array([r[1][:10] for r in res]); J0s_nf = np.array([r[0] for r in res])                              # noise-free exact forward sensitivity
        out.update(J0_scen=J0s.tolist(), g_scen=gs.tolist(), g=g.tolist(), J0_scen_noisefree=J0s_nf.tolist(), g_scen_noisefree=gs_nf.tolist(), g_noisefree=gs_nf.mean(0).tolist())
        say('gradient at x0 done (%.0f s): |g| %.4g, J0 per scenario %s, noise-free J0 %s' % (time.time() - t0, np.linalg.norm(g), np.round(J0s, 5), np.round(J0s_nf, 5)))
        d = -g / np.linalg.norm(g); steps = [1e-5, 1e-4, 1e-3, 1e-2]
        deltas = [s * d for s in steps] + [-0.01 * np.sign(g)]; names = ['%g' % s for s in steps] + ['adam_lr0.01']
        xs = [x0] + [x0 + np.r_[dl, np.zeros(len(psi))] for dl in deltas]
        Jn = J_of(xs, True); Jf = J_of(xs, False)
        a = []
        for k, (nm, dl) in enumerate(zip(names, deltas)):
            a.append(dict(step=nm, delta_norm=float(np.linalg.norm(dl)), J_noisy=float(Jn[k + 1].mean()), dJ_noisy=float(Jn[k + 1].mean() - Jn[0].mean()), pred_noisy=float(g @ dl),
                          dJ_scen_noisy=(Jn[k + 1] - Jn[0]).tolist(), pred_scen_noisy=[float(gs[i] @ dl) for i in range(n)],
                          J_noisefree=float(Jf[k + 1].mean()), dJ_noisefree=float(Jf[k + 1].mean() - Jf[0].mean()), pred_noisefree=float(gs_nf.mean(0) @ dl),
                          dJ_scen_noisefree=(Jf[k + 1] - Jf[0]).tolist(), pred_scen_noisefree=[float(gs_nf[i] @ dl) for i in range(n)]))
            say('a: step %-12s dJ noisy %+.3e (pred %+.3e)   noise-free %+.3e (pred %+.3e)' % (nm, a[-1]['dJ_noisy'], a[-1]['pred_noisy'], a[-1]['dJ_noisefree'], a[-1]['pred_noisefree']))
        out['a'] = a; out['J0_noisy'] = float(Jn[0].mean()); out['J0_noisefree'] = float(Jf[0].mean())
        json.dump(out, open(HERE / 'results' / ('diag7_%s.json' % C), 'w'), indent=1)
        if '--skip-b' not in sys.argv:
            ulim = np.log(1.25); b = {}
            u = np.linspace(-ulim, ulim, 21)
            for j, nm in enumerate(L.GAIN_NAMES):
                xs = [x0 + np.r_[np.eye(10)[j] * uu, np.zeros(len(psi))] for uu in u]; Jj = J_of(xs, True)
                b[nm] = dict(u=u.tolist(), J_scen=Jj.tolist(), slope_g=float(g[j])); say('b: gain %-6s J mean %s ... done (%.0f s)' % (nm, np.round(Jj.mean(1)[[0, 10, 20]], 5), time.time() - t0))
            t = np.linspace(-0.05, 0.05, 21); xs = [x0 + np.r_[t_ * d, np.zeros(len(psi))] for t_ in t]; b['neg_g'] = dict(t=t.tolist(), J_scen=J_of(xs, True).tolist())
            out['b'] = b; json.dump(out, open(HERE / 'results' / ('diag7_%s.json' % C), 'w'), indent=1)
    finally:
        pool.close()
    say('done')
