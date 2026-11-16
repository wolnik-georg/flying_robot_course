#!/usr/bin/env python3
"""Stage 7 step 1 (follow-up of diag7.py): the noisy A1 task gradient of the compiled reverse mode (kind task_R, what the tuning step used) against the exact compiled forward sensitivity (task_C, same noise seeds), per scenario,
at the points of the Stage 5c "RPM delay fixed" dry run: round l = 0 ... 3 (theta_l = rounds[l]['phi'], beta = rounds[l]['psi_after_fit'], noise seeds 6000 + 2l, 6001 + 2l), plus a directional finite-difference check on A1 at round 0
(J of +-h along the forward gradient direction, noisy, h 1e-6 / 1e-7 / 1e-8) and the descent test of diag7.py with the forward gradient (-g/|g| steps 1e-5 ... 1e-2, one Adam step).
Usage: DIFFCOTUNE_CONTROLLER=<C> DIFFCOTUNE_GRAD_MODE=mixed DIFFCOTUNE_FIX_RD=1 python3 diag7_fix.py -> results/diag7_fix_<C>.json"""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent))
import loop_lib as L  # noqa
def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)
if __name__ == '__main__':
    C = L.CTRL; rec = json.load(open(HERE.parent / 'stage5c_rdfix' / 'results' / ('loop_%s.json' % C))); n = len(L.TUNING); out = dict(controller=C, rounds=[])
    nr = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    pool = L.Pool(4)
    try:
        for l in range(nr):
            r = rec['rounds'][l]; phi = np.array(r['phi']); psi = np.array(r['psi_after_fit']); NS = (6000 + 2 * l, 6001 + 2 * l); x0 = np.r_[phi, psi]; t0 = time.time()
            def gr(kind):
                res = pool.pool.map(L.worker_rollout, [(kind, s, c, x0, None, pool.states, NS) for s, c in L.TUNING], chunksize=1); return np.array([q[0] for q in res]), np.array([q[1][:10] for q in res])
            Jr, Gr = gr('task_R'); Jc, Gc = gr('task_C')
            nrm = lambda a: np.linalg.norm(a, axis=1)
            cos = [float(Gr[i] @ Gc[i] / (np.linalg.norm(Gr[i]) * np.linalg.norm(Gc[i]) + 1e-300)) for i in range(n)]
            ent = dict(round=l, noise_seeds=NS, J_rev=Jr.tolist(), J_fwd=Jc.tolist(), norm_rev=nrm(Gr).tolist(), norm_fwd=nrm(Gc).tolist(), cos_rev_fwd=cos, rel_diff=(nrm(Gr - Gc) / (nrm(Gc) + 1e-300)).tolist(), g_rev=Gr.tolist(), g_fwd=Gc.tolist())
            say('round %d: |g_rev| per scenario %s, |g_fwd| %s, cos %s (%.0f s)' % (l, np.array2string(nrm(Gr), precision=3), np.array2string(nrm(Gc), precision=3), np.round(cos, 3), time.time() - t0))
            if l == 0:
                g = Gc.mean(0); d = g / np.linalg.norm(g); fdc = {}
                for h in (1e-6, 1e-7, 1e-8):
                    xs = [x0 + np.r_[s * h * d, np.zeros(len(psi))] for s in (1, -1)]
                    JJ = [np.array([q[0] for q in pool.pool.map(L.worker_rollout, [('task_fwd', s_, c_, x, None, pool.states, NS) for s_, c_ in L.TUNING], chunksize=1)]) for x in xs]
                    fd = (JJ[0] - JJ[1]) / (2 * h); fdc[str(h)] = dict(fd_dir_deriv_per_scen=fd.tolist(), fwd_pred=[float(Gc[i] @ d) for i in range(n)], rev_pred=[float(Gr[i] @ d) for i in range(n)])
                    say('  FD directional derivative h %g: per scenario %s ; forward %s ; reverse %s' % (h, np.array2string(fd, precision=4), np.array2string(np.array(fdc[str(h)]['fwd_pred']), precision=4), np.array2string(np.array(fdc[str(h)]['rev_pred']), precision=4)))
                ent['fd_dir'] = fdc
                steps = [1e-5, 1e-4, 1e-3, 1e-2]; deltas = [-s * d for s in steps] + [-0.01 * np.sign(g)]; names = ['%g' % s for s in steps] + ['adam_lr0.01']
                xs = [x0] + [x0 + np.r_[dl, np.zeros(len(psi))] for dl in deltas]
                JJ = np.array([[q[0] for q in pool.pool.map(L.worker_rollout, [('task_fwd', s_, c_, x, None, pool.states, NS) for s_, c_ in L.TUNING], chunksize=1)] for x in xs]); desc = []
                for k, (nm, dl) in enumerate(zip(names, deltas)):
                    desc.append(dict(step=nm, dJ=float(JJ[k + 1].mean() - JJ[0].mean()), pred=float(g @ dl), dJ_scen=(JJ[k + 1] - JJ[0]).tolist())); say('  descent with forward gradient: step %-12s dJ %+.3e (pred %+.3e)' % (nm, desc[-1]['dJ'], desc[-1]['pred']))
                ent['descent_fwd'] = desc
            out['rounds'].append(ent); json.dump(out, open(HERE / 'results' / ('diag7_fix_%s.json' % C), 'w'), indent=1)
    finally:
        pool.close()
    say('done')
