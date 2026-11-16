#!/usr/bin/env python3
"""Stage 3d gradient check with noise: autograd vs central finite differences of the COMBINED J_task (mean of the 4 tuning scenarios, each the mean over
the 2 noise seeds 1000 / 1001 of frsim's sequence), all 10 gain components, at 2 points (default gains, round-1 gains of Stage 3b loop (d)), beta = P1 truth.
Gate (spec 5c.1, our choice carried over): same sign and (relative <= 1 % or absolute <= 0.1 % of the largest component). h = 1e-5 (log-factor) by default, argv[1] otherwise (the diagnostic showed the finite difference itself depends on h)."""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import loop_lib as L  # noqa
import noise_forward_check as N  # noqa

if __name__ == '__main__':
    pool = L.Pool(4, noise_seeds=(1000, 1001)); h = float(sys.argv[1]) if len(sys.argv) > 1 else 1e-5; out = {}
    try:
        for name in ('default', 'round1'):
            t0 = time.time()
            x = np.r_[N.load_phis()[name], L.PSI_TRUE]
            J, g = pool.rollouts('task', x, None)
            fd = np.zeros(10)
            for i in range(10):
                xp = x.copy(); xm = x.copy(); xp[i] += h; xm[i] -= h
                fd[i] = (pool.rollouts('task_fwd', xp, None)[0] - pool.rollouts('task_fwd', xm, None)[0]) / (2 * h)
            big = np.abs(g[:10]).max(); rows = []
            for i in range(10):
                rel = abs(g[i] - fd[i]) / max(abs(fd[i]), 1e-300); ab = abs(g[i] - fd[i]) / big
                ok = (np.sign(g[i]) == np.sign(fd[i])) and (rel <= 0.01 or ab <= 0.001)
                rows.append(dict(gain=L.GAIN_NAMES[i], autograd=float(g[i]), fd=float(fd[i]), rel=float(rel), abs_of_largest=float(ab), ok=bool(ok)))
                print('%-7s %-6s autograd %12.5e fd %12.5e rel %.4f abs/largest %.5f %s' % (name, L.GAIN_NAMES[i], g[i], fd[i], rel, ab, 'ok' if ok else 'FAIL'), flush=True)
            out[name] = dict(J=J, rows=rows, passed=all(r['ok'] for r in rows), seconds=time.time() - t0)
            print(name, 'J %.6g' % J, 'PASS' if out[name]['passed'] else 'FAIL', '%.0f s' % (time.time() - t0), flush=True)
    finally:
        pool.close()
    out['passed'] = all(out[k]['passed'] for k in ('default', 'round1'))
    json.dump(out, open(HERE / ('noise_gradient_check.json' if h == 1e-5 else 'noise_gradient_check_h%g.json' % h), 'w'), indent=1)
    print('GRADIENT CHECK WITH NOISE:', 'PASS' if out['passed'] else 'FAIL')
