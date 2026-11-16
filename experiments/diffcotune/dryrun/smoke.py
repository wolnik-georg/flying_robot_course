import sys, time
sys.path.insert(0, '/home/georg/Desktop/flying_robot_course/experiments/diffcotune/dryrun')
import numpy as np
import loop_lib as L
if __name__ == '__main__':
    pool = L.Pool(4)
    try:
        t = time.time(); fl = L.fly_set(pool, np.zeros(10), L.P2, L.TUNING, [0, 1, 2, 3]); print('fly', time.time() - t, L.round_cost(fl), flush=True)
        traces = {i: [f['trace']] for i, f in enumerate(fl)}
        x = np.zeros(14)
        t = time.time(); J, g = pool.rollouts('task', x, None); print('task epoch 1 (compile)', time.time() - t, J, g[10:], flush=True)
        t = time.time(); J, g = pool.rollouts('task', x, None); print('task epoch 2', time.time() - t, J, flush=True)
        t = time.time(); J, g = pool.rollouts('sysid', x, traces); print('sysid epoch 1 (compile)', time.time() - t, J, g[10:], flush=True)
        t = time.time(); J, g = pool.rollouts('sysid', x, traces); print('sysid epoch 2', time.time() - t, J, flush=True)
        t = time.time(); xx, rec = L.adam_inner(lambda y: pool.rollouts('sysid', y, traces), x, np.arange(10, 14), 0.03, K2=4); print('adam 4 epochs', time.time() - t, rec['reason'], [r['Jn'] for r in rec['log']], np.round(xx[10:], 3), flush=True)
        t = time.time(); print(L.fd_check(pool, 'sysid', x, np.arange(10, 14), traces), time.time() - t)
        t = time.time(); ev = L.evaluate(pool, np.zeros(10), L.P1, seeds=(1000, 1001)); print('eval', time.time() - t, [(e['scenario'], round(e['rms3d_cm'], 2), e['crashed']) for e in ev])
    finally:
        pool.close()
