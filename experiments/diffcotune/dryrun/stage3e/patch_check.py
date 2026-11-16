#!/usr/bin/env python3
"""Stage 3e Part A1: compiled + Inductor-loop-split-patched batched rollout (B = 2 rows = the 2 noise seeds, as in the A / A + B runs) against the eager
float64 model, same inputs. Cells: 4 tuning scenarios x {default, theta_1, theta_3 of the Stage 3d A + B run} x P1 beta. Reports J_task and dJ_task/dtheta (10 gains).
Gate (our choice, numerical identity up to reordering): relative J difference <= 1e-9, each gradient component <= 1e-6 relative (|diff| / |eager|)."""
import json, os, sys, time
from pathlib import Path
import numpy as np
os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent.parent))
import loop_lib as L  # noqa
import torch
torch.set_num_threads(2)
import costs as C, s0_model as S, reference as R  # noqa

SEEDS = (5000, 5001)          # round-0 noise seeds of the Stage 3d variant A / A + B runs


def one(args):
    """One (cell, mode) per child process: the eager graph of one cell takes about 10 GB, so a fresh process per task (maxtasksperchild=1) and no two at once."""
    scen, cell, phi, compiled = args
    ref = R.build(scen, cell)
    z = torch.tensor(np.r_[phi, L.PSI_TRUE][None], dtype=torch.float64, requires_grad=True)
    nz = np.stack([S.frsim_noise(sd, ref.n_ticks) for sd in SEEDS], 1)
    out = S.rollout(ref, z[:, :10].expand(2, 10), z[:, 10:].expand(2, 4), compile=compiled, noise=nz)
    J = C.j_task(out, ref.scenario_s).mean(); J.backward()
    return float(J), z.grad[0, :10].numpy().copy()


if __name__ == '__main__':
    ab = json.load(open(HERE / 'results' / '..' / '..' / 'stage3d' / 'results' / 'loop_AB.json'))
    sets = {'default': np.zeros(10), 'theta1': np.array(ab['rounds'][1]['phi']), 'theta3': np.array(ab['rounds'][3]['phi'])}
    rows = []; ok_all = True
    import multiprocessing as mp
    pool = mp.get_context('spawn').Pool(1, maxtasksperchild=1)
    for name, phi in sets.items():
        for scen, cell in L.TUNING:
            t0 = time.time()
            (Je, ge), (Jc, gc) = pool.map(one, [(scen, cell, phi, False), (scen, cell, phi, True)], chunksize=1)
            rj = abs(Jc - Je) / abs(Je); rg = np.abs(gc - ge) / np.maximum(np.abs(ge), 1e-300); ag = np.abs(gc - ge).max() / np.abs(ge).max()
            ok = rj <= 1e-9 and rg.max() <= 1e-6; ok_all &= ok
            rows.append(dict(gains=name, scenario=scen, J_eager=Je, J_compiled=Jc, rel_J=rj, max_rel_grad=float(rg.max()), max_abs_over_largest=float(ag), ok=bool(ok)))
            print('%-8s %-7s J eager %.12e compiled %.12e relJ %.2e  max rel grad %.2e (abs/largest %.2e) %s %.0f s' % (name, scen, Je, Jc, rj, rg.max(), ag, 'ok' if ok else 'FAIL', time.time() - t0), flush=True)
    json.dump(dict(rows=rows, passed=bool(ok_all)), open(HERE / 'patch_check.json', 'w'), indent=1)
    print('PATCH GATE:', 'PASS' if ok_all else 'FAIL')
