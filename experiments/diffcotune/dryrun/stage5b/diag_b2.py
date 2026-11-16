"""Stage 5b step 1b: observability of the unstable mode of the one-block state Jacobian of S1 (A1, theta = beta = 0), at blocks 500, 1250, 2000 (t about 1, 2.5, 4 s).
 right r_i, left l_i eigenvectors (unit norm) of all modes of the 156 x 156 state Jacobian A. p = position error direction (cost gradient dJ/dx_pos at that block, 2 (x - x_ref), embedded in the state vector).
 Reverse mode carries the adjoint a^T = p^T A^m; its component along mode i is p . r_i.   Forward mode carries the tangent s' = A^m B u; its component along mode i is l_i . B u.
 Reported: |p . r_i| and |l_i . B_theta|, |l_i . B_beta| (norm over columns) for the mode of the largest |lambda| relative to the largest value over all modes. Literal pairing of the task text
 (p . l_i and r_i . B) is reported too."""
import sys, json
from pathlib import Path
import numpy as np, torch
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parents[0]))
src = open(HERE.parent / 'stage5' / 'jac1.py').read().split("names = []")[0]
src = src.replace("b0, b1, stride = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]); h = float(sys.argv[4]) if len(sys.argv) > 4 else 1e-6", "b0, b1, stride = 500, 2000, 750; h = 1e-6")
src = src.replace("HERE = Path(__file__).resolve().parent\n", "").replace("sys.path.insert(0, str(HERE.parents[0])); sys.path.insert(0, str(HERE.parents[1]))\n", "")
exec(src)
out = {}
for k in sorted(rec):
    f, sh, sz = make_f(k); v0 = torch.cat([pack(rec[k][0]), z0[0]]); ns = sum(sz)
    Jf = torch.autograd.functional.jacobian(f, v0).numpy(); A = Jf[:, :ns]; B = Jf[:, ns:]
    ev, V = np.linalg.eig(A); evl, U = np.linalg.eig(A.T); used = set(); W = np.zeros_like(V)     # A is singular / defective (history states): left eigenvectors from eig(A^T), matched by eigenvalue
    for a_ in np.argsort(-abs(ev)):
        d_ = np.abs(evl - ev[a_]); d_[list(used)] = np.inf; j_ = int(np.argmin(d_)); used.add(j_); W[:, a_] = U[:, j_]
    V = V / np.linalg.norm(V, axis=0); Wn = W / np.linalg.norm(W, axis=0)
    st = rec[k][0]; x_ref = torch.as_tensor(ref.pos[2 * k], dtype=torch.float64); e = 2 * (st[0][0] - x_ref).numpy()
    p = np.zeros(ns); p[:3] = e                                  # state ordering: x first
    i = int(np.argmax(abs(ev)))
    q = {}
    q['adjoint_on_r'] = np.abs(p @ V); q['adjoint_on_l'] = np.abs(p @ Wn)
    q['excite_theta_l'] = np.linalg.norm(Wn.T @ B[:, :10], axis=1); q['excite_beta_l'] = np.linalg.norm(Wn.T @ B[:, 10:], axis=1)
    q['excite_theta_r'] = np.linalg.norm(V.T @ B[:, :10], axis=1); q['excite_beta_r'] = np.linalg.norm(V.T @ B[:, 10:], axis=1)
    rel = {kk: float(v[i] / v.max()) for kk, v in q.items()}
    top = np.argsort(-abs(V[:, i]))[:5]; labs = [(nm, a) for nm, z_ in zip(ST, sz) for a in range(z_)]
    out[k] = dict(t_s=2e-3 * k, max_abs_eig=float(abs(ev[i])), n_unstable=int((abs(ev) > 1 + 1e-9).sum()), rel=rel, abs_p=float(np.linalg.norm(p)), eigvec_top=[(labs[j][0], labs[j][1], float(abs(V[j, i]))) for j in top])
    print('block %d t=%.2f s |lambda|max %.6f n(|l|>1) %d  rel projections (largest-|lambda| mode / max over modes): %s' % (k, 2e-3 * k, abs(ev[i]), out[k]['n_unstable'], {kk: round(v, 4) for kk, v in rel.items()}), flush=True)
json.dump(out, open(HERE / 'results' / 'diag_b2.json', 'w'), indent=1)
