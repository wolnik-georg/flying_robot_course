"""Stage 5 diagnostic: eigenvalues and singular values of the one-block state Jacobian (A1, theta=beta=0) at chosen blocks; names the state components of the largest eigenvector."""
import sys
exec(open('jac1.py').read().split("names = []")[0].replace("b0, b1, stride = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]); h = float(sys.argv[4]) if len(sys.argv) > 4 else 1e-6", "b0, b1, stride = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]); h = 1e-6"))
for k in sorted(rec):
    f, sh, sz = make_f(k); v0 = torch.cat([pack(rec[k][0]), z0[0]]); ns = sum(sz)
    J = torch.autograd.functional.jacobian(f, v0).numpy()[:, :ns]
    ev, V = np.linalg.eig(J); i = np.argmax(abs(ev)); sv = np.linalg.svd(J, compute_uv=False)
    labs = [(nm, a) for nm, z_ in zip(ST, sz) for a in range(z_)]; top = np.argsort(-abs(V[:, i]))[:5]
    print('block %d t=%.3f  max|eig| %.6f  (n>1: %d)  top sing %.3e  eigvec top: %s' % (k, -4.5 + 2e-3 * k, abs(ev[i]), int((abs(ev) > 1 + 1e-9).sum()), sv[0], [(labs[j][0], labs[j][1], round(float(abs(V[j, i])), 3)) for j in top]), flush=True)
