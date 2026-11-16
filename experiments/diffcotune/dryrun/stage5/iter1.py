"""Stage 5 diagnostic: iterate one recorded S1 block (frozen exogenous inputs) from the state and from the state + eps * unstable eigenvector; distance after N blocks."""
import sys
exec(open('jac1.py').read().split("names = []")[0].replace("b0, b1, stride = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]); h = float(sys.argv[4]) if len(sys.argv) > 4 else 1e-6", "b0, b1, stride = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]); h = 1e-6"))
k = b0; f, sh, sz = make_f(k); ns = sum(sz); v0 = torch.cat([pack(rec[k][0]), z0[0]])
J = torch.autograd.functional.jacobian(f, v0).numpy()[:, :ns]; ev, V = np.linalg.eig(J); i = np.argmax(abs(ev)); d = np.real(V[:, i]); d /= np.linalg.norm(d)
print('eig %s' % ev[i], flush=True)
for eps in (1e-10, 1e-8):
    a = v0.clone(); b = v0.clone(); b[:ns] += eps * torch.as_tensor(d)
    for it in range(1, 1201):
        a = torch.cat([f(a)[:ns], z0[0]]); b = torch.cat([f(b)[:ns], z0[0]])
        if it in (1, 10, 50, 100, 200, 400, 600, 800, 1000, 1200): print('eps %.0e  after %4d blocks: |dx| %.3e' % (eps, it, float((a - b).norm())), flush=True)
