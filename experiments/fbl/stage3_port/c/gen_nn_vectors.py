#!/usr/bin/env python3
"""Gate 4 vectors for the network term of fbl_residual.c (fbl_nn_pair) from the AUTHORS' code: DownwashResidual._acceleration_nn_analytical_jvp
(bundle, imported unmodified) with (a) the bundle's sample checkpoint (net_f 9 neurons) and (b) a 4-neuron net_f (paper hardware size; random weights
here, our trained weights go through the same path).
Each row also carries the bundle evaluated in float64 on the same float32 inputs (ground truth for the float32 rounding). Random states with the neighbour above (r0 0.005-0.6 m, z0 0.05-1.2 m, velocities up to 1 m/s).
  ~/.pyenv/versions/flying_robots/bin/python gen_nn_vectors.py   -> nn_vectors_sample.txt, nn_vectors_h4.txt"""
import sys
from pathlib import Path
import numpy as np
import torch
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE.parents[1]))
from common import bundle as B  # noqa: E402
b = B.load()


def write(path, ode, seed):
    import copy
    dw = b.dw.DownwashResidual(ode, use_nn=True, use_nominal=False)
    dw64 = b.dw.DownwashResidual(copy.deepcopy(ode).double(), use_nn=True, use_nominal=False)       # float64 truth (same code, double precision)
    W1, b1 = ode.net_f[0].weight.detach().numpy(), ode.net_f[0].bias.detach().numpy(); W2, b2 = ode.net_f[2].weight.detach().numpy(), ode.net_f[2].bias.detach().numpy()
    rng = np.random.default_rng(seed); N = 3000
    with open(path, "w") as f:
        f.write(f"{W1.shape[0]}\n" + " ".join(f"{x:.9g}" for x in [*W1.ravel(), *b1, *W2.ravel(), *b2]) + "\n" + f"{N}\n")
        for k in range(N):
            r0 = rng.uniform(0.005, 0.6) if k % 10 else rng.uniform(0.005, 0.03); z0 = rng.uniform(0.05, 1.2); th = rng.uniform(-np.pi, np.pi)
            p = rng.normal(0, 0.5, 3); p[2] = rng.uniform(0.2, 1.0); pt = p + np.array([r0 * np.cos(th), r0 * np.sin(th), z0])
            v = rng.normal(0, 0.5, 3); vt = rng.normal(0, 0.5, 3)
            y = np.zeros(30); y[0:3] = p; y[3:6] = v; y[9] = 1.0; y[17:20] = pt; y[20:23] = vt; y[26] = 1.0
            y32 = torch.tensor(y, dtype=torch.float32)
            with torch.no_grad():
                a, j = dw._acceleration_nn_analytical_jvp(y32)
                a64, j64 = dw64._acceleration_nn_analytical_jvp(y32.double())          # same float32 inputs, evaluated in double
            f.write(" ".join(f"{x:.9g}" for x in [*y32[0:3].numpy(), *y32[3:6].numpy(), *y32[17:20].numpy(), *y32[20:23].numpy(), *a[0].numpy(), *j[0].numpy()])
                    + " " + " ".join(f"{x:.17g}" for x in [*a64[0].numpy(), *j64[0].numpy()]) + "\n")


ode = b.nnm.ODEFunc(); ck = torch.load(B.BUNDLE / "KNODE/Models/sample_model.pth", map_location="cpu", weights_only=False); ode.load_state_dict(ck["state_dict"] if "state_dict" in ck else ck)
write(HERE / "nn_vectors_sample.txt", ode, 31)
torch.manual_seed(4); ode4 = b.nnm.ODEFunc(); ode4.net_f = torch.nn.Sequential(torch.nn.Linear(2, 4), torch.nn.Tanh(), torch.nn.Linear(4, 3))
for m in ode4.net_f.modules():
    if isinstance(m, torch.nn.Linear): torch.nn.init.normal_(m.weight, 0, 1.0); torch.nn.init.normal_(m.bias, 0, 0.5)
write(HERE / "nn_vectors_h4.txt", ode4, 32)

# (c) the trained flight network (KNODE on our S0 flights, 4 neurons, C_D 0.55; 2026-10-25) -> nn_vectors_trained.txt
ck = torch.load(HERE.parents[1] / "out/stage4/real_v0/knode_s0977_h4.pth", map_location="cpu", weights_only=False)
odet = b.nnm.ODEFunc(); h = ck.get("hidden_f", 9)
odet.net_f = torch.nn.Sequential(torch.nn.Linear(2, h), torch.nn.Tanh(), torch.nn.Linear(h, 3)); odet.load_state_dict(ck["state_dict"])
write(HERE / "nn_vectors_trained.txt", odet, 33)
print("done")
