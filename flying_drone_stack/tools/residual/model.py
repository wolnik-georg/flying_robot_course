"""The deep-sets residual model in PyTorch, shaped to match the firmware exactly.

This file is one half of a contract. The other half is
`firmware_app/src/residual_nn.rs`, which indexes a flat array of 987 floats by hand. Nothing in
either language checks that the two agree about which float is which, so the layout is written
down here once (`flatten`) and verified numerically against the compiled firmware by
`test_pipeline.py`. If you change a layer size, change it in both places and re-run that test --
a mismatch does not crash, it produces a plausible wrong answer.

Normalisation
-------------
The firmware has none, on purpose: a second set of constants onboard could drift out of step with
the trained model. Instead `fold_normalisation` pushes the input scaling into the first layer's
weights and biases, which is exactly equivalent:

    W1 @ ((x - mu) / sigma) + b1  ==  (W1 / sigma) @ x + (b1 - (W1 / sigma) @ mu)

So the network that ships is arithmetically identical to the network that was trained, and the
firmware only ever sees raw metres and metres per second.
"""

import numpy as np
import torch
from torch import nn

# ── Architecture. Mirrors residual_nn.rs; changing either alone is a silent bug. ──
PHI_IN, PHI_H1, PHI_H2 = 6, 16, 16
LATENT = 8
RHO_H1, RHO_H2, RHO_OUT = 16, 16, 3

PHI_SHAPES = [(PHI_IN, PHI_H1), (PHI_H1, PHI_H2), (PHI_H2, LATENT)]
RHO_SHAPES = [(LATENT, RHO_H1), (RHO_H1, RHO_H2), (RHO_H2, RHO_OUT)]

N_WEIGHTS = sum(i * o + o for i, o in PHI_SHAPES + RHO_SHAPES)  # 987

MAX_NEIGHBOURS = 3
OUT_CLAMP = 8.0
MIN_DIST, MAX_DIST = 0.04, 2.0


class DeepSets(nn.Module):
    """a_res = rho(sum_j phi(rel_j)).

    phi is applied per neighbour and the results summed, which makes the output
    permutation-invariant and independent of neighbour count -- the two properties the physics
    has and a fixed-input MLP does not.
    """

    def __init__(self):
        super().__init__()
        self.phi = nn.Sequential(
            nn.Linear(PHI_IN, PHI_H1), nn.ReLU(),
            nn.Linear(PHI_H1, PHI_H2), nn.ReLU(),
            nn.Linear(PHI_H2, LATENT), nn.ReLU(),
        )
        # No activation on the output: an interaction force can point either way.
        self.rho = nn.Sequential(
            nn.Linear(LATENT, RHO_H1), nn.ReLU(),
            nn.Linear(RHO_H1, RHO_H2), nn.ReLU(),
            nn.Linear(RHO_H2, RHO_OUT),
        )

    def forward(self, rel, mask):
        """rel: (B, K, 6) relative states. mask: (B, K) 1 where a neighbour is present.

        The mask is applied after phi rather than by dropping rows, so a batch can mix samples
        with one, two and three neighbours without reshaping. Note that phi(0) is generally
        NOT zero -- the biases see to that -- so masking after the sum would quietly add a
        phantom neighbour to every sample.
        """
        latent = self.phi(rel) * mask.unsqueeze(-1)
        return self.rho(latent.sum(dim=1))

    # ── Export ──────────────────────────────────────────────────────────────
    def linears(self):
        return [m for m in list(self.phi) + list(self.rho) if isinstance(m, nn.Linear)]

    def flatten(self):
        """Weights in firmware order: layer by layer (phi then rho); within a layer the weight
        block row-major as [n_out][n_in], then the bias block."""
        out = []
        for lin in self.linears():
            out.append(lin.weight.detach().cpu().numpy().astype(np.float32).ravel())
            out.append(lin.bias.detach().cpu().numpy().astype(np.float32).ravel())
        w = np.concatenate(out)
        assert w.size == N_WEIGHTS, f"{w.size} != {N_WEIGHTS}"
        return w

    def load_flat(self, w):
        """Inverse of flatten(), so a round trip through the wire format can be checked."""
        w = np.asarray(w, np.float32)
        assert w.size == N_WEIGHTS
        off = 0
        with torch.no_grad():
            for lin in self.linears():
                n_out, n_in = lin.weight.shape
                lin.weight.copy_(torch.from_numpy(
                    w[off:off + n_out * n_in].reshape(n_out, n_in).copy()))
                off += n_out * n_in
                lin.bias.copy_(torch.from_numpy(w[off:off + n_out].copy()))
                off += n_out
        assert off == N_WEIGHTS


def fold_normalisation(model, mu, sigma):
    """Return a copy whose first layer absorbs (x - mu) / sigma.

    After this the model takes raw relative states, exactly as the firmware feeds it. Call it
    once at export; calling it twice would fold the normalisation in twice, which is why it
    returns a new model instead of mutating in place.
    """
    mu = np.asarray(mu, np.float64).reshape(PHI_IN)
    sigma = np.asarray(sigma, np.float64).reshape(PHI_IN)
    if np.any(sigma < 1e-9):
        raise ValueError(f"sigma has near-zero entries {sigma}; a constant input cannot be "
                         f"normalised. Usually means a channel was never excited in the data.")

    out = DeepSets()
    out.load_state_dict(model.state_dict())
    lin = out.linears()[0]
    with torch.no_grad():
        W = lin.weight.detach().cpu().numpy().astype(np.float64)
        b = lin.bias.detach().cpu().numpy().astype(np.float64)
        W_new = W / sigma                      # broadcast over input columns
        b_new = b - W_new @ mu
        lin.weight.copy_(torch.from_numpy(W_new.astype(np.float32)))
        lin.bias.copy_(torch.from_numpy(b_new.astype(np.float32)))
    return out


def firmware_forward(w, rel, apply_clamp=True):
    """The firmware's evaluation, in NumPy, including its distance guards and output clamp.

    Used to check a set of exported weights without a drone or the SIL bindings, and to make the
    guards visible to the training code -- a model trained on inputs the firmware would never
    present is a model evaluated on the wrong distribution.

    rel: iterable of (dp, dv) in the world frame, dp = peer - own.
    """
    w = np.asarray(w, np.float32)
    layers, off = [], 0
    for n_in, n_out in PHI_SHAPES + RHO_SHAPES:
        W = w[off:off + n_out * n_in].reshape(n_out, n_in); off += n_out * n_in
        b = w[off:off + n_out]; off += n_out
        layers.append((W, b))

    latent, used = np.zeros(LATENT, np.float32), False
    for dp, dv in list(rel)[:MAX_NEIGHBOURS]:
        dp = np.asarray(dp, np.float32)
        d = float(np.linalg.norm(dp))
        if d > MAX_DIST:
            continue
        scale = MIN_DIST / d if (MIN_DIST > d > 1e-6) else 1.0
        x = np.concatenate([dp * scale, np.asarray(dv, np.float32)]).astype(np.float32)
        for W, b in layers[:3]:
            x = np.maximum(W @ x + b, 0.0).astype(np.float32)
        latent += x
        used = True
    if not used:
        return np.zeros(3, np.float32), False

    x = latent
    for W, b in layers[3:5]:
        x = np.maximum(W @ x + b, 0.0).astype(np.float32)
    W, b = layers[5]
    y = (W @ x + b).astype(np.float32)

    mag = float(np.linalg.norm(y))
    if apply_clamp and mag > OUT_CLAMP:
        return (y * (OUT_CLAMP / mag)).astype(np.float32), True
    return y, False
