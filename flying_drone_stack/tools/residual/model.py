"""The Neural-Swarm2 residual model in PyTorch, shaped to match the firmware exactly.

This file is one half of a contract. The other half is
`firmware_app/src/residual_nn.rs`, which indexes a flat array of 19297 floats by hand. Nothing in
either language checks that the two agree about which float is which, so the layout is written
down here once (`flatten`) and verified numerically against the compiled firmware by
`test_pipeline.py`. If you change a layer size, change it in both places and re-run that test --
a mismatch does not crash, it produces a plausible wrong answer.

2026-09-14: rewritten for the Neural-Swarm2 architecture port. The previous model here was a
smaller custom deep-sets network (phi 6->16->16->8, rho 8->16->16->3, 987 weights, 3-vector
output). The firmware no longer contains that network, so this file no longer describes it.

Architecture (exact, from `neuralswarm.py`'s `phi_Net` / `rho_Net`)
------------------------------------------------------------------
    phi_Net(input_dim):  input_dim -> 25 -> 40 -> 40 -> H=20   (ReLU, ReLU, ReLU, LINEAR)
    rho_Net:             H=20      -> 40 -> 40 -> 40 -> 1      (ReLU, ReLU, ReLU, LINEAR)

Five weight sets, in this flat order: phi_S | phi_L | phi_G | rho_S | rho_L.

* `phi_S` / `rho_S` -- neighbour interaction for a "small" vehicle. Every Crazyflie is small, so
  these are the only sets this project ever evaluates.
* `phi_L` / `rho_L` -- the "large" vehicle path. Never exercised here (see `cftype` in the
  reference), but present in the layout because the firmware's flat array reserves space for
  them. `DeepSets.flatten()` emits them as **zeros** unless they are explicitly set: there is no
  data to train them on, and a zero block is honest about that. Do not read a prediction made
  with the large path as meaningful.
* `phi_G` -- ground effect, on `[0 - own_z, -own_vx, -own_vy, -own_vz]`. Evaluated
  unconditionally, every tick, with no neighbour to gate it on.

Forward pass (`compute_Fa`)
---------------------------
    rho_input = phi_G(ground_term)
    for each neighbour j:
        if |dx| < 0.2 and |dy| < 0.2 and |dvx| < 1.5:      # NOT a distance cutoff
            rho_input += phi_S([dx, dy, dz, dvx, dvy, dvz])
    Fa = (0, 0, rho_S(rho_input))                          # scalar, Z only

The output is a force in the reference's equivalent-grams unit. This project's residual
convention is an acceleration, so the firmware converts grams -> N -> /mass once at the very end;
`firmware_forward` reproduces that conversion so exported weights can be checked without a drone.

Normalisation
-------------
The firmware has none, on purpose: a second set of constants onboard could drift out of step with
the trained model. Instead `fold_normalisation` pushes the input scaling into the first layer's
weights and biases, which is exactly equivalent:

    W1 @ ((x - mu) / sigma) + b1  ==  (W1 / sigma) @ x + (b1 - (W1 / sigma) @ mu)

So the network that ships is arithmetically identical to the network that was trained, and the
firmware only ever sees raw metres and metres per second. This folds *values*, never shapes, so
it does not touch the ported architecture. Note there are now two first layers to fold into --
`phi_S`'s (6 inputs) and `phi_G`'s (4 inputs) -- and they need separate statistics, because a
neighbour's relative state and a vehicle's own height are not the same distribution.
"""

import numpy as np
import torch
from torch import nn

# ── Architecture. Mirrors residual_nn.rs; changing either alone is a silent bug. ──
PHI_IN_SL = 6          # neighbour [dx, dy, dz, dvx, dvy, dvz]
PHI_IN_G = 4           # ground    [0 - z, -vx, -vy, -vz]
HIDDEN = 20            # "H" in the reference
PHI_H = (25, 40, 40)
RHO_H = (40, 40, 40)
RHO_OUT = 1            # scalar; only ever the Z component of Fa


def phi_shapes(input_dim):
    a, b, c = PHI_H
    return [(input_dim, a), (a, b), (b, c), (c, HIDDEN)]


RHO_SHAPES = [(HIDDEN, RHO_H[0]), (RHO_H[0], RHO_H[1]), (RHO_H[1], RHO_H[2]),
              (RHO_H[2], RHO_OUT)]

N_PHI_SL = sum(i * o + o for i, o in phi_shapes(PHI_IN_SL))   # 3675
N_PHI_G = sum(i * o + o for i, o in phi_shapes(PHI_IN_G))     # 3625
N_RHO = sum(i * o + o for i, o in RHO_SHAPES)                 # 4161

OFF_PHI_S = 0
OFF_PHI_L = OFF_PHI_S + N_PHI_SL
OFF_PHI_G = OFF_PHI_L + N_PHI_SL
OFF_RHO_S = OFF_PHI_G + N_PHI_G
OFF_RHO_L = OFF_RHO_S + N_RHO
N_WEIGHTS = OFF_RHO_L + N_RHO                                 # 19297

MAX_NEIGHBOURS = 3
OUT_CLAMP = 8.0                 # m/s^2, this project's safety net, not the reference's
GATE_DXY = 0.2                  # neuralswarm.py:79 -- dx and dy specifically
GATE_DVX = 1.5                  # ...and dvx. Not dz, not dvy, not dvz.
GRAMS_TO_NEWTONS = 9.81 / 1000.0
DEFAULT_MASS = 0.0364           # kg; g_indi_mass's default, overridable at call time


def _mlp(shapes):
    """phi_Net / rho_Net: ReLU on every layer except the last, which is linear."""
    layers = []
    for k, (n_in, n_out) in enumerate(shapes):
        layers.append(nn.Linear(n_in, n_out))
        if k != len(shapes) - 1:
            layers.append(nn.ReLU())
    return nn.Sequential(*layers)


def neighbour_gate(rel):
    """The reference's three-term proximity gate, as a boolean mask over (..., 6) inputs.

    Deliberately asymmetric -- it tests dx, dy and dvx and ignores dz, dvy, dvz. Replicated
    literally rather than regularised into something tidier; it IS the reference's cutoff, and
    "fixing" it would change which neighbours the network is trained to see.

    WARNING: an all-zero row PASSES this gate (0 < 0.2 and 0 < 1.5), so applying it to a
    zero-padded neighbour array marks the padding as a neighbour sitting exactly on top of the
    vehicle -- the single strongest input the network can receive. Always combine it with a
    presence mask; `build_mask` does that for you.
    """
    rel = np.asarray(rel)
    return ((np.abs(rel[..., 0]) < GATE_DXY)
            & (np.abs(rel[..., 1]) < GATE_DXY)
            & (np.abs(rel[..., 3]) < GATE_DVX))


def build_mask(rel, present):
    """The mask `NeuralSwarm2.forward` wants: present AND inside the gate.

    `rel` is (..., K, 6) and `present` is (..., K), 1 where a neighbour actually exists. Keeping
    the two separate and combining here is the whole point -- see `neighbour_gate`'s warning.
    """
    return (np.asarray(present, np.float32)
            * neighbour_gate(rel).astype(np.float32))


class NeuralSwarm2(nn.Module):
    """Fa_z = rho_S(phi_G(ground) + sum_j gated phi_S(rel_j)), in the reference's grams unit.

    phi is applied per neighbour and the results summed, which makes the output
    permutation-invariant and independent of neighbour count -- the two properties the physics
    has and a fixed-input MLP does not. The ground term joins the same sum, which is how the
    reference folds ground effect into a network that otherwise only sees neighbours.

    Only the "small" path is trained here: `phi_L`/`rho_L` exist in the exported layout but have
    no data behind them (this fleet is Crazyflies only).
    """

    def __init__(self):
        super().__init__()
        self.phi_s = _mlp(phi_shapes(PHI_IN_SL))
        self.phi_g = _mlp(phi_shapes(PHI_IN_G))
        self.rho_s = _mlp(RHO_SHAPES)

    def forward(self, rel, mask, ground):
        """rel: (B, K, 6) relative states. mask: (B, K) 1 where a neighbour is present AND
        passes the gate. ground: (B, 4). Returns (B,) in grams.

        The mask is applied after phi rather than by dropping rows, so a batch can mix samples
        with zero, one, two and three neighbours without reshaping. Note that phi(0) is generally
        NOT zero -- the biases see to that -- so masking after the sum would quietly add a
        phantom neighbour to every sample.
        """
        summed = (self.phi_s(rel) * mask.unsqueeze(-1)).sum(dim=1)
        return self.rho_s(summed + self.phi_g(ground)).squeeze(-1)

    # ── Export ──────────────────────────────────────────────────────────────
    @staticmethod
    def _flat(seq):
        out = []
        for m in seq:
            if isinstance(m, nn.Linear):
                out.append(m.weight.detach().cpu().numpy().astype(np.float32).ravel())
                out.append(m.bias.detach().cpu().numpy().astype(np.float32).ravel())
        return np.concatenate(out)

    def flatten(self, phi_l=None, rho_l=None):
        """Weights in firmware order: phi_S | phi_L | phi_G | rho_S | rho_L.

        Within a net: layer by layer; within a layer the weight block row-major as
        [n_out][n_in] (PyTorch nn.Linear convention), then the bias block.

        `phi_l`/`rho_l` default to zeros -- this project never evaluates the large-vehicle path,
        and shipping zeros states that plainly rather than shipping an untrained copy of the
        small path that would look trained.
        """
        phi_s = self._flat(self.phi_s)
        phi_g = self._flat(self.phi_g)
        rho_s = self._flat(self.rho_s)
        phi_l = np.zeros(N_PHI_SL, np.float32) if phi_l is None else np.asarray(phi_l, np.float32)
        rho_l = np.zeros(N_RHO, np.float32) if rho_l is None else np.asarray(rho_l, np.float32)
        assert phi_s.size == N_PHI_SL and phi_g.size == N_PHI_G and rho_s.size == N_RHO
        assert phi_l.size == N_PHI_SL and rho_l.size == N_RHO
        w = np.concatenate([phi_s, phi_l, phi_g, rho_s, rho_l])
        assert w.size == N_WEIGHTS, f"{w.size} != {N_WEIGHTS}"
        return w

    def load_flat(self, w):
        """Inverse of flatten(), so a round trip through the wire format can be checked.

        Reads only the small and ground paths; the phi_L/rho_L blocks are skipped, since this
        module has no parameters for them.
        """
        w = np.asarray(w, np.float32)
        assert w.size == N_WEIGHTS, f"{w.size} != {N_WEIGHTS}"

        def load(seq, off):
            with torch.no_grad():
                for m in seq:
                    if not isinstance(m, nn.Linear):
                        continue
                    n_out, n_in = m.weight.shape
                    m.weight.copy_(torch.from_numpy(
                        w[off:off + n_out * n_in].reshape(n_out, n_in).copy()))
                    off += n_out * n_in
                    m.bias.copy_(torch.from_numpy(w[off:off + n_out].copy()))
                    off += n_out
            return off

        assert load(self.phi_s, OFF_PHI_S) == OFF_PHI_L
        assert load(self.phi_g, OFF_PHI_G) == OFF_RHO_S
        assert load(self.rho_s, OFF_RHO_S) == OFF_RHO_L


def fold_normalisation(model, mu_rel, sigma_rel, mu_g, sigma_g):
    """Return a copy whose first layers absorb (x - mu) / sigma, for phi_S and phi_G separately.

    After this the model takes raw metres and m/s, exactly as the firmware feeds it. Call it once
    at export; calling it twice would fold the normalisation in twice, which is why it returns a
    new model instead of mutating in place.
    """
    def fold(lin, mu, sigma, n):
        mu = np.asarray(mu, np.float64).reshape(n)
        sigma = np.asarray(sigma, np.float64).reshape(n)
        if np.any(sigma < 1e-9):
            raise ValueError(f"sigma has near-zero entries {sigma}; a constant input cannot be "
                             f"normalised. Usually means a channel was never excited in the "
                             f"data.")
        with torch.no_grad():
            W = lin.weight.detach().cpu().numpy().astype(np.float64)
            b = lin.bias.detach().cpu().numpy().astype(np.float64)
            W_new = W / sigma                      # broadcast over input columns
            b_new = b - W_new @ mu
            lin.weight.copy_(torch.from_numpy(W_new.astype(np.float32)))
            lin.bias.copy_(torch.from_numpy(b_new.astype(np.float32)))

    out = NeuralSwarm2()
    out.load_state_dict(model.state_dict())
    fold(out.phi_s[0], mu_rel, sigma_rel, PHI_IN_SL)
    fold(out.phi_g[0], mu_g, sigma_g, PHI_IN_G)
    return out


def _forward_flat(w, off, shapes, x):
    """One phi_Net/rho_Net stack straight out of the flat vector, matching residual_nn.rs."""
    x = np.asarray(x, np.float32)
    for k, (n_in, n_out) in enumerate(shapes):
        W = w[off:off + n_out * n_in].reshape(n_out, n_in)
        off += n_out * n_in
        b = w[off:off + n_out]
        off += n_out
        x = (W @ x + b).astype(np.float32)
        if k != len(shapes) - 1:
            x = np.maximum(x, 0.0).astype(np.float32)
    return x


def firmware_forward(w, rel, own_z, own_vel, mass=DEFAULT_MASS, apply_clamp=True):
    """The firmware's evaluation, in NumPy, including the gate, the unit conversion and the clamp.

    Used to check a set of exported weights without a drone or the SIL bindings, and to make the
    gate visible to the training code -- a model trained on neighbours the firmware would never
    present is a model evaluated on the wrong distribution.

    rel:     iterable of (dp, dv) in the world frame, dp = peer - own, dv = peer_vel - own_vel.
    own_z:   this vehicle's altitude [m].
    own_vel: this vehicle's velocity [m/s], 3 components.

    Returns (a_z [m/s^2], clamped). The x and y components are zero by construction and are not
    returned; `compute_Fa` produces a scalar.
    """
    w = np.asarray(w, np.float32)
    own_vel = np.asarray(own_vel, np.float32)

    ground = np.array([0.0 - own_z, -own_vel[0], -own_vel[1], -own_vel[2]], np.float32)
    acc = _forward_flat(w, OFF_PHI_G, phi_shapes(PHI_IN_G), ground)

    for dp, dv in list(rel)[:MAX_NEIGHBOURS]:
        dp = np.asarray(dp, np.float32)
        dv = np.asarray(dv, np.float32)
        if not (abs(dp[0]) < GATE_DXY and abs(dp[1]) < GATE_DXY and abs(dv[0]) < GATE_DVX):
            continue
        x = np.concatenate([dp, dv]).astype(np.float32)
        acc = acc + _forward_flat(w, OFF_PHI_S, phi_shapes(PHI_IN_SL), x)

    grams = float(_forward_flat(w, OFF_RHO_S, RHO_SHAPES, acc)[0])
    a_z = grams * GRAMS_TO_NEWTONS / mass

    if not np.isfinite(a_z):
        return 0.0, True
    if apply_clamp and abs(a_z) > OUT_CLAMP:
        return float(np.sign(a_z) * OUT_CLAMP), True
    return float(a_z), False


def grams_to_accel(grams, mass=DEFAULT_MASS):
    """The network's output unit -> this project's residual unit. One conversion, one place."""
    return np.asarray(grams, np.float64) * GRAMS_TO_NEWTONS / mass


def accel_to_grams(accel, mass=DEFAULT_MASS):
    """Inverse of `grams_to_accel`, for turning a measured a_res into a training target."""
    return np.asarray(accel, np.float64) * mass / GRAMS_TO_NEWTONS
