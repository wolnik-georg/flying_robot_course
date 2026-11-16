# Copy of experiments/tuning/diff/ns2_torch.py (2026-11-11, Stage 2). Only change: this header and the default weights path (simulator/frsim/data/ns2_v2.npz, the same weights as the old path).
#!/usr/bin/env python3
"""Torch port of the NS2 v2 downwash network used as the force truth in plant_core (flying_drone_stack/firmware_app/host/test_residual_nn.py `reference`):
Fa_z = rho(phi_G(ground) + gate * phi_S(neighbour relative state)), one neighbour (the top drone), clamped at 8 m/s^2. Batched, differentiable (the gate is a 0/1 mask)."""
import numpy as np, torch

HIDDEN, PHI_IN_SL, PHI_IN_G = 20, 6, 4
RHO_LAYERS = [(HIDDEN, 40), (40, 40), (40, 40), (40, 1)]
phi_layers = lambda d: [(d, 25), (25, 40), (40, 40), (40, HIDDEN)]
N_PHI_SL = sum(i * o + o for i, o in phi_layers(PHI_IN_SL)); N_PHI_G = sum(i * o + o for i, o in phi_layers(PHI_IN_G)); N_RHO = sum(i * o + o for i, o in RHO_LAYERS)
OFF_PHI_S = 0; OFF_PHI_L = N_PHI_SL; OFF_PHI_G = OFF_PHI_L + N_PHI_SL; OFF_RHO_S = OFF_PHI_G + N_PHI_G
GATE_DXY, GATE_DVX, OUT_CLAMP, GRAMS_TO_NEWTONS = 0.2, 1.5, 8.0, 9.81 / 1000.0


def _stack(w, off, layers, dtype, dev):
    out = []
    for n_in, n_out in layers:
        W = torch.as_tensor(w[off:off + n_out * n_in].reshape(n_out, n_in), dtype=dtype, device=dev); off += n_out * n_in
        b = torch.as_tensor(w[off:off + n_out], dtype=dtype, device=dev); off += n_out; out.append((W, b))
    return out


def load(path, dtype=torch.float32, dev='cuda'):
    w = np.asarray(np.load(path)['weights'], np.float32)
    return dict(phi_s=_stack(w, OFF_PHI_S, phi_layers(PHI_IN_SL), dtype, dev), phi_g=_stack(w, OFF_PHI_G, phi_layers(PHI_IN_G), dtype, dev), rho=_stack(w, OFF_RHO_S, RHO_LAYERS, dtype, dev))


def _fwd(layers, x):
    for k, (W, b) in enumerate(layers):
        x = x @ W.T + b
        if k < len(layers) - 1: x = torch.relu(x)
    return x


def faz(net, dp, dv, own_z, own_v, mass, active):
    """dp, dv: (B, 3) top minus own; own_z (B,); own_v (B, 3); active (B,) bool (top above own) -> vertical acceleration (B,) [m/s^2]."""
    ground = torch.stack([-own_z, -own_v[..., 0], -own_v[..., 1], -own_v[..., 2]], -1)
    gate = (active & (dp[..., 0].abs() < GATE_DXY) & (dp[..., 1].abs() < GATE_DXY) & (dv[..., 0].abs() < GATE_DVX)).to(dp.dtype)[..., None]
    x = _fwd(net['phi_g'], ground) + gate * _fwd(net['phi_s'], torch.cat([dp, dv], -1))
    return (_fwd(net['rho'], x)[..., 0] * GRAMS_TO_NEWTONS / mass).clamp(-OUT_CLAMP, OUT_CLAMP)


from pathlib import Path
DEFAULT_WEIGHTS = Path(__file__).resolve().parents[2] / 'simulator' / 'frsim' / 'data' / 'ns2_v2.npz'
