# Copy of experiments/tuning/diff/s0_torch.py (2026-11-11, Stage 2 of docs/plan_diffcotune_2026-11-11.md). Only change: this header; behaviour identical (the old file is the record and stays untouched).
#!/usr/bin/env python3
"""Differentiable copy of S0 (controller_omar_indi.c with indi 0, NS2 off, FBL off) + mixer, in PyTorch, batched over B independent drones.
The C firmware is NOT changed; equivalence is checked by diff/check_s0_equiv.py against the host bindings.
Same constants as experiments/tuning/plant_core.py (THRUST_MAX 0.2 N, ARM 0.707*0.05, T2T). Float quantisation of the uint16 motor command is not modelled (differentiable)."""
import torch

G = 9.81
DT = 1.0 / 500.0
IDLE = 7000 / 65535 * 0.2          # firmware idle thrust (powerDistributionGetIdleThrust = 7000 counts): motors never go below it while armed
THRUST_MAX, ARM, T2T = 0.2, 0.707106781 * 0.050, 0.00569278844371417
J_CTRL = (16.571710e-6, 16.655602e-6, 29.261652e-6)      # firmware default controller inertia (g_self.J); the plant body inertia is plant_core.J_OMAR
GAINS0 = dict(Kpos_P=(7., 7., 7.), Kpos_D=(4., 4., 4.), Kpos_Iz=1.5, KR=(.007, .007, .008), Komega=(.00115, .00115, .002), KI=(.03, .03, .03), Kpos_I_limit=2.0)


_CACHE = {}
def K(x, dtype, dev):
    """Constant tensor memoised per (value, dtype, device): host-to-device copies are not allowed inside CUDA graph capture (the warm-up call fills the cache)."""
    key = (id(x) if torch.is_tensor(x) and x.device.type == 'cpu' else repr(x) if not torch.is_tensor(x) else None, dtype, str(dev))
    if key[0] is None: return x.to(dtype=dtype, device=dev)          # tensor already on the device: no copy
    if key not in _CACHE: _CACHE[key] = torch.as_tensor(x, dtype=dtype, device=dev)
    return _CACHE[key]


def vcross(a, b): return torch.linalg.cross(a, b, dim=-1)
def vdot(a, b): return (a * b).sum(-1)
def vnorm(a): return a / torch.linalg.norm(a, dim=-1, keepdim=True)


def quat2rot(q):
    x, y, z, w = q.unbind(-1)
    return torch.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
                        2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
                        2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1).reshape(*q.shape[:-1], 3, 3)


class S0:
    """gains: dict of tensors broadcastable to (B,3) (Kpos_P, Kpos_D, KR, Komega, KI) and (B,) / scalar (Kpos_Iz, Kpos_I_limit)."""
    def __init__(self, B, gains, mass=0.0427, J=J_CTRL, dtype=torch.float64, device='cpu'):
        self.B, self.dt_, self.dev, self.dtype = B, DT, device, dtype
        t = lambda v: K(v, dtype, device)
        self.g = {k: t(v) for k, v in gains.items()}; self.mass = mass; self.J = t(J)
        self.i_pos = torch.zeros(B, 3, dtype=dtype, device=device); self.i_att = torch.zeros(B, 3, dtype=dtype, device=device)

    def step(self, pos_d, vel_d, acc_d, jerk_d, pos, vel, q, gyro_dps, snap_d=None):
        g, m = self.g, self.mass
        pos_e, vel_e = pos_d - pos, vel_d - vel
        self.i_pos = self.i_pos + DT * pos_e
        iz = torch.clamp(self.i_pos[..., 2], -g['Kpos_I_limit'], g['Kpos_I_limit']); self.i_pos = torch.cat([self.i_pos[..., :2], iz[..., None]], -1)
        i_term = torch.cat([torch.zeros_like(iz)[..., None], torch.zeros_like(iz)[..., None], (g['Kpos_Iz'] * iz)[..., None]], -1)   # Kpos_Iz != 0: x, y integral terms off
        a_d = acc_d + K([0, 0, G], self.dtype, self.dev) + g['Kpos_D'] * vel_e + g['Kpos_P'] * pos_e + i_term
        R = quat2rot(q); zw = R[..., :, 2]
        thrust = m * vdot(a_d, zw)
        on = thrust >= 0.01
        self.i_pos = self.i_pos * on[..., None]; self.i_att = self.i_att * on[..., None]      # controllerOmarIndiReset while on the ground
        yc = K([0., 1., 0.], self.dtype, self.dev).expand_as(a_d); xc = K([1., 0., 0.], self.dtype, self.dev).expand_as(a_d)
        xb = vnorm(vcross(yc, a_d)); yb = vnorm(vcross(a_d, xb)); zb = vcross(xb, yb)
        Rd = torch.stack([xb, yb, zb], -1)
        eRM = Rd.transpose(-1, -2) @ R - R.transpose(-1, -2) @ Rd
        eR = 0.5 * torch.stack([eRM[..., 2, 1], eRM[..., 0, 2], eRM[..., 1, 0]], -1)
        om = torch.deg2rad(gyro_dps)
        c = thrust / m; B1 = c; B3 = -vdot(yc, zb); C3 = torch.linalg.norm(vcross(yc, zb), dim=-1)
        D1 = vdot(xb, jerk_d); D2 = -vdot(yb, jerk_d)
        od = torch.stack([D2 / B1, D1 / B1, (-B3 * D1) / (B1 * C3)], -1)            # yaw rate setpoint 0 -> D3 = 0
        snap = torch.zeros_like(a_d) if snap_d is None else snap_d
        c_dot = vdot(zb, jerk_d)
        E1 = vdot(xb, snap) - 2 * c_dot * od[..., 1] - c * od[..., 0] * od[..., 2]
        E2 = -vdot(yb, snap) - 2 * c_dot * od[..., 0] + c * od[..., 1] * od[..., 2]
        E3 = -od[..., 0] * od[..., 1] * vdot(yc, yb) - od[..., 0] * od[..., 2] * vdot(yc, zb)
        odd = torch.stack([E2 / B1, E1 / B1, (-B3 * E1 + B1 * E3) / (B1 * C3)], -1)
        RtRd = R.transpose(-1, -2) @ Rd
        om_r = (RtRd @ od[..., None])[..., 0]
        self.i_att = self.i_att + DT * eR
        Jo = self.J * om
        u = -g['KR'] * eR - g['Komega'] * (om - om_r) - g['KI'] * self.i_att + vcross(om, Jo) - self.J * (vcross(om, om_r) - (RtRd @ odd[..., None])[..., 0])
        return thrust, u


def mixer(thrust, u):
    """powerDistributionForceTorque + cap (shift all motors down by the excess over full scale) -> per-motor force [N]."""
    roll, pitch = 0.25 / ARM * u[..., 0], 0.25 / ARM * u[..., 1]; tp = 0.25 * thrust; yaw = 0.25 * u[..., 2] / T2T
    Fm = torch.stack([tp - roll - pitch - yaw, tp - roll + pitch + yaw, tp + roll + pitch - yaw, tp + roll - pitch + yaw], -1).clamp(min=0.0)
    over = (Fm.max(-1).values - THRUST_MAX).clamp(min=0.0)
    return (Fm - over[..., None]).clamp(min=IDLE)
