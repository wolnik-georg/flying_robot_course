#!/usr/bin/env python3
"""Analytic solo reference trajectories (pos / vel / acc / jerk), batched over rows with randomised parameters (option A: no second drone).
Families: circle, figure-8, point-to-point move (smoothstep, incl. landing), takeoff from the ground. Speeds 0.1-0.5 m/s like the NS2 grid."""
import math, torch

def _ss(u):      # quintic smoothstep and its derivatives w.r.t. u
    s = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5; d1 = 30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4; d2 = 60 * u - 180 * u ** 2 + 120 * u ** 3; d3 = 60 - 360 * u + 360 * u ** 2
    return s, d1, d2, d3


def sample_family(B, gen, dtype, dev, z0=0.5):
    """Return per-row parameter dict. fam: 0 circle, 1 figure-8, 2 move, 3 takeoff."""
    U = lambda lo, hi: lo + (hi - lo) * torch.rand(B, generator=gen, dtype=dtype, device=dev)
    fam = torch.randint(0, 4, (B,), generator=gen, device=dev)
    speed = U(0.1, 0.5); rad = U(0.3, 0.7); ang = U(0, 2 * math.pi); phase = U(0, 2 * math.pi)
    az = U(0.0, 0.08) * (torch.rand(B, generator=gen, dtype=dtype, device=dev) < 0.5); wz = U(0.5, 1.5)
    amp = U(0.1, 0.4); dur = U(1.0, 1.6); land = (torch.rand(B, generator=gen, dtype=dtype, device=dev) < 0.25) & (fam == 2); zc = torch.full((B,), z0, dtype=dtype, device=dev)
    return dict(fam=fam, speed=speed, rad=rad, ang=ang, phase=phase, az=az, wz=wz, amp=amp, dur=dur, land=land, zc=zc)


def evaluate(p, t):
    """t: (n, B) times [s] -> pos, vel, acc, jerk each (n, B, 3)."""
    fam = p['fam'][None]; n, B = t.shape; z = torch.zeros_like(t)
    # circle: w = v / r
    w = (p['speed'] / p['rad'])[None]; ph = w * t + p['phase'][None]; r = p['rad'][None]
    cx, cy = r * torch.cos(ph), r * torch.sin(ph)
    c = [torch.stack([cx, cy, z], -1), torch.stack([-r * w * torch.sin(ph), r * w * torch.cos(ph), z], -1), torch.stack([-r * w ** 2 * torch.cos(ph), -r * w ** 2 * torch.sin(ph), z], -1), torch.stack([r * w ** 3 * torch.sin(ph), -r * w ** 3 * torch.cos(ph), z], -1)]
    # figure-8: x = a sin(w t), y = (a/2) sin(2 w t), peak speed ~ a w
    a = r; w8 = p['speed'][None] / a; ph8 = w8 * t + p['phase'][None]
    f = [torch.stack([a * torch.sin(ph8), 0.5 * a * torch.sin(2 * ph8), z], -1), torch.stack([a * w8 * torch.cos(ph8), a * w8 * torch.cos(2 * ph8), z], -1),
         torch.stack([-a * w8 ** 2 * torch.sin(ph8), -2 * a * w8 ** 2 * torch.sin(2 * ph8), z], -1), torch.stack([-a * w8 ** 3 * torch.cos(ph8), -4 * a * w8 ** 3 * torch.cos(2 * ph8), z], -1)]
    # move: smoothstep of amplitude amp along direction ang after 1 s of hold; landing: z from zc to 0 over 3 s
    land = p['land'][None]; D = torch.where(p['land'], torch.full_like(p['dur'], 3.0), p['dur'])[None]; u = ((t - 1.0) / D).clamp(0, 1); s, d1, d2, d3 = _ss(u)
    inside = ((t > 1.0) & (t < 1.0 + D)).to(t.dtype); A = p['amp'][None]; cs, sn = torch.cos(p['ang'])[None], torch.sin(p['ang'])[None]
    ax_ = torch.where(land, z, cs * A); ay_ = torch.where(land, z, sn * A); az_ = torch.where(land, -p['zc'][None], z)
    m = [torch.stack([ax_ * s, ay_ * s, az_ * s], -1), torch.stack([ax_ * d1 / D, ay_ * d1 / D, az_ * d1 / D], -1) * inside[..., None],
         torch.stack([ax_ * d2 / D ** 2, ay_ * d2 / D ** 2, az_ * d2 / D ** 2], -1) * inside[..., None], torch.stack([ax_ * d3 / D ** 3, ay_ * d3 / D ** 3, az_ * d3 / D ** 3], -1) * inside[..., None]]
    # takeoff: ground to zc over 3 s
    T0 = 3.0; u2 = (t / T0).clamp(0, 1); s2, e1, e2, e3 = _ss(u2); ins2 = (t < T0).to(t.dtype); zc = p['zc'][None]
    k = [torch.stack([z, z, zc * s2], -1), torch.stack([z, z, zc * e1 / T0 * ins2], -1), torch.stack([z, z, zc * e2 / T0 ** 2 * ins2], -1), torch.stack([z, z, zc * e3 / T0 ** 3 * ins2], -1)]
    out = []
    for q in range(4):
        sel = torch.stack([c[q], f[q], m[q], k[q]], 0)           # (4, n, B, 3)
        v = sel[fam.expand(n, B), torch.arange(n, device=t.device)[:, None], torch.arange(B, device=t.device)[None]]
        out.append(v)
    pos, vel, acc, jerk = out
    zterm = p['az'][None] * torch.sin(p['wz'][None] * t); zd = p['az'][None] * p['wz'][None] * torch.cos(p['wz'][None] * t)
    zdd = -p['az'][None] * p['wz'][None] ** 2 * torch.sin(p['wz'][None] * t); zddd = -p['az'][None] * p['wz'][None] ** 3 * torch.cos(p['wz'][None] * t)
    nz = (fam != 3) & ~(fam == 2) | (fam == 2) & ~p['land'][None]           # z oscillation only for circle / figure-8 / non-landing moves
    nzf = nz.to(t.dtype)
    pos = pos + torch.stack([z, z, zterm * nzf], -1) + torch.stack([z, z, (p['zc'][None] * nzf).expand_as(t)], -1)
    vel = vel + torch.stack([z, z, zd * nzf], -1); acc = acc + torch.stack([z, z, zdd * nzf], -1); jerk = jerk + torch.stack([z, z, zddd * nzf], -1)
    pos = pos + torch.stack([z, z, torch.where(land & (fam == 2), p['zc'][None], z)], -1)           # landing starts at the hover height
    return dict(pos=pos, vel=vel, acc=acc, jerk=jerk)
