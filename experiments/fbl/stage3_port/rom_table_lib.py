"""ROM downwash table for the onboard FBL port: per-unit-C_D acceleration a1(z0, r0) (world z, m/s^2, negative = pushed down) and its
partial derivatives, from the authors' analytic ROM (their code imported, not edited), on OUR vehicle constants.
Runtime: acc_z = C_D * a1; jerk_z = C_D * (da1/dz0 * dz0dot + da1/dr0 * dr0dot)  (upper vehicle treated as upright, as in the bundle).
Grid: z0 uniform, r0 quadratic (r = RMAX * u^2) because the jet profile is narrow (~0.1 m) and the table must stay small."""
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
import torch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import bundle as B  # noqa: E402
from common.vehicle import OUR  # noqa: E402

ZMIN, ZMAX, RMAX = 0.05, 2.0, 0.6


def rom_exact(z, r):
    b = B.load(); B.set_vehicle(b, OUR["mass"], OUR["arm_length"])
    out = []
    z = np.atleast_1d(np.asarray(z, float)); r = np.atleast_1d(np.asarray(r, float))
    for i in range(0, len(z), 4000):
        F, dz, dr = b.force.downwash_force_and_derivs_fo(torch.tensor(z[i:i + 4000]), torch.tensor(r[i:i + 4000]), C_d=1.0, S=0.0997)
        out.append(np.stack([-F.numpy(), -dz.numpy(), -dr.numpy()], 1) / OUR["mass"])
    return np.concatenate(out)          # columns: a1, da1/dz0, da1/dr0


def grid(nz, nr):
    zs = np.linspace(ZMIN, ZMAX, nz); us = np.linspace(0, 1, nr); rs = RMAX * us ** 2
    return zs, rs


def build(nz, nr):
    zs, rs = grid(nz, nr); Z, R = np.meshgrid(zs, rs, indexing="ij")
    t = rom_exact(Z.ravel(), R.ravel()).reshape(nz, nr, 3)
    return zs, rs, t


def _hermite(f0, f1, d0, d1, s, h):
    s2, s3 = s * s, s ** 3
    return (2 * s3 - 3 * s2 + 1) * f0 + (s3 - 2 * s2 + s) * h * d0 + (-2 * s3 + 3 * s2) * f1 + (s3 - s2) * h * d1


def lookup(zs, rs, t, z, r):
    """Bilinear in the value tables (value and both partials interpolated separately); returns a1, da/dz0, da/dr0 and zero outside the grid."""
    z = np.asarray(z, float); r = np.asarray(r, float)
    u = np.sqrt(np.clip(r, 0, RMAX) / RMAX) * (len(rs) - 1)
    i = np.clip(np.floor((z - ZMIN) / (ZMAX - ZMIN) * (len(zs) - 1)).astype(int), 0, len(zs) - 2)
    j = np.clip(np.floor(u).astype(int), 0, len(rs) - 2)
    sz = (z - zs[i]) / (zs[i + 1] - zs[i]); sr = (r - rs[j]) / (rs[j + 1] - rs[j])
    out = np.zeros(z.shape + (3,))
    for k in range(3):
        a = t[i, j, k]; b_ = t[i + 1, j, k]; c = t[i, j + 1, k]; d = t[i + 1, j + 1, k]
        out[..., k] = (1 - sz) * (1 - sr) * a + sz * (1 - sr) * b_ + (1 - sz) * sr * c + sz * sr * d
    return out
