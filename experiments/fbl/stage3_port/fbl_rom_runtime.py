"""Reference of the onboard residual evaluation (float32, same steps the C code will take): gate, geometry, table lookup, fade, jerk.
The C port must reproduce this function; this function is tested against the authors' DownwashResidual (ROM only)."""
from __future__ import annotations
import numpy as np

Z_LO, Z_FADE0, Z_FADE1 = 0.05, 1.2, 1.4     # neighbour-above gate; linear fade to zero between 1.2 and 1.4 m (fit data ends at 1.3 m)
R_MAX_USE = 0.6


def fade(z0):
    return float(np.clip((Z_FADE1 - z0) / (Z_FADE1 - Z_FADE0), 0.0, 1.0))


def lookup32(tab, z0, r0):
    """tab: dict(zs, rs, t[nz,nr,3]) float32. Bilinear in z0 (uniform) and u = sqrt(r/RMAX) (uniform). Returns (a1, da/dz0, da/dr0)."""
    zs, rs, t = tab["zs"], tab["rs"], tab["t"]; nz, nr = len(zs), len(rs)
    fz = (z0 - zs[0]) / (zs[-1] - zs[0]) * (nz - 1); i = int(np.clip(np.floor(fz), 0, nz - 2)); sz = np.float32(fz - i)
    u = np.sqrt(np.float32(min(r0, rs[-1])) / np.float32(rs[-1])) * (nr - 1); j = int(np.clip(np.floor(u), 0, nr - 2))
    sr = np.float32((r0 - rs[j]) / (rs[j + 1] - rs[j]))
    return ((1 - sz) * (1 - sr) * t[i, j] + sz * (1 - sr) * t[i + 1, j] + (1 - sz) * sr * t[i, j + 1] + sz * sr * t[i + 1, j + 1]).astype(np.float32)


def residual(tab, c_d, p, v, p_top, v_top):
    """World-frame (acc[3], jerk[3]) the upper vehicle causes on the ego vehicle. Upper vehicle treated as upright (axis = world z)."""
    d = np.asarray(p, np.float32) - np.asarray(p_top, np.float32)      # ego - top
    dd = np.asarray(v, np.float32) - np.asarray(v_top, np.float32)
    z0 = float(-d[2])                                                   # top above ego -> positive
    if not (Z_LO < z0 < Z_FADE1):
        return np.zeros(3, np.float32), np.zeros(3, np.float32)
    rv = d[:2]; r0 = float(np.linalg.norm(rv))
    if r0 >= R_MAX_USE:
        return np.zeros(3, np.float32), np.zeros(3, np.float32)
    a1, da_dz, da_dr = lookup32(tab, z0, r0)
    dz0 = -float(dd[2])                                                 # z0 = |d.z| with sign: d(z0)/dt = -ddot_z
    dr0 = float(rv @ dd[:2]) / max(r0, 1e-6)
    w = fade(z0)
    acc = np.zeros(3, np.float32); jerk = np.zeros(3, np.float32)
    acc[2] = np.float32(w * c_d * a1)
    jerk[2] = np.float32(w * c_d * (da_dz * dz0 + da_dr * dr0))         # fade derivative neglected (inside the fade band only)
    return acc, jerk
