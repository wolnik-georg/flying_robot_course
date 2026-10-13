#!/usr/bin/env python3
"""Plant downwash from exported full-bank RNN weights (same forward as onboard firmware)."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from test_residual_nn import N_WEIGHTS, reference


class BankPlantDisturbance:
    """Z-only specific-force disturbance → external force on the np quad plant."""

    def __init__(self, weights_path: str | Path):
        data = np.load(weights_path)
        self.w = np.asarray(data["weights"], np.float32)
        if self.w.size != N_WEIGHTS:
            raise ValueError(f"expected {N_WEIGHTS} weights, got {self.w.size}")

    symmetrize: bool = False  # average over x/y mirror images (physical symmetry of the downwash)
    above_only: bool = True   # only a neighbour above the drone produces downwash on it (see _fa_raw)

    def _fa_raw(
        self,
        index: int,
        positions: list[np.ndarray],
        velocities: list[np.ndarray],
        mass: float,
        *,
        scale: float = 1.0,
    ) -> np.ndarray:
        pos = np.asarray(positions[index], float)
        vel = np.asarray(velocities[index], float)
        rel = []
        for j, (p, v) in enumerate(zip(positions, velocities)):
            if j == index:
                continue
            # 2026-10-12: the learned downwash acts on the drone that has the other one ABOVE it (the network was trained only with
            # a neighbour above, dz > 0). Without this gate the TOP drone was evaluated with a neighbour below, extrapolated and
            # received a spurious downward force (-10 cm dip at every A8 crossing in the simulation, -0.2 cm on hardware).
            if self.above_only and float(np.asarray(p, float)[2]) <= float(pos[2]):
                continue
            rel.append((np.asarray(p, float) - pos, np.asarray(v, float) - vel))
        acc, _clamped = reference(self.w, rel, float(pos[2]), vel, float(mass))
        return np.asarray(acc, float) * float(mass) * float(scale)

    def compute_fa_newtons(self, index, positions, velocities, mass, *, scale=1.0):
        if not self.symmetrize:
            return self._fa_raw(index, positions, velocities, mass, scale=scale)
        own = np.asarray(positions[index], float)
        acc = np.zeros(3)
        for sx in (1.0, -1.0):
            for sy in (1.0, -1.0):
                m = np.array([sx, sy, 1.0])
                pp = [own + m * (np.asarray(p, float) - own) for p in positions]
                vv = [m * np.asarray(v, float) for v in velocities]
                f = self._fa_raw(index, pp, vv, mass, scale=scale)
                acc += f * np.array([1.0, 1.0, 1.0])  # only Fz is used downstream
        return acc / 4.0

    def compute_tau_nm(
        self,
        index: int,
        positions: list[np.ndarray],
        velocities: list[np.ndarray],
        mass: float,
        *,
        c_lever2: float,
        delta: float = 0.02,
    ) -> np.ndarray:
        """Body roll/pitch torque from the lateral gradient of the vertical downwash force.

        tau_x = c * dFz/dy, tau_y = -c * dFz/dx  (force acting off-centre across the airframe,
        r x F with F = (0,0,Fz)); c [m^2] is the single calibrated scalar (about lever^2).
        """
        if c_lever2 == 0.0:
            return np.zeros(3)
        grads = []
        for ax in (0, 1):
            fz = []
            for sgn in (+1.0, -1.0):
                pp = [np.asarray(p, float).copy() for p in positions]
                pp[index][ax] += sgn * delta
                fz.append(self.compute_fa_newtons(index, pp, velocities, mass)[2])
            grads.append((fz[0] - fz[1]) / (2.0 * delta))
        return np.array([c_lever2 * grads[1], -c_lever2 * grads[0], 0.0])
