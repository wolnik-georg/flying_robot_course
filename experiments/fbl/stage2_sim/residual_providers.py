"""Residual providers for the Stage 2 closed loop. Each returns (acc[3], jerk[3], ang_acc[3]) in the bundle's convention
(world-frame acceleration the OTHER vehicle causes on the ego vehicle; negative z = pushed down), from ego and neighbour 13-states.
The bundle code is imported, never edited; the only in-memory change is the drag coefficient C_D (module default arguments)."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(REPO / "flying_drone_stack/firmware_app/host"))
from common import bundle as B  # noqa: E402
from common.vehicle import OUR  # noqa: E402

CKPT = REPO / "experiments/fbl/out/stage1/v0/fbl_residual_v0_rom_nn_h9.pth"
NS2_W = REPO / "flying_drone_stack/tools/residual/weights/ns2_v2.npz"
Z_MIN, Z_MAX = 0.10, 1.30     # region the Stage 1 fit was made on; no residual outside (ROM is singular at z0 -> 0)


def _set_cd(res, b, c_d: float) -> None:
    """The ROM drag coefficient is a default argument of the bundle's analytic method and a module global; set both in memory."""
    f = type(res)._acceleration_nominal_analytical_jvp
    f.__defaults__ = (float(c_d), f.__defaults__[1])
    b.dw.C_d = float(c_d)


class FBLResidual:
    """ROM (+ optional fitted net_f) through the bundle's DownwashResidual. Force only: angular acceleration is zeroed (no torque label)."""

    def __init__(self, c_d: float, use_nn: bool, ckpt: Path = CKPT):
        self.b = B.load(); B.set_vehicle(self.b, OUR["mass"], OUR["arm_length"])
        ode = None
        if use_nn:
            c = torch.load(ckpt, map_location="cpu", weights_only=False)
            ode = self.b.nnm.ODEFunc()
            ode.net_f.load_state_dict({k[len("net_f."):]: v for k, v in c["state_dict_net_f"].items()})
            ode.eval()
        self.res = self.b.dw.DownwashResidual(ode, use_nn=use_nn, use_nominal=True).eval()
        self.res.nominal_model.mass = OUR["mass"]   # the bundle divides the ROM force by ITS vehicle mass (0.03 kg), set_vehicle() only changes the force side; without this the ROM is 1.42x too large for our 0.0427 kg
        self.c_d = float(c_d)

    def __call__(self, t, ego, nbr):
        _set_cd(self.res, self.b, self.c_d)   # the coefficient is process-global in the bundle: set it on every call, instances differ
        z0 = nbr[2] - ego[2]
        if not (Z_MIN < z0 < Z_MAX):
            return np.zeros(3), np.zeros(3), np.zeros(3)
        y = torch.from_numpy(np.concatenate([ego, np.zeros(4), nbr])).float()
        with torch.no_grad():
            acc, jerk, _ = self.res(t, y)
        return acc.squeeze(0).numpy().astype(float), jerk.squeeze(0).numpy().astype(float), np.zeros(3)


class NS2Residual:
    """NS2 v2 network as FBL residual (z only, neighbour above, ground term included); jerk by backward difference."""

    def __init__(self, scale: float = 1.0):
        sys.path.insert(0, str(REPO / "experiments/analysis"))
        from unittest.mock import MagicMock
        sys.modules.setdefault('cffirmware', MagicMock())   # the reference forward is pure numpy; the module only imports the firmware bindings for other tests
        from test_residual_nn import reference
        self.ref = reference; self.w = np.asarray(np.load(NS2_W)["weights"], np.float32); self.scale = scale
        self.prev = None

    def acc(self, ego, nbr):
        rel = [(nbr[:3] - ego[:3], nbr[3:6] - ego[3:6])] if nbr[2] > ego[2] else []
        a, _ = self.ref(self.w, rel, float(ego[2]), ego[3:6], OUR["mass"])
        return np.asarray(a, float) * self.scale

    def __call__(self, t, ego, nbr):
        a = self.acc(ego, nbr)
        jerk = np.zeros(3) if self.prev is None or t <= self.prev[0] else (a - self.prev[1]) / (t - self.prev[0])
        self.prev = (t, a)
        return a, jerk, np.zeros(3)


class Zero:
    def __call__(self, t, ego, nbr):
        return np.zeros(3), np.zeros(3), np.zeros(3)
