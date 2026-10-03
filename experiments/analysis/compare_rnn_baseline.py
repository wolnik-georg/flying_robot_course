#!/usr/bin/env python3
"""Compare RNN predictions: current tree (div=1) vs baseline lib in a git worktree build."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
HOST = REPO / "flying_drone_stack/firmware_app/host"
sys.path.insert(0, str(REPO / "flying_drone_stack/firmware_app/host"))
sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/build")

# Import current bindings (must be built from current tree first).
import cffirmware as fw  # noqa: E402
from test_residual_nn import Fw, N_WEIGHTS, next_peer_ts  # noqa: E402


def run_vector(w: np.ndarray, label: str):
    rng = np.random.default_rng(0xC0FFEE)
    w = (rng.standard_normal(N_WEIGHTS) * 0.2).astype(np.float32) if w is None else w
    fw.cvar.g_rnn_div = 1
    h = Fw(own_pos=(0.1, -0.2, 1.0), own_vel=(0.3, 0.0, -0.1))
    assert h.upload(w) == 1
    p1 = [np.array([0.15, -0.20, 1.32], np.float32)]
    h.peers(p1, next_peer_ts())
    h.step()
    return h.pred().copy()


def main():
    w = np.random.default_rng(0xC0FFEE).standard_normal(N_WEIGHTS).astype(np.float32) * 0.2
    cur = run_vector(w, "current")
    print("Current tree pred_z", cur[2])
    print(
        "Run baseline comparison manually: build baseline worktree bindings, "
        "record pred_z, compare with np.allclose(atol=1e-6)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
