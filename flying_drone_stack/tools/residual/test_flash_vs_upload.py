#!/usr/bin/env python3
"""Parity: RAM upload build vs flash-resident build (host x86_64, same weights).

  python3 flying_drone_stack/tools/residual/test_flash_vs_upload.py
"""
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
FW_APP = ROOT / "flying_drone_stack/firmware_app"
CF_FW = Path.home() / "Desktop/crazyflie-firmware"
HOST_TEST = FW_APP / "host/test_residual_nn.py"

N_WEIGHTS = 19297


def cargo_host(features: str, npz: Path | None) -> None:
    env = os.environ.copy()
    env["DRONE_PLATFORM"] = "bl"
    env["RUSTFLAGS"] = "-C panic=abort"
    if npz is not None:
        env["CF_RNN_WEIGHTS_NPZ"] = str(npz)
    subprocess.check_call(
        [
            "cargo",
            "build",
            "--release",
            "--target",
            "x86_64-unknown-linux-gnu",
            "--features",
            features,
        ],
        cwd=FW_APP,
        env=env,
    )
    subprocess.check_call(["make", "bindings_python"], cwd=CF_FW)


def run_predict(w: np.ndarray, use_upload: bool) -> float:
    # Re-import after each bindings rebuild.
    for mod in list(sys.modules):
        if mod == "cffirmware" or mod.startswith("cffirmware."):
            del sys.modules[mod]
    sys.path.insert(0, str(CF_FW / "build"))
    import importlib.util

    spec = importlib.util.spec_from_file_location("test_residual_nn", HOST_TEST)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    h = mod.Fw(own_pos=(0.0, 0.0, 0.8), own_vel=(0.1, -0.05, 0.02))
    h.peers([(0.05, 0.05, 1.0)], 1000)
    if use_upload:
        if h.upload(w) != 1:
            raise RuntimeError("upload failed")
    else:
        import cffirmware as fw  # noqa: E402

        fw.oot_rnn_service()
        if fw.cvar.g_rnn_ready != 1:
            raise RuntimeError("flash build: g_rnn_ready != 1 after service")
    h.step(5)
    return float(h.pred()[2])


def main() -> int:
    rng = np.random.default_rng(42)
    w = rng.standard_normal(N_WEIGHTS).astype(np.float32)
    with tempfile.TemporaryDirectory() as td:
        npz = Path(td) / "parity.npz"
        np.savez(npz, weights=w)

        cargo_host("residual_nn", None)
        z_upload = run_predict(w, use_upload=True)

        cargo_host("residual_nn_flash", npz)
        z_flash = run_predict(w, use_upload=False)

    diff = abs(z_upload - z_flash)
    print(f"upload z={z_upload:.9f}  flash z={z_flash:.9f}  |diff|={diff:.3e}")
    if not np.isfinite(z_upload) or not np.isfinite(z_flash):
        print("FAIL: non-finite prediction")
        return 1
    if diff > 1e-5:
        print("FAIL: predictions differ")
        return 1
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
