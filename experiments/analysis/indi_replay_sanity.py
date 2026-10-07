#!/usr/bin/env python3
"""Omar C vs Rust on first replay ticks (must match ~1e-9 scale on identical inputs)."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
HOST = REPO / "flying_drone_stack/firmware_app/host"
BUILD = Path("/home/georg/Desktop/crazyflie-firmware/build")


def main() -> None:
    test = HOST / "test_omar_indi_rust_vs_c.py"
    out = subprocess.run([sys.executable, str(test), str(BUILD)], capture_output=True, text=True)
    print(out.stdout)
    if out.returncode != 0:
        print(out.stderr, file=sys.stderr)
        sys.exit(out.returncode)
    npz = REPO / "experiments/analysis/out/indi_replay/A8/inputs.npz"
    if npz.is_file():
        import numpy as np

        z = np.load(npz)
        i = int(z["warmup_start"]) + 800
        case = {
            "pos": tuple(float(x) for x in z["pos"][i]),
            "vel": tuple(float(x) for x in z["vel"][i]),
            "rpy": tuple(float(x) for x in z["rpy"][i]),
            "gyro": tuple(float(x) for x in z["gyro_deg_s"][i]),
            "sp_pos": tuple(float(x) for x in z["sp_pos"][i]),
            "sp_vel": tuple(float(x) for x in z["sp_vel"][i]),
            "sp_acc": tuple(float(x) for x in z["sp_acc"][i]),
            "yaw_d": float(z["yaw_d_rad"][i]),
            "rpm": tuple(int(x) for x in z["rpm"][i]),
        }
        runner_c = HOST / "_omar_indi_rust_case_runner.py"
        import os

        env = os.environ.copy()
        env["PYTHONPATH"] = str(BUILD)
        for side in ("c", "rust"):
            r = subprocess.run(
                [sys.executable, str(runner_c), side, str(BUILD), json.dumps(case)],
                capture_output=True,
                text=True,
                env=env,
                check=True,
            )
            print(side, r.stdout.strip())
    print(json.dumps({"harness": "ok"}))


if __name__ == "__main__":
    main()
