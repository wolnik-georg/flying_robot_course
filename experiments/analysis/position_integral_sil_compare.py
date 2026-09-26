#!/usr/bin/env python3
"""SIL hover: geometric controller (oot mode 0) with position integral on vs off.

Desk-only — expects two pre-built cffirmware .so files:
  OFF (default): crazyflie-firmware/build/_cffirmware*.so
  ON:  /tmp/cffirmware_posint_on/_cffirmware*.so  (build via --rebuild-on)

Does not modify committed lib.rs when run normally; --rebuild-on patches locally, rebuilds, restores.
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
LIB_RS = ROOT / "flying_drone_stack/firmware_app/src/lib.rs"
FW = Path("/home/georg/Desktop/crazyflie-firmware")
BUILD = FW / "build"


def rebuild_with_integral(enabled: bool) -> Path:
    text = LIB_RS.read_text()
    new_val = "true" if enabled else "false"
    patched, n = re.subn(
        r"const ENABLE_POSITION_INTEGRAL: bool = (true|false);",
        f"const ENABLE_POSITION_INTEGRAL: bool = {new_val};",
        text,
        count=1,
    )
    if n != 1:
        raise SystemExit("ENABLE_POSITION_INTEGRAL not found in lib.rs")
    LIB_RS.write_text(patched)
    try:
        subprocess.run(
            ["make", "bindings_python"],
            cwd=FW,
            check=True,
            env={**os.environ, "PYTHON": sys.executable},
        )
    finally:
        LIB_RS.write_text(text)
    so = glob.glob(str(BUILD / "_cffirmware*.so"))
    if not so:
        raise SystemExit("bindings build produced no .so")
    return Path(so[0])


def _mock_ros_imports() -> None:
    """np.py imports rclpy at module load; desk SIL only needs Quadrotor."""
    from unittest.mock import MagicMock

    for name in (
        "rclpy",
        "rclpy.node",
        "rclpy.time",
        "rosgraph_msgs",
        "rosgraph_msgs.msg",
    ):
        sys.modules.setdefault(name, MagicMock())


def run_hover(
    so_path: Path,
    label: str,
    duration: float = 14.0,
    height: float = 1.0,
    f_ext_z: float = -0.008,
) -> dict:
    cs2_sim = Path("/home/georg/Desktop/crazyswarm2/crazyflie_sim")
    code = f'''
import sys
from unittest.mock import MagicMock
for name in ("rclpy", "rclpy.node", "rclpy.time", "rosgraph_msgs", "rosgraph_msgs.msg"):
    sys.modules.setdefault(name, MagicMock())
sys.path.insert(0, "{so_path.parent}")
sys.path.insert(0, "{cs2_sim}")
import numpy as np
import rowan
import cffirmware as firm
from crazyflie_sim.crazyflie_sil import CrazyflieSIL
from crazyflie_sim import sim_data_types
from crazyflie_sim.backend.np import Quadrotor

c = firm.cvar
for k, v in dict(kr=2400.0, kw=170.0, kr_z=2400.0, kw_z=170.0, fc_bw=60.0, mass=0.041,
                 kt1=4.1623e-10, kt2=4.0592e-10, kt3=4.1116e-10, kt4=4.0631e-10, ff_free=0,
                 filt_order=1, filt_tau=1, j_scale=1.0, clamp_en=11, tau_xy_max=0.045,
                 tau_z_max=0.0025, tilt_max_deg=30.0, thrust_max=0.8, notch_en=0,
                 notch_f0=6.9, notch_bw=3.0).items():
    setattr(c, "g_indi_" + k, v)
c.g_kp_xy, c.g_kp_z, c.g_kv_xy, c.g_kv_z = 64.0, 48.0, 8.0, 7.0
c.g_controller_mode = 0
CrazyflieSIL._oot_count = 0
J = [firm.oot_inertia(i) for i in range(3)]
PH = dict(mass=0.041, kt=[c.g_indi_kt1, c.g_indi_kt2, c.g_indi_kt3, c.g_indi_kt4],
          arm_length=firm.oot_arm_length(), t2t=firm.oot_thrust2torque(),
          inertia=J, motor_tau=0.044)
p0 = np.array([0., 0., 0.])
t = [0.0]
cf = CrazyflieSIL("cf", p0, "oot", lambda: t[0])
q = Quadrotor(sim_data_types.State(pos=p0.copy()), PH)
cf.takeoff({height}, 3.0)
dt = 1e-3
T = {duration}
zs, rolls, pitchs, thrusts = [], [], [], []
for k in range(1, int(T / dt) + 1):
    t[0] = k * dt
    cf.setState(q.state)
    cf.getSetpoint()
    act = cf.executeController()
    fa = np.array([0.0, 0.0, {f_ext_z}]) if t[0] > 5.0 else np.zeros(3)
    q.step(act, dt, fa)
    if t[0] > 8.0:
        zs.append(q.state.pos[2])
        r, p, _ = rowan.to_euler(q.state.quat, convention="xyz")
        rolls.append(np.degrees(r))
        pitchs.append(np.degrees(p))
        thrusts.append(float(np.mean(act.rpm)))
z = np.array(zs)
print("LABEL", "{label}")
print("Z_MEAN", float(np.mean(z)))
print("Z_STD", float(np.std(z)))
print("Z_ERR_MEAN", float(np.mean(z - {height})))
print("Z_ERR_RMSE", float(np.sqrt(np.mean((z - {height})**2))))
print("ROLL_RMS", float(np.sqrt(np.mean(np.array(rolls)**2))))
print("PITCH_RMS", float(np.sqrt(np.mean(np.array(pitchs)**2))))
print("RPM_MEAN_STD", float(np.std(thrusts)))
'''
    py = shutil.which("python3.10") or sys.executable
    env = os.environ.copy()
    env["PYTHONPATH"] = f"{so_path.parent}:{cs2_sim}"
    out = subprocess.check_output([py, "-c", code], env=env, text=True)
    metrics = {}
    for line in out.strip().splitlines():
        if " " in line:
            k, v = line.split(" ", 1)
            metrics[k] = float(v) if k != "LABEL" else v
    return metrics


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rebuild-on", action="store_true", help="temporarily patch lib.rs, rebuild ON .so")
    ap.add_argument("--on-so", type=Path, default=None)
    args = ap.parse_args()

    off_candidates = sorted(BUILD.glob("_cffirmware.cpython-310*.so"))
    if not off_candidates:
        sys.exit("missing default cffirmware build (python3.10)")
    off_path = off_candidates[-1]

    on_path = args.on_so
    if args.rebuild_on:
        dest = Path("/tmp/cffirmware_posint_on")
        dest.mkdir(exist_ok=True)
        off_backup = dest / off_path.name
        if not off_backup.is_file():
            shutil.copy2(off_path, off_backup)
        built = rebuild_with_integral(True)
        on_path = dest / built.name
        shutil.copy2(built, on_path)
        subprocess.run(["make", "bindings_python"], cwd=FW, check=True)

    if on_path is None or not on_path.is_file():
        sys.exit("provide --on-so or run with --rebuild-on")

    off_so = off_path
    if args.rebuild_on and (Path("/tmp/cffirmware_posint_on") / off_path.name).is_file():
        off_so = Path("/tmp/cffirmware_posint_on") / off_path.name
    off = run_hover(off_so, "integral_off")
    on = run_hover(on_path, "integral_on")
    print("=== position integral SIL hover (oot geometric, z=1.0 m, f_ext after t=5s, stats t>8s) ===")
    for key in ("Z_ERR_MEAN", "Z_ERR_RMSE", "Z_STD", "ROLL_RMS", "PITCH_RMS", "RPM_MEAN_STD"):
        print(f"  {key:12}  off={off[key]:+.5f}  on={on[key]:+.5f}  delta={on[key]-off[key]:+.5f}")


if __name__ == "__main__":
    main()
