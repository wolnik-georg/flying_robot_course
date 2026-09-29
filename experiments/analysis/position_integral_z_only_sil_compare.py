#!/usr/bin/env python3
"""SIL: geometric (oot mode 0) with Z-only integral OFF vs ON (ENABLE_Z_INTEGRAL).

Mirrors position_integral_sil_compare.py — host cargo + bindings_python, SHA256 check.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FW_APP = ROOT / "flying_drone_stack/firmware_app"
LIB_RS = FW_APP / "src/lib.rs"
FW = Path("/home/georg/Desktop/crazyflie-firmware")
BUILD = FW / "build"
ARTIFACT_DIR = Path("/tmp/cffirmware_zint_on")
DEFAULT_JSON = ROOT / "experiments/analysis/out/position_integral_z_only_sil_2026-09-29.json"

DISTURBANCE_SUITE_N = [0.0, -0.008, -0.040, -0.120, -0.200]


def host_env() -> dict[str, str]:
    env = os.environ.copy()
    env.setdefault("DRONE_PLATFORM", "bl")
    env["RUSTFLAGS"] = "-C panic=abort"
    env.setdefault("CRAZYFLIE_BASE", str(FW))
    return env


def patch_z_integral(enabled: bool) -> tuple[str, str]:
    text = LIB_RS.read_text()
    new_val = "true" if enabled else "false"
    patched, n = re.subn(
        r"const ENABLE_Z_INTEGRAL: bool = (true|false);",
        f"const ENABLE_Z_INTEGRAL: bool = {new_val};",
        text,
        count=1,
    )
    if n != 1:
        raise SystemExit("ENABLE_Z_INTEGRAL not found in lib.rs")
    return text, patched


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def rebuild_with_z_integral(enabled: bool) -> Path:
    original, patched = patch_z_integral(enabled)
    LIB_RS.write_text(patched)
    try:
        subprocess.run(
            ["cargo", "build", "--release", "--target", "x86_64-unknown-linux-gnu"],
            cwd=FW_APP,
            check=True,
            env=host_env(),
        )
        subprocess.run(["make", "bindings_python"], cwd=FW, check=True, env=host_env())
    finally:
        LIB_RS.write_text(original)

    so_candidates = sorted(BUILD.glob("_cffirmware.cpython-310*.so"))
    if not so_candidates:
        raise SystemExit("bindings build produced no python3.10 .so")
    return so_candidates[-1]


def run_hover(so_path: Path, label: str, duration: float = 14.0, height: float = 1.0, f_ext_z: float = -0.008) -> dict:
    cs2_sim = Path("/home/georg/Desktop/crazyswarm2/crazyflie_sim")
    use_force = "True" if f_ext_z != 0.0 else "False"
    fval = float(f_ext_z)
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
c.g_ki_z, c.g_ki_z_limit = 8.0, 1.5
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
USE_F = {use_force}
F = {fval}
zs, rolls, pitchs = [], [], []
roll_all, pitch_all = [], []
for k in range(1, int(T / dt) + 1):
    t[0] = k * dt
    cf.setState(q.state)
    cf.getSetpoint()
    act = cf.executeController()
    fa = np.array([0.0, 0.0, F]) if (USE_F and t[0] > 5.0) else np.zeros(3)
    q.step(act, dt, fa)
    r, p, _ = rowan.to_euler(q.state.quat, convention="xyz")
    roll_all.append(abs(np.degrees(r)))
    pitch_all.append(abs(np.degrees(p)))
    if t[0] > 8.0:
        zs.append(q.state.pos[2])
        rolls.append(np.degrees(r))
        pitchs.append(np.degrees(p))
z = np.array(zs)
print("LABEL", "{label}")
print("Z_ERR_MEAN", float(np.mean(z - {height})))
print("Z_ERR_RMSE", float(np.sqrt(np.mean((z - {height})**2))))
print("ROLL_MAX_ALL", float(np.max(roll_all)))
print("PITCH_MAX_ALL", float(np.max(pitch_all)))
'''
    py = "/usr/bin/python3.10" if Path("/usr/bin/python3.10").is_file() else sys.executable
    env = os.environ.copy()
    env["PYTHONPATH"] = f"{so_path.parent}:{cs2_sim}"
    out = subprocess.check_output([py, "-c", code], env=env, text=True)
    metrics: dict = {}
    for line in out.strip().splitlines():
        if " " not in line:
            continue
        k, v = line.split(" ", 1)
        metrics[k] = float(v) if k != "LABEL" else v
    return metrics


def full_suite(forces: list[float], json_path: Path) -> dict:
    ARTIFACT_DIR.mkdir(exist_ok=True)
    off_dir = ARTIFACT_DIR / "off"
    on_dir = ARTIFACT_DIR / "on"

    print("Building Z integral OFF (host cargo + bindings)...")
    off_so = rebuild_with_z_integral(False)
    off_dir.mkdir(parents=True, exist_ok=True)
    on_dir.mkdir(parents=True, exist_ok=True)
    off_copy = off_dir / off_so.name
    shutil.copy2(off_so, off_copy)
    shutil.copy2(BUILD / "cffirmware.py", off_dir / "cffirmware.py")

    print("Building Z integral ON...")
    on_so = rebuild_with_z_integral(True)
    on_copy = on_dir / on_so.name
    shutil.copy2(on_so, on_copy)
    shutil.copy2(BUILD / "cffirmware.py", on_dir / "cffirmware.py")

    off_hash = sha256_file(off_copy)
    on_hash = sha256_file(on_copy)
    if off_hash == on_hash:
        raise SystemExit(f"OFF and ON .so identical — stale build? {off_hash}")

    results = {
        "build": {"off_sha256": off_hash, "on_sha256": on_hash, "distinct": True},
        "runs": [],
    }
    for f in forces:
        row = {
            "f_ext_z_N": f,
            "z_integral_off": run_hover(off_copy, "off", f_ext_z=f),
            "z_integral_on": run_hover(on_copy, "on", f_ext_z=f),
        }
        d = row["z_integral_on"]["Z_ERR_MEAN"] - row["z_integral_off"]["Z_ERR_MEAN"]
        row["delta_z_err_mean_m"] = d
        results["runs"].append(row)
        print(
            f"  f_ext={f:+.3f} N  dZ_mean={d*1000:+.2f} mm  "
            f"off={row['z_integral_off']['Z_ERR_MEAN']*1000:+.2f} mm  "
            f"on={row['z_integral_on']['Z_ERR_MEAN']*1000:+.2f} mm"
        )

    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(results, indent=2))
    print(f"wrote {json_path}")
    return results


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--full-suite", action="store_true")
    ap.add_argument("--json", type=Path, default=DEFAULT_JSON)
    args = ap.parse_args()
    if not args.full_suite:
        ap.print_help()
        return 1
    full_suite(DISTURBANCE_SUITE_N, args.json)
    if "const ENABLE_Z_INTEGRAL: bool = false;" not in LIB_RS.read_text():
        raise SystemExit("lib.rs not restored to ENABLE_Z_INTEGRAL=false")
    print("lib.rs verified: ENABLE_Z_INTEGRAL=false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
