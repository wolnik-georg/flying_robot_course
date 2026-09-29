#!/usr/bin/env python3
"""Numerical verification: omar_indi_rust.rs (controller=10) vs controller_omar_indi.c (c=9)."""
import json
import math
import subprocess
import sys

RUNNER = __file__.replace("test_omar_indi_rust_vs_c.py", "_omar_indi_rust_case_runner.py")

CASES = [
    dict(name="exact hover",
         pos=(0, 0, 0.5), vel=(0, 0, 0), rpy=(0, 0, 0), gyro=(0, 0, 0),
         sp_pos=(0, 0, 0.5), sp_vel=(0, 0, 0), sp_acc=(0, 0, 0), yaw_d=0.0,
         rpm=(24150, 24150, 24150, 24150)),
    dict(name="5cm low, level",
         pos=(0, 0, 0.45), vel=(0, 0, 0), rpy=(0, 0, 0), gyro=(0, 0, 0),
         sp_pos=(0, 0, 0.5), sp_vel=(0, 0, 0), sp_acc=(0, 0, 0), yaw_d=0.0,
         rpm=(24150, 24150, 24150, 24150)),
    dict(name="10deg roll error + xy velocity error",
         pos=(0.1, -0.05, 0.5), vel=(0.3, -0.2, 0.05),
         rpy=(math.radians(10), 0, 0), gyro=(5, -3, 1),
         sp_pos=(0, 0, 0.5), sp_vel=(0, 0, 0), sp_acc=(0, 0, 0), yaw_d=0.0,
         rpm=(23000, 25000, 25000, 23000)),
    dict(name="yaw 90deg + asymmetric RPM (yaw torque)",
         pos=(0, 0, 0.5), vel=(0, 0, 0), rpy=(0, 0, math.radians(90)), gyro=(0, 0, 20),
         sp_pos=(0, 0, 0.5), sp_vel=(0, 0, 0), sp_acc=(0, 0, 0), yaw_d=math.radians(90),
         rpm=(24500, 23800, 24500, 23800)),
    dict(name="aggressive: tilt+climb setpoint, all axes moving",
         pos=(0.5, 0.3, 1.0), vel=(0.5, -0.4, 0.3),
         rpy=(math.radians(-8), math.radians(6), math.radians(30)), gyro=(-10, 15, -5),
         sp_pos=(0.2, 0.1, 1.2), sp_vel=(1.0, -0.5, 0.2), sp_acc=(0.5, 0.2, -0.1),
         yaw_d=math.radians(35),
         rpm=(26000, 22000, 27000, 21000)),
    dict(name="asymmetric RPM per motor",
         pos=(0, 0, 0.5), vel=(0, 0, 0), rpy=(0, 0, 0), gyro=(2, -4, 3),
         sp_pos=(0, 0, 0.5), sp_vel=(0, 0, 0), sp_acc=(0, 0, 0), yaw_d=0.0,
         rpm=(20000, 24000, 28000, 22000)),
    dict(name="zero-thrust: modeDisable low thrust cmd",
         pos=(0, 0, 0.5), vel=(0, 0, 0), rpy=(0, 0, 0), gyro=(0, 0, 0),
         sp_pos=(0, 0, 0.5), sp_vel=(0, 0, 0), sp_acc=(0, 0, 0), yaw_d=0.0,
         rpm=(24150, 24150, 24150, 24150),
         mode_disable_z=True, thrust_cmd=500),
]


def run_case(side, so_dir, case):
    out = subprocess.run(
        [sys.executable, RUNNER, side, so_dir, json.dumps(case)],
        capture_output=True, text=True, check=True,
    )
    json_line = next(line for line in out.stdout.splitlines() if line.strip().startswith("{"))
    result = json.loads(json_line)
    return result["thrust"], tuple(result["tau"])


def main():
    if len(sys.argv) != 2:
        print("usage: test_omar_indi_rust_vs_c.py <cffirmware_build_dir>")
        sys.exit(1)
    so_dir = sys.argv[1]

    n_pass = 0
    worst_d = 0.0
    for case in CASES:
        t_c, tau_c = run_case("c", so_dir, case)
        t_r, tau_r = run_case("rust", so_dir, case)
        d_thrust = abs(t_c - t_r)
        d_tau = max(abs(a - b) for a, b in zip(tau_c, tau_r))
        worst_d = max(worst_d, d_thrust, d_tau)
        ok = d_thrust < 1e-4 and d_tau < 1e-5
        n_pass += ok
        status = "PASS" if ok else "FAIL"
        print(f"  {status}  {case['name']:52s} "
              f"thrust c={t_c:+.9f} rust={t_r:+.9f} d={d_thrust:.2e}  "
              f"tau d={d_tau:.2e}")

    print(f"\n{n_pass}/{len(CASES)} cases match (worst delta {worst_d:.2e})")
    sys.exit(0 if n_pass == len(CASES) else 1)


if __name__ == "__main__":
    main()
