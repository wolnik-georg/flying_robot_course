#!/usr/bin/env python3
"""Numerical test vectors: controller_omar_indi.c (stabilizer.controller=9, this project's
literal C copy of the supervisor's controller_lee.c) vs the reference controllerLee() compiled
directly from ~/Desktop/crazyflie-firmware-omar (his own, completely unmodified source).

Unlike the Briesewitz comparison (test_naindi_reference.py), no gain/mass/kt pinning is needed:
controller_omar_indi.c is a byte-for-byte copy of his file (verified by diff after normalizing
renamed identifiers -- see docs/41_Pure_INDI_Implementation_Comparison.md), and both sides are
built for the SAME platform (CONFIG_PLATFORM_CF21BL, no CONFIG_MODIFIED_CF_MASS override), so
mass/J/arm/t2t/MOTORRPM2FORCE all come from the identical platform_defaults_cf21bl.h logic on
both sides automatically. This test isolates "did the integration (renaming, header backports,
Kconfig wiring) introduce any behavior change" -- not "is the port correct", which is close to
tautological here since it IS his source, just renamed.

See host/naindi_reference_build_notes.md for the build-recipe precedent this mirrors, and
host/README (or LOCAL_MODIFICATIONS.md) for how to rebuild the scratch reference .so.
"""
import json
import math
import subprocess
import sys

RUNNER = __file__.replace("test_omar_indi_reference.py", "_omar_indi_case_runner.py")

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
    dict(name="asymmetric RPM per motor (catches per-motor arm/t2t mixing errors)",
         pos=(0, 0, 0.5), vel=(0, 0, 0), rpy=(0, 0, 0), gyro=(2, -4, 3),
         sp_pos=(0, 0, 0.5), sp_vel=(0, 0, 0), sp_acc=(0, 0, 0), yaw_d=0.0,
         rpm=(20000, 24000, 28000, 22000)),
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
    if len(sys.argv) != 3:
        print("usage: test_omar_indi_reference.py <ref_build_dir> <ours_build_dir>")
        sys.exit(1)
    ref_dir, ours_dir = sys.argv[1], sys.argv[2]

    n_pass = 0
    for case in CASES:
        t_ref, tau_ref = run_case("reference", ref_dir, case)
        t_ours, tau_ours = run_case("ours", ours_dir, case)

        d_thrust = abs(t_ref - t_ours)
        d_tau = max(abs(a - b) for a, b in zip(tau_ref, tau_ours))
        ok = d_thrust < 1e-4 and d_tau < 1e-5
        n_pass += ok
        status = "PASS" if ok else "FAIL"
        print(f"  {status}  {case['name']:52s} "
              f"thrust ref={t_ref:+.6f} ours={t_ours:+.6f} d={d_thrust:.2e}  "
              f"tau ref=({tau_ref[0]:+.6f},{tau_ref[1]:+.6f},{tau_ref[2]:+.6f}) "
              f"ours=({tau_ours[0]:+.6f},{tau_ours[1]:+.6f},{tau_ours[2]:+.6f}) d={d_tau:.2e}")

    print(f"\n{n_pass}/{len(CASES)} cases match")
    sys.exit(0 if n_pass == len(CASES) else 1)


if __name__ == "__main__":
    main()
