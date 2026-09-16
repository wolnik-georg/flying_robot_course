#!/usr/bin/env python3
"""Numerical test vectors: our naindi_hybrid.rs (controller=8, `use_nn=1`) vs the reference
controllerLee() compiled directly from ~/Desktop/NA-INDI-firmware (Cobo & Briesewitz's own C
source), with their `use_nn` bits also enabled.

This is test_naindi_reference.py's exact pattern (controller=7, use_nn=0), extended to also
exercise the real trained network (`nn.c`/`nn_utils.c`) rather than just linking it unused. Two
separate compiled `cffirmware` extensions, each in its OWN subprocess for the same
sys.modules-caching reason as before -- see _naindi_hybrid_case_runner.py.

Building the reference side needs one addition beyond naindi_reference_build_notes.md's
recipe: `bindings/host_stubs.c`'s `motorsGetRatio()` stub must become SETTABLE (the NN's own
input vector reads it directly -- controller_lee.c line ~293), via a new
`oot_test_set_pwm_ratio()` hook mirroring the existing `oot_test_set_rpm()`. See
naindi_reference_build_notes.md's "NA-INDI hybrid (controller=8)" addendum for the exact
diff. mass/J/kt are pinned identically on both sides as before (now via
`naindi_hybrid_test_set_j()`, this file's own hook -- controller=8 does NOT share
naindi.rs's `naindi_test_set_j()`, kept isolated per this project's convention).
"""
import json
import math
import subprocess
import sys

RUNNER = __file__.replace("test_naindi_hybrid_reference.py", "_naindi_hybrid_case_runner.py")

CASES = [
    dict(name="exact hover, NN sees plausible cruise state",
         pos=(0, 0, 0.5), vel=(0, 0, 0), rpy=(0, 0, 0), gyro=(0, 0, 0), gyro_reg=(0, 0, 0),
         sp_pos=(0, 0, 0.5), sp_vel=(0, 0, 0), sp_acc=(0, 0, 0), yaw_d=0.0,
         rpm=(24150, 24150, 24150, 24150), acc=(0, 0, 0), pwm=(32768, 32768, 32768, 32768)),
    dict(name="5cm low, level, zero NN input (matches the earlier smoke test)",
         pos=(0, 0, 0.45), vel=(0, 0, 0), rpy=(0, 0, 0), gyro=(0, 0, 0), gyro_reg=(0, 0, 0),
         sp_pos=(0, 0, 0.5), sp_vel=(0, 0, 0), sp_acc=(0, 0, 0), yaw_d=0.0,
         rpm=(24150, 24150, 24150, 24150), acc=(0, 0, 0), pwm=(0, 0, 0, 0)),
    dict(name="10deg roll error + xy velocity error, nonzero acc/pwm",
         pos=(0.1, -0.05, 0.5), vel=(0.3, -0.2, 0.05),
         rpy=(math.radians(10), 0, 0), gyro=(5, -3, 1), gyro_reg=(5, -3, 1),
         sp_pos=(0, 0, 0.5), sp_vel=(0, 0, 0), sp_acc=(0, 0, 0), yaw_d=0.0,
         rpm=(23000, 25000, 25000, 23000), acc=(0.05, -0.02, 0.01),
         pwm=(28000, 30000, 30000, 28000)),
    dict(name="gyro != gyro_reg -- catches a gyro/gyroNoLpf wiring swap",
         pos=(0, 0, 0.5), vel=(0, 0, 0), rpy=(0, 0, 0), gyro=(40, -25, 10), gyro_reg=(2, 1, -1),
         sp_pos=(0, 0, 0.5), sp_vel=(0, 0, 0), sp_acc=(0, 0, 0), yaw_d=0.0,
         rpm=(24150, 24150, 24150, 24150), acc=(0, 0, 0), pwm=(31000, 31000, 31000, 31000)),
    dict(name="yaw 90deg + asymmetric RPM (yaw torque)",
         pos=(0, 0, 0.5), vel=(0, 0, 0), rpy=(0, 0, math.radians(90)), gyro=(0, 0, 20),
         gyro_reg=(0, 0, 20),
         sp_pos=(0, 0, 0.5), sp_vel=(0, 0, 0), sp_acc=(0, 0, 0), yaw_d=math.radians(90),
         rpm=(24500, 23800, 24500, 23800), acc=(0, 0, 0.02), pwm=(29500, 27800, 29500, 27800)),
    dict(name="aggressive: tilt+climb setpoint, all axes moving, all inputs nonzero",
         pos=(0.5, 0.3, 1.0), vel=(0.5, -0.4, 0.3),
         rpy=(math.radians(-8), math.radians(6), math.radians(30)), gyro=(-10, 15, -5),
         gyro_reg=(-12, 18, -6),
         sp_pos=(0.2, 0.1, 1.2), sp_vel=(1.0, -0.5, 0.2), sp_acc=(0.5, 0.2, -0.1),
         yaw_d=math.radians(35),
         rpm=(26000, 22000, 27000, 21000), acc=(0.12, -0.08, 0.15),
         pwm=(33000, 25000, 36000, 22000)),
]

REFERENCE_GAINS = dict(
    mass=0.034,
    J=(16.571710e-6, 16.655602e-6, 29.261652e-6),
    Kpos_P=(12.0, 12.0, 12.0), Kpos_P_limit=100.0,
    Kpos_D=(10.5, 10.5, 10.5), Kpos_D_limit=100.0,
    Kpos_I=(2.0, 2.0, 2.0), Kpos_I_limit=100.0,
    KR=(0.007, 0.007, 0.01),
    Komega=(0.002, 0.002, 0.002),
    KI=(0.01, 0.01, 0.01),
    kt=(1.5e-10, 1.5e-10, 1.5e-10, 1.5e-10),
)


def run_case(side, so_dir, case):
    out = subprocess.run(
        [sys.executable, RUNNER, side, so_dir, json.dumps(case), json.dumps(REFERENCE_GAINS)],
        capture_output=True, text=True, check=True,
    )
    json_line = next(line for line in out.stdout.splitlines() if line.strip().startswith("{"))
    result = json.loads(json_line)
    return result["thrust"], tuple(result["tau"])


def main():
    if len(sys.argv) != 3:
        print("usage: test_naindi_hybrid_reference.py <ref_build_dir> <ours_build_dir>")
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
        print(f"  {status}  {case['name']:55s} "
              f"thrust ref={t_ref:+.6f} ours={t_ours:+.6f} d={d_thrust:.2e}  "
              f"tau ref=({tau_ref[0]:+.6f},{tau_ref[1]:+.6f},{tau_ref[2]:+.6f}) "
              f"ours=({tau_ours[0]:+.6f},{tau_ours[1]:+.6f},{tau_ours[2]:+.6f}) d={d_tau:.2e}")

    print(f"\n{n_pass}/{len(CASES)} cases match")
    sys.exit(0 if n_pass == len(CASES) else 1)


if __name__ == "__main__":
    main()
