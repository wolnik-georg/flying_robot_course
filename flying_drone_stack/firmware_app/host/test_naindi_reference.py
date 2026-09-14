#!/usr/bin/env python3
"""Numerical test vectors: our naindi.rs (controller=7) vs the reference controllerLee()
compiled directly from ~/Desktop/NA-INDI-firmware (Cobo & Briesewitz's own C source).

The reference has no rich test vectors of its own (test_controller_lee.py there is a
trivial all-zero smoke test), so this generates our own: hand-picked non-trivial states run
through both compiled implementations, comparing thrustSi and torque[x,y,z] bit-for-bit
(to float tolerance).

Two separate compiled `cffirmware` extensions are required, each loaded in its OWN
subprocess (both are named `cffirmware` / `_cffirmware...so`, so importing both in one
process would silently reuse the first via sys.modules caching -- see
_naindi_case_runner.py, which each subprocess runs):
  - `cffirmware_ours`  : this project's own bindings (crazyflie-firmware + naindi.rs),
                         built the usual way (`cd crazyflie-firmware && make bindings_python`).
  - `cffirmware_ref`   : the reference's OWN controller_lee.c, compiled standalone from a
                         throwaway copy of ~/Desktop/NA-INDI-firmware (that repo is read-only,
                         never modified in place -- see LOCAL_MODIFICATIONS.md). Build notes
                         for this scratch copy are in naindi_reference_build_notes.md; the
                         .so itself is not checked in (build artifact, and depends on a
                         throwaway copy outside this repo).

Because our port intentionally uses THIS project's own mass/J/kt (see naindi.rs's module
doc -- the deliberate, physically-necessary deviations), the two controllers are NOT
compared using each side's own native defaults. Instead mass/J/kt on BOTH sides are pinned
to the same literal values for the duration of this test (via oot_test_set_kappa_f /
naindi_test_set_j -- both test-only hooks, never used in flight), so the comparison
isolates "did we port the algorithm correctly" from "do the two projects fly different
hardware". Every other constant (gains, filter cutoffs, arm/t2t, rate gate, dt handling) is
the reference's own value already hardcoded into naindi.rs, so no override needed there.
"""
import json
import math
import subprocess
import sys

RUNNER = __file__.replace("test_naindi_reference.py", "_naindi_case_runner.py")

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
        print("usage: test_naindi_reference.py <ref_build_dir> <ours_build_dir>")
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
        print(f"  {status}  {case['name']:42s} "
              f"thrust ref={t_ref:+.6f} ours={t_ours:+.6f} d={d_thrust:.2e}  "
              f"tau ref=({tau_ref[0]:+.6f},{tau_ref[1]:+.6f},{tau_ref[2]:+.6f}) "
              f"ours=({tau_ours[0]:+.6f},{tau_ours[1]:+.6f},{tau_ours[2]:+.6f}) d={d_tau:.2e}")

    print(f"\n{n_pass}/{len(CASES)} cases match")
    sys.exit(0 if n_pass == len(CASES) else 1)


if __name__ == "__main__":
    main()
