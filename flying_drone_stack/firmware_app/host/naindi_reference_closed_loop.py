#!/usr/bin/env python3
"""Closed-loop test of Briesewitz's OWN compiled controller_lee.c (not naindi.rs, our Rust
port of it) -- flown against the SAME rigid-body plant physics (crazyswarm2's np.py
Quadrotor) that controller=7's crashing SIL hover uses, through the same climb-then-hover
shape, to separate "port bug" from "their algorithm is genuinely unstable here".

Reuses the exact scratch-build recipe and host stubs documented in
naindi_reference_build_notes.md (same one test_naindi_reference.py's static single-tick
vectors already validate to ~1e-9) -- this script is the closed-loop extension of that same
harness, not a new implementation. New host stub needed beyond the static-test recipe:
pmGetBatteryVoltage (missing symbol at link time, not called by controller_lee.c itself but
needed elsewhere in the link chain).

Supports BOTH sides through the identical tick loop / plant / trajectory, so a run with
--which=reference and one with --which=ours differ ONLY in which compiled controller is
called -- the single rigorous way to separate "port bug" from "algorithm problem" (a
standalone reference-only run leaves the trajectory-shape difference from the CS2 SIL test
as an uncontrolled variable; this removes it).

Airframe: mass, kt (their own measured kappa_f from NA-INDI/pwm2thrust.py, not this
project's), arm, t2t, J, and every gain -- ALL from the reference authors' own values,
fully self-consistent, matching the same "run it exactly as they built it" config that flew
controller=7 clean through hover (state_naindi/2026-09-17_203435) before crashing later.

Usage:
    python3 naindi_reference_closed_loop.py --which reference [--so-dir DIR] [--out CSV] [--duration S]
    python3 naindi_reference_closed_loop.py --which ours      [--so-dir DIR] [--out CSV] [--duration S]

    --which     reference (controllerLee, scratch NA-INDI-firmware build) or
                ours (controllerOutOfTree2, this project's own cffirmware build)
    --so-dir    default /tmp/na_indi_build/build for reference,
                ~/Desktop/crazyflie-firmware/build for ours
    --out       default /tmp/briesewitz_closed_loop_<which>.csv
    --duration  default 20.0 seconds

Output CSV matches crazyswarm2/state_*/csv/*.csv's own format (timestamp,x,y,z,qw,qx,qy,qz)
so the same analysis snippets used throughout docs/07's controller=7 investigation apply
unchanged.
"""
import argparse
import csv
import sys

import numpy as np
import rowan

# Reference authors' own airframe -- controller_lee.c's own .mass=0.034 literal, their own J
# (same literal), their own measured per-motor kt (NA-INDI/pwm2thrust.py's kappa_f, NOT this
# project's kt1-4 -- see docs/07, 2026-09-17), and the standard CF2.1 arm/t2t their firmware
# was built for. Nothing here is this project's own number.
MASS = 0.034
J = (16.571710e-6, 16.655602e-6, 29.261652e-6)
KT = (2.139974655714972e-10, 2.3783777845095615e-10, 1.9693330742680727e-10,
      2.559402652634741e-10)
ARM_LENGTH = 0.046
T2T = 0.006

# controller_lee.c's own default gains (g_self), unchanged -- same values naindi.rs ports.
KPOS_P, KPOS_D, KPOS_I = 12.0, 10.5, 2.0
KR_XY, KR_Z, KOMEGA, KI_ATT = 0.007, 0.01, 0.002, 0.01

TARGET_Z = 1.0
CLIMB_T = 2.0
HOVER_T = 8.0   # matches --duration 8 in the CS2 SIL tests that actually crashed
LAND_T = 2.0    # every crash observed to date happened during hover or landing, never climb
DT = 0.002  # 500 Hz -- matches usecTimestamp()'s fixed +2000us/call stub and ATTITUDE_RATE


def quintic(tau):
    """s, ds/dtau, d2s/dtau2 for the minimum-jerk profile, tau in [0,1]."""
    s = 6 * tau**5 - 15 * tau**4 + 10 * tau**3
    ds = 30 * tau**4 - 60 * tau**3 + 30 * tau**2
    dds = 120 * tau**3 - 180 * tau**2 + 60 * tau
    return s, ds, dds


def setpoint_zvaz(t, feedforward):
    """Climb -> hold -> land, matching the CS2 SIL tests' own --duration 8 --height 1.0
    (docs/07, 2026-09-17/18) -- the one phase (landing) no earlier version of this script
    exercised, despite every real crash observed happening during hover or landing, never
    climb."""
    land_start = CLIMB_T + HOVER_T
    if t < CLIMB_T:
        tau = t / CLIMB_T
        s, ds, dds = quintic(tau) if feedforward else (tau, 0.0, 0.0)
        return TARGET_Z * s, TARGET_Z * ds / CLIMB_T, TARGET_Z * dds / CLIMB_T**2
    elif t < land_start:
        return TARGET_Z, 0.0, 0.0
    elif t < land_start + LAND_T:
        tau = (t - land_start) / LAND_T
        s, ds, dds = quintic(tau) if feedforward else (tau, 0.0, 0.0)
        z = TARGET_Z * (1.0 - s)
        return z, -TARGET_Z * ds / LAND_T, -TARGET_Z * dds / LAND_T**2
    else:
        return 0.0, 0.0, 0.0


def setup_reference(fw):
    ctrl = fw.controllerLee_t()
    fw.controllerLeeInit(ctrl)
    ctrl.mass = MASS
    ctrl.J = fw.mkvec(*J)
    ctrl.Kpos_P = fw.mkvec(KPOS_P, KPOS_P, KPOS_P); ctrl.Kpos_P_limit = 100.0
    ctrl.Kpos_D = fw.mkvec(KPOS_D, KPOS_D, KPOS_D); ctrl.Kpos_D_limit = 100.0
    ctrl.Kpos_I = fw.mkvec(KPOS_I, KPOS_I, KPOS_I); ctrl.Kpos_I_limit = 100.0
    ctrl.KR = fw.mkvec(KR_XY, KR_XY, KR_Z)
    ctrl.Komega = fw.mkvec(KOMEGA, KOMEGA, KOMEGA)
    ctrl.KI = fw.mkvec(KI_ATT, KI_ATT, KI_ATT)
    ctrl.indi = 3       # position + attitude INDI, their own standard flown config
    ctrl.use_nn = 0      # dead code in every flown config either project has found
    fw.oot_test_set_kappa_f(*KT)

    def step(sp, sensors, st, tick, rpm_meas):
        # controllerLee() gates on RATE_DO_EXECUTE(ATTITUDE_RATE=500, tick) =
        # (tick % 2 == 0) at these rates (stabilizer_types.h) -- tick must be even on every
        # real call or it silently no-ops (control_t unchanged), halving the effective rate.
        fw.oot_test_set_rpm(int(rpm_meas[0]), int(rpm_meas[1]), int(rpm_meas[2]),
                            int(rpm_meas[3]))
        control = fw.control_t()
        fw.controllerLee(ctrl, control, sp, sensors, st, tick)
        return control
    return step


def setup_ours(fw):
    fw.controllerOutOfTree2Init()
    fw.cvar.g_indi_mass = MASS
    fw.cvar.g_indi_kt1, fw.cvar.g_indi_kt2, fw.cvar.g_indi_kt3, fw.cvar.g_indi_kt4 = KT
    fw.naindi_test_set_j(*J)
    fw.naindi_test_set_arm(0.707106781 * ARM_LENGTH, T2T)

    def step(sp, sensors, st, tick, rpm_meas):
        # naindi.rs's own module doc: the RATE_DO_EXECUTE(ATTITUDE_RATE, tick) gate is
        # "copied as literally as the port allows" -- same tick%2==0 requirement.
        fw.oot_set_rpm(int(rpm_meas[0]), int(rpm_meas[1]), int(rpm_meas[2]), int(rpm_meas[3]))
        control = fw.control_t()
        fw.controllerOutOfTree2(control, sp, sensors, st, tick)
        return control
    return step


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--which", choices=["reference", "ours"], required=True)
    ap.add_argument("--so-dir", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--feedforward", action="store_true",
                    help="smooth minimum-jerk climb with real velocity/acceleration "
                         "feedforward, instead of a bare position ramp with sp.velocity/"
                         "sp.acceleration held at 0 -- matching what an HLC uploadTrajectory "
                         "polynomial actually sends, unlike the position-only ramp above")
    args = ap.parse_args()

    so_dir = args.so_dir or (
        "/tmp/na_indi_build/build" if args.which == "reference"
        else "/home/georg/Desktop/crazyflie-firmware/build")
    out_csv = args.out or f"/tmp/briesewitz_closed_loop_{args.which}.csv"

    sys.path.insert(0, so_dir)
    import cffirmware as fw  # noqa: E402

    sys.path.insert(0, "/home/georg/Desktop/crazyswarm2/crazyflie_sim")
    from crazyflie_sim.backend.np import Quadrotor  # noqa: E402
    from crazyflie_sim.sim_data_types import Action, State  # noqa: E402

    step_controller = (setup_reference if args.which == "reference" else setup_ours)(fw)

    plant = Quadrotor(State(pos=np.array([0.0, 0.0, 0.0])), params={
        'mass': MASS, 'inertia': list(J), 'kt': list(KT),
        'arm_length': ARM_LENGTH, 't2t': T2T,
    })
    B0_inv = np.linalg.inv(plant.B0)

    n_ticks = int(args.duration / DT)
    rpm_meas = np.zeros(4)
    rows = []
    crashed_at = None

    for loop_i in range(n_ticks):
        tick = 2 * loop_i  # see setup_reference's comment on RATE_DO_EXECUTE
        t = loop_i * DT
        z_sp, vz_sp, az_sp = setpoint_zvaz(t, args.feedforward)

        sp = fw.setpoint_t()
        sp.position.x, sp.position.y, sp.position.z = 0.0, 0.0, z_sp
        sp.velocity.x = sp.velocity.y = 0.0; sp.velocity.z = vz_sp
        sp.acceleration.x = sp.acceleration.y = 0.0; sp.acceleration.z = az_sp
        sp.attitude.yaw = 0.0
        sp.mode.x = fw.modeAbs; sp.mode.y = fw.modeAbs; sp.mode.z = fw.modeAbs
        sp.mode.yaw = fw.modeAbs

        gyro_deg = np.degrees(plant.state.omega)
        sensors = fw.sensorData_t()
        sensors.gyro.x, sensors.gyro.y, sensors.gyro.z = gyro_deg
        sensors.gyroNoLpf.x, sensors.gyroNoLpf.y, sensors.gyroNoLpf.z = gyro_deg

        st = fw.state_t()
        st.position.x, st.position.y, st.position.z = plant.state.pos
        st.velocity.x, st.velocity.y, st.velocity.z = plant.state.vel
        qw, qx, qy, qz = plant.state.quat
        st.attitudeQuaternion.w = qw
        st.attitudeQuaternion.x = qx
        st.attitudeQuaternion.y = qy
        st.attitudeQuaternion.z = qz
        # state->acc: world-frame, gravity-EXCLUDED, in g's (controller_lee.c line 329:
        # a_imu = 9.81 * state->acc). Quadrotor's own state.acc is the body-frame equivalent
        # -- rotate it into world frame to match what state_t.acc means here.
        acc_world_g = rowan.rotate(plant.state.quat, plant.state.acc)
        st.acc.x, st.acc.y, st.acc.z = acc_world_g

        control = step_controller(sp, sensors, st, tick, rpm_meas)

        rhs = np.array([control.thrustSi, control.torqueX, control.torqueY, control.torqueZ])
        force = np.maximum(B0_inv @ rhs, 0.0)
        rpm_cmd = np.sqrt(force / np.array(KT))

        plant.step(Action(rpm_cmd), DT)
        rpm_meas = plant.state.rpm  # post-lag "measured", feeds the NEXT tick

        qw, qx, qy, qz = plant.state.quat
        rows.append([t, *plant.state.pos, qw, qx, qy, qz])

        if plant.state.pos[2] < -0.05 or not np.all(np.isfinite(plant.state.pos)):
            crashed_at = t
            print(f"CRASHED at t={t:.3f}s (z={plant.state.pos[2]:.3f})")
            break

    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "x", "y", "z", "qw", "qx", "qy", "qz"])
        w.writerows(rows)

    print(f"[{args.which}] wrote {len(rows)} rows ({rows[-1][0]:.3f}s simulated) to {out_csv}")
    if crashed_at is None:
        print(f"[{args.which}] No crash detected through the full {args.duration:.1f}s.")


if __name__ == "__main__":
    main()
