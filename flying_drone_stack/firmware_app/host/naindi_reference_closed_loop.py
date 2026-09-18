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
DT_PHYS = 0.0005  # 2 kHz -- crazyflie_sil.py's actual physics substep rate; see --real-substeps


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
    ap.add_argument("--mass", type=float, default=None,
                    help="override MASS (kg) -- e.g. to match the airframe a replayed log "
                         "was actually captured under, if different from the reference's own")
    ap.add_argument("--kt", type=float, nargs=4, default=None, metavar=("K1", "K2", "K3", "K4"))
    ap.add_argument("--arm-length", type=float, default=None)
    ap.add_argument("--t2t", type=float, default=None)
    ap.add_argument("--j", type=float, nargs=3, default=None, metavar=("JXX", "JYY", "JZZ"))
    ap.add_argument("--zero-state-acc", action="store_true",
                    help="replicate a real, CONFIRMED bug in crazyflie_sil.py's setState(): "
                         "it only ever writes self.sensors.acc, never self.state.acc, which "
                         "naindi.rs actually reads (state->acc, not sensors->acc -- see the "
                         "module doc and line ~389). state.acc stays zero-initialized for "
                         "the entire real SIL flight. Off by default, this script computes "
                         "the CORRECT world-frame value -- pass this to instead feed a "
                         "permanent zero, exactly matching the real (buggy) SIL behavior.")
    ap.add_argument("--motor-tau", type=float, default=None,
                    help="first-order rotor-speed lag [s]. crazyflie_server.py's own "
                         "_setup_oot builds sim['physics'] with motor_tau=0.044 by default "
                         "(the measured brushless rotor time constant) whenever no explicit "
                         "sim.physics block overrides it -- server_sim_naindi.yaml has none, "
                         "so the real SIL run this log came from had this active. This "
                         "script's plant defaults to None (instant, lag-free thrust), a real, "
                         "previously-missed difference for an RPM-feedback controller.")
    ap.add_argument("--real-substeps", action="store_true",
                    help="replicate crazyflie_sil.py's actual discretization: physics at "
                         "2 kHz (dt=0.0005), the controller only recomputes once every 4 "
                         "physics substeps (matching the observed real tick spacing of 2ms "
                         "between genuine computes), holding the same Action across the "
                         "other 3 -- instead of this script's own default direct 1:1 "
                         "500 Hz stepping. The one remaining untested hypothesis after "
                         "gains, airframe, and even the exact real setpoint sequence "
                         "(--replay-log) all failed to reproduce the CS2 SIL crash.")
    ap.add_argument("--replay-log", default=None,
                    help="path to a NAINDI_DEBUG_LOG CSV (crazyflie_sil.py, "
                         "'real_compute'==1 rows) -- replays the REAL logged sp_x/y/z/"
                         "vx/vy/vz/ax/ay/az sequence (linearly interpolated by tick/1000s) "
                         "instead of this script's own hand-picked trajectory. Ignores "
                         "--feedforward. The direct test of 'is the real trajectory shape "
                         "the missing ingredient', per the Investigation plan.")
    args = ap.parse_args()

    global MASS, KT, ARM_LENGTH, T2T, J
    if args.mass is not None:
        MASS = args.mass
    if args.kt is not None:
        KT = tuple(args.kt)
    if args.arm_length is not None:
        ARM_LENGTH = args.arm_length
    if args.t2t is not None:
        T2T = args.t2t
    if args.j is not None:
        J = tuple(args.j)

    replay = None
    if args.replay_log:
        import csv as _csv
        with open(args.replay_log) as f:
            log_rows = [r for r in _csv.DictReader(f) if r["real_compute"] == "1"]
        replay = {
            "t": np.array([int(r["tick"]) / 1000.0 for r in log_rows]),
            "x": np.array([float(r["sp_x"]) for r in log_rows]),
            "y": np.array([float(r["sp_y"]) for r in log_rows]),
            "z": np.array([float(r["sp_z"]) for r in log_rows]),
            "vx": np.array([float(r["sp_vx"]) for r in log_rows]),
            "vy": np.array([float(r["sp_vy"]) for r in log_rows]),
            "vz": np.array([float(r["sp_vz"]) for r in log_rows]),
            "ax": np.array([float(r["sp_ax"]) for r in log_rows]),
            "ay": np.array([float(r["sp_ay"]) for r in log_rows]),
            "az": np.array([float(r["sp_az"]) for r in log_rows]),
        }
        replay["t"] -= replay["t"][0]  # re-zero to this run's own t=0
        print(f"--replay-log: {len(log_rows)} real-compute rows, "
              f"{replay['t'][-1]:.3f}s of logged setpoint")

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

    x0 = replay["x"][0] if replay is not None else 0.0
    y0 = replay["y"][0] if replay is not None else 0.0
    plant = Quadrotor(State(pos=np.array([x0, y0, 0.0])), params={
        'mass': MASS, 'inertia': list(J), 'kt': list(KT),
        'arm_length': ARM_LENGTH, 't2t': T2T,
        'motor_tau': args.motor_tau,
    })
    B0_inv = np.linalg.inv(plant.B0)

    duration = replay["t"][-1] if replay is not None else args.duration
    n_ticks = int(duration / DT)
    rpm_meas = np.zeros(4)
    rows = []
    crashed_at = None

    for loop_i in range(n_ticks):
        tick = 2 * loop_i  # see setup_reference's comment on RATE_DO_EXECUTE
        t = loop_i * DT

        if replay is not None:
            x_sp = np.interp(t, replay["t"], replay["x"])
            y_sp = np.interp(t, replay["t"], replay["y"])
            z_sp = np.interp(t, replay["t"], replay["z"])
            vx_sp = np.interp(t, replay["t"], replay["vx"])
            vy_sp = np.interp(t, replay["t"], replay["vy"])
            vz_sp = np.interp(t, replay["t"], replay["vz"])
            ax_sp = np.interp(t, replay["t"], replay["ax"])
            ay_sp = np.interp(t, replay["t"], replay["ay"])
            az_sp = np.interp(t, replay["t"], replay["az"])
        else:
            x_sp = y_sp = vx_sp = vy_sp = ax_sp = ay_sp = 0.0
            z_sp, vz_sp, az_sp = setpoint_zvaz(t, args.feedforward)

        sp = fw.setpoint_t()
        sp.position.x, sp.position.y, sp.position.z = x_sp, y_sp, z_sp
        sp.velocity.x, sp.velocity.y, sp.velocity.z = vx_sp, vy_sp, vz_sp
        sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = ax_sp, ay_sp, az_sp
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
        if args.zero_state_acc:
            st.acc.x = st.acc.y = st.acc.z = 0.0
        else:
            # state->acc: world-frame, gravity-EXCLUDED, in g's (controller_lee.c line 329:
            # a_imu = 9.81 * state->acc, compared against a_rpm which has gravity already
            # subtracted -- reads (0,0,0) at hover). Quadrotor's own state.acc is body-frame
            # SPECIFIC FORCE (reads (0,0,1) at hover, the thrust/gravity-reaction
            # convention) -- rotate to world frame, then subtract the gravity-cancelling
            # (0,0,1) hover offset to get what state->acc actually means. An earlier version
            # of this script (and, identically, crazyflie_sil.py before the 2026-09-18 fix)
            # omitted the "- 1.0" -- a constant z-offset that happened not to visibly
            # destabilize a clean run, unlike --zero-state-acc's dynamically-varying error.
            acc_world_g = rowan.rotate(plant.state.quat, plant.state.acc)
            st.acc.x, st.acc.y, st.acc.z = acc_world_g[0], acc_world_g[1], acc_world_g[2] - 1.0

        try:
            control = step_controller(sp, sensors, st, tick, rpm_meas)
        except OverflowError:
            crashed_at = t
            print(f"DIVERGED at t={t:.3f}s (loop_i={loop_i}) -- rpm_meas overflowed "
                  f"uint16 feeding the controller: rpm_meas={rpm_meas}")
            break

        rhs = np.array([control.thrustSi, control.torqueX, control.torqueY, control.torqueZ])
        force = np.maximum(B0_inv @ rhs, 0.0)
        with np.errstate(invalid="ignore"):
            rpm_cmd = np.sqrt(force / np.array(KT))
        if not np.all(np.isfinite(rpm_cmd)) or np.any(rpm_cmd > 1e6):
            crashed_at = t
            print(f"DIVERGED at t={t:.3f}s -- non-physical rpm_cmd={rpm_cmd} "
                  f"(control: thrust={control.thrustSi:.4g}, "
                  f"tau=({control.torqueX:.4g},{control.torqueY:.4g},{control.torqueZ:.4g}))")
            break

        if args.real_substeps:
            # crazyflie_sil.py's actual pattern: physics at 2 kHz, a controller call
            # attempted every other substep (~1 kHz), RATE_DO_EXECUTE gating that down
            # again to 500 Hz real computes -- 4 physics substeps of dt=0.0005 elapse,
            # holding this SAME rpm_cmd, between one real compute and the next.
            for _ in range(4):
                plant.step(Action(rpm_cmd), DT_PHYS)
        else:
            plant.step(Action(rpm_cmd), DT)
        rpm_meas = plant.state.rpm  # post-lag "measured", feeds the NEXT tick

        qw, qx, qy, qz = plant.state.quat
        rows.append([t, *plant.state.pos, qw, qx, qy, qz])

        if not np.all(np.isfinite(plant.state.pos)) or not np.all(np.isfinite(rpm_meas)) \
                or np.any(np.abs(rpm_meas) > 1e6):
            crashed_at = t
            print(f"DIVERGED at t={t:.3f}s -- plant state or rpm_meas non-physical "
                  f"(pos={plant.state.pos}, rpm_meas={rpm_meas})")
            break

        if plant.state.pos[2] < -0.05 or not np.all(np.isfinite(plant.state.pos)):
            crashed_at = t
            print(f"CRASHED at t={t:.3f}s (z={plant.state.pos[2]:.3f})")
            break

    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "x", "y", "z", "qw", "qx", "qy", "qz"])
        w.writerows(rows)

    last_t = rows[-1][0] if rows else 0.0
    print(f"[{args.which}] wrote {len(rows)} rows ({last_t:.3f}s simulated) to {out_csv}")
    if crashed_at is None:
        print(f"[{args.which}] No crash detected through the full {args.duration:.1f}s.")


if __name__ == "__main__":
    main()
