#!/usr/bin/env python3
"""Gate 1 of the FBL stock inner loop (ctrlFbl.inner = 1, docs/fbl/HANDOVER_FBL_1TO1_PROMPT_2026-10-24.md WP1).

(a) FBL with inner = 1 (controller_omar_indi.c, fbl.en 1) is driven with smooth random state / sensor / setpoint sequences; its body-rate command
    w_des and its legacy thrust are recorded. (b) The SAME state / sensor sequences go into the stock controllerPid (controller_pid.c) in rate mode:
    mode.x/y/z = modeDisable, mode.roll/pitch/yaw = modeVelocity, attitudeRate = (deg w_x, -deg w_y, deg w_z) as the stock interfaces expect,
    thrust = the same per-motor uint16 value. A thrust-0 tick first resets the stock PIDs and its yaw setpoint (what stock does on the ground).
PASS: roll / pitch / yaw (int16), thrust and the four motor values of powerDistribution are identical in every tick.
The stock attitude / rate PIDs are file-static and shared, so (a) and (b) run one after the other, never interleaved.
Sequences start with gyro = 0 (stock resets on the ground; FBL primes its rate PIDs with the gyro at the in-air switch-on, identical at gyro 0).

  python3 flying_drone_stack/firmware_app/host/test_fbl_inner_stock.py [--build ~/Desktop/crazyflie-firmware/build]   (system python3, 3.10)"""
import argparse, os, sys
import numpy as np

ap = argparse.ArgumentParser(); ap.add_argument("--build", default=os.path.expanduser("~/Desktop/crazyflie-firmware/build")); a = ap.parse_args()
sys.path.insert(0, a.build)
import cffirmware as fw  # noqa: E402

F32 = np.float32
DEG = F32(180.0) / F32(np.pi)            # degrees() of math3d.h: (180.0f / M_PI_F) * rad, in float


def smooth(rng, n, scale, tau=200):
    w = rng.normal(0, 1, n); y = np.zeros(n); a_ = 1.0 / tau
    for k in range(1, n): y[k] = (1 - a_) * y[k - 1] + a_ * w[k] * np.sqrt(tau)
    return scale * y / (np.abs(y).max() + 1e-9)


def make_sequence(seed, n, yaw0, yaw_span):
    rng = np.random.default_rng(seed)
    seq = dict(
        roll=smooth(rng, n, 15.0), pitch=smooth(rng, n, 15.0), yaw=yaw0 + smooth(rng, n, yaw_span),
        gx=smooth(rng, n, 120.0), gy=smooth(rng, n, 120.0), gz=smooth(rng, n, 60.0),
        px=smooth(rng, n, 0.3), py=smooth(rng, n, 0.3), pz=0.5 + smooth(rng, n, 0.1),
        vx=smooth(rng, n, 0.5), vy=smooth(rng, n, 0.5), vz=smooth(rng, n, 0.2),
        sx=smooth(rng, n, 0.3), sy=smooth(rng, n, 0.3), sz=0.5 + smooth(rng, n, 0.05))
    for k in ("gx", "gy", "gz"): seq[k] = seq[k] - seq[k][0]          # gyro 0 at the first tick
    seq["yaw"] = (seq["yaw"] + 180.0) % 360.0 - 180.0
    return seq


def fill_state(st, sens, s, k):
    r, p, y = np.radians([s["roll"][k], s["pitch"][k], s["yaw"][k]])
    cr, sr, cp, sp_, cy, sy = np.cos(r / 2), np.sin(r / 2), np.cos(p / 2), np.sin(p / 2), np.cos(y / 2), np.sin(y / 2)
    st.attitudeQuaternion.w = float(cr * cp * cy + sr * sp_ * sy); st.attitudeQuaternion.x = float(sr * cp * cy - cr * sp_ * sy)
    st.attitudeQuaternion.y = float(cr * sp_ * cy + sr * cp * sy); st.attitudeQuaternion.z = float(cr * cp * sy - sr * sp_ * cy)
    st.attitude.roll, st.attitude.pitch, st.attitude.yaw = float(s["roll"][k]), float(s["pitch"][k]), float(s["yaw"][k])
    st.position.x, st.position.y, st.position.z = float(s["px"][k]), float(s["py"][k]), float(s["pz"][k])
    st.velocity.x, st.velocity.y, st.velocity.z = float(s["vx"][k]), float(s["vy"][k]), float(s["vz"][k])
    sens.gyro.x, sens.gyro.y, sens.gyro.z = float(s["gx"][k]), float(s["gy"][k]), float(s["gz"][k])


def motors(ctl):
    m = fw.motors_thrust_uncapped_t(); fw.powerDistribution(ctl, m); return (m.motors.m1, m.motors.m2, m.motors.m3, m.motors.m4)


def run_fbl(s, n):
    omar = fw.controllerOmarIndi_t(); fw.controllerOmarIndiInit(omar); omar.indi = 0; omar.fbl.en = 1; omar.fbl.inner = 1; omar.fbl.c_d = 0.0
    sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
    sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs; sp.mode.yaw = fw.modeAbs; sp.attitude.yaw = 0.0
    out = []; rng = np.random.default_rng(99)
    for k in range(n):
        if s.get("track") and k > 0:     # gyro follows the previous rate command (deg/s) plus noise: keeps the PID outputs mostly unsaturated
            w = out[-1]["w"]
            s["gx"][k], s["gy"][k], s["gz"][k] = (np.degrees(w[i]) + rng.normal(0, s["track"]) for i in range(3))
        fill_state(st, sens, s, k)
        sp.position.x, sp.position.y, sp.position.z = float(s["sx"][k]), float(s["sy"][k]), float(s["sz"][k])
        fw.controllerOmarIndi(omar, ctl, sp, sens, st, 2 * k)
        assert omar.fbl.active == 1 and omar.fbl.tripped == 0, f"FBL not active at tick {k}"
        assert ctl.controlMode == fw.controlModeLegacy
        out.append(dict(w=(omar.fbl.w_des.x, omar.fbl.w_des.y, omar.fbl.w_des.z), thrust=ctl.thrust, cmd=(ctl.roll, ctl.pitch, ctl.yaw), mot=motors(ctl)))
    return out


def run_stock(s, n, fbl):
    fw.controllerPidInit()
    sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
    sp.mode.x = sp.mode.y = sp.mode.z = fw.modeDisable; sp.mode.roll = sp.mode.pitch = sp.mode.yaw = fw.modeVelocity
    fill_state(st, sens, s, 0); sp.thrust = 0.0; fw.controllerPid(ctl, sp, sens, st, 0)       # ground reset tick (stock: thrust 0)
    out = []
    for k in range(n):
        fill_state(st, sens, s, k)
        w = [F32(x) for x in fbl[k]["w"]]
        sp.attitudeRate.roll = float(DEG * w[0]); sp.attitudeRate.pitch = float(-(DEG * w[1])); sp.attitudeRate.yaw = float(DEG * w[2])
        sp.thrust = fbl[k]["thrust"]
        fw.controllerPid(ctl, sp, sens, st, 2 * (k + 1))
        out.append(dict(thrust=ctl.thrust, cmd=(ctl.roll, ctl.pitch, ctl.yaw), mot=motors(ctl)))
    return out


cases = [("hover yaw 0", 11, 0.0, 20.0, 0), ("yaw across +-180", 12, 175.0, 30.0, 0), ("large rates", 13, -40.0, 60.0, 0),
         ("tracking gyro, noise 2", 14, 10.0, 20.0, 2.0), ("tracking gyro, noise 10", 15, 178.0, 20.0, 10.0)]
N = 3000; ok = True
for name, seed, yaw0, span, track in cases:
    s = make_sequence(seed, N, yaw0, span)
    if track: s["track"] = track; s["px"] = 0.2 * s["px"] + s["sx"]; s["py"] = 0.2 * s["py"] + s["sy"]; s["vx"] *= 0.2; s["vy"] *= 0.2
    a_ = run_fbl(s, N); b = run_stock(s, N, a_)
    dc = max(max(abs(x - y) for x, y in zip(p["cmd"], q["cmd"])) for p, q in zip(a_, b))
    dm = max(max(abs(x - y) for x, y in zip(p["mot"], q["mot"])) for p, q in zip(a_, b))
    dt_ = max(abs(p["thrust"] - q["thrust"]) for p, q in zip(a_, b))
    sat = sum(any(abs(c) >= 32767 for c in p["cmd"]) for p in a_)
    rng_ = [max(abs(p["cmd"][i]) for p in a_) for i in range(3)]
    passed = dc == 0 and dm == 0 and dt_ == 0.0
    ok &= passed
    print(f"{name:20s} {N} ticks: max |d cmd| {dc}, max |d motor| {dm}, max |d thrust| {dt_:.3g}; |cmd| max roll/pitch/yaw {rng_}, saturated ticks {sat}  -> {'PASS' if passed else 'FAIL'}")
print("ALL PASS" if ok else "FAIL"); sys.exit(0 if ok else 1)
