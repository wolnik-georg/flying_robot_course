#!/usr/bin/env python3
"""controller=11 (tk_indi.rs, f32) vs experiments/analysis/tk_indi_reference.py (f64).

Drives both with the same smooth time-varying inputs for N ticks (filters, finite differences,
flatness feed-forward all exercised) and compares thrust and torque at every tick.
Use system python3 with PYTHONPATH=~/Desktop/crazyflie-firmware/build.
"""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.expanduser("~/Desktop/crazyflie-firmware/build"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../../experiments/analysis"))
import cffirmware as fw
from tk_indi_reference import TKReference

def quat_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r/2), math.sin(r/2), math.cos(p/2), math.sin(p/2), math.cos(y/2), math.sin(y/2)
    return np.array([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy])

def rot(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])

def run(n=600, yaw_literal=True, seed=1, rpm_ok_case=True, impl=6):
    rng = np.random.default_rng(seed)
    fw.cvar.g_tk_yaw_lit = 1 if yaw_literal else 0
    m = fw.cvar.g_indi_mass
    kt = [fw.cvar.g_indi_kt1, fw.cvar.g_indi_kt2, fw.cvar.g_indi_kt3, fw.cvar.g_indi_kt4]
    ref = TKReference(mass=m, kt=kt, jrz=fw.cvar.g_tk_jrz, yaw_literal=yaw_literal)
    getattr(fw, f'oot{impl}_select_drone')(0)
    getattr(fw, f'controllerOutOfTree{impl}Init')()
    ctl_fn = getattr(fw, f'controllerOutOfTree{impl}')
    rpm_h = math.sqrt(m*9.81/4/kt[0])
    ph = rng.uniform(0, 6.28, 20)
    worst_T = worst_tau = 0.0
    for k in range(n):
        t = k*1e-3
        s = lambda i, a, f: a*math.sin(2*math.pi*f*t + ph[i])
        pos = np.array([0.3+s(0, .2, 1), s(1, .2, 0.8), 1.0+s(2, .1, 0.5)])
        vel = np.array([s(3, .3, 1.2), s(4, .3, 0.7), s(5, .1, 0.9)])
        rpy = (s(6, .15, 1.3), s(7, .15, 0.9), 0.3+s(8, .2, 0.4))
        q = quat_rpy(*rpy)
        gyro = np.array([s(9, 1.5, 2.1), s(10, 1.5, 1.7), s(11, .6, 1.1)])
        acc = rot(q).T @ np.array([s(12, .8, 1.5), s(13, .8, 1.1), 9.81 + s(14, 1.0, 1.9)]) / 9.81
        rpm = np.array([rpm_h*(1+0.05*math.sin(2*math.pi*(2+0.3*i)*t+ph[15+i%4])) for i in range(4)]).round().astype(int)
        r = dict(pos=[0.3, 0, 1.0], vel=[s(16, .2, .6), s(17, .2, .5), 0.0],
                 acc=[s(18, 1.0, .7), s(19, 1.0, .6), s(0, .3, .4)],
                 jerk=[s(1, 2.0, .7), s(2, 2.0, .6), s(3, .5, .4)], snap=[s(4, 8.0, .7), s(5, 8.0, .6), s(6, 2.0, .4)],
                 yaw=0.3+s(8, .2, .4), yaw_rate=s(9, .3, .5), yaw_acc=s(10, .3, .5))
        # --- firmware ---
        sp = fw.setpoint_t()
        sp.position.x, sp.position.y, sp.position.z = r['pos']
        sp.velocity.x, sp.velocity.y, sp.velocity.z = r['vel']
        sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = r['acc']
        sp.jerk.x, sp.jerk.y, sp.jerk.z = r['jerk']
        sp.snap.x, sp.snap.y, sp.snap.z = r['snap']
        sp.attitude.yaw = math.degrees(r['yaw'])
        sp.attitudeRate.yaw = math.degrees(r['yaw_rate'])
        sp.attitudeAcc.yaw = math.degrees(r['yaw_acc'])
        sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs
        sp.mode.yaw = fw.modeAbs
        se = fw.sensorData_t()
        se.gyroNoLpf.x, se.gyroNoLpf.y, se.gyroNoLpf.z = [math.degrees(v) for v in gyro]
        se.gyro.x, se.gyro.y, se.gyro.z = [math.degrees(v) for v in gyro]
        se.acc.x, se.acc.y, se.acc.z = acc
        st = fw.state_t()
        st.position.x, st.position.y, st.position.z = pos
        st.velocity.x, st.velocity.y, st.velocity.z = vel
        st.attitudeQuaternion.w, st.attitudeQuaternion.x, st.attitudeQuaternion.y, st.attitudeQuaternion.z = q
        fw.oot_set_rpm(*[int(v) for v in rpm])
        ctl = fw.control_t()
        ctl_fn(ctl, sp, se, st, k)
        # --- reference ---
        out = ref.step(pos, vel, q, gyro, acc, rpm, r)
        eT = abs(ctl.thrustSi - out['thrust'])
        etau = max(abs(ctl.torqueX-out['torque'][0]), abs(ctl.torqueY-out['torque'][1]), abs(ctl.torqueZ-out['torque'][2]))
        worst_T, worst_tau = max(worst_T, eT), max(worst_tau, etau)
        if k in (0, 1, n-1):
            print(f"  k={k:4d} T rust/ref {ctl.thrustSi:.6f}/{out['thrust']:.6f}  tau rust {ctl.torqueX:+.3e} {ctl.torqueY:+.3e} {ctl.torqueZ:+.3e}  ref {out['torque'][0]:+.3e} {out['torque'][1]:+.3e} {out['torque'][2]:+.3e}")
    return worst_T, worst_tau

if __name__ == "__main__":
    ok = True
    for impl in (6, 7):
        if not hasattr(fw, f"controllerOutOfTree{impl}"):
            continue
        for lit in (True, False):
            wT, wt = run(yaw_literal=lit, impl=impl)
            print(f"controller {5+impl} ({'Rust' if impl == 6 else 'C'}) vs reference, yaw_literal={lit}: worst |dT|={wT:.2e} N, worst |dtau|={wt:.2e} Nm")
            ok &= wT < 1e-4 and wt < 1e-6
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)
