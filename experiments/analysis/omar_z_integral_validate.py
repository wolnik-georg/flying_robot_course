#!/usr/bin/env python3
"""Omar z-integral validation (Claude, 2026-10-08): takeoff/landing windup, real figure-8, both controllers.
Solo direct plant (consistent RPM<->force, delivered thrust scaled by cmd_gain), Omar native gains.
Usage: omar_z_integral_validate.py '<json cfg>'   -> prints one JSON line (fresh process per run)."""
import json, math, os, sys
from pathlib import Path
import numpy as np, rowan
sys.argv_cfg = json.loads(sys.argv[1]) if len(sys.argv) > 1 else {}
sys.path.insert(0, str(Path(__file__).resolve().parent))
import omar_z_integral_sil as S   # noqa: E402  (loads firmware bindings, plant, helpers)
fw, Quadrotor, State, Action = S.fw, S.Quadrotor, S.State, S.Action

def quintic(t, t0, dur, a, b):
    if t <= t0: return a
    if t >= t0 + dur: return b
    s = (t - t0) / dur
    return a + (b - a) * (6 * s**5 - 15 * s**4 + 10 * s**3)

def dquintic(t, t0, dur, a, b):
    if t <= t0 or t >= t0 + dur: return 0.0
    s = (t - t0) / dur
    return (b - a) / dur * (30 * s**4 - 60 * s**3 + 30 * s**2)

def run(cfg):
    ctrl_name, cg, kiz, profile = cfg["controller"], cfg["cmd_gain"], cfg["kiz"], cfg["profile"]
    import importlib.util
    spec = importlib.util.spec_from_file_location("u", "/home/georg/Desktop/crazyswarm2/crazyflie_py/crazyflie_py/uav_trajectory.py")
    u = importlib.util.module_from_spec(spec); spec.loader.exec_module(u)
    tr8 = u.Trajectory(); tr8.loadcsv(str(S.FIG8_CSV))
    HOVER = 1.0
    T_CLIMB, T_LAND = 2.0, 3.0
    if profile == "hover":
        t_fly0, t_fly1 = 8.0, 16.0
    else:
        t_fly0, t_fly1 = 5.0, 5.0 + 2 * tr8.duration
    t_land0 = t_fly1 + 1.0 if profile == "fig8" else 16.0
    t_end = t_land0 + T_LAND + 1.0
    devnull = os.open(os.devnull, os.O_WRONLY); o1, o2 = os.dup(1), os.dup(2)
    os.dup2(devnull, 1); os.dup2(devnull, 2)
    try:
        if ctrl_name == "oot4":
            c = fw.controllerOmarIndi_t(); fw.controllerOmarIndiInit(c); c.indi = 3; c.Kpos_I = fw.mkvec(0.0, 0.0, float(kiz))
            def step(sp, sens, st, tick):
                o = fw.control_t(); fw.controllerOmarIndi(c, o, sp, sens, st, tick); return o
        else:
            fw.controllerOutOfTree5Init(); fw.omar_indi_rust_set_indi(3); fw.omar_indi_rust_set_kpos_iz(float(kiz))
            def step(sp, sens, st, tick):
                o = fw.control_t(); fw.controllerOutOfTree5(o, sens and sp, sens, st, tick) if False else fw.controllerOutOfTree5(o, sp, sens, st, tick); return o
        plant = Quadrotor(State(pos=np.zeros(3)), dict(mass=S.MASS, inertia=S.J, kt=[S.KT] * 4, arm_length=S.ARM, t2t=S.T2T, motor_tau=0.044))
        B0_inv = np.linalg.inv(plant.B0); rpm = np.zeros(4)
        N = int(t_end / S.DT); T = np.zeros(N); P = np.zeros((N, 3)); R = np.zeros((N, 3)); V = np.zeros((N, 3)); TILT = np.zeros(N); G = np.zeros((N, 3))
        for i in range(N):
            t = i * S.DT
            ref = np.array([0.0, 0.0, 0.0]); rv = np.zeros(3); ra = np.zeros(3)
            if t < t_land0:
                zc = quintic(t, 0.0, T_CLIMB, 0.0, HOVER); vzc = dquintic(t, 0.0, T_CLIMB, 0.0, HOVER)
                ref[2] = zc; rv[2] = vzc
                if profile == "fig8" and t_fly0 <= t < t_fly1:
                    e = tr8.eval(((t - t_fly0) % tr8.duration) if ((t - t_fly0) % tr8.duration) < tr8.duration else tr8.duration); ref[:2] = e.pos[:2]; rv[:2] = e.vel[:2]; ra[:2] = e.acc[:2]
            else:
                ref[2] = quintic(t, t_land0, T_LAND, HOVER, 0.02); rv[2] = dquintic(t, t_land0, T_LAND, HOVER, 0.02)
            sp = fw.setpoint_t()
            sp.position.x, sp.position.y, sp.position.z = map(float, ref)
            sp.velocity.x, sp.velocity.y, sp.velocity.z = map(float, rv)
            sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = map(float, ra)
            sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs; sp.mode.yaw = fw.modeAbs
            st = fw.state_t()
            st.position.x, st.position.y, st.position.z = plant.state.pos
            st.velocity.x, st.velocity.y, st.velocity.z = plant.state.vel
            qw, qx, qy, qz = plant.state.quat
            st.attitudeQuaternion.w, st.attitudeQuaternion.x, st.attitudeQuaternion.y, st.attitudeQuaternion.z = qw, qx, qy, qz
            aw = rowan.rotate(plant.state.quat, plant.state.acc); st.acc.x, st.acc.y, st.acc.z = aw[0], aw[1], aw[2] - 1.0
            sens = fw.sensorData_t(); sens.gyro.x, sens.gyro.y, sens.gyro.z = map(float, np.degrees(plant.state.omega))
            fw.oot_set_rpm(*[int(x) for x in rpm])
            o = step(sp, sens, st, 2 * i)
            rhs = np.array([o.thrustSi, o.torqueX, o.torqueY, o.torqueZ])
            rc = np.sqrt(np.maximum(np.maximum(B0_inv @ rhs, 0.0) / S.KT, 0.0)) * math.sqrt(cg)
            plant.step(Action(rc), S.DT, np.zeros(3)); rpm = plant.state.rpm
            T[i] = t; P[i] = plant.state.pos; R[i] = ref; V[i] = plant.state.vel; G[i] = np.degrees(plant.state.omega)
            zb = rowan.rotate(plant.state.quat, np.array([0, 0, 1.0])); TILT[i] = math.degrees(math.acos(max(-1, min(1, zb[2]))))
    finally:
        os.dup2(o1, 1); os.dup2(o2, 2); os.close(devnull)
    m_fly = (T >= t_fly0 + 2.0) & (T < t_fly1 - 0.5) if profile == "hover" else (T >= t_fly0 + 0.5) & (T < t_fly1)
    m_climb = (T >= T_CLIMB) & (T < T_CLIMB + 4.0)
    out = dict(cfg=cfg, finite=bool(np.isfinite(P).all()),
               z_err_mean_cm=float(np.mean(P[m_fly, 2] - R[m_fly, 2]) * 100), z_err_rms_cm=float(np.sqrt(np.mean((P[m_fly, 2] - R[m_fly, 2]) ** 2)) * 100),
               xy_err_rms_cm=float(np.sqrt(np.mean(np.sum((P[m_fly, :2] - R[m_fly, :2]) ** 2, axis=1))) * 100),
               climb_overshoot_cm=float((np.max(P[m_climb, 2]) - HOVER) * 100), max_tilt_deg=float(TILT.max()),
               gyro_rms_deg_s=float(np.sqrt(np.mean(np.sum(G[m_fly] ** 2, axis=1)))))
    ml = T >= t_land0
    touch = np.where(ml & (P[:, 2] <= 0.04))[0]
    out["land_min_z_cm"] = float(P[ml, 2].min() * 100)
    out["touchdown_vz_mps"] = float(V[touch[0], 2]) if len(touch) else None
    out["z_at_end_cm"] = float(P[-1, 2] * 100)
    return out

if __name__ == "__main__":
    print("RESULT " + json.dumps(run(sys.argv_cfg)), file=sys.__stderr__ if False else sys.stdout)
