#!/usr/bin/env python3
"""WP2 of the FBL 1:1 work (docs/fbl/HANDOVER_FBL_1TO1_PROMPT_2026-10-24.md): FBL with the stock rate-PID inner loop (ctrlFbl.inner = 1) against the
flown Eq. 19 inner loop (inner = 0), both through the REAL firmware code (controller_omar_indi.c with attitude_pid_controller.c / pid.c via the host
bindings), in a rigid-body plant of our vehicle:
  - actuation through the real firmware mixer for both modes: powerDistribution (Legacy or ForceTorque) -> powerDistributionCap -> per-motor force
    u / 65535 * THRUST_MAX (force-linear uint16, battery compensation assumed ideal) -> thrust / torque with the firmware ARM_LENGTH / THRUST2TORQUE;
  - command delay 4 ms, first-order motor lag per motor (44 ms nominal = GAIN_TUNING_2026-10-20; sweep 15 / 30 / 60), gyro / position / velocity noise;
  - takeoff from the ground under the Omar controller (smooth ramp to the scenario start), FBL switches on at z_on 0.15 m (as on hardware);
  - downwash truth NS2 v2 (z only) under a scripted top vehicle; FBL residual = ROM table, C_D 0.55, + trained 4-neuron force net (nn_en 1, inner 1 runs; nn=0 runs = ROM only); peer feed every 10 ms (oot_set_peer_count 1, missing before 2026-10-25);
  - optional EKF-like state hold (new state every 4-8 ms) with ctrlFbl.abg_on_new 0 / 1;
  - body-torque disturbance: none, constant (+80 / -300 / -30 uN m, typical cf5 need, review §a #29), slowly varying (sign flips over the flight,
    amplitudes 300 / 400 / 60 uN m = the S0 range).
Gain presets: 1 = ours (position loop 0.25x), 0 = bundle "Deployed" values (56/38/48, 45/21.6/3). inner = 1 uses the stock CF21BL rate / yaw PIDs.
  CFFIRMWARE_DIR=<firmware>/build python3 experiments/fbl/stage3_port/stock_inner_sim.py [--quick]   (system python3 3.10)"""
import argparse, json, os, sys, time
from multiprocessing import Pool
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot
sys.argv_saved = sys.argv; sys.argv = sys.argv[:1]; sys.path.insert(0, str(Path(__file__).parent)); import c_in_the_loop_real as C  # noqa: E402
sys.argv = sys.argv_saved
fw = C.fw
M, G = C.M, C.G
J_FBL = np.diag([2.395e-5, 2.395e-5, 3.235e-5]); J_OMAR = np.diag([1.657e-5, 1.666e-5, 2.926e-5])
THRUST_MAX, ARM, T2T = 0.2, 0.707106781 * 0.050, 0.00569278844371417        # platform_defaults_cf21bl.h with battery compensation (bindings: CF21BL)
PRESET = {1: dict(kp_xy=14.0, kp_z=11.25, kv_xy=9.5, kv_z=5.4, ki_xy=12.0, ki_z=12.0, kr_xy=59.5, kr_z=35.0),
          0: dict(kp_xy=56.0, kp_z=45.0, kv_xy=38.0, kv_z=21.6, ki_xy=48.0, ki_z=3.0, kr_xy=238.0, kr_z=140.0)}
TAU_CONST = np.array([80e-6, -300e-6, -30e-6]); TAU_VAR_AMP = np.array([300e-6, 400e-6, 60e-6])
OUT = Path(__file__).parents[1] / "out/stage3"


def scenario(name):
    if name == "A1": return C.FSC.A1(dz=0.45, hold=15.0)
    if name == "A4": return C.FSC.A4(dz=0.4, offset=0.125, axis='y', motion='line', speed=0.35)
    return C.FSC.A8(dz=0.4, span=1.0, duration=6.25, settle=2.0, passes=2)


def mixer_forces(ctl):
    u = fw.motors_thrust_uncapped_t(); fw.powerDistribution(ctl, u); pwm = fw.motors_thrust_pwm_t(); fw.powerDistributionCap(u, pwm)
    return np.array([pwm.motors.m1, pwm.motors.m2, pwm.motors.m3, pwm.motors.m4], float) / 65535.0 * THRUST_MAX


def wrench(F):
    return F.sum(), np.array([ARM * (-F[0] - F[1] + F[2] + F[3]), ARM * (-F[0] + F[1] + F[2] - F[3]), T2T * (-F[0] + F[1] - F[2] + F[3])])


def run(cfg):
    scen, inner, preset, dist, lag_ms, jname, seed = (cfg[k] for k in ("scen", "inner", "preset", "dist", "lag", "J", "seed"))
    hold_ms, abg = cfg.get("hold", 0), cfg.get("abg", 0)
    rng = np.random.default_rng(seed); Jp = J_FBL if jname == "fbl" else J_OMAR
    sc = scenario(scen); anchor = np.array([0, 0, 0.5]); T_sc = min(r.curve.duration for r in sc.robots)
    T_TO, T_HOLD = 3.0, 1.5; T = T_TO + T_HOLD + T_sc
    omar = fw.controllerOmarIndi_t(); fw.controllerOmarIndiInit(omar); omar.mass = M; omar.indi = 0
    for k, v in PRESET[preset].items(): setattr(omar.fbl, k, v)
    fw.oot_set_peer_count(1)       # host peer feed: without it FBL never sees the top drone (residual off); fixed 2026-10-25
    omar.fbl.en = 1; omar.fbl.inner = inner; omar.fbl.abg_on_new = abg; omar.fbl.nn_en = cfg.get("nn", 1 if inner == 1 else 0);   # inner 0 = the 10-24 controller (ROM only)
    omar.fbl.c_d = 0.55; fw.cvar.g_rnn_div = 10; fw.cvar.g_rnn_en = 0
    sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
    sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs; sp.mode.yaw = fw.modeAbs; sp.attitude.yaw = 0.0
    p_start = C.kin(sc, 0, 0.0, anchor)[0]; x = np.array([p_start[0], p_start[1], 0.0]); v = np.zeros(3); q = np.array([0, 0, 0, 1.0]); w = np.zeros(3)
    dt = 1e-3; n = int(T / dt); F = np.full(4, M * G / 4); cmd_q = []; F_c = F.copy(); fext = np.zeros(3); a_m = dt / (lag_ms * 1e-3 + dt)
    rec = []; t_on = None; tilt_ho = 0.0; next_state = 0.0
    for i in range(n):
        t = i * dt; ts = t - T_TO - T_HOLD
        tp, tv, *_ = C.kin(sc, 1, max(ts, 0.0), anchor)
        if i % C.PEER_EVERY == 0: fw.oot_set_peer(0, float(tp[0]), float(tp[1]), float(tp[2]), int(i))
        if ts < 0:          # takeoff: smoothstep from the ground to the scenario start, then hold
            u_ = min(max(t / T_TO, 0.0), 1.0); s_ = 10 * u_ ** 3 - 15 * u_ ** 4 + 6 * u_ ** 5; ds = (30 * u_ ** 2 - 60 * u_ ** 3 + 30 * u_ ** 4) / T_TO
            dds = (60 * u_ - 180 * u_ ** 2 + 120 * u_ ** 3) / T_TO ** 2 if 0 < u_ < 1 else 0.0; ddds = (60 - 360 * u_ + 360 * u_ ** 2) / T_TO ** 3 if 0 < u_ < 1 else 0.0
            h = p_start[2] - 0.0; pr = np.array([p_start[0], p_start[1], h * s_]); vr = np.array([0, 0, h * ds]); ar = np.array([0, 0, h * dds]); jr = np.array([0, 0, h * ddds])
        else:
            pr, vr, ar, jr, _ = C.kin(sc, 0, ts, anchor)
        sp.position.x, sp.position.y, sp.position.z = map(float, pr); sp.velocity.x, sp.velocity.y, sp.velocity.z = map(float, vr)
        sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = map(float, ar); sp.jerk.x, sp.jerk.y, sp.jerk.z = map(float, jr)
        if t >= next_state:      # hold > 0: the state estimate changes only every 4-8 ms (EKF on the drone: median ~5.6 ms), held in between
            xm = x + rng.normal(0, 0.0005, 3); vm = v + rng.normal(0, 0.005, 3)
            st.position.x, st.position.y, st.position.z = map(float, xm); st.velocity.x, st.velocity.y, st.velocity.z = map(float, vm)
            st.attitudeQuaternion.x, st.attitudeQuaternion.y, st.attitudeQuaternion.z, st.attitudeQuaternion.w = map(float, q)
            next_state = t + (rng.uniform(0.004, 0.008) if hold_ms else 0.0)
        eul = Rot.from_quat(q).as_euler("xyz", degrees=True)                          # roll, pitch, yaw (deg); state->attitude uses the legacy pitch sign
        st.attitude.roll, st.attitude.pitch, st.attitude.yaw = float(eul[0]), float(-eul[1]), float(eul[2])
        gd = np.degrees(w) + rng.normal(0, 0.5, 3); sens.gyro.x, sens.gyro.y, sens.gyro.z = map(float, gd)
        was_on = omar.fbl.active
        fw.controllerOmarIndi(omar, ctl, sp, sens, st, i)
        if omar.fbl.active and not was_on and t_on is None: t_on = t
        if i % 2 == 0:
            Fc = mixer_forces(ctl)
            if np.isfinite(Fc).all(): cmd_q.append((t + 0.004, Fc))
        while cmd_q and cmd_q[0][0] <= t: _, F_c = cmd_q.pop(0)
        F = F + a_m * (F_c - F); thrust, tau = wrench(F)
        if dist == "const": tau = tau - TAU_CONST
        elif dist == "var": tau = tau - TAU_VAR_AMP * np.sin(2 * np.pi * t / T * np.array([1.0, 1.5, 2.0]) + np.array([0.0, 1.0, 2.0]))
        if i % 2 == 0:
            rel = [(tp - x, tv - v)] if tp[2] > x[2] else []
            a_nn, _ = C.reference(C.W, rel, float(x[2]), v, M); fext = np.asarray(a_nn, float) * M
        R = Rot.from_quat(q).as_matrix()
        v = v + (np.array([0, 0, -G]) + R[:, 2] * thrust / M + fext / M) * dt; x = x + v * dt
        if x[2] <= 0.0: x[2] = 0.0; v[2] = max(v[2], 0.0); v[:2] *= 0.0 if thrust < M * G else 1.0
        w = w + np.linalg.solve(Jp, tau - np.cross(w, Jp @ w)) * dt; q = (Rot.from_rotvec(R @ w * dt) * Rot.from_quat(q)).as_quat()
        tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))
        if t_on is not None and t < t_on + 1.0: tilt_ho = max(tilt_ho, tilt)
        if i % 10 == 0: rec.append([t, *(x - pr), eul[2], tilt, omar.fbl.active, omar.fbl.tripped, *x])
        if not np.isfinite(x).all() or tilt > 120: break
    r = np.array(rec); ts_ = r[:, 0] - T_TO - T_HOLD; win = ts_ > 2.0; late = ts_ > (T_sc / 2)
    rms = lambda a: 100 * float(np.sqrt(np.mean(a ** 2))) if a.size else float("nan")
    ok = len(r) * 10 >= n - 10
    res = dict(cfg, x_rms_cm=rms(r[win, 1]), y_rms_cm=rms(r[win, 2]), z_rms_cm=rms(r[win, 3]), xy_steady_cm=100 * float(np.hypot(r[late, 1].mean(), r[late, 2].mean())) if late.any() else float("nan"),
               yaw_absmax_deg=float(np.abs(r[ts_ > 0, 4]).max()) if (ts_ > 0).any() else float("nan"), tilt_rms_deg=float(np.sqrt(np.mean(r[win, 5] ** 2))) if win.any() else float("nan"),
               tilt_max_deg=float(r[ts_ > 0, 5].max()) if (ts_ > 0).any() else float("nan"), t_on_s=t_on, tilt_handover_deg=tilt_ho, tripped=bool(r[:, 7].max()),
               fbl_active_pct=100 * float(r[ts_ > 0, 6].mean()) if (ts_ > 0).any() else 0.0, completed=bool(ok))
    res["nn"] = int(omar.fbl.nn_en)
    res["stable"] = bool(ok and not res["tripped"] and res["tilt_max_deg"] < 45 and res["fbl_active_pct"] > 99)
    np.save(OUT / "stock_inner_traces" / (f"{scen}_i{inner}_p{preset}_{dist}_lag{lag_ms}_{jname}" + (f"_hold_abg{abg}" if hold_ms else "") + ("_nn0" if cfg.get("nn", 1) == 0 else "") + ".npy"), r)
    return res


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--quick", action="store_true"); ap.add_argument("--procs", type=int, default=16); a = ap.parse_args()
    (OUT / "stock_inner_traces").mkdir(parents=True, exist_ok=True)
    base = dict(lag=44, J="fbl", seed=0)
    cfgs = [dict(base, scen=s, inner=i, preset=p, dist=d) for s in ("A1", "A4", "A8") for i in (1, 0) for p in (1, 0) for d in ("none", "const", "var")]
    cfgs += [dict(base, scen=s, inner=1, preset=p, dist="const", lag=l) for s in ("A4", "A8") for p in (1, 0) for l in (15, 30, 60)]
    cfgs += [dict(base, scen=s, inner=1, preset=0, dist="const", lag=l) for s in ("A1", "A4", "A8") for l in (18, 21, 24, 27)]     # lag margin of preset 0
    cfgs += [dict(base, scen=s, inner=i, preset=1, dist="const", J="omar") for s in ("A1", "A4", "A8") for i in (1, 0)]
    cfgs += [dict(base, scen=s, inner=i, preset=1, dist="const", hold=6, abg=b) for s in ("A1", "A4", "A8") for i in (1, 0) for b in (0, 1)]
    cfgs += [dict(base, scen=s, inner=1, preset=p, dist="const", nn=0) for s in ("A1", "A4", "A8") for p in (1, 0)]      # network off (physics model only)
    if a.quick: cfgs = [dict(base, scen="A1", inner=1, preset=1, dist="const")]
    t0 = time.time()
    with Pool(min(a.procs, len(cfgs))) as pool: res = pool.map(run, cfgs)
    json.dump(res, open(OUT / ("stock_inner_sim_quick.json" if a.quick else "stock_inner_sim.json"), "w"), indent=1)
    for r in res:
        print(f"{r['scen']} inner {r['inner']} preset {r['preset']} {r['dist']:5s} lag {r['lag']:2d} J {r['J']:4s} hold {r.get('hold', 0)} abg {r.get('abg', 0)} nn {r.get('nn', 1)} | x {r['x_rms_cm']:6.2f} y {r['y_rms_cm']:6.2f} z {r['z_rms_cm']:6.2f} cm, "
              f"steady xy {r['xy_steady_cm']:6.2f} cm, |yaw| max {r['yaw_absmax_deg']:6.2f}, tilt rms {r['tilt_rms_deg']:5.2f} max {r['tilt_max_deg']:6.1f}, handover tilt {r['tilt_handover_deg']:5.1f} "
              f"| tripped {r['tripped']} stable {r['stable']}", flush=True)
    print(f"{len(res)} runs, {time.time() - t0:.0f} s")
