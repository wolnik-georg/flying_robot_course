# Step B diagnostic 2026-10-28: stock_inner_sim.py plus options ad (lateral accel), md (measurement delay ms), quad (quadratic motor law), kappa (plume torque from NS2 at the rotors). System python3 + CFFIRMWARE_DIR.
#!/usr/bin/env python3
"""WP2 of the FBL 1:1 work (docs/fbl/HANDOVER_FBL_1TO1_PROMPT_2026-10-24.md): FBL with the stock rate-PID inner loop (ctrlFbl.inner = 1) against the
flown Eq. 19 inner loop (inner = 0), both through the REAL firmware code (controller_omar_indi.c with attitude_pid_controller.c / pid.c via the host
bindings), in a rigid-body plant of our vehicle:
  - actuation through the real firmware mixer for both modes: powerDistribution (Legacy or ForceTorque) -> powerDistributionCap -> per-motor force
    u / 65535 * THRUST_MAX (force-linear uint16, battery compensation assumed ideal) -> thrust / torque with the firmware ARM_LENGTH / THRUST2TORQUE;
  - command delay 4 ms, first-order motor lag per motor (44 ms nominal = GAIN_TUNING_2026-10-20; sweep 15 / 30 / 60), gyro / position / velocity noise;
  - takeoff from the ground and landing back on it (smoothstep ramps); inner 1 = firmware defaults since 2026-10-25: FBL flies the whole flight (no handover); inner 0 = the 10-24 controller (Omar below 0.15 m, tilt latch, rate limit);
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
sys.argv_saved = sys.argv; sys.argv = sys.argv[:1]; sys.path.insert(0, '/home/georg/Desktop/flying_robot_course/experiments/fbl/stage3_port'); import c_in_the_loop_real as C  # noqa: E402
sys.argv = sys.argv_saved
fw = C.fw
M, G = C.M, C.G
ARMS = [(0.0354, -0.0354), (-0.0354, -0.0354), (-0.0354, 0.0354), (0.0354, 0.0354)]
J_FBL = np.diag([2.395e-5, 2.395e-5, 3.235e-5]); J_OMAR = np.diag([1.657e-5, 1.666e-5, 2.926e-5])
THRUST_MAX, ARM, T2T = 0.2, 0.707106781 * 0.050, 0.00569278844371417        # platform_defaults_cf21bl.h with battery compensation (bindings: CF21BL)
PRESET = {1: dict(kp_xy=14.0, kp_z=11.25, kv_xy=9.5, kv_z=5.4, ki_xy=12.0, ki_z=12.0, kr_xy=59.5, kr_z=35.0),
          0: dict(kp_xy=56.0, kp_z=45.0, kv_xy=38.0, kv_z=21.6, ki_xy=48.0, ki_z=3.0, kr_xy=238.0, kr_z=140.0)}
TAU_CONST = np.array([80e-6, -300e-6, -30e-6]); TAU_VAR_AMP = np.array([300e-6, 400e-6, 60e-6])
OUT = Path('/home/georg/Desktop/flying_robot_course/experiments/fbl/out/stage3/sysid'); OUT.mkdir(parents=True, exist_ok=True)


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
    T_TO, T_HOLD, T_LAND, T_GND = 3.0, 1.5, 3.0, 1.0; T = T_TO + T_HOLD + T_sc + T_LAND + T_GND
    omar = fw.controllerOmarIndi_t(); fw.controllerOmarIndiInit(omar); omar.mass = M; omar.indi = 0
    for k, v in PRESET[preset].items(): setattr(omar.fbl, k, v)
    fw.oot_set_peer_count(1)       # host peer feed: without it FBL never sees the top drone (residual off); fixed 2026-10-25
    omar.fbl.en = 1; omar.fbl.inner = inner; omar.fbl.abg_on_new = abg; omar.fbl.nn_en = cfg.get("nn", 1 if inner == 1 else 0);   # inner 0 = the 10-24 controller (ROM only)
    omar.fbl.c_d = 0.55; fw.cvar.g_rnn_div = 10; fw.cvar.g_rnn_en = 0
    if inner == 0: omar.fbl.z_on, omar.fbl.z_off, omar.fbl.tilt_max_deg, omar.fbl.w_max = 0.15, 0.10, 60.0, 20.0   # the 10-24 controller (handover, latch, limit)
    # inner 1: firmware defaults since 2026-10-25 = FBL from takeoff to landing, no latch, no rate limit
    sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
    sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs; sp.mode.yaw = fw.modeAbs; sp.attitude.yaw = 0.0
    p_start = C.kin(sc, 0, 0.0, anchor)[0]; x = np.array([p_start[0], p_start[1], 0.0]); v = np.zeros(3); q = np.array([0, 0, 0, 1.0]); w = np.zeros(3)
    dt = 1e-3; n = int(T / dt); F = np.full(4, M * G / 4); cmd_q = []; F_c = F.copy(); fext = np.zeros(3); a_m = dt / (lag_ms * 1e-3 + dt)
    mq = []; tau_ns2 = np.zeros(3); rec = []; t_on = None; tilt_ho = 0.0; next_state = 0.0; td_vz = None; tilt_land = 0.0
    for i in range(n):
        t = i * dt; ts = t - T_TO - T_HOLD
        tp, tv, *_ = C.kin(sc, 1, max(ts, 0.0), anchor)
        if cfg.get('top') is not None:      # logged trajectory of the top drone instead of the scripted one (columns t, x, y, z; t = 0 at the scenario start)
            TL = cfg['top']; tq = min(max(ts, TL[0, 0]), TL[-1, 0]); tp = np.array([np.interp(tq, TL[:, 0], TL[:, k]) for k in (1, 2, 3)])
            tv = np.array([np.interp(tq, TL[:, 0], TL[:, k]) for k in (4, 5, 6)])
        if i % C.PEER_EVERY == 0: fw.oot_set_peer(0, float(tp[0]), float(tp[1]), float(tp[2]), int(i))
        if ts < 0:          # takeoff: smoothstep from the ground to the scenario start, then hold
            u_ = min(max(t / T_TO, 0.0), 1.0); s_ = 10 * u_ ** 3 - 15 * u_ ** 4 + 6 * u_ ** 5; ds = (30 * u_ ** 2 - 60 * u_ ** 3 + 30 * u_ ** 4) / T_TO
            dds = (60 * u_ - 180 * u_ ** 2 + 120 * u_ ** 3) / T_TO ** 2 if 0 < u_ < 1 else 0.0; ddds = (60 - 360 * u_ + 360 * u_ ** 2) / T_TO ** 3 if 0 < u_ < 1 else 0.0
            h = p_start[2] - 0.0; pr = np.array([p_start[0], p_start[1], h * s_]); vr = np.array([0, 0, h * ds]); ar = np.array([0, 0, h * dds]); jr = np.array([0, 0, h * ddds])
        elif ts <= T_sc:
            pr, vr, ar, jr, _ = C.kin(sc, 0, ts, anchor)
        else:               # landing: smoothstep from the last scenario point to the ground, then on the ground (as the HLC land)
            if 'p_end' not in locals(): p_end = C.kin(sc, 0, T_sc, anchor)[0]
            u_ = min(max((ts - T_sc) / T_LAND, 0.0), 1.0); s_ = 10 * u_ ** 3 - 15 * u_ ** 4 + 6 * u_ ** 5; ds = (30 * u_ ** 2 - 60 * u_ ** 3 + 30 * u_ ** 4) / T_LAND
            dds = (60 * u_ - 180 * u_ ** 2 + 120 * u_ ** 3) / T_LAND ** 2 if 0 < u_ < 1 else 0.0; ddds = (60 - 360 * u_ + 360 * u_ ** 2) / T_LAND ** 3 if 0 < u_ < 1 else 0.0
            h = p_end[2]; pr = np.array([p_end[0], p_end[1], h * (1 - s_)]); vr = np.array([0, 0, -h * ds]); ar = np.array([0, 0, -h * dds]); jr = np.array([0, 0, -h * ddds])
        sp.position.x, sp.position.y, sp.position.z = map(float, pr); sp.velocity.x, sp.velocity.y, sp.velocity.z = map(float, vr)
        if cfg.get('ff') == 'nojerk': jr = jr * 0.0
        if cfg.get('ff') == 'noacc': jr = jr * 0.0; ar = ar * 0.0
        sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = map(float, ar); sp.jerk.x, sp.jerk.y, sp.jerk.z = map(float, jr)
        if t >= next_state:      # hold > 0: the state estimate changes only every 4-8 ms (EKF on the drone: median ~5.6 ms), held in between
            xm = x + rng.normal(0, 0.0005, 3); vm = v + rng.normal(0, 0.005, 3)
            md = cfg.get('md', 0)
            if md:
                mq.append((xm.copy(), vm.copy()))
                if len(mq) > int(md) + 1: mq.pop(0)
                xm, vm = mq[0]
            st.position.x, st.position.y, st.position.z = map(float, xm); st.velocity.x, st.velocity.y, st.velocity.z = map(float, vm)
            st.attitudeQuaternion.x, st.attitudeQuaternion.y, st.attitudeQuaternion.z, st.attitudeQuaternion.w = map(float, q)
            next_state = t + (rng.uniform(0.004, 0.008) if hold_ms else 0.0)
        eul = Rot.from_quat(q).as_euler("xyz", degrees=True)                          # roll, pitch, yaw (deg); state->attitude uses the legacy pitch sign
        st.attitude.roll, st.attitude.pitch, st.attitude.yaw = float(eul[0]), float(-eul[1]), float(eul[2])
        gd = np.degrees(w) + rng.normal(0, cfg.get('gn', 0.5), 3); sens.gyro.x, sens.gyro.y, sens.gyro.z = map(float, gd)
        was_on = omar.fbl.active
        fw.controllerOmarIndi(omar, ctl, sp, sens, st, i)
        if omar.fbl.active and not was_on and t_on is None: t_on = t
        if i % 2 == 0:
            Fc = mixer_forces(ctl)
            if np.isfinite(Fc).all(): cmd_q.append((t + 0.004, Fc))
        while cmd_q and cmd_q[0][0] <= t: _, F_c = cmd_q.pop(0)
        F = F + a_m * (F_c - F); thrust, tau = wrench(F)
        tau = tau + (1.0 if cfg.get('plume') else cfg.get('kappa', 0)) * tau_ns2
        if dist == "const": tau = tau - TAU_CONST
        elif dist == "var": tau = tau - TAU_VAR_AMP * np.sin(2 * np.pi * t / T * np.array([1.0, 1.5, 2.0]) + np.array([0.0, 1.0, 2.0]))
        if i % 2 == 0:
            rel = [(tp - x, tv - v)] if tp[2] > x[2] else []
            a_nn, _ = C.reference(C.W, rel, float(x[2]), v, M); fext = np.asarray(a_nn, float) * M
            tau_ns2 = np.zeros(3)
            if cfg.get('plume') and tp[2] > x[2]:      # fitted plume model (fit_plume_torque.py): lift change per rotor, Gaussian in the distance to the top drone's axis
                Ap, sp_, np_ = cfg['plume']; Rm = Rot.from_quat(q).as_matrix()
                for rk in ARMS:
                    pk = x + Rm @ np.array([rk[0], rk[1], 0.0]); rho2 = (pk[0] - tp[0]) ** 2 + (pk[1] - tp[1]) ** 2; dFk = -Ap * (0.4 / max(tp[2] - pk[2], 0.05)) ** np_ * np.exp(-rho2 / (2 * sp_ ** 2))
                    tau_ns2 += np.array([rk[1] * dFk, -rk[0] * dFk, 0.0])
            if cfg.get('kappa') and not cfg.get('plume') and tp[2] > x[2]:
                Rm = Rot.from_quat(q).as_matrix()
                for rk in ARMS:
                    pk = x + Rm @ np.array([rk[0], rk[1], 0.0]); a_k, _ = C.reference(C.W, [(tp - pk, tv - v)], float(pk[2]), v, M); Fk = M * float(np.asarray(a_k, float)[2]) / 4
                    tau_ns2 += np.array([rk[1] * Fk, -rk[0] * Fk, 0.0])
        R = Rot.from_quat(q).as_matrix()
        v = v + (np.array([0, 0, -G]) + R[:, 2] * thrust / M + fext / M + (np.array([cfg['ad'][0], cfg['ad'][1], 0.0]) if (cfg.get('ad') and ts < cfg['ad'][2]) else 0.0)) * dt; x = x + v * dt
        if x[2] <= 0.0:
            if ts > T_sc and td_vz is None and v[2] < 0: td_vz = float(v[2])         # touchdown speed
            x[2] = 0.0; v[2] = max(v[2], 0.0); v[:2] *= 0.0 if thrust < M * G else 1.0
        w = w + np.linalg.solve(Jp, tau - np.cross(w, Jp @ w)) * dt; q = (Rot.from_rotvec(R @ w * dt) * Rot.from_quat(q)).as_quat()
        tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))
        if t_on is not None and t < t_on + 1.0: tilt_ho = max(tilt_ho, tilt)
        if ts > T_sc: tilt_land = max(tilt_land, tilt)
        if i % 10 == 0: rec.append([t, *(x - pr), eul[2], tilt, omar.fbl.active, omar.fbl.tripped, *x])
        if not np.isfinite(x).all() or tilt > 120: break
    r = np.array(rec); ts_ = r[:, 0] - T_TO - T_HOLD; win = (ts_ > 2.0) & (ts_ <= T_sc); late = (ts_ > (T_sc / 2)) & (ts_ <= T_sc)
    rms = lambda a: 100 * float(np.sqrt(np.mean(a ** 2))) if a.size else float("nan")
    ok = len(r) * 10 >= n - 10
    res = dict(cfg, x_rms_cm=rms(r[win, 1]), y_rms_cm=rms(r[win, 2]), z_rms_cm=rms(r[win, 3]), xy_steady_cm=100 * float(np.hypot(r[late, 1].mean(), r[late, 2].mean())) if late.any() else float("nan"),
               yaw_absmax_deg=float(np.abs(r[(ts_ > 0) & (ts_ <= T_sc), 4]).max()) if (ts_ > 0).any() else float("nan"), tilt_rms_deg=float(np.sqrt(np.mean(r[win, 5] ** 2))) if win.any() else float("nan"),
               tilt_max_deg=float(r[(ts_ > 0) & (ts_ <= T_sc), 5].max()) if (ts_ > 0).any() else float("nan"), t_on_s=t_on, tilt_handover_deg=tilt_ho, tripped=bool(r[:, 7].max()),
               fbl_active_pct=100 * float(r[(ts_ > 0) & (ts_ <= T_sc), 6].mean()) if (ts_ > 0).any() else 0.0, completed=bool(ok))
    res["nn"] = int(omar.fbl.nn_en); res["touchdown_vz"] = td_vz; res["tilt_landing_deg"] = tilt_land; res["t_on_height"] = None
    res["stable"] = bool(ok and not res["tripped"] and res["tilt_max_deg"] < 45 and res["fbl_active_pct"] > 99)
    np.save(OUT / "stock_inner_traces" / (f"{scen}_i{inner}_p{preset}_{dist}_lag{lag_ms}_{jname}_md{cfg.get('md',0)}_ad{cfg.get('ad')}_k{cfg.get('kappa',0)}_{cfg.get('nm','')}_{cfg.get('tag','')}_{'top' if cfg.get('top') is not None else 'scr'}" + (f"_hold_abg{abg}" if hold_ms else "") + ("_nn0" if cfg.get("nn", 1) == 0 else "") + ".npy"), r)
    return res


def top_from_log(name, scen):
    """Logged top-drone trajectory in the SIMULATION frame: the lab frame differs from the scenario frame (rotation + shift), found by a 2-D rigid fit
    of the simulated planned bottom path to the logged setpoint (ctrltarget)."""
    import pandas as pd
    from scipy.ndimage import gaussian_filter1d
    p = f'/home/georg/Desktop/flying_robot_course/experiments/logs/bank_fbl_b1_ours_2026-10-26/{name}/{name}_merged_usd.csv'
    df = pd.read_csv(p, comment='#'); P0 = [c.split('.')[0] + '.' for c in df.columns if c.startswith('cf5')][0]; tp_ = [c for c in df.columns if c.startswith('cf_second')][0].split('.')[0]
    t = df.t.values; dt = np.median(np.diff(t)); sc = scenario(scen); anchor = np.array([0, 0, 0.5]); Tsc = min(r.curve.duration for r in sc.robots)
    m = (t > 0.5) & (t < Tsc - 0.5)
    lab = np.c_[df[P0 + 'ctrltarget_x'].values[m], df[P0 + 'ctrltarget_y'].values[m]]
    sim = np.array([C.kin(sc, 0, float(ti), anchor)[0][:2] for ti in t[m]])
    mu_l, mu_s = lab.mean(0), sim.mean(0); H = (lab - mu_l).T @ (sim - mu_s); U, _, Vt = np.linalg.svd(H); d = np.sign(np.linalg.det(Vt.T @ U.T)); Rm = Vt.T @ np.diag([1, d]) @ U.T   # sim = Rm (lab - mu_l) + mu_s
    P = np.c_[[gaussian_filter1d(df[tp_ + '.' + a].values.astype(float), 0.1 / dt) for a in 'xyz']].T
    xy = (P[:, :2] - mu_l) @ Rm.T + mu_s; Ps = np.c_[xy, P[:, 2]]
    fit_err = np.abs((lab - mu_l) @ Rm.T + mu_s - sim).max()
    V = np.gradient(Ps, dt, axis=0); print(name[-8:], 'frame fit max err %.3f m, top at t=0 (sim frame):' % fit_err, Ps[0].round(3), 'scripted top:', C.kin(sc, 1, 0.0, anchor)[0].round(3)); return np.c_[t, Ps, V]


HW = {'A4': 10.2, 'A8': 5.6, 'A1': 3.0}
FLIGHTS = [('A4', 'A4_2026-10-26_17-25-15'), ('A4', 'A4_2026-10-26_17-26-40'), ('A8', 'A8_2026-10-26_17-23-37'), ('A1', 'A1_2026-10-26_17-28-19')]
if __name__ == "__main__":
    import json as _j
    from scipy.ndimage import gaussian_filter1d
    (OUT / "stock_inner_traces").mkdir(parents=True, exist_ok=True)
    pf = _j.load(open(Path(__file__).parent / 'out/plume_fit.json')); PL = (pf['A'], pf['sigma'], pf['n'])
    base = dict(J="fbl", inner=1, preset=1, lag=30, quad=1, dist="none", plume=PL, seed=0)
    TL = top_from_log('A8_2026-10-26_17-23-37', 'A8'); cfgs = []
    for hold, abg in ((0, 0), (6, 0), (6, 1), (3, 0), (3, 1)): cfgs.append(dict(base, scen='A8', top=TL, nm='h%d_a%d' % (hold, abg), gn=5.0, hold=hold, abg=abg, tag='ff'))
    with Pool(5) as pool: res = pool.map(run, cfgs)
    for c, r in zip(cfgs, res):
        tr = np.load(OUT / "stock_inner_traces" / (f"A8_i1_p1_none_lag30_fbl_md0_adNone_k0_{c['nm']}_ff_top" + (f"_hold_abg{c['abg']}" if c['hold'] else "") + ".npy")); t = tr[:, 0]; dt = np.median(np.diff(t)); m = t > 5.5
        xs = tr[:, 9]; pr = xs - tr[:, 2]; ar = np.gradient(np.gradient(gaussian_filter1d(pr, 0.1 / dt), dt), dt); am = np.gradient(np.gradient(gaussian_filter1d(xs, 0.1 / dt), dt), dt)
        a_, b_ = ar[m], am[m]; best = max(((np.corrcoef(a_[: len(a_) - k], b_[k:])[0, 1], k) for k in range(0, int(1.2 / dt))), key=lambda z: z[0])
        print("A8 sim EKF hold %d ms abg_on_new %d | xy %.1f cm | accel lag %3.0f ms corr %.2f | hardware: xy 5.6 cm, lag 364 ms" % (c['hold'], c['abg'], (r['x_rms_cm']**2 + r['y_rms_cm']**2) ** .5, best[1] * dt * 1000, best[0]))
    raise SystemExit
