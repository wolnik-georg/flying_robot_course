#!/usr/bin/env python3
"""Runtime checks of the learned-FBL bundle (docs/73 phase 4, part 2): flatness consistency and a closed-loop smoke test.
Runs on a COPY of the bundle. usage: FBL_ROOT=<copy> PYTHONDONTWRITEBYTECODE=1 python fbl_checks_part2.py"""
import os, sys, json, warnings
from pathlib import Path
import numpy as np, torch
from scipy.spatial.transform import Rotation as Rot
ROOT = Path(os.environ["FBL_ROOT"]); sys.path.insert(0, str(ROOT)); torch.set_num_threads(1)
warnings.filterwarnings("ignore")
from KNODE.Training.flatness_utils import flat_output_to_state_and_control
from KNODE.Training.Velocity_Field.force_batch_fo import downwash_force_fo
from KNODE.Training.Velocity_Field.torque_batch_fo import downwash_torque_fo
from KNODE.Training.nominal_dynamics import NominalModel
from simulator_multi_python.Controllers.fbl_control import compute_fbl_command, FBLControl
OUT = {}
m, g = 0.03, 9.81; J = np.diag([1.43e-5, 1.43e-5, 2.89e-5]); K = np.diag([238., 238., 140.])

# ------------------------------------------------------------ A. flatness / Eq. 15 consistency
print("=== A. compute_fbl_command vs the flatness inversion (independent) on a circle")
w0, rad, zc = 0.8, 0.4, 0.5
def flat(t):
    c, s = np.cos(w0 * t), np.sin(w0 * t)
    return dict(x=np.array([rad * c, rad * s, zc]), x_dot=w0 * rad * np.array([-s, c, 0]), x_ddot=-w0 ** 2 * rad * np.array([c, s, 0]),
                x_dddot=-w0 ** 3 * rad * np.array([-s, c, 0]), x_ddddot=w0 ** 4 * rad * np.array([c, s, 0]), yaw=0.0, yaw_dot=0.0, yaw_ddot=0.0)
t = 1.3; f = flat(t)
st, ct = flat_output_to_state_and_control(f, m, g, J)
R = Rot.from_quat(st[6:10]).as_matrix()
thr, om, tau = compute_fbl_command(R, st[10:13], f["x_ddot"], f["x_dddot"], 0.0, 0.0, np.zeros(3), np.zeros(3), np.zeros(3), m, g, J, K)
print("  no residual: thrust flatness %.6f / FBL %.6f N ; omega flatness %s / FBL %s ; torque (should be ~0) %s" % (ct[0], thr, np.round(st[10:13], 5), np.round(om, 5), np.round(tau, 8)))
OUT["A_no_residual"] = dict(thrust_flat=float(ct[0]), thrust_fbl=float(thr), omega_flat=list(map(float, st[10:13])), omega_fbl=list(map(float, om)))
# with a time-varying residual: a_res(t) = a0 + j_res t ; numerical omega from R(t+-dt) of the flatness inversion using the residual acceleration
a0 = np.array([0.04, -0.03, -1.2]); j_res = np.array([0.3, -0.2, 0.9])
def Rof(tt): return Rot.from_quat(flat_output_to_state_and_control(flat(tt), m, g, J, a_downwash=a0 + j_res * tt)[0][6:10]).as_matrix()
dt = 1e-4; Rd = (Rof(t + dt) - Rof(t - dt)) / (2 * dt); W = Rof(t).T @ Rd; om_num = np.array([W[2, 1], W[0, 2], W[1, 0]])
st2, ct2 = flat_output_to_state_and_control(f, m, g, J, a_downwash=a0 + j_res * t); R2 = Rot.from_quat(st2[6:10]).as_matrix()
thr2, om2, _ = compute_fbl_command(R2, om_num, f["x_ddot"], f["x_dddot"], 0.0, 0.0, a0 + j_res * t, j_res, np.zeros(3), m, g, J, K)
thr3, om3, _ = compute_fbl_command(R2, om_num, f["x_ddot"], f["x_dddot"], 0.0, 0.0, a0 + j_res * t, np.zeros(3), np.zeros(3), m, g, J, K)
print("  with residual (a_res + j_res t): thrust flatness %.6f / FBL %.6f N" % (ct2[0], thr2))
print("    omega from numerical differentiation of R(t) : %s" % np.round(om_num, 5))
print("    omega compute_fbl_command (with j_res)       : %s   max err %.2e rad/s" % (np.round(om2, 5), np.max(np.abs(om2 - om_num))))
print("    omega compute_fbl_command (j_res dropped)    : %s   max err %.2e rad/s  (shows the term matters)" % (np.round(om3, 5), np.max(np.abs(om3 - om_num))))
print("    omega from flatness_utils (legacy, ignores j_res): %s   max err %.2e rad/s" % (np.round(st2[10:13], 5), np.max(np.abs(st2[10:13] - om_num))))
OUT["A_with_residual"] = dict(thrust_flat=float(ct2[0]), thrust_fbl=float(thr2), omega_num=list(map(float, om_num)), err_fbl=float(np.max(np.abs(om2 - om_num))),
                              err_fbl_without_jerk=float(np.max(np.abs(om3 - om_num))), err_flatness_utils=float(np.max(np.abs(st2[10:13] - om_num))))

# ------------------------------------------------------------ B. closed-loop smoke test (own plant, force + torque downwash with a mismatched drag coefficient)
CTRL_HZ = float(os.environ.get("CTRL_HZ", "500"))
print("\n=== B. closed-loop smoke test (control rate %.0f Hz):" % CTRL_HZ, end=" ")
print(" FBLControl (bundle) on my own 6-DoF plant; top drone fixed at 0.9 m, bottom translates beneath at <= 0.4 m/s")
CD_PLANT = float(os.environ.get("CD_PLANT", "0.55")); CTRL_HZ = float(os.environ.get("CTRL_HZ", "500"))
def quintic(t, T, L):
    s = np.clip(t / T, 0, 1); p = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5; v = (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / T
    a = (60 * s - 180 * s ** 2 + 120 * s ** 3) / T ** 2; j = (60 - 360 * s + 360 * s ** 2) / T ** 3
    inside = (t >= 0) & (t <= T)
    return L * p, L * v * inside, L * a * inside, L * j * inside
def ref(t, hold0=1.0, T=6.0, L=1.2, x0=-0.6, z=0.5):
    p, v, a, j = quintic(t - hold0, T, L)
    return dict(x=np.array([x0 + p, 0.0, z]), x_dot=np.array([v, 0, 0]), x_ddot=np.array([a, 0, 0]), x_dddot=np.array([j, 0, 0]), yaw=0.0, yaw_dot=0.0)
p_top = np.array([0.0, 0.0, 0.9])
nom = NominalModel()
def plant_dist(p, Rm, cd):
    d = p - p_top; r0 = np.hypot(d[0], d[1]); z0 = abs(d[2])
    z = torch.tensor([z0], dtype=torch.float64); r = torch.tensor([r0], dtype=torch.float64); zero = torch.zeros(1, dtype=torch.float64)
    Fz = float(downwash_force_fo(z, r, zero, C_d=cd, S=0.0997)[0])
    r_hat = torch.tensor([p_top - p], dtype=torch.float64); zb = torch.tensor([[0.0, 0.0, 1.0]], dtype=torch.float64)
    tw = downwash_torque_fo(z, r, zero, r_hat, zb, C_d=cd, S=0.0997).numpy()[0]
    return np.array([0.0, 0.0, -Fz]), Rm.T @ tw          # force on the lower drone (world, N), torque in body frame
LAST = {}
def run(mode, use_torque_dist=True, T_end=9.0):
    downwash = mode in ("ROM", "ROM+NN"); use_nn = mode == "ROM+NN"
    ctrl = FBLControl(dt_c if False else 0.01, 10, downwash, use_nn, False, 0.3, 0.0997, 0, "quad_bot", 1)
    p = ref(0.0)["x"].copy(); v = np.zeros(3); Rm = np.eye(3); w = np.zeros(3); dt_c = 1.0 / CTRL_HZ; nsub = max(1, int(round(dt_c / 5e-4))); dt_s = dt_c / nsub; rows = []
    thr_cmd, mom_cmd = m * g, np.zeros(3)
    for k in range(int(round(T_end / dt_c))):
        tt = k * dt_c; q = Rot.from_matrix(Rm).as_quat()
        state = dict(x=p.copy(), v=v.copy(), q=q, w=w.copy(), u=np.array([thr_cmd]))
        drones = [dict(name="quad_bot", state=np.r_[p, v, q, w, thr_cmd]), dict(name="quad_top", state=np.r_[p_top, np.zeros(3), 0, 0, 0, 1, np.zeros(3), 0])]
        ctrl.update(tt, state, ref(tt), drones, None, None, 1)
        thr_cmd, mom_cmd = ctrl.cmd_thrust, np.array(ctrl.cmd_moment)
        for _ in range(nsub):
            fd, td = plant_dist(p, Rm, CD_PLANT)
            a = (thr_cmd * (Rm @ np.array([0, 0, 1.0])) + fd) / m + np.array([0, 0, -g])
            tau = mom_cmd + (td if use_torque_dist else 0.0)
            wdot = np.linalg.solve(J, tau - np.cross(w, J @ w))
            p = p + v * dt_s + 0.5 * a * dt_s ** 2; v = v + a * dt_s
            ang = w * dt_s; Rm = Rm @ Rot.from_rotvec(ang).as_matrix(); w = w + wdot * dt_s
        rows.append((tt, *p, *ref(tt)["x"], *v, np.degrees(Rot.from_matrix(Rm).as_euler("xyz")[:2]).tolist()[0], np.degrees(Rot.from_matrix(Rm).as_euler("xyz")[:2]).tolist()[1], thr_cmd, float(np.max(np.abs(w)))))
    A = np.array(rows); LAST[mode] = A; err = A[:, 1:4] - A[:, 4:7]; win = A[:, 0] >= 0.5
    return dict(rmse=float(np.sqrt(np.mean(np.sum(err[win] ** 2, axis=1)))), zmax=float(np.max(np.abs(err[win, 2]))), zbias=float(np.mean(err[win, 2])),
                tilt_max=float(np.max(np.abs(A[:, 10:12]))), thr_max=float(A[:, 12].max()), w_max=float(A[:, 13].max()))
res = {}
for dist in (True, False):
    for mode in ("nominal", "ROM", "ROM+NN"):
        try: res[(dist, mode)] = run(mode, dist)
        except Exception as e: res[(dist, mode)] = dict(error=repr(e)[:200])
for dist in (True, False):
    print("  plant C_D = %.2f, torque disturbance %s:" % (CD_PLANT, "ON" if dist else "OFF"))
    for mode in ("nominal", "ROM", "ROM+NN"):
        r_ = res[(dist, mode)]
        print("    %-8s " % mode + ("RMSE %.4f m  z_max %.4f m  mean z error %+.4f m  max tilt %.1f deg  max rate %.2f rad/s  max thrust %.3f N" % (r_["rmse"], r_["zmax"], r_["zbias"], r_["tilt_max"], r_["w_max"], r_["thr_max"]) if "rmse" in r_ else r_["error"]))
OUT["B_closed_loop"] = {("torque_dist_%s_%s" % (d, m_)): v_ for (d, m_), v_ in res.items()}; OUT["B_cd_plant"] = CD_PLANT
json.dump(OUT, open(Path(__file__).parent / "out" / ("part2_results_cd%.2f_hz%.0f.json" % (CD_PLANT, CTRL_HZ)), "w"), indent=1, default=float)
if os.environ.get("FBL_DEBUG_SERIES"):
    A = LAST["nominal"]
    print("  time series nominal (last run): t, x, z, z_ref, vz, tilt_x, tilt_y, thrust")
    for i in range(0, len(A), 40): print("   %.2f  x %+.3f  z %.4f  zref %.3f  vz %+.4f  tilt %+.2f %+.2f  thr %.4f" % (A[i,0], A[i,1], A[i,3], A[i,6], A[i,9] if A.shape[1] > 9 else 0, A[i,10], A[i,11], A[i,12]))
