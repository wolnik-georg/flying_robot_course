#!/usr/bin/env python3
"""Runtime checks of the learned-FBL bundle (docs/73 phase 4, part 1). Runs on a COPY of the bundle (never the original).
usage: FBL_ROOT=<copy of learned_fbl_core> PYTHONDONTWRITEBYTECODE=1 ~/.pyenv/versions/flying_robots/bin/python fbl_checks_part1.py"""
import os, sys, json, time, subprocess
from pathlib import Path
import numpy as np, torch
ROOT = Path(os.environ["FBL_ROOT"]); sys.path.insert(0, str(ROOT))
torch.set_num_threads(1)
from KNODE.Training.nn_models_full_only_zr import ODEFunc, DownwashResidual
from KNODE.Training.nominal_dynamics import C_d, S, NominalModel
from KNODE.Training.Velocity_Field.force_batch_fo import downwash_force_fo, downwash_force_and_derivs_fo
from KNODE.Training.Velocity_Field.torque_batch_fo import downwash_torque_fo
from KNODE.Training.residual_derivatives import network_value_and_jvp
from simulator_multi_python.Controllers.fbl_control import compute_fbl_command
OUT = {}
ck = torch.load(ROOT / "KNODE/Models/sample_model.pth", map_location="cpu", weights_only=True)
net = ODEFunc(); net.load_state_dict(ck["state_dict"]); net.eval()
import copy
net64 = copy.deepcopy(net).double()   # the shipped nets are float32; float64 copies are used for derivative checks (dtype mixing otherwise raises)
def model(use_nn, use_nom, n=None): return DownwashResidual(n or net, use_nn=use_nn, use_nominal=use_nom).eval()
M_full, M_nom, M_nn = model(True, True), model(False, True), model(True, False)
M64_full, M64_nom, M64_nn = model(True, True, net64), model(False, True, net64), model(True, False, net64)

def pair(pe, ve, pt, vt):
    ego = np.r_[pe, ve, [0, 0, 0, 1.0], 0, 0, 0]; top = np.r_[pt, vt, [0, 0, 0, 1.0], 0, 0, 0]
    return torch.tensor(np.r_[ego, np.zeros(4), top], dtype=torch.float64)

# ---------------------------------------------------------------- 1. example
print("=== 1. authors' example (run in the copy)")
r = subprocess.run([sys.executable, str(ROOT / "examples/learned_fbl_inference.py")], capture_output=True, text=True, cwd=str(ROOT),
                   env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
print(r.stdout.strip()); print("stderr:", (r.stderr.strip()[:400] or "(empty)")); OUT["example_stdout"] = r.stdout; OUT["example_rc"] = r.returncode

# ---------------------------------------------------------------- 2. analytic jerk vs central finite difference
print("\n=== 2. analytic time derivative of the residual acceleration vs central finite difference (float64 copies of the nets, h = 5 ms)")
rng = np.random.default_rng(0); h = 5e-3; worst = {"nom": 0, "nn": 0, "full": 0}; rel = {"nom": [], "nn": [], "full": []}
for _ in range(300):
    dz = rng.uniform(0.15, 0.9); r0 = rng.uniform(0.03, 0.4); az = rng.uniform(0, 2 * np.pi)
    pt = np.array([0, 0, 1.0]); pe = pt + np.array([r0 * np.cos(az), r0 * np.sin(az), -dz])
    ve = rng.uniform(-0.5, 0.5, 3); vt = rng.uniform(-0.5, 0.5, 3)
    for key, M in (("nom", M64_nom), ("nn", M64_nn), ("full", M64_full)):
        with torch.no_grad():
            a0, ad = M.acceleration(pair(pe, ve, pt, vt))
            ap, _ = M.acceleration(pair(pe + ve * h, ve, pt + vt * h, vt)); am, _ = M.acceleration(pair(pe - ve * h, ve, pt - vt * h, vt))
        num = ((ap - am) / (2 * h)).numpy()[0]; ana = ad.numpy()[0]
        err = np.max(np.abs(num - ana)); worst[key] = max(worst[key], err); rel[key].append(err / (np.linalg.norm(ana) + 1e-9))
for k in worst: print("  %-5s max |analytic − numeric| = %.4g m/s³ ; median rel. error %.2e ; max rel. error %.2e" % (k, worst[k], np.median(rel[k]), np.max(rel[k])))
OUT["jerk_fd_check"] = {k: {"max_abs": float(worst[k]), "median_rel": float(np.median(rel[k])), "max_rel": float(np.max(rel[k]))} for k in worst}
# NN jvp vs autograd
f = torch.tensor(rng.uniform(0.02, 0.6, (50, 2)), dtype=torch.float32); tg = torch.tensor(rng.uniform(-0.5, 0.5, (50, 2)), dtype=torch.float32)
v1, d1 = network_value_and_jvp(net.net_f, f, tg)
d2 = torch.autograd.functional.jvp(lambda x: net.net_f(x), f, tg)[1]
print("  network_value_and_jvp vs autograd JVP: max abs diff %.2e" % float((d1 - d2).abs().max())); OUT["nn_jvp_vs_autograd"] = float((d1 - d2).abs().max())

# ---------------------------------------------------------------- 3. force quadrature vs fine integration; analytic partials vs finite difference
print("\n=== 3. ROM force: 10x10 quadrature vs fine integration; analytic partials vs finite difference of the 10x10 sum")
rho = 1.293; m = 0.03; g = 9.81; rp = 0.0225; U_H = np.sqrt(m * g / (2 * rho * np.pi * rp ** 2 * 4)); l = 2 * (0.046 + rp); Bd = 6.84; z00 = -3.08
def V(zz, R):
    zeta = zz / l - z00; eta = R / (l * S * zeta); return U_H * Bd / zeta / (1 + (np.sqrt(2) - 1) * eta ** 2) ** 2
def F_fine(zz, r0, nr=300, nt=720):
    rr = (np.arange(nr) + 0.5) * (0.072 / nr); th = (np.arange(nt) + 0.5) * (2 * np.pi / nt)
    Rg, Tg = np.meshgrid(rr, th, indexing="ij"); R = np.sqrt(r0 ** 2 + Rg ** 2 + 2 * r0 * Rg * np.cos(Tg))
    return 0.5 * C_d * rho * np.sum(V(zz, R) ** 2 * Rg) * (0.072 / nr) * (2 * np.pi / nt)
print("  U_H = %.3f m/s (hand value 4.23)" % U_H)
rows = []
for zz, r0 in [(0.2, 0.0), (0.2, 0.05), (0.3, 0.0), (0.3, 0.1), (0.5, 0.0), (0.5, 0.1), (0.5, 0.3), (0.8, 0.0), (0.8, 0.2)]:
    z_t = torch.tensor([zz], dtype=torch.float64); r_t = torch.tensor([r0], dtype=torch.float64)
    Fc = float(downwash_force_fo(z_t, r_t, torch.zeros(1, dtype=torch.float64), C_d=C_d, S=S)[0])
    Fa, dz_a, dr_a = [float(x[0]) for x in downwash_force_and_derivs_fo(z_t, r_t, C_d=C_d, S=S)]
    e = 1e-5
    dz_n = (float(downwash_force_fo(z_t + e, r_t, torch.zeros(1, dtype=torch.float64), C_d=C_d, S=S)[0]) - float(downwash_force_fo(z_t - e, r_t, torch.zeros(1, dtype=torch.float64), C_d=C_d, S=S)[0])) / (2 * e)
    dr_n = (float(downwash_force_fo(z_t, r_t + e, torch.zeros(1, dtype=torch.float64), C_d=C_d, S=S)[0]) - float(downwash_force_fo(z_t, torch.clamp(r_t - e, min=0), torch.zeros(1, dtype=torch.float64), C_d=C_d, S=S)[0])) / (e + float(min(e, r0)))
    Ff = F_fine(zz, r0)
    rows.append((zz, r0, Fc, Ff, (Fc - Ff) / Ff, Fa - Fc, dz_a, dz_n, dr_a, dr_n)); 
    print("  dz %.2f r0 %.2f | F code %.5f N  fine %.5f N  quad err %+.2f %% | F(analytic fn) - F(code) %.1e | dF/dz analytic %.4f num %.4f | dF/dr analytic %.4f num %.4f" % (rows[-1][0], rows[-1][1], rows[-1][2], rows[-1][3], 100 * rows[-1][4], rows[-1][5], rows[-1][6], rows[-1][7], rows[-1][8], rows[-1][9]))
OUT["force_quadrature"] = [dict(dz=a, r0=b, F_code=c, F_fine=d, rel_err=e_, d_dz_ana=g_, d_dz_num=h_, d_dr_ana=i_, d_dr_num=j_) for a, b, c, d, e_, f_, g_, h_, i_, j_ in rows]
print("  acceleration scale: F/m at dz 0.3, r0 0 = %.2f m/s² (ROM downwash on the lower drone)" % (rows[2][2] / 0.03))

# ---------------------------------------------------------------- 4. small-radius sweep
print("\n=== 4. r0 -> 0 sweep (dz = 0.5 m, relative lateral velocity 0.3 m/s along x, top fixed)")
print("   r0[m]   |f_nn local (x,y,z)|            acc_NN world (x,y,z)            jerk_NN world (x,y,z)       |jerk_nom|")
tab = []
for r0 in [0.3, 0.1, 0.03, 0.01, 0.003, 1e-3, 1e-4, 1e-5, 1e-6, 0.0]:
    pt = np.array([0, 0, 1.0]); pe = pt + np.array([r0, 0, -0.5]); ve = np.array([0.3, 0, 0]); vt = np.zeros(3)
    y = pair(pe, ve, pt, vt)
    with torch.no_grad():
        a_nn, j_nn, _, _, _, _ = (lambda t: (t[4], t[5], 0, 0, 0, 0))(M64_full.acceleration_split_jvp(y))
        j_nom = M64_nom.acceleration(y)[1]
        fl = net.net_f(torch.tensor([[r0, 0.5]], dtype=torch.float32))[0].numpy()
    tab.append((r0, fl, a_nn.numpy()[0], j_nn.numpy()[0], float(torch.norm(j_nom))))
    print("  %.0e  [% .3f % .3f % .3f]   [% .3f % .3f % .3f]   [% .3f % .3f % .3f]   %.3f" % (r0, *fl, *a_nn.numpy()[0], *j_nn.numpy()[0], float(torch.norm(j_nom))))
OUT["r0_sweep"] = [dict(r0=a, f_local=list(map(float, b)), acc_nn=list(map(float, c)), jerk_nn=list(map(float, d)), jerk_nom=e) for a, b, c, d, e in tab]
# same sweep, lateral velocity zero but vertical relative velocity (stack breathing), and tiny lateral jitter velocity
print("  --- jerk_NN norm vs r0 for different relative lateral velocities (dz = 0.5): ")
for vlat in (0.0, 0.05, 0.3):
    out = []
    for r0 in [0.1, 0.03, 0.01, 0.003, 1e-3, 1e-4]:
        pt = np.array([0, 0, 1.0]); pe = pt + np.array([r0, 0, -0.5]); ve = np.array([0.0, vlat, 0.0])
        with torch.no_grad(): out.append(float(torch.norm(M64_full.acceleration_split_jvp(pair(pe, ve, pt, np.zeros(3)))[5])))
    print("   v_lat(perp. to offset) %.2f m/s: |jerk_NN| =" % vlat, " ".join("%.2f" % x for x in out), "(r0 = 0.1, 0.03, 0.01, 0.003, 1e-3, 1e-4)")

# ---------------------------------------------------------------- 5. torque ROM frame consistency for a tilted flow axis
print("\n=== 5. ROM torque: world-frame z_bm used with flow-frame disk points (tilted flow axis) — consistency check")
def torque_ref(z0, r0, r_hat, z_bm, Cd=C_d, Sx=S):
    """independent reference: disk points in the flow frame F, rotate the result to the world with R_F^W; z_bm only defines F."""
    lq = l + 0.002; rr = (np.arange(60) + 0.5) * (0.0695 / 60); th = (np.arange(180) + 0.5) * (2 * np.pi / 180)
    Rg, Tg = np.meshgrid(rr, th, indexing="ij"); R = np.sqrt(r0 ** 2 + Rg ** 2 - 2 * r0 * Rg * np.cos(Tg))
    zeta = z0 / lq - z00; eta = R / (lq * Sx * zeta); Vv = U_H * Bd / zeta / (1 + (np.sqrt(2) - 1) * eta ** 2) ** 2
    px, py = Rg * np.cos(Tg), Rg * np.sin(Tg)
    w = Vv ** 2 * Rg * (0.0695 / 60) * (2 * np.pi / 180)
    tF = -np.array([np.sum(py * w), -np.sum(px * w), 0.0])  # −(p × e_z) with p = (px, py, 0): (py, −px, 0) → minus sign as in the code
    zf = z_bm / np.linalg.norm(z_bm); xv = r_hat / (np.linalg.norm(r_hat) + 1e-18); yv = np.cross(zf, xv); yv /= np.linalg.norm(yv); xv = np.cross(yv, zf); xv /= np.linalg.norm(xv)
    return 0.5 * C_d * rho * (np.stack([xv, yv, zf], axis=1) @ tF)
cases = []
for tilt in (0, 10, 20, 30):
    zb = np.array([0, np.sin(np.radians(tilt)), np.cos(np.radians(tilt))]); r_hat = np.array([0.12, 0.05, 0.45])
    # features for the tilted axis, as in the training wrapper (point_line_distance with the tilted z_bm)
    pt = np.zeros(3); pe = -r_hat; pv = pe - pt; proj = (pv @ zb) / (zb @ zb); closest = pt + proj * zb
    r0v = np.linalg.norm(pe - closest); z0v = np.linalg.norm(pt - closest)
    code = downwash_torque_fo(torch.tensor([z0v], dtype=torch.float64), torch.tensor([r0v], dtype=torch.float64), torch.zeros(1, dtype=torch.float64),
                              torch.tensor([r_hat], dtype=torch.float64), torch.tensor([zb], dtype=torch.float64), C_d=C_d, S=S).numpy()[0]
    ref = torque_ref(z0v, r0v, r_hat, zb)
    cases.append((tilt, np.linalg.norm(code), np.linalg.norm(ref), np.linalg.norm(code - ref) / (np.linalg.norm(ref) + 1e-12)))
    print("  tilt %2d deg: |tau_code| = %.3e  |tau_ref| = %.3e  relative difference %.1f %%" % cases[-1])
OUT["torque_tilt"] = [dict(tilt=a, code=b, ref=c, rel=d) for a, b, c, d in cases]

# ---------------------------------------------------------------- 6. timing
print("\n=== 6. timing on this CPU (single thread, torch %s)" % torch.__version__)
y32 = pair(np.array([0.05, 0.0, 0.5]), np.array([0.2, 0, 0]), np.array([0, 0, 1.0]), np.zeros(3)).float()
def t_it(fn, n=300):
    for _ in range(20): fn()
    t0 = time.perf_counter()
    for _ in range(n): fn()
    return (time.perf_counter() - t0) / n * 1e3
with torch.no_grad():
    T = {"residual forward (acc, jerk, ang_acc), ROM+NN": t_it(lambda: M_full(0., y32)),
         "  acceleration only": t_it(lambda: M_full.acceleration(y32)),
         "  angular acceleration only": t_it(lambda: M_full.angular_acceleration(y32)),
         "ROM only (acc, jerk, ang_acc)": t_it(lambda: M_nom(0., y32)),
         "NN only (acc, jerk, ang_acc)": t_it(lambda: M_nn(0., y32))}
for k, v in T.items(): print("  %-52s %.3f ms" % (k, v))
R3 = np.eye(3); kw = dict(rotation=R3, body_rates=np.zeros(3), acc_cmd=np.zeros(3), jerk_cmd=np.zeros(3), yaw=0., yaw_rate=0., residual_acc=np.array([0, 0, -3.0]),
                          residual_jerk=np.zeros(3), residual_ang_acc=np.zeros(3), mass=.03, gravity=9.81, inertia=np.diag([1.43e-5, 1.43e-5, 2.89e-5]), rate_gains=np.diag([238., 238., 140.]))
T["compute_fbl_command (numpy)"] = t_it(lambda: compute_fbl_command(**kw), 3000); print("  %-52s %.4f ms" % ("compute_fbl_command (numpy)", T["compute_fbl_command (numpy)"]))
OUT["timing_ms"] = T
json.dump(OUT, open(Path(__file__).parent / "out" / "part1_results.json", "w"), indent=1, default=float)
