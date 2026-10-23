#!/usr/bin/env python3
"""Was the NS2 term applied (rnn.en = 1) in a uSD log of the Omar C tree?  Better than tree_log_check.py for geometric (INDI off) hold flights, where
the prediction (which depends on the own height) is confounded with the position error.  Model of the logged thrust:
   thrust/(m R_zz) = a_ff,z + g + Kd e_v,z + Kp e_p,z + Kiz * J(t) - s * a_nn,z + c        J = cumulative integral of e_p,z (integral term, constant c = unknown start value)
Least squares on [1, J, pred]: the coefficient of J must come out near Kpos_Iz (1.5) (sanity of the model), the coefficient of the prediction is -s: about -1 when NS2 is applied, about 0 when not.
  python3 experiments/analysis/ns2_applied_check.py <log.bin> [...]"""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "flying_drone_stack/tools"))
import decode_usd_log as D
M, G, KP, KV = 0.0427, 9.81, 7.0, 4.0

def fit(path):
    d = D.load(path); t = np.asarray(d["t"], float); dt = np.median(np.diff(t))
    t = t - t[0]; air = np.asarray(d["z"]) > 0.30
    if air.sum() < 1000: return None
    thr = np.asarray(d["omar_thrust_si"]); ph, th, ps = (np.asarray(d["omar_rpy_x"]), np.asarray(d["omar_rpy_y"]), np.asarray(d["omar_rpy_z"]))
    zb = np.c_[np.cos(ph) * np.sin(th) * np.cos(ps) + np.sin(ph) * np.sin(ps), np.cos(ph) * np.sin(th) * np.sin(ps) - np.sin(ph) * np.cos(ps), np.cos(ph) * np.cos(th)]   # body z axis in the world (ZYX euler)
    def g(n): return np.asarray(d[n])
    ad = np.c_[g("ctrltarget_ax") + KV * g("omar_err_vel_x") + KP * g("omar_err_pos_x"), g("ctrltarget_ay") + KV * g("omar_err_vel_y") + KP * g("omar_err_pos_y"),
               g("ctrltarget_az") + G + KV * g("omar_err_vel_z") + KP * g("omar_err_pos_z")]      # position-loop acceleration without the integral and without NS2 (thrust = m a . z_b)
    ep = g("omar_err_pos_z"); pred = g("rnn_pred_z")
    est_logged = np.abs(g("omar_a_rpm_fz")[air]).mean() > 1e-6
    J = np.cumsum(np.where(air, ep, 0.0)) * dt; w = zb[:, 2]      # y = z_b,z * (c + Kiz J - s a_nn)
    ok = air
    corr = (np.c_[g("omar_a_rpm_fx") - g("omar_a_imu_fx"), g("omar_a_rpm_fy") - g("omar_a_imu_fy"), g("omar_a_rpm_fz") - g("omar_a_imu_fz")] * zb).sum(1)
    X = np.c_[w[ok], w[ok] * (J[ok] - J[ok][0]), w[ok] * pred[ok]]
    def run(indi):
        y = thr / M - (ad * zb).sum(1) - (corr if indi else 0.0)
        coef, *_ = np.linalg.lstsq(X, y[ok], rcond=None); res = y[ok] - X @ coef
        return coef, res, y
    # Old images log the INDI estimate only with INDI on (zero otherwise). The estlog image (2026-10-22) logs it under every strategy, so then the
    # flag is decided by which thrust model explains the logged thrust better (residual variance).
    if not est_logged: indi = False
    else: indi = bool(run(True)[1].var() < run(False)[1].var())
    coef, res, y = run(indi); r2 = 1 - res.var() / y[ok].var()
    X0 = X[:, :2]; c0, *_ = np.linalg.lstsq(X0, y[ok], rcond=None); res0 = y[ok] - X0 @ c0
    return dict(indi=bool(indi), coef_pred=float(coef[2]), coef_J=float(coef[1]), r2=float(r2), r2_without_pred=float(1 - res0.var() / y[ok].var()))


def verdict(f):
    return "NS2 APPLIED" if f["coef_pred"] < -0.6 else "NS2 NOT applied" if f["coef_pred"] > -0.3 else "unclear"


def check(path):
    f = fit(path)
    if f is None: return f"{Path(path).name[:55]}: not enough airborne samples"
    return (f"{Path(path).name[:58]:58s} INDI {'on ' if f['indi'] else 'off'} | coef(pred) {f['coef_pred']:+.2f}  coef(J) {f['coef_J']:+.2f} (expect ~1.5 with INDI, 1.5 geometric)  R2 {f['r2']:.2f} (without pred {f['r2_without_pred']:.2f})  => {verdict(f)}")




if __name__ == "__main__":
    for p in sys.argv[1:]:
        try: print(check(p))
        except Exception as e: print(f"{Path(p).name}: ERROR {e}")
