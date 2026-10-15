#!/usr/bin/env python3
"""Which strategy did a uSD log of the Omar C tree actually fly? (S0..S3), from the log itself.
  bit 0 (position INDI) : a_rpm_f / a_imu_f are only computed when it is on -> non-zero iff on.
  NS2 term (rnn.en)     : thrust = m (a_d + a_indi - a_nn).R e3. a_d is rebuilt from the logged feedforward, errors and gains;
                          the slow vertical integral is removed by a high-pass; the slope of the remainder on the logged
                          prediction is ~ -1 with rnn.en = 1 and ~ 0 with rnn.en = 0 (only meaningful when the prediction
                          varies, i.e. when a neighbour passes through the gate).
  python3 experiments/analysis/tree_log_check.py <log.bin> [<log.bin> ...]       (system python3 not needed)"""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "flying_drone_stack/tools"))
import decode_usd_log as D

M, G, KP, KV = 0.0427, 9.81, 7.0, 4.0


def hp(x, n=1000):  # subtract a ~2 s moving average at 500 Hz
    k = np.ones(n) / n
    return x - np.convolve(np.pad(x, (n // 2, n - n // 2 - 1), mode="edge"), k, mode="valid")


def check(path):
    d = D.load(path)
    air = np.asarray(d["z"]) > 0.30
    if air.sum() < 500:
        return f"{Path(path).name}: not enough airborne samples"
    bit0 = float(np.mean(np.abs(np.asarray(d["omar_a_rpm_fz"])[air]) > 1e-6))
    pred = np.asarray(d["rnn_pred_z"])
    thr = np.asarray(d["omar_thrust_si"])
    roll, pitch = np.asarray(d["omar_rpy_x"]), np.asarray(d["omar_rpy_y"])
    rzz = np.cos(roll) * np.cos(pitch)
    a_ff = np.asarray(d["ctrltarget_az"])
    a_d_z = a_ff + G + KV * np.asarray(d["omar_err_vel_z"]) + KP * np.asarray(d["omar_err_pos_z"])
    a_indi_z = np.asarray(d["omar_a_rpm_fz"]) - np.asarray(d["omar_a_imu_fz"])          # in m/s^2 (logged filtered values)
    y = thr / M / np.maximum(rzz, 0.5) - a_d_z - a_indi_z                                   # = -a_nn + I_z(t) + model error
    ok = air & (np.abs(pred) > 0)
    yh, ph = hp(y)[ok], hp(pred)[ok]
    var = float(np.var(ph))
    slope = float(np.dot(yh, ph) / np.dot(ph, ph)) if var > 1e-6 else float("nan")
    corr = float(np.corrcoef(yh, ph)[0, 1]) if var > 1e-6 else float("nan")
    s_indi = "INDI on (bit 0)" if bit0 > 0.5 else "INDI off"
    s_ns2 = ("NS2 on" if slope < -0.5 else "NS2 off" if abs(slope) < 0.25 else "NS2 unclear") if np.isfinite(slope) else "NS2: prediction constant (cannot tell)"
    return (f"{Path(path).name[:60]}: {s_indi} (a_rpm_f active {100 * bit0:.0f} % of airborne); {s_ns2}  "
            f"[slope of thrust residual on prediction {slope:+.2f}, corr {corr:+.2f}, pred std {np.sqrt(var):.2f} m/s^2]")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        try:
            print(check(p))
        except Exception as e:
            print(f"{Path(p).name}: ERROR {e}")
