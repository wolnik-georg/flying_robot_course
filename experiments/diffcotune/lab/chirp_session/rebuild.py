"""REPORT_51 U2: rebuild of the INDI correction (tau_rpm_f - tau_gyro_f, roll / pitch) of controller_omar_indi.c (the flashed image fw/ tree, lines 391-400 + 627-652) from quantities that are in the existing uSD logs:
 rpm telemetry `motor.m1..m4_rpm` (raw DShot, 500 Hz rows) -> rpm_filter_step (DShot rules: reject > 28000, slew > 10000 vs the previous accepted value, 0xFFFF -> 0; no hold) -> t_i = MOTORRPM2FORCE (2 pi rpm / 60)^2 -> tau_x = arm(-t1 - t2 + t3 + t4), tau_y = arm(-t1 + t2 + t3 - t4), arm = 0.70710678 ARM_LENGTH (0.050 m, cf21bl) -> 2nd-order Butterworth 30 Hz at 500 Hz (filter.h: K = tan(dt / 2 tau), Q = 0.7071) = tau_rpm_f;
 gyro (deg/s -> rad/s) -> (w - w_prev) / dt -> same Butterworth -> times J (controller J: 1.657e-5 / 1.666e-5 kg m^2) = tau_gyro_f.   The firmware runs this only when indi & 2 and the rpm log variables exist (S1, S3); for S0 / S2 both terms stay zero."""
import numpy as np
MOTORRPM2FORCE = 3.91194027330774e-08; ARM = 0.707106781 * 0.050; J = np.array([1.65717101e-05, 1.66556019e-05])
def rpm_filter(raw, slew=10000, amax=28000):
    out = np.zeros(len(raw)); prev = 0
    for i, v in enumerate(raw):
        v = int(v) if np.isfinite(v) else 0
        if v == 0xFFFF: v = 0
        if v > 0:
            rej = v > amax or (prev > 500 and abs(v - prev) > slew)
            if rej and prev > 0: v = prev
            elif not rej: prev = v
        out[i] = v
    return out
def butter2(x, dt=0.002, fc=30.0):
    tau = 1.0 / (2 * np.pi * fc); K = np.tan(dt / (2 * tau)); poly = K * K + K / 0.7071 + 1.0; a0 = 2 * (K * K - 1) / poly; a1 = (K * K - K / 0.7071 + 1) / poly; b0 = K * K / poly; b1 = 2 * b0
    y = np.zeros(len(x)); i0 = i1 = o0 = o1 = 0.0
    for k, v in enumerate(x):
        o = b0 * v + b1 * i0 + b0 * i1 - a0 * o0 - a1 * o1; i1 = i0; i0 = v; o1 = o0; o0 = o; y[k] = o
    return y
def rebuild(rpm4, gyro_deg, dt=0.002):
    """rpm4 (n, 4) raw rpm, gyro_deg (n, 3) deg/s -> tau_rpm_f (n, 2), tau_gyro_f (n, 2) [N m]"""
    t = [MOTORRPM2FORCE * (rpm_filter(rpm4[:, i]) * 2 * np.pi / 60.0) ** 2 for i in range(4)]
    tau = np.stack([ARM * (-t[0] - t[1] + t[2] + t[3]), ARM * (-t[0] + t[1] + t[2] - t[3])], 1)
    w = np.radians(gyro_deg[:, :2]); aa = np.zeros_like(w); aa[1:] = (w[1:] - w[:-1]) / dt
    return np.stack([butter2(tau[:, 0]), butter2(tau[:, 1])], 1), np.stack([J[0] * butter2(aa[:, 0]), J[1] * butter2(aa[:, 1])], 1)
