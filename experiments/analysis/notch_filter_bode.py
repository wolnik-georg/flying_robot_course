#!/usr/bin/env python3
"""
Offline bode/phase analysis of the INDI notch filter chain, ported directly from
firmware_app/src/lib.rs's Butterworth2/NotchFilter coefficient formulas (same math,
same discretisation) -- no flight data needed, this is a pure filter-design check.

Chain under notch_en=1 (see lib.rs ~1795-1948): each of alpha_meas, alpha_ref,
tau_current passes through Butterworth2(fc_bw) THEN NotchFilter(notch_f0, notch_bw)
in series, at the real firmware tick rate (1 kHz, NOT the 500 Hz some filters were
originally designed around).
"""
import numpy as np

FS = 1000.0          # real firmware tick rate, confirmed 1 kHz (not 500 Hz)
DT = 1.0 / FS

# Current yaml values (flying_robot_course session, 2026-09-13)
FC_BW = 206.0
FILT_PREWARP = True
NOTCH_F0 = 6.9
NOTCH_BW = 3.0

# Attitude loop's own closed-loop dynamics (kr=2400, kw=170)
KR, KW = 2400.0, 170.0
WN = np.sqrt(KR)                  # rad/s
ZETA = KW / (2 * WN)
F_LOOP = WN / (2 * np.pi)         # Hz


def butterworth2_coeffs(fc, dt, prewarp=True):
    tau = 1.0 / (2 * np.pi * fc)
    if prewarp:
        q = 0.7071
        k = np.tan(dt / (2 * tau))
        poly = k * k + k / q + 1.0
        b = k * k / poly
        a1 = 2 * (k * k - 1.0) / poly
        a2 = (k * k - k / q + 1.0) / poly
    else:
        SQRT2 = np.sqrt(2.0)
        denom = tau * tau + SQRT2 * tau * dt + dt * dt
        b = dt * dt / denom
        a1 = 2 * (dt * dt - tau * tau) / denom
        a2 = (tau * tau - SQRT2 * tau * dt + dt * dt) / denom
    # numerator b*(1 + 2z^-1 + z^-2), denominator 1 + a1 z^-1 + a2 z^-2
    return np.array([b, 2 * b, b]), np.array([1.0, a1, a2])


def notch_coeffs(f0, bw, dt):
    w0 = 2 * np.pi * f0 * dt
    q = max(f0 / max(bw, 0.1), 0.1)
    alpha = np.sin(w0) / (2 * q)
    cos_w0 = np.cos(w0)
    a0 = 1.0 + alpha
    b0 = 1.0 / a0
    b1 = -2 * cos_w0 / a0
    b2 = 1.0 / a0
    a1 = -2 * cos_w0 / a0
    a2 = (1.0 - alpha) / a0
    return np.array([b0, b1, b2]), np.array([1.0, a1, a2])


def freqz_manual(b, a, freqs_hz, dt):
    """H(e^{jwT}) evaluated directly -- no scipy dependency."""
    w = 2 * np.pi * freqs_hz * dt
    z_inv = np.exp(-1j * w)
    num = sum(bk * z_inv**k for k, bk in enumerate(b))
    den = sum(ak * z_inv**k for k, ak in enumerate(a))
    return num / den


def mag_phase(H):
    return np.abs(H), np.degrees(np.unwrap(np.angle(H)))


freqs = np.linspace(0.5, 25, 500)

b_bw, a_bw = butterworth2_coeffs(FC_BW, DT, FILT_PREWARP)
b_nt, a_nt = notch_coeffs(NOTCH_F0, NOTCH_BW, DT)

H_bw = freqz_manual(b_bw, a_bw, freqs, DT)
H_nt = freqz_manual(b_nt, a_nt, freqs, DT)
H_series = H_bw * H_nt  # BW then notch, in series -- matches the firmware order exactly

mag_bw, ph_bw = mag_phase(H_bw)
mag_nt, ph_nt = mag_phase(H_nt)
mag_s, ph_s = mag_phase(H_series)

print(f"Attitude loop: wn={WN:.2f} rad/s = {F_LOOP:.2f} Hz, zeta={ZETA:.3f}")
print(f"Sample rate assumed: {FS:.0f} Hz (dt={DT*1e6:.0f} us)")
print()
print(f"{'f (Hz)':>8} | {'BW mag':>8} {'BW deg':>8} | {'Notch mag':>9} {'Notch deg':>9} | {'SERIES mag':>10} {'SERIES deg':>10}")
for f_target in [1, 2, 3, 4, 5, 6, 6.9, 7, F_LOOP, 8, 9, 10, 12, 15, 20]:
    i = np.argmin(np.abs(freqs - f_target))
    print(f"{freqs[i]:8.2f} | {mag_bw[i]:8.3f} {ph_bw[i]:8.1f} | {mag_nt[i]:9.3f} {ph_nt[i]:9.1f} | {mag_s[i]:10.3f} {ph_s[i]:10.1f}")

print()
i_loop = np.argmin(np.abs(freqs - F_LOOP))
print(f"At the loop's own bandwidth ({F_LOOP:.2f} Hz):")
print(f"  Notch alone:  {mag_nt[i_loop]:.3f}x magnitude, {ph_nt[i_loop]:.1f} deg phase")
print(f"  BW+Notch series: {mag_s[i_loop]:.3f}x magnitude, {ph_s[i_loop]:.1f} deg phase")
print(f"  (compare: the fc_bw=100 rung that also crashed added measured phase lag in the")
print(f"   same low-frequency range -- this is the same class of concern)")
