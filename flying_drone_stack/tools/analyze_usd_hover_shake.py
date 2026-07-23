#!/usr/bin/env python
"""Answers the four §16.5 questions from docs/investigation_indi_oscillation_2026-07-21.md
against a decoded USD hover log, and plots the results into docs/.

Usage:
  ~/.pyenv/versions/flying_robots/bin/python analyze_usd_hover_shake.py <path-to-log-file> <out.png>
"""
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import signal

from decode_usd_log import load


def bandpass(sig, f0, fs, bw=2.0):
    lo, hi = max(f0 - bw, 0.5), f0 + bw
    b, a = signal.butter(2, [lo, hi], btype="band", fs=fs)
    return signal.filtfilt(b, a, sig)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    d = load(sys.argv[1])
    out_path = sys.argv[2]

    t = d["t"]
    motor_mean = np.mean([d["motor_m1"], d["motor_m2"], d["motor_m3"], d["motor_m4"]], axis=0)
    active = motor_mean > 1000
    idx = np.where(active)[0]
    i0, i1 = idx[0] + 250, idx[-1] - 250  # drop 0.5s arm/land transients
    sl = slice(i0, i1)
    fs = 1.0 / np.median(np.diff(t))
    tt = t[sl] - t[sl][0]
    print(f"Steady-hover window: {tt[-1] - tt[0]:.1f}s at {fs:.1f}Hz")

    # Dominant shake frequency
    gx = d["gyro_x"][sl] - d["gyro_x"][sl].mean()
    freqs = np.fft.rfftfreq(len(tt), 1 / fs)
    psd = np.abs(np.fft.rfft(gx)) ** 2
    band = (freqs > 3) & (freqs < 12)
    f_peak = freqs[band][np.argmax(psd[band])]
    print(f"Dominant gyro_x oscillation freq: {f_peak:.2f} Hz")

    # steady-state sub-window (excludes the amplitude-onset ramp seen 2026-07-23)
    onset = (tt > 7) & (tt < min(tt[-1], 25))

    fig, axs = plt.subplots(2, 3, figsize=(18, 9))
    fig.suptitle("USD 500Hz diagnostic — hover shake analysis")

    zoom = (tt > 10) & (tt < 12)
    axs[0, 0].plot(tt[zoom], d["alp_raw_x"][sl][zoom], label="alp_raw_x", alpha=0.7, lw=1)
    axs[0, 0].plot(tt[zoom], d["alp_x"][sl][zoom], label="alp_x (filtered)", lw=1.5)
    axs[0, 0].set_title("Q1: Filter aliasing")
    axs[0, 0].set_xlabel("t [s]"); axs[0, 0].set_ylabel("alp_x [rad/s²]")
    axs[0, 0].legend(fontsize=8); axs[0, 0].grid(alpha=0.3)

    Araw = np.abs(np.fft.rfft(d["alp_raw_x"][sl] - d["alp_raw_x"][sl].mean()))
    Afilt = np.abs(np.fft.rfft(d["alp_x"][sl] - d["alp_x"][sl].mean()))
    fband = freqs < 25
    ratio = Afilt[np.argmin(np.abs(freqs - f_peak))] / Araw[np.argmin(np.abs(freqs - f_peak))]
    axs[0, 1].plot(freqs[fband], Araw[fband], label="alp_raw_x", alpha=0.7, lw=1)
    axs[0, 1].plot(freqs[fband], Afilt[fband], label="alp_x", lw=1.5)
    axs[0, 1].axvline(f_peak, color="r", ls="--", alpha=0.5, label=f"{f_peak:.2f}Hz peak")
    axs[0, 1].set_title(f"Q1: Spectrum (filt/raw ratio={ratio:.2f} @ peak)")
    axs[0, 1].set_xlabel("Hz"); axs[0, 1].set_ylabel("|FFT|")
    axs[0, 1].legend(fontsize=8); axs[0, 1].grid(alpha=0.3)

    roll = d["roll_deg"][sl]
    dt = np.diff(tt, prepend=tt[0] - 1 / fs)
    roll_dr = np.cumsum(d["gyro_x"][sl] * dt)
    roll_dr -= roll_dr.mean()
    roll_bp = bandpass(roll - roll.mean(), f_peak, fs)
    dr_bp = bandpass(roll_dr, f_peak, fs)
    corr = signal.correlate(roll_bp, dr_bp, mode="full")
    lags = signal.correlation_lags(len(roll_bp), len(dr_bp))
    lag_ms = lags[np.argmax(corr)] / fs * 1000
    zoom2 = (tt > 10) & (tt < 11.5)
    axs[1, 0].plot(tt[zoom2], dr_bp[zoom2], label="gyro dead-reckoned roll", alpha=0.8, lw=1.3)
    axs[1, 0].plot(tt[zoom2], roll_bp[zoom2], label="stabilizer.roll (EKF)", lw=1.3)
    axs[1, 0].set_title(f"Q2: EKF attitude lag = {lag_ms:.1f}ms @ {f_peak:.2f}Hz")
    axs[1, 0].set_xlabel("t [s]"); axs[1, 0].set_ylabel("roll [deg, bandpassed]")
    axs[1, 0].legend(fontsize=8); axs[1, 0].grid(alpha=0.3)
    print(f"Q2: roll lags gyro dead-reckoning by {lag_ms:.1f} ms")

    moc = d["motor_m1"][sl][onset] - d["motor_m1"][sl][onset].mean()
    rpc = d["rpm_m1"][sl][onset] - d["rpm_m1"][sl][onset].mean()
    moc /= moc.std(); rpc /= rpc.std()
    xcorr = signal.correlate(rpc, moc, mode="full") / len(moc)
    lagsamp = signal.correlation_lags(len(rpc), len(moc))
    lag_ms_axis = lagsamp / fs * 1000
    win = (lag_ms_axis > -100) & (lag_ms_axis < 100)
    peak_i = np.argmax(xcorr)
    axs[1, 1].plot(lag_ms_axis[win], xcorr[win], lw=1.3)
    axs[1, 1].axvline(lag_ms_axis[peak_i], color="r", ls="--", label=f"peak @ {lag_ms_axis[peak_i]:.1f}ms")
    axs[1, 1].set_title("Q3: motor.m1 -> rpm.m1 cross-correlation")
    axs[1, 1].set_xlabel("lag [ms]"); axs[1, 1].set_ylabel("normalized correlation")
    axs[1, 1].legend(fontsize=8); axs[1, 1].grid(alpha=0.3)

    zoom4 = (tt > 10) & (tt < 10.05)
    axs[0, 2].step(tt[zoom4] * 1000, d["rpm_m1"][sl][zoom4], where="post", marker="o", ms=3)
    axs[0, 2].set_title(f"Q4: RPM update rate ~{fs:.0f}Hz")
    axs[0, 2].set_xlabel("t [ms]"); axs[0, 2].set_ylabel("rpm_m1")
    axs[0, 2].grid(alpha=0.3)

    lag_by_motor = []
    for m in ["1", "2", "3", "4"]:
        moc2 = d[f"motor_m{m}"][sl][onset] - d[f"motor_m{m}"][sl][onset].mean()
        rpc2 = d[f"rpm_m{m}"][sl][onset] - d[f"rpm_m{m}"][sl][onset].mean()
        c2 = signal.correlate(rpc2, moc2, mode="full")
        l2 = signal.correlation_lags(len(rpc2), len(moc2))
        lag_by_motor.append(l2[np.argmax(c2)] / fs * 1000)
    axs[1, 2].bar(["m1", "m2", "m3", "m4"], lag_by_motor, color="steelblue")
    axs[1, 2].set_title("Q3: Actuator lag by motor [ms]")
    axs[1, 2].set_ylabel("lag [ms]")
    for i, v in enumerate(lag_by_motor):
        axs[1, 2].text(i, v + 0.5, f"{v:.1f}", ha="center", fontsize=9)
    axs[1, 2].grid(alpha=0.3, axis="y")
    print(f"Q3: motor->rpm lag by motor [ms]: {lag_by_motor}")

    changes = np.where(np.diff(d["rpm_m1"][sl][onset]) != 0)[0]
    if len(changes) > 1:
        step_intervals = np.diff(changes) / fs
        print(f"Q4: rpm_m1 true update rate ~{1 / np.median(step_intervals):.1f} Hz")

    plt.tight_layout()
    plt.savefig(out_path, dpi=130)
    print("saved", out_path)


if __name__ == "__main__":
    main()
