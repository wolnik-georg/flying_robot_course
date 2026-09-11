#!/usr/bin/env python3
"""DShot-vs-deck RPM source investigation — analysis half.

Companion to `usd_dshot_investigation_config.txt` (both RPM sources logged
simultaneously, regardless of which one is actually feeding INDI via the new
`indi_gains.rpm_source` runtime switch — see traj_iface.c) and to
`docs/23_DShot_RPM_Investigation.md`, which has the full history and the flight
procedure this script's numbers are meant to answer.

Two things this script does NOT do, on purpose:
  - It does not fly anything. It reads a decoded uSD log after the fact.
  - It does not decide "DShot is fine" or "DShot is bad". It reports the four
    numbers the 2026-07-15..18 investigation actually used to make that call
    (gyro sigma, tau_x/y FFT peak frequency+amplitude, deck-vs-DShot measurement
    agreement, deck-vs-DShot lag) so a human makes that call from real numbers,
    not from a repeat of the "conservative gains didn't fix it" guesswork that
    cost three flights in July.

Usage:
    ~/.pyenv/versions/flying_robots/bin/python investigate_dshot_rpm.py <decoded.csv>
    ~/.pyenv/versions/flying_robots/bin/python investigate_dshot_rpm.py <usd-log-file>  # decodes first

Needs matplotlib/numpy -> pyenv env, same reason as plot_flight.py (system
matplotlib on this machine is broken, numpy 1.x/2.x ABI mismatch).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
import decode_usd_log as U  # noqa: E402

FS = 500.0  # uSD log rate, from usd_dshot_investigation_config.txt line 6


def load(path: str) -> dict:
    """Accepts either a raw uSD log file or an already-decoded CSV."""
    p = Path(path)
    if p.suffix == ".csv":
        import csv
        rows = list(csv.DictReader(open(p)))
        out = {}
        for k in rows[0]:
            out[k] = np.array([float(r[k]) for r in rows])
        return out
    return U.load(str(p))


def fft_peak(sig: np.ndarray, fs: float, band=(2.0, 15.0)):
    """Return (peak_freq_hz, peak_amplitude) of the dominant tone in `band`.

    Same band the 07-15..18 investigation tracked the limit cycle in (their
    observed range was 5.8-8.2 Hz; widened here to 2-15 Hz so a shifted peak is
    still caught rather than silently clipped by an assumed band).
    """
    sig = sig - np.mean(sig)
    n = len(sig)
    if n < 8:
        return float("nan"), float("nan")
    spec = np.abs(np.fft.rfft(sig * np.hanning(n)))
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    mask = (freqs >= band[0]) & (freqs <= band[1])
    if not mask.any():
        return float("nan"), float("nan")
    i = np.argmax(spec[mask])
    return float(freqs[mask][i]), float(spec[mask][i])


def cross_corr_lag(a: np.ndarray, b: np.ndarray, fs: float, max_lag_s: float = 0.05):
    """Lag [ms] that best aligns b to a, positive = b lags a. Direct measurement of
    the 'documented cmd->actuation delay' the CF21BL paper flags for DShot, instead
    of assuming a number.
    """
    a = a - np.mean(a)
    b = b - np.mean(b)
    max_lag = int(max_lag_s * fs)
    if len(a) < 4 * max_lag or len(b) < 4 * max_lag:
        return float("nan")
    corr = np.correlate(a, b, mode="full")
    lags = np.arange(-len(b) + 1, len(a))
    window = (lags >= -max_lag) & (lags <= max_lag)
    best = lags[window][np.argmax(corr[window])]
    # np.correlate's own lag sign is the OPPOSITE of "b lags a" (verified against a
    # synthetic signal with a known, injected delay before trusting this in the lab) --
    # negate so the docstring's stated convention is the one actually returned.
    return -best / fs * 1000.0


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    d = load(sys.argv[1])

    t = d.get("t")
    if t is None:
        sys.exit("no 't' column -- is this a decode_usd_log.py output?")
    dur = t[-1] - t[0]
    print(f"log duration: {dur:.1f} s, {len(t)} samples ({len(t)/dur:.0f} Hz effective)")

    # -- hover window: z > 0.5 m, same convention as check_flight.py -----------
    z = d.get("z")
    if z is not None and (z > 0.5).any():
        idx = np.where(z > 0.5)[0]
        a, b = idx[0], idx[-1]
        print(f"hover window: t={t[a]-t[0]:.1f}..{t[b]-t[0]:.1f}s ({b-a} samples)")
    else:
        a, b = 0, len(t)
        print("WARNING: no z>0.5 window found -- using the whole log")

    def seg(k):
        v = d.get(k)
        return None if v is None else v[a:b]

    print("\n=== 1. Physical oscillation (gyro) -- what the airframe actually did ===")
    for ax in ("gyro_x", "gyro_y", "gyro_z"):
        v = seg(ax)
        if v is None:
            print(f"  {ax}: not in log"); continue
        f0, amp = fft_peak(v, FS)
        print(f"  {ax}: sigma={np.std(v):6.2f} deg/s   peak {f0:5.2f} Hz  (amp {amp:.0f})")

    print("\n=== 2. Commanded-torque chain -- 'the clean loop signal' (07-16 finding) ===")
    for ax in ("tau_x", "tau_y", "tau_z"):
        v = seg(ax)
        if v is None:
            print(f"  {ax}: not in log"); continue
        f0, amp = fft_peak(v, FS)
        print(f"  {ax}: sigma={np.std(v):8.5f} Nm   peak {f0:5.2f} Hz  (amp {amp:.2f})")

    print("\n=== 3. Deck vs DShot -- do the two RPM sources agree, and by how much do they lag? ===")
    have_both = all(d.get(f"rpm_m{i}") is not None for i in (1, 2, 3, 4)) and \
                all(d.get(f"motor_m{i}_rpm") is not None for i in (1, 2, 3, 4))
    if not have_both:
        print("  Not both sources present in this log -- fly with "
              "usd_dshot_investigation_config.txt to get both simultaneously.")
    else:
        for i in (1, 2, 3, 4):
            rd = seg(f"rpm_m{i}")
            rs = seg(f"motor_m{i}_rpm")
            # DShot's own "invalid" sentinel (0xffff) is already zeroed by rpm_get_all()
            # for the value fed to INDI, but the raw log channel here is read straight from
            # the firmware log system, independent of that guard -- filter it here too.
            valid = (rd > 0) & (rs > 0) & (rs < 60000)
            if valid.sum() < 50:
                print(f"  m{i}: too few valid concurrent samples ({valid.sum()}) -- skip")
                continue
            rd_v, rs_v = rd[valid], rs[valid]
            bias = np.mean(rs_v - rd_v)
            pct = 100.0 * bias / np.mean(rd_v)
            lag_ms = cross_corr_lag(rd_v.astype(float), rs_v.astype(float), FS)
            print(f"  m{i}: deck={np.mean(rd_v):7.0f} rpm  dshot={np.mean(rs_v):7.0f} rpm  "
                  f"bias={bias:+7.1f} ({pct:+.2f}%)  measured lag={lag_ms:+.2f} ms")

    print("\n=== 4. Loop timing sanity (this session's filter-rate finding, docs/22 s2f) ===")
    dtus = seg("dt_us")
    if dtus is not None:
        print(f"  dt: mean={np.mean(dtus):.1f} us  std={np.std(dtus):.2f}  "
              f"min={np.min(dtus):.0f}  max={np.max(dtus):.0f}")
        print("  (expect a flat ~1000 -- filt_dt_us default of 2000 assumes 500 Hz, which this "
              "log's own dt_us column disproves or confirms directly, see docs/22 s2f)")
    else:
        print("  indi.dt_us not in log")

    # -- plot: the actual comparison, not just numbers -------------------------
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    tt = t[a:b] - t[a]

    ax0 = axes[0]
    for axis in ("gyro_x", "gyro_y"):
        v = seg(axis)
        if v is not None:
            ax0.plot(tt, v, lw=0.8, label=axis)
    ax0.set_ylabel("gyro [deg/s]"); ax0.legend(fontsize=8); ax0.set_title("Raw gyro")

    ax1 = axes[1]
    for axis in ("tau_x", "tau_y"):
        v = seg(axis)
        if v is not None:
            ax1.plot(tt, v, lw=0.8, label=axis)
    ax1.set_ylabel("tau [Nm]"); ax1.legend(fontsize=8); ax1.set_title("Commanded torque")

    ax2 = axes[2]
    if have_both:
        for i in (1,):
            rd = seg(f"rpm_m{i}"); rs = seg(f"motor_m{i}_rpm")
            if rd is not None:
                ax2.plot(tt, rd, lw=0.8, label=f"deck m{i}")
            if rs is not None:
                rs_plot = np.where(rs > 60000, np.nan, rs)
                ax2.plot(tt, rs_plot, lw=0.8, ls="--", label=f"dshot m{i}")
    ax2.set_ylabel("RPM"); ax2.legend(fontsize=8); ax2.set_title("Deck vs DShot, motor 1")

    ax3 = axes[3]
    e_r = seg("e_r_norm")
    if e_r is not None:
        ax3.plot(tt, e_r, lw=0.8, color="tab:red")
    ax3.set_ylabel("|e_R|"); ax3.set_xlabel("t [s]"); ax3.set_title("Geometric attitude error")

    fig.tight_layout()
    out = Path(sys.argv[1]).with_suffix("").as_posix() + "_dshot_investigation.png"
    fig.savefig(out, dpi=130)
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
