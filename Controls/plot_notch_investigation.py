#!/usr/bin/env python3
"""Compare pre/post-notch alpha chain (raw -> Butterworth -> notch) directly stacked
with commanded tau on a shared time axis, so the filter's effect and its consequence
on commanded torque can be read together.

Run: ~/.pyenv/versions/flying_robots/bin/python Controls/plot_notch_investigation.py
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

LOG_DIR = Path(__file__).parent / "logs"

FLIGHTS = [
    ("hover_mode0_2026-07-29_18-44-24.csv", "notch_en=0 (baseline, post-fix)"),
    ("hover_mode0_2026-07-29_18-20-38.csv", "notch_en=1 f0=6.9 bw=3 (stable)"),
    ("hover_mode0_2026-07-29_18-49-47.csv", "notch_en=1 f0=6.9 bw=1 (stable, worse)"),
    ("hover_mode0_2026-07-29_18-50-54.csv", "notch_en=1 f0=6.9 bw=0.5 (stable)"),
]


def load(fname):
    df = pd.read_csv(LOG_DIR / fname, comment="#")
    meta = {}
    with open(LOG_DIR / fname) as f:
        for line in f:
            if not line.startswith("#"):
                break
            if line.startswith("# meta:"):
                k, _, v = line[7:].strip().partition("=")
                meta[k] = v
    df = df.copy()
    df["time_s"] = df["time_s"] - df["time_s"].iloc[0]
    return df, meta


def make_figure(xlim, out_name, title_suffix):
    fig, axes = plt.subplots(len(FLIGHTS), 2, figsize=(16, 3.2 * len(FLIGHTS)), sharex=False)

    loaded = []
    alpha_max = 0.0
    tau_max = 0.0
    for fname, label in FLIGHTS:
        df, meta = load(fname)
        t = df["time_s"].values
        mask = (t >= xlim[0]) & (t <= xlim[1])
        loaded.append((df, meta, t))
        alpha_max = max(alpha_max, df["alp_raw_x"].values[mask].__abs__().max(),
                         df["alp_x"].values[mask].__abs__().max(),
                         df["alp_notch_x"].values[mask].__abs__().max())
        tau_max = max(tau_max, df["tau_x"].values[mask].__abs__().max(),
                      df["tau_y"].values[mask].__abs__().max())
    alpha_max *= 1.05
    tau_max *= 1.05

    for row, (fname, label) in enumerate(FLIGHTS):
        df, meta, t = loaded[row]

        ax_alpha = axes[row, 0]
        ax_alpha.plot(t, df["alp_raw_x"], label="alp_raw (pre-filter)", color="tab:gray", alpha=0.6, lw=0.7)
        ax_alpha.plot(t, df["alp_x"], label="alp (Butterworth)", color="tab:orange", alpha=0.85, lw=0.9)
        ax_alpha.plot(t, df["alp_notch_x"], label="alp_notch (post-notch)", color="tab:green", alpha=0.95, lw=1.0)
        ax_alpha.axvline(6.08, color="k", ls="--", lw=0.8, alpha=0.6)
        ax_alpha.set_title(f"{label}  (bw={meta.get('indi_notch_bw','?')}, en={meta.get('indi_notch_en','?')})\nalpha_x filter chain")
        ax_alpha.legend(fontsize=6, loc="upper right")
        ax_alpha.set_ylabel("rad/s^2")
        ax_alpha.set_xlim(*xlim)
        ax_alpha.set_ylim(-alpha_max, alpha_max)

        ax_tau = axes[row, 1]
        ax_tau.plot(t, df["tau_x"], label="tau_x", color="tab:blue", lw=0.9)
        ax_tau.plot(t, df["tau_y"], label="tau_y", color="tab:red", lw=0.9)
        ax_tau.axvline(6.08, color="k", ls="--", lw=0.8, alpha=0.6, label="ctrl switch ~6.08s")
        ax_tau.set_title("commanded torque (same time axis)")
        ax_tau.legend(fontsize=6, loc="upper right")
        ax_tau.set_ylabel("N*m")
        ax_tau.set_xlim(*xlim)
        ax_tau.set_ylim(-tau_max, tau_max)

    for ax in axes[-1, :]:
        ax.set_xlabel("time [s]")

    fig.suptitle(f"Notch filter chain vs commanded torque — 2026-07-29 {title_suffix}\n(shared y-axis scale across all rows)", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out = LOG_DIR / out_name
    fig.savefig(out, dpi=140)
    print(f"saved {out}")


# Full-duration view
make_figure(xlim=(0, 27), out_name="notch_chain_vs_tau_full.png", title_suffix="(full flight)")

# Zoomed view around the geometric->INDI handover, where the divergence starts
make_figure(xlim=(4, 10), out_name="notch_chain_vs_tau_zoom.png", title_suffix="(zoom on handover, t=4-10s)")


# ── Spectral view: where the raw/BW/notch overlap in time-domain, PSD shows the cut ──
import numpy as np

PSD_FLIGHTS = [
    ("hover_mode0_2026-07-29_18-44-24.csv", "notch_en=0 (baseline, post-fix)"),
    ("hover_mode0_2026-07-29_18-20-38.csv", "notch_en=1 f0=6.9 bw=3 (stable)"),
    ("hover_mode0_2026-07-29_18-49-47.csv", "notch_en=1 f0=6.9 bw=1 (stable, worse)"),
    ("hover_mode0_2026-07-29_18-50-54.csv", "notch_en=1 f0=6.9 bw=0.5 (stable)"),
]

from scipy.signal import welch

def psd(v, dt):
    v = v - np.mean(v)
    fs = 1.0 / dt
    freqs, pxx = welch(v, fs=fs, nperseg=256, noverlap=192)
    return freqs, pxx

fig, axes = plt.subplots(len(PSD_FLIGHTS), 2, figsize=(14, 4.5 * len(PSD_FLIGHTS)))

psd_loaded = []
alpha_pmax = 0.0
tau_pmax = 0.0
for fname, label in PSD_FLIGHTS:
    df, meta = load(fname)
    dt = np.median(np.diff(df["time_s"].values))
    psd_loaded.append((df, meta, dt))
    for col in ("alp_raw_x", "alp_x", "alp_notch_x"):
        f_, p_ = psd(df[col].values, dt)
        alpha_pmax = max(alpha_pmax, p_[(f_ >= 0) & (f_ <= 20)].max())
    for col in ("tau_x", "tau_y"):
        f_, p_ = psd(df[col].values, dt)
        tau_pmax = max(tau_pmax, p_[(f_ >= 0) & (f_ <= 20)].max())

for row, (fname, label) in enumerate(PSD_FLIGHTS):
    df, meta, dt = psd_loaded[row]

    ax = axes[row, 0]
    for col, name, color in [
        ("alp_raw_x", "alp_raw (pre-filter)", "tab:gray"),
        ("alp_x", "alp (Butterworth)", "tab:orange"),
        ("alp_notch_x", "alp_notch (post-notch)", "tab:green"),
    ]:
        f_, p_ = psd(df[col].values, dt)
        ax.semilogy(f_, p_, label=name, color=color, lw=1.0)
    ax.axvspan(4.7, 9.7, color="red", alpha=0.08, label="target notch band")
    ax.set_xlim(0, 20)
    ax.set_ylim(1e0, alpha_pmax * 2)
    ax.set_title(f"{label} (bw={meta.get('indi_notch_bw')})\nalpha_x PSD")
    ax.legend(fontsize=7)
    ax.set_xlabel("Hz")

    ax = axes[row, 1]
    for col, color in [("tau_x", "tab:blue"), ("tau_y", "tab:red")]:
        f_, p_ = psd(df[col].values, dt)
        ax.semilogy(f_, p_, label=col, color=color, lw=1.0)
    ax.axvspan(4.7, 9.7, color="red", alpha=0.08, label="target notch band")
    ax.set_xlim(0, 20)
    ax.set_ylim(1e-9, tau_pmax * 2)
    ax.set_title("commanded torque PSD")
    ax.legend(fontsize=7)
    ax.set_xlabel("Hz")

fig.suptitle("Notch filter effect in frequency domain — 2026-07-29\n(shared y-axis scale across rows)", fontsize=14)
fig.tight_layout(rect=[0, 0, 1, 0.96])
out = LOG_DIR / "notch_psd_comparison.png"
fig.savefig(out, dpi=140)
print(f"saved {out}")


# ── Bode plot: theoretical magnitude/phase response of the notch filter itself,
# to explain WHY a wider notch_bw behaved worse, not better ──────────────────
def notch_coeffs(f0, bw, dt):
    w0 = 2 * np.pi * f0 * dt
    Q = f0 / bw
    alpha = np.sin(w0) / (2 * Q)
    a0 = 1 + alpha
    b0 = 1 / a0
    b1 = -2 * np.cos(w0) / a0
    b2 = 1 / a0
    a1 = -2 * np.cos(w0) / a0
    a2 = (1 - alpha) / a0
    return b0, b1, b2, a1, a2


def freqresp(coeffs, freqs, dt):
    b0, b1, b2, a1, a2 = coeffs
    w = 2 * np.pi * freqs * dt
    z = np.exp(1j * w)
    num = b0 + b1 * z**-1 + b2 * z**-2
    den = 1 + a1 * z**-1 + a2 * z**-2
    return num / den


dt_ctrl = 0.002
freqs_bode = np.linspace(0.1, 25, 800)
fig, (ax_mag, ax_phase) = plt.subplots(2, 1, figsize=(9, 8), sharex=True)
for bw, color in [(5, "tab:blue"), (10, "tab:orange"), (20, "tab:green")]:
    H = freqresp(notch_coeffs(7.2, bw, dt_ctrl), freqs_bode, dt_ctrl)
    ax_mag.plot(freqs_bode, 20 * np.log10(np.abs(H)), color=color, label=f"notch_bw={bw} (Q={7.2/bw:.2f})")
    phase = np.angle(H, deg=True)
    ax_phase.plot(freqs_bode, phase, color=color, label=f"notch_bw={bw}")

ax_mag.axvline(6.9, color="k", ls=":", lw=1, alpha=0.6, label="observed peak 6.9Hz")
ax_phase.axvline(6.9, color="k", ls=":", lw=1, alpha=0.6)
ax_mag.set_ylabel("Magnitude [dB]")
ax_mag.set_ylim(-40, 2)
ax_mag.legend(fontsize=8)
ax_mag.set_title("Notch filter frequency response vs bandwidth (f0=7.2Hz)")
ax_mag.grid(alpha=0.3)

ax_phase.set_ylabel("Phase [deg]")
ax_phase.set_xlabel("Hz")
ax_phase.axhline(0, color="gray", lw=0.5)
ax_phase.legend(fontsize=8)
ax_phase.grid(alpha=0.3)
ax_phase.set_title("Phase lag — wider bandwidth pushes phase distortion down to lower, control-critical frequencies")

fig.tight_layout()
out = LOG_DIR / "notch_bode_response.png"
fig.savefig(out, dpi=140)
print(f"saved {out}")
