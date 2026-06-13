#!/usr/bin/env python3
"""
plot_indi_filters.py — Visualise INDI Butterworth filter effect for fc_bw tuning.

Loads a CS2 log CSV that contains indi.alp_raw_x/y/z (pre-filter) logged at
100 Hz from the firmware (works with mode=0 geometric hover — passive logging).

Two modes:
  --sweep (default): apply multiple cutoff frequencies offline from ONE log.
      Plots time domain overlay + PSD overlay for all sweep freqs per axis.
      Use this to choose fc_bw without reflashing.

  --compare: overlay raw vs the single fc_bw value currently in the firmware.
      Useful for verifying a chosen fc_bw matches what the drone actually sees
      (requires indi.alp_x/y/z post-filter columns, only present in mode>=2).

Usage:
    python3 plot_indi_filters.py                            # auto-picks latest CSV, sweep mode
    python3 plot_indi_filters.py logs/hover_....csv
    python3 plot_indi_filters.py logs/hover_....csv --compare --fc-bw 40
    python3 plot_indi_filters.py logs/hover_....csv --sweep --freqs 10 20 30 50 70 100

Notes:
  - Offline filter uses scipy sosfilt (causal) to match firmware behaviour.
  - Log rate should be >=100 Hz (Nyquist 50 Hz) to see motor-noise suppression.
    At 20 Hz you only see phase-lag effect, not noise rejection.
  - Works with mode=0 geometric hover (passive logging — no INDI needed).
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, sosfilt, welch

LOGS_DIR = Path(__file__).resolve().parent / "logs"
DEFAULT_FREQS = [10, 20, 30, 50, 70, 100]


def load_csv(path: Path) -> dict:
    with open(path) as f:
        lines = f.readlines()
    header_idx = next(i for i, l in enumerate(lines) if not l.startswith("#"))
    header = lines[header_idx].strip().split(",")
    cols = {k: [] for k in header}
    for line in lines[header_idx + 1:]:
        if not line.strip():
            continue
        vals = line.strip().split(",")
        if len(vals) != len(header):
            continue
        try:
            parsed = [float(v) for v in vals]
        except ValueError:
            continue
        for k, v in zip(header, parsed):
            cols[k].append(v)
    return {k: np.array(v) for k, v in cols.items()}


def detect_fs(cols: dict) -> float:
    """Infer sample rate from timestamp column if present, else default 100 Hz."""
    for key in ("timestamp", "time_s", "time"):
        if key in cols and len(cols[key]) > 10:
            diffs = np.diff(cols[key][:200])
            diffs = diffs[diffs > 0]
            if len(diffs):
                dt = float(np.median(diffs))
                if key == "timestamp":
                    dt /= 1000.0   # ms → s
                return round(1.0 / dt)
    return 100.0


def bw2_causal(sig: np.ndarray, fc: float, fs: float) -> np.ndarray:
    """2nd-order causal Butterworth (matches firmware sosfilt chain)."""
    nyq = fs / 2.0
    if fc >= nyq:
        return sig.copy()
    sos = butter(2, fc / nyq, btype="low", output="sos")
    return sosfilt(sos, sig)


def psd(sig, fs):
    nperseg = min(len(sig), 256)
    f, p = welch(sig, fs=fs, nperseg=nperseg)
    return f, p


def find_col(cols, options):
    for o in options:
        if o in cols:
            return cols[o]
    return None


def trim_leading_zeros(arrays):
    """Drop leading samples where all raw signals are near zero (not yet armed)."""
    ref = arrays[0]
    for i in range(len(ref)):
        if any(abs(a[i]) > 0.5 for a in arrays):
            return i
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="?")
    parser.add_argument("--compare", action="store_true",
                        help="Compare raw vs single fc_bw (needs post-filter columns)")
    parser.add_argument("--fc-bw", type=float, default=None,
                        help="Firmware fc_bw [Hz] for --compare mode or PSD marker")
    parser.add_argument("--freqs", type=float, nargs="+", default=DEFAULT_FREQS,
                        help="Cutoff frequencies [Hz] to sweep (default: 10 20 30 50 70 100)")
    args = parser.parse_args()

    if args.csv:
        path = Path(args.csv)
    else:
        candidates = sorted(LOGS_DIR.glob("*.csv")) if LOGS_DIR.exists() else []
        if not candidates:
            print(f"No CSV in {LOGS_DIR}. Pass a path explicitly.")
            sys.exit(1)
        path = candidates[-1]
        print(f"[auto] {path.name}")

    cols = load_csv(path)
    fs   = detect_fs(cols)
    print(f"[info] Detected sample rate: {fs:.0f} Hz  (Nyquist: {fs/2:.0f} Hz)")
    if fs < 80:
        print(f"[warn] Log rate {fs:.0f} Hz is low — raise indi_state frequency to 100 Hz "
              f"in crazyflies.yaml for full noise visibility.")

    axes_names = ["x", "y", "z"]
    raw_cols = [find_col(cols, [f"indi.alp_raw_{a}", f"alp_raw_{a}"]) for a in axes_names]

    if raw_cols[0] is None:
        print("ERROR: indi.alp_raw_* columns not found in CSV.")
        print("Available columns:", list(cols.keys())[:20])
        print("Make sure firmware is reflashed (passive logging in all modes).")
        sys.exit(1)

    start = trim_leading_zeros(raw_cols)
    raw_cols = [c[start:] for c in raw_cols]
    t = np.arange(len(raw_cols[0])) / fs

    if args.compare:
        # ── Compare mode: raw vs firmware post-filter (requires mode>=2 log) ──
        filt_cols = [find_col(cols, [f"indi.alp_{a}", f"alp_{a}"]) for a in axes_names]
        if filt_cols[0] is None:
            print("ERROR: indi.alp_* (post-filter) columns not found.")
            print("--compare requires a log from INDI mode (mode>=2). "
                  "Use sweep mode for mode=0 geometric logs.")
            sys.exit(1)
        filt_cols = [c[start:] for c in filt_cols]

        fig, axs = plt.subplots(3, 2, figsize=(14, 10))
        fig.suptitle(f"INDI filter comparison (firmware fc_bw={args.fc_bw} Hz) — {path.name}", fontsize=11)

        for i, ax_name in enumerate(["Roll (X)", "Pitch (Y)", "Yaw (Z)"]):
            raw  = raw_cols[i]
            filt = filt_cols[i]

            ax_t = axs[i, 0]
            ax_t.plot(t, raw,  lw=0.5, alpha=0.6, label="alpha_raw (pre-BW)")
            ax_t.plot(t, filt, lw=1.0, color="C1", label="alpha_meas (firmware post-BW)")
            ax_t.set_ylabel(f"{ax_name}\n[rad/s²]", fontsize=9)
            ax_t.legend(fontsize=8)
            if i == 0: ax_t.set_title("Time domain")
            if i == 2: ax_t.set_xlabel("t [s]")

            ax_f = axs[i, 1]
            fr, pr = psd(raw, fs); ff, pf = psd(filt, fs)
            ax_f.semilogy(fr, pr, lw=0.7, alpha=0.7, label="raw")
            ax_f.semilogy(ff, pf, lw=1.2, color="C1", label="firmware filtered")
            if args.fc_bw:
                ax_f.axvline(args.fc_bw, color="r", ls="--", lw=1, label=f"fc_bw={args.fc_bw} Hz")
            ax_f.set_ylabel("PSD [(rad/s²)²/Hz]", fontsize=9)
            ax_f.legend(fontsize=8)
            ax_f.set_xlim(0, fs / 2)
            if i == 0: ax_f.set_title("PSD")
            if i == 2: ax_f.set_xlabel("Frequency [Hz]")

        plt.tight_layout()
        out = path.with_name(path.stem + "_filter_compare.png")

    else:
        # ── Sweep mode: apply multiple fc_bw values offline from raw ──────────
        freqs = sorted(args.freqs)
        # clip to valid range
        freqs = [f for f in freqs if 0 < f < fs / 2]
        if not freqs:
            print(f"ERROR: no valid frequencies after clipping to (0, {fs/2:.0f}) Hz.")
            sys.exit(1)

        cmap   = plt.cm.viridis(np.linspace(0.0, 0.85, len(freqs)))
        labels = [f"{int(f)} Hz" for f in freqs]

        fig, axs = plt.subplots(3, 2, figsize=(15, 10))
        fig.suptitle(f"Offline BW filter sweep (causal, fs={fs:.0f} Hz) — {path.name}", fontsize=11)

        for i, ax_name in enumerate(["Roll (X)", "Pitch (Y)", "Yaw (Z)"]):
            raw = raw_cols[i]

            ax_t = axs[i, 0]
            ax_t.plot(t, raw, lw=0.4, alpha=0.4, color="gray", label="raw", zorder=0)
            ax_f = axs[i, 1]
            fr_raw, pr_raw = psd(raw, fs)
            ax_f.semilogy(fr_raw, pr_raw, lw=0.7, alpha=0.5, color="gray", label="raw")

            for fc, color, label in zip(freqs, cmap, labels):
                filtered = bw2_causal(raw, fc, fs)
                ax_t.plot(t, filtered, lw=0.9, color=color, label=label)
                ff, pf = psd(filtered, fs)
                ax_f.semilogy(ff, pf, lw=1.0, color=color, label=label)
                ax_f.axvline(fc, color=color, lw=0.5, ls=":")

            if args.fc_bw:
                ax_t.set_title(f"Time domain  (current fc_bw={args.fc_bw} Hz)", fontsize=9)
                ax_f.axvline(args.fc_bw, color="red", lw=1.2, ls="--",
                             label=f"current {args.fc_bw} Hz")
            elif i == 0:
                ax_t.set_title("Time domain")
                ax_f.set_title("PSD")

            ax_t.set_ylabel(f"{ax_name}\n[rad/s²]", fontsize=9)
            ax_t.legend(fontsize=7, loc="upper right", ncol=2)
            ax_f.set_ylabel("PSD [(rad/s²)²/Hz]", fontsize=9)
            ax_f.legend(fontsize=7, ncol=2)
            ax_f.set_xlim(0, fs / 2)
            if i == 2:
                ax_t.set_xlabel("t [s]")
                ax_f.set_xlabel("Frequency [Hz]")

        # Summary guidance
        fig.text(0.01, 0.01,
                 "Choosing fc_bw:  too low → large phase lag (curves lag raw in time domain)  |  "
                 "too high → noise passes through (PSD curves close to raw above fc_bw)  |  "
                 "good → noise suppressed, phase lag <5 ms",
                 fontsize=7, color="dimgray")
        plt.tight_layout(rect=[0, 0.025, 1, 1])
        out = path.with_name(path.stem + "_filter_sweep.png")

    plt.savefig(out, dpi=150)
    print(f"Saved: {out}")
    plt.show()


if __name__ == "__main__":
    main()
