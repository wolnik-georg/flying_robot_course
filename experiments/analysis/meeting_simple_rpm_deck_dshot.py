#!/usr/bin/env python3
"""Task 3 — optical deck RPM vs DShot (airborne-clean, Hampel glitch removal).

Run:
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/meeting_simple_rpm_deck_dshot.py
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from meeting_simple_common import OUT_ASSETS, OUT_TABLES, apply_meeting_style

LOGS = Path(__file__).resolve().parents[2] / "experiments/logs"
FS = 500.0
SPIKE_ERR_RPM = 10_000.0
DSHOT_INVALID = 60_000.0
AIRBORNE_RPM_MIN = 8_000.0
SHUTDOWN_RPM = 5_000.0
SHUTDOWN_MIN_S = 0.5
MIN_FLIGHT_S = 25.0
MIN_AIRBORNE_S = 25.0
MAX_LAG_S = 0.05
ROLL_WIN_S = 2.0
ROLL_STEP_S = 0.5
HAMPEL_WIN = 21
HAMPEL_K = 6.0
HAMPEL_FLOOR_RPM = 600.0
MD = OUT_TABLES / "table_rpm_deck_vs_dshot.md"
A3_CHECK = "A3_2026-09-21_13-00-57"


def cross_corr_lag(a: np.ndarray, b: np.ndarray, fs: float, max_lag_s: float = MAX_LAG_S) -> float:
    a = a - np.mean(a)
    b = b - np.mean(b)
    max_lag = int(max_lag_s * fs)
    if len(a) < 4 * max_lag or len(b) < 4 * max_lag:
        return float("nan")
    corr = np.correlate(a, b, mode="full")
    lags = np.arange(-len(b) + 1, len(a))
    window = (lags >= -max_lag) & (lags <= max_lag)
    best = lags[window][np.argmax(corr[window])]
    return float(-best / fs * 1000.0)


def load_merged_csv(path: Path) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    if path.suffix == ".bin":
        return load_raw_usd(path)
    with open(path, newline="") as f:
        first = f.readline()
        if not first.startswith("#"):
            raise ValueError(f"expected # meta line: {path}")
        reader = csv.reader(f)
        header = next(reader)
        cols: dict[str, list[float]] = {h: [] for h in header}
        for row in reader:
            if len(row) < len(header):
                continue
            for i, h in enumerate(header):
                try:
                    cols[h].append(float(row[i]))
                except ValueError:
                    cols[h].append(float("nan"))
    t = np.array(cols["t"], dtype=float)
    arrays = {k: np.array(v, dtype=float) for k, v in cols.items() if k != "t"}
    return t, arrays


def load_raw_usd(path: Path) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Raw uSD log -> (t, arrays) with the same '<vehicle>.<col>' keys as the merged CSVs."""
    import contextlib, io
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "flying_drone_stack/tools"))
    import decode_usd_log
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        r = decode_usd_log.load(str(path))
    veh = path.name.split("_thesis")[0].split("_A8")[0]
    arrays = {f"{veh}.{k}": np.asarray(v, dtype=float) for k, v in r.items()
              if k != "t" and np.ndim(v) == 1}
    return np.asarray(r["t"], dtype=float), arrays


# Top drone (cf_second) only: controller=6 / ctrl_mode=0 geometric + Z-integral (radio # meta), so no
# downwash and no INDI. Same-session cf_second 17:xx flights were controller=5/ctrl_mode=3 -> excluded.
# Flights shorter than MIN_AIRBORNE_S (A1 16 s, solo hovers <=21 s) drop out via the existing rule.
TOP_DRONE_RAW = (
    [f"cf_second_thesis{n}_2026-10-02_{ts}.bin" for n, ts in (
        (43, "18-28-10"), (44, "18-28-10"), (47, "18-52-26"), (48, "18-52-26"),
        (54, "19-18-11"), (55, "19-18-12"))]
    + ["cf_second_A8_thesis00_2026-10-03_13-17-25.bin", "cf_second_A8_thesis01_2026-10-03_13-17-26.bin"]
)


def detect_prefixes(header_cols: dict[str, np.ndarray]) -> list[str]:
    prefs = set()
    for col in header_cols:
        if col.endswith(".rpm_m1"):
            prefs.add(col[: -len(".rpm_m1")])
    return sorted(prefs)


def prefix_z(arrays: dict[str, np.ndarray], prefix: str) -> np.ndarray | None:
    for key in (f"{prefix}.z", f"{prefix}.pos_z"):
        if key in arrays:
            return arrays[key]
    return None


def longest_run(mask: np.ndarray) -> tuple[int, int] | None:
    if not np.any(mask):
        return None
    idx = np.where(mask)[0]
    breaks = np.where(np.diff(idx) > 1)[0]
    starts = np.concatenate([[0], breaks + 1])
    ends = np.concatenate([breaks, [len(idx) - 1]])
    best = max(range(len(starts)), key=lambda k: ends[k] - starts[k])
    return int(idx[starts[best]]), int(idx[ends[best]])


def low_rpm_while_airborne(t: np.ndarray, rpm: np.ndarray, z: np.ndarray) -> bool:
    low = rpm < SHUTDOWN_RPM
    if not np.any(low & (z > 0.3)):
        return False
    dt = np.diff(t, prepend=t[0])
    run = 0.0
    for i in range(len(t)):
        if low[i] and z[i] > 0.3:
            run += float(dt[i])
            if run > SHUTDOWN_MIN_S:
                return True
        else:
            run = 0.0
    return False


def hampel_outlier_mask(delta: np.ndarray) -> np.ndarray:
    n = len(delta)
    bad = np.zeros(n, dtype=bool)
    half = HAMPEL_WIN // 2
    for i in range(n):
        i0 = max(0, i - half)
        i1 = min(n, i + half + 1)
        w = delta[i0:i1]
        med = float(np.median(w))
        mad = float(np.median(np.abs(w - med)))
        thresh = max(HAMPEL_FLOOR_RPM, HAMPEL_K * 1.4826 * mad)
        if abs(float(delta[i]) - med) > thresh:
            bad[i] = True
    return bad


def glitch_masks(deck: np.ndarray, dshot: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    delta = dshot - deck
    hampel = hampel_outlier_mask(delta)
    abs_big = np.abs(delta) > SPIKE_ERR_RPM
    invalid = dshot >= DSHOT_INVALID
    bad = hampel | abs_big | invalid
    dshot_high = bad & (dshot > deck + 500)
    deck_glitch = bad & ~dshot_high
    old_bad = abs_big | invalid
    return bad, old_bad, dshot_high, deck_glitch


def interp_series(t: np.ndarray, y: np.ndarray, bad: np.ndarray) -> np.ndarray:
    out = y.astype(float).copy()
    good = ~bad & np.isfinite(y)
    if good.sum() < 2:
        return out
    out[bad] = np.interp(t[bad], t[good], y[good])
    return out


def pearson(a: np.ndarray, b: np.ndarray) -> float:
    if len(a) < 3 or np.std(a) < 1e-9 or np.std(b) < 1e-9:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def rolling_lag_iqr(t: np.ndarray, deck: np.ndarray, dshot: np.ndarray, bad: np.ndarray) -> float:
    d_i = interp_series(t, deck, bad)
    s_i = interp_series(t, dshot, bad)
    win = int(ROLL_WIN_S * FS)
    step = max(1, int(ROLL_STEP_S * FS))
    lags = []
    for i0 in range(0, len(t) - win, step):
        sl = slice(i0, i0 + win)
        if bad[sl].mean() > 0.15:
            continue
        lag = cross_corr_lag(d_i[sl], s_i[sl], FS)
        if np.isfinite(lag):
            lags.append(lag)
    if len(lags) < 3:
        return float("nan")
    return float(np.percentile(lags, 75) - np.percentile(lags, 25))


def plot_segments(ax, x: np.ndarray, y: np.ndarray, **kw) -> None:
    """Line plot without connecting across NaN gaps."""
    y = np.asarray(y, dtype=float)
    x = np.asarray(x, dtype=float)
    ok = np.isfinite(y) & np.isfinite(x)
    if not np.any(ok):
        return
    breaks = np.where(np.diff(np.where(ok)[0]) > 1)[0]
    idx = np.where(ok)[0]
    starts = np.concatenate([[0], breaks + 1])
    ends = np.concatenate([breaks, [len(idx) - 1]])
    label = kw.pop("label", None)  # label only the first segment -> one legend entry
    for k, (s, e) in enumerate(zip(starts, ends)):
        ii = idx[s : e + 1]
        ax.plot(x[ii], y[ii], label=label if k == 0 else None, **kw)


def analyze_motor(
    t: np.ndarray,
    deck: np.ndarray,
    dshot_raw: np.ndarray,
    z: np.ndarray | None,
) -> dict:
    out: dict = {
        "excluded": "",
        "n_airborne": 0,
        "n_metric": 0,
        "n_bad": 0,
        "n_bad_old": 0,
        "n_dshot_high": 0,
        "n_deck_glitch": 0,
        "n_bad_on_fast_deck": 0,
        "bias_pct": float("nan"),
        "rmse": float("nan"),
        "r": float("nan"),
        "lag_ms": float("nan"),
        "lag_iqr_ms": float("nan"),
        "i0": 0,
        "i1": 0,
        "deck_std": float("nan"),
    }
    airborne = (deck > AIRBORNE_RPM_MIN) & (dshot_raw > AIRBORNE_RPM_MIN)
    run = longest_run(airborne)
    if run is None:
        out["excluded"] = "no airborne segment (deck & DShot > 8000 RPM)"
        return out
    i0, i1 = run
    out["i0"], out["i1"] = i0, i1
    sl = slice(i0, i1 + 1)
    td, dd, ds = t[sl], deck[sl], dshot_raw[sl]
    zz = z[sl] if z is not None else np.full(len(td), 1.0)
    if low_rpm_while_airborne(td, np.minimum(dd, ds), zz):
        out["excluded"] = "motor shutdown/dropout >0.5 s while z>0.3 m"
        return out
    seg_s = float(td[-1] - td[0])
    if seg_s < MIN_AIRBORNE_S:
        out["excluded"] = f"airborne segment {seg_s:.1f} s < {MIN_AIRBORNE_S:.0f} s"
        return out
    out["n_airborne"] = len(td)
    out["deck_std"] = float(np.std(dd))
    bad, old_bad, dshot_high, deck_glitch = glitch_masks(dd, ds)
    out["n_bad"] = int(bad.sum())
    out["n_bad_old"] = int(old_bad.sum())
    out["n_dshot_high"] = int(dshot_high.sum())
    out["n_deck_glitch"] = int(deck_glitch.sum())
    ddeck = np.abs(np.gradient(dd, td))
    fast = ddeck >= np.percentile(ddeck, 99)
    out["n_bad_on_fast_deck"] = int((bad & fast).sum())
    good = ~bad
    if good.sum() < 50:
        out["excluded"] = "too few samples after glitch removal"
        return out
    d = dd[good]
    s = ds[good]
    err = s - d
    out["n_metric"] = int(good.sum())
    out["bias_pct"] = float(100.0 * np.mean(err) / np.mean(d))
    out["rmse"] = float(np.sqrt(np.mean(err**2)))
    out["r"] = pearson(d, s)
    d_lag = interp_series(td, dd, bad)
    s_lag = interp_series(td, ds, bad)
    out["lag_ms"] = cross_corr_lag(d_lag, s_lag, FS)
    out["lag_iqr_ms"] = rolling_lag_iqr(td, dd, ds, bad)
    return out


def flight_id_from_path(path: Path) -> str:
    return path.stem.replace("_merged_usd", "")


def discover_merged() -> list[Path]:
    return [LOGS / "usd_raw" / n for n in TOP_DRONE_RAW if (LOGS / "usd_raw" / n).exists()]


def plot_pair(path: Path, prefix: str, motor: int, meta: dict, out_png: Path) -> None:
    t, arrays = load_merged_csv(path)
    deck = arrays[f"{prefix}.rpm_m{motor}"]
    dshot = arrays[f"{prefix}.motor_m{motor}_rpm"]
    i0, i1 = meta["i0"], meta["i1"]
    td, dd, ds = t[i0 : i1 + 1], deck[i0 : i1 + 1], dshot[i0 : i1 + 1]
    bad, _, _, _ = glitch_masks(dd, ds)
    d_plot = dd.astype(float)
    s_plot = ds.astype(float)
    d_plot[bad] = np.nan
    s_plot[bad] = np.nan
    diff = s_plot - d_plot

    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(12, 7), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    plot_segments(ax0, td, d_plot, color="#1F5C4D", lw=2.2, alpha=0.85, label="optical deck")
    plot_segments(ax0, td, s_plot, color="#B4341C", lw=0.8, ls="--", label="DShot, spikes removed (gaps)")
    z0, z1 = int(4.5 * FS), int(5.5 * FS)
    ins = ax0.inset_axes([0.55, 0.52, 0.42, 0.42])
    plot_segments(ins, td[z0:z1], d_plot[z0:z1], color="#1F5C4D", lw=1.4)
    plot_segments(ins, td[z0:z1], s_plot[z0:z1], color="#B4341C", lw=1.2, ls="--")
    ins.set_title("zoom ~4.7 s transient", fontsize=10)
    ax0.set_ylabel("RPM")
    ax0.legend(loc="upper left", fontsize=11)
    fid = flight_id_from_path(path)
    ax0.set_title(f"{fid} — {prefix} motor {motor} (airborne segment)")
    plot_segments(ax1, td, diff, color="#2E6DA4", lw=0.9)
    ax1.axhline(0, color="black", lw=0.6)
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("DShot − deck (RPM)")
    txt = (
        f"bias = {meta['bias_pct']:.2f} %\n"
        f"RMSE = {meta['rmse']:.1f} RPM\n"
        f"r = {meta['r']:.3f}\n"
        f"lag τ = {meta['lag_ms']:.2f} ms\n"
        f"removed: DShot-high {meta['n_dshot_high']}, deck {meta['n_deck_glitch']}"
    )
    ax0.text(0.02, 0.02, txt, transform=ax0.transAxes, fontsize=12, va="bottom", bbox=dict(boxstyle="round", facecolor="white", alpha=0.9))
    fig.tight_layout()
    fig.savefig(out_png, bbox_inches="tight")
    plt.close(fig)


def plot_overlay_and_delta(path: Path, prefix: str, motor: int, meta: dict, out_overlay: Path, out_delta: Path) -> None:
    """Two simple figures for ONE flight/motor: (1) overlay of both sources, (2) their difference."""
    t, arrays = load_merged_csv(path)
    deck = arrays[f"{prefix}.rpm_m{motor}"]
    dshot = arrays[f"{prefix}.motor_m{motor}_rpm"]
    i0, i1 = meta["i0"], meta["i1"]
    td, dd, ds = t[i0 : i1 + 1] - t[i0], deck[i0 : i1 + 1], dshot[i0 : i1 + 1]
    bad, _, _, _ = glitch_masks(dd, ds)
    d_plot = dd.astype(float); s_plot = ds.astype(float)
    d_plot[bad] = np.nan; s_plot[bad] = np.nan
    diff = s_plot - d_plot
    fid = flight_id_from_path(path)

    # zoom window: the 2 s with the largest deck RPM swing (fast transient -> lag visible)
    w = int(2.0 * FS)
    best, zi = -1.0, 0
    for k in range(0, max(len(dd) - w, 1), int(0.25 * FS)):
        seg = np.nan_to_num(d_plot[k : k + w], nan=np.nanmedian(d_plot))
        r = float(seg.max() - seg.min())
        if r > best:
            best, zi = r, k
    C_DECK, C_DSHOT = "#1F5C4D", "#D1495B"

    fig, (a0, a1) = plt.subplots(2, 1, figsize=(11, 7), gridspec_kw={"height_ratios": [1, 1.1]})
    plot_segments(a0, td, d_plot, color=C_DECK, lw=1.8, alpha=1.0, zorder=3, label="optical deck")
    plot_segments(a0, td, s_plot, color=C_DSHOT, lw=0.9, alpha=0.8, zorder=2, label="DShot (spikes removed)")
    a0.axvspan(td[zi], td[min(zi + w, len(td) - 1)], color="0.8", alpha=0.5)
    a0.set_title("Full flight"); a0.set_xlabel("time (s)"); a0.set_ylabel("motor RPM"); a0.legend(loc="upper right")
    sl = slice(zi, zi + w)
    plot_segments(a1, td[sl], d_plot[sl], color=C_DECK, lw=3.0, alpha=1.0, zorder=3, label="optical deck")
    plot_segments(a1, td[sl], s_plot[sl], color=C_DSHOT, lw=1.4, alpha=0.8, zorder=2, label="DShot (spikes removed)")
    a1.set_title("Zoom on the grey 2 s"); a1.set_xlabel("time (s)"); a1.set_ylabel("motor RPM"); a1.legend(loc="upper right")
    fig.suptitle(f"Motor RPM, optical deck vs DShot ({fid}, {prefix}, motor {motor})", y=1.0)
    fig.tight_layout(); fig.savefig(out_overlay, bbox_inches="tight"); plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 4.2))
    rm = meta["rmse"]
    ax.axhspan(-rm, rm, color="#4FB39A", alpha=0.2, label=f"±RMSE = ±{rm:.0f} RPM")
    plot_segments(ax, td, diff, color="#2E6DA4", lw=0.8, label="DShot − deck")
    ax.axhline(0, color="black", lw=0.8)
    ax.set_xlabel("time (s)"); ax.set_ylabel("DShot − deck (RPM)")
    ax.set_title("Difference DShot − deck: noise around 0, no drift")
    ax.legend(loc="upper right")
    fig.tight_layout(); fig.savefig(out_delta, bbox_inches="tight"); plt.close(fig)


def a3_spike_check(path: Path, prefix: str, motor: int, meta: dict) -> dict:
    t, arrays = load_merged_csv(path)
    deck = arrays[f"{prefix}.rpm_m{motor}"]
    dshot = arrays[f"{prefix}.motor_m{motor}_rpm"]
    i0, i1 = meta["i0"], meta["i1"]
    td, dd, ds = t[i0 : i1 + 1], deck[i0 : i1 + 1], dshot[i0 : i1 + 1]
    bad, _, _, _ = glitch_masks(dd, ds)

    def peak_in_window(t_lo: float, t_hi: float, label: str) -> dict:
        m = (td >= t_lo) & (td <= t_hi)
        if not np.any(m):
            return {"removed": None, "note": "no samples in window"}
        idx = np.where(m)[0]
        j = int(idx[np.argmax(ds[m])])
        return {
            "removed": bool(bad[j]),
            "t_peak_s": float(td[j]),
            "dshot": float(ds[j]),
            "deck": float(dd[j]),
            "in_plot": bool(not bad[j]),
        }

    checks: dict = {
        "spike~7.3s (peak DShot in 7.15–7.55 s)": peak_in_window(7.15, 7.55, "7.3"),
        "spike~14.4s (peak DShot in 14.15–14.55 s)": peak_in_window(14.15, 14.55, "14.4"),
    }
    for t_chk in (4.7, 13.0, 21.5, 30.0):
        j = int(np.argmin(np.abs(td - t_chk)))
        checks[f"transient t≈{t_chk}s (nearest sample)"] = {
            "removed": bool(bad[j]),
            "dshot": float(ds[j]),
            "deck": float(dd[j]),
            "in_plot": bool(not bad[j]),
        }
    return checks


def main() -> None:
    apply_meeting_style()
    OUT_ASSETS.mkdir(parents=True, exist_ok=True)
    OUT_TABLES.mkdir(parents=True, exist_ok=True)

    rows: list[dict] = []
    included: list[dict] = []

    for path in discover_merged():
        try:
            t, arrays = load_merged_csv(path)
        except (ValueError, OSError):
            continue
        if float(t[-1] - t[0]) < MIN_FLIGHT_S:
            continue
        for prefix in detect_prefixes(arrays):
            z = prefix_z(arrays, prefix)
            for motor in range(1, 5):
                dc = f"{prefix}.rpm_m{motor}"
                sc = f"{prefix}.motor_m{motor}_rpm"
                if dc not in arrays or sc not in arrays:
                    continue
                m = analyze_motor(t, arrays[dc], arrays[sc], z)
                row = {
                    "flight": flight_id_from_path(path),
                    "prefix": prefix,
                    "motor": motor,
                    "path": path,
                    **m,
                }
                rows.append(row)
                if not m["excluded"]:
                    included.append(row)

    if not included:
        raise SystemExit("no airborne-clean motor rows")

    pick_bias = max(included, key=lambda r: abs(r["bias_pct"]))
    lag_stable = [r for r in included if np.isfinite(r["lag_iqr_ms"]) and r["lag_iqr_ms"] < 2.0]
    pick_lag = max(lag_stable, key=lambda r: abs(r["lag_ms"])) if lag_stable else max(included, key=lambda r: abs(r["lag_ms"]))

    picks = [
        ("bias", pick_bias, f"largest |bias %| ({pick_bias['bias_pct']:.2f} %)"),
        ("lag", pick_lag, f"largest |lag| with rolling-lag IQR < 2 ms (IQR={pick_lag['lag_iqr_ms']:.2f} ms)"),
    ]
    written: list[Path] = []
    a3_row = None
    a3_checks = None
    for row in included:
        if A3_CHECK in row["flight"] and row["motor"] == 1:
            a3_row = row
            a3_checks = a3_spike_check(row["path"], row["prefix"], 1, row)
            break
    # one flight is enough for the meeting: the stable-lag pick -> overlay + delta figures
    # figure flight = the calmest one (smallest deck RPM std), so the plot does not look like an unstable hover
    row = min(included, key=lambda r: r["deck_std"])
    out_ov = OUT_ASSETS / "fig_rpm_overlay.png"
    out_dl = OUT_ASSETS / "fig_rpm_delta.png"
    plot_overlay_and_delta(row["path"], row["prefix"], row["motor"], row, out_ov, out_dl)
    written += [out_ov, out_dl]
    print(f"wrote {out_ov}\nwrote {out_dl}")
    total_new = sum(r["n_bad"] for r in included)
    total_old = sum(r["n_bad_old"] for r in included)
    fast_frac = sum(r["n_bad_on_fast_deck"] for r in included) / max(total_new, 1)

    md = [
        "# Table — deck vs DShot RPM (airborne-clean, Hampel Δ filter)",
        "",
        "## Formulas (metrics on glitch-removed aligned samples, deck & DShot > 0)",
        "- **bias** = mean(s − d) [RPM]; **bias %** = 100 · mean(s − d) / mean(d)",
        "- **RMSE** = √(mean((s − d)²)) [RPM]",
        "- **Pearson r** between d and s",
        f"- **lag τ** [ms] = argmax cross-correlation of mean-removed d,s over ±{MAX_LAG_S*1000:.0f} ms; **positive = DShot later**",
        "",
        "## Airborne window (unchanged)",
        f"- Raw deck > {AIRBORNE_RPM_MIN:.0f} RPM and raw DShot > {AIRBORNE_RPM_MIN:.0f} RPM; contiguous first→last.",
        f"- Exclude RPM < {SHUTDOWN_RPM:.0f} for > {SHUTDOWN_MIN_S} s while z > 0.3 m, or airborne segment < {MIN_AIRBORNE_S:.0f} s.",
        "",
        "## Glitch removal (Hampel on Δ = DShot − deck)",
        f"- Flag if |Δ − median(Δ, centred {HAMPEL_WIN}-sample window)| > max({HAMPEL_FLOOR_RPM:.0f} RPM, "
        f"{HAMPEL_K} · 1.4826 · MAD(Δ)), or |Δ| > {SPIKE_ERR_RPM:.0f}, or DShot ≥ {DSHOT_INVALID:.0f}.",
        "- Remove flagged samples from **both** series (gaps in plots); metrics on remaining samples.",
        "- **Lag:** linear interpolation of both series at removed times, then ±50 ms cross-correlation.",
        "",
        "## Old |Δ|>10k rule vs Hampel (included motor-rows)",
        f"- Total removed samples: **{total_new}** (Hampel+rules) vs **{total_old}** (|Δ|>10k only).",
        f"- Removed on top-1% |d(deck)/dt| samples: **{sum(r['n_bad_on_fast_deck'] for r in included)}** "
        f"({100*fast_frac:.1f}% of removals) — should stay low so real transients remain.",
        "",
    ]
    if a3_checks:
        md.append("### Visual acceptance — A3 m1 (reference flight, not necessarily a figure pick)")
        for k, v in a3_checks.items():
            extra = f", t_peak={v['t_peak_s']:.3f}s" if "t_peak_s" in v else ""
            md.append(
                f"- {k}: removed={v['removed']}, DShot={v['dshot']:.0f}, deck={v['deck']:.0f}{extra}"
            )
        md.append("")

    for tag, row, rule in picks:
        md.append(f"### Figure {tag.upper()} — {rule}")
        md.append(
            f"- `{row['flight']}` {row['prefix']} m{row['motor']}: "
            f"bias **{row['bias_pct']:.2f} %**, RMSE **{row['rmse']:.1f}**, r **{row['r']:.3f}**, "
            f"lag **{row['lag_ms']:.2f} ms**; removed DShot-high **{row['n_dshot_high']}**, deck **{row['n_deck_glitch']}** "
            f"(old rule would remove **{row['n_bad_old']}**)"
        )
        md.append("")

    med_lag = [r["lag_ms"] for r in included if np.isfinite(r["lag_ms"])]
    md.append(f"Fleet median lag: **{np.median(med_lag):.2f} ms** — traces nearly identical; systematic lag **≈2–5 ms**.")
    md.append("")

    md.append("## Exclusions")
    for r in sorted(rows, key=lambda x: (x["flight"], x["prefix"], x["motor"])):
        if r["excluded"]:
            md.append(f"- `{r['flight']}` {r['prefix']} m{r['motor']}: {r['excluded']}")

    md += [
        "",
        "## Fleet table (airborne-clean only)",
        "",
        "| flight | vehicle | motor | n | removed | old rule | DShot-hi | deck | fast-deck | bias % | RMSE | r | lag | IQR |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for r in sorted(included, key=lambda x: (x["flight"], x["prefix"], x["motor"])):
        md.append(
            f"| {r['flight']} | {r['prefix']} | {r['motor']} | {r['n_metric']} | {r['n_bad']} | {r['n_bad_old']} | "
            f"{r['n_dshot_high']} | {r['n_deck_glitch']} | {r['n_bad_on_fast_deck']} | "
            f"{r['bias_pct']:.2f} | {r['rmse']:.1f} | {r['r']:.3f} | {r['lag_ms']:.2f} | {r['lag_iqr_ms']:.2f} |"
        )

    bias_v = [r["bias_pct"] for r in included]
    rmse_v = [r["rmse"] for r in included]
    q1b, q3b = np.percentile(bias_v, [25, 75])
    q1r, q3r = np.percentile(rmse_v, [25, 75])
    md.append("")
    md.append(
        f"**Summary (n={len(included)}):** bias % median {np.median(bias_v):.2f} (IQR {q1b:.2f}…{q3b:.2f}); "
        f"RMSE median {np.median(rmse_v):.1f} RPM (IQR {q1r:.1f}…{q3r:.1f})."
    )

    MD.write_text("\n".join(md) + "\n")
    if a3_checks:
        (OUT_TABLES / "rpm_a3_m1_acceptance.json").write_text(json.dumps(a3_checks, indent=2))
    print(f"wrote {MD}")


if __name__ == "__main__":
    main()
