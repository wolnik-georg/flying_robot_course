#!/usr/bin/env python3
"""Per-crossing A8 dip stats, eval-rate / neighbour checks, z-loop fit (2026-10-05 uSD)."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import numpy as np
from scipy import stats

REPO = Path(__file__).resolve().parents[2]
LOG = REPO / "experiments/logs"
USD = LOG / "usd_raw"
OUT = Path(__file__).resolve().parent / "out" / "ns2_2026_10_05"

sys.path.insert(0, str(REPO / "flying_drone_stack/tools"))
from decode_usd_log import load as load_usd  # noqa: E402

# lab_sessions/2026-10-05.md §6 thesis cohort: 2× en=0 (17:39/17:41 → uSD 17-44-01),
# 4× en=1 (17:59 A8rnn1 thesis02 + 19:17/19:19/19:23 fresh).
FRESH_BATTERY_STAMPS = {"19-17-04", "19-19-27", "19-23-26"}


def rnn_en_and_cohort(path: Path) -> tuple[int, bool]:
    """Return (rnn_en, thesis_cohort). thesis_cohort excludes crash/uSD garbage flights."""
    name = path.name
    if "A1" in name:
        return -1, False
    if "17-44-01" in name and "_A8_thesis" in name:
        return 0, True
    if name.endswith("19-17-04.bin") or "19-19-27" in name or "19-23-26" in name:
        return 1, True
    if "A8rnn1_thesis02_" in name and "18-05-08" in name:
        return 1, True
    # Known non-cohort / crash logs (still decoded for eval-rate)
    if "18-18-58" in name or "18-34-10" in name:
        return 0, False
    if "A8rnn1" in name:
        return 1, False
    return -1, False


def stamp_from_name(name: str) -> str | None:
    m = re.search(r"2026-10-05_(\d{2}-\d{2}-\d{2})", name)
    return m.group(1) if m else None


def find_crossing_times(t: np.ndarray, y: np.ndarray, n_expect: int = 4) -> list[float]:
    """Bottom drone A8: horizontal axis y; crossings ≈ |y| minima."""
    y = np.asarray(y, float)
    t = np.asarray(t, float)
    if len(t) < 100:
        return []
    dy = np.abs(y)
    # Smooth lightly
    w = min(31, len(dy) // 10 * 2 + 1)
    if w >= 5:
        k = np.ones(w) / w
        dy_s = np.convolve(dy, k, mode="same")
    else:
        dy_s = dy
    order = max(1, len(t) // 200)
    mins = []
    for i in range(order, len(t) - order):
        if dy_s[i] <= dy_s[i - order : i + order + 1].min() + 1e-6:
            if not mins or t[i] - mins[-1] > 2.5:
                mins.append(float(t[i]))
    if len(mins) > n_expect:
        # keep deepest n_expect by |y|
        mins.sort(key=lambda tc: np.min(dy[(t >= tc - 0.5) & (t <= tc + 0.5)]))
        mins = sorted(mins[:n_expect])
    return mins


def crossing_dips_usd(path: Path) -> list[dict]:
    d = load_usd(str(path))
    t, y, z = d["t"], d["y"], d["z"]
    z_sp = d.get("ctrltarget_z", np.full_like(z, 0.5))
    a_res = d.get("a_res_z", np.full_like(z, np.nan))
    pred = d.get("rnn_pred_z", np.full_like(z, np.nan))
    stamp = stamp_from_name(path.name)
    rnn_en, thesis_cohort = rnn_en_and_cohort(path)
    rows = []
    for tc in find_crossing_times(t, y):
        m = (t >= tc - 1.0) & (t <= tc + 1.0)
        if not np.any(m):
            continue
        e_z = (z[m] - z_sp[m]) * 100.0
        dip_cm = float(np.min(e_z))
        rows.append(
            {
                "file": path.name,
                "stamp": stamp,
                "rnn_en": rnn_en,
                "thesis_cohort": thesis_cohort,
                "fresh_battery": stamp in FRESH_BATTERY_STAMPS,
                "t_cross_s": tc,
                "dip_cm": dip_cm,
                "a_res_z_mean": float(np.nanmean(a_res[m])),
                "pred_z_mean": float(np.nanmean(pred[m])),
                "pred_z_std": float(np.nanstd(pred[m])),
            }
        )
    return rows


def eval_rate_and_gate(path: Path) -> dict:
    d = load_usd(str(path))
    t = d["t"]
    pred = np.asarray(d.get("rnn_pred_z", []), float)
    dt = np.diff(t)
    fs = 1.0 / float(np.median(dt)) if len(dt) else float("nan")
    dp = np.diff(pred)
    changed = np.abs(dp) > 1e-5
    change_rate_hz = float(np.sum(changed) / (t[-1] - t[0])) if len(t) > 1 else float("nan")
    # Plateau length between changes (samples at ~500 Hz)
    runs = []
    n = 0
    for c in changed:
        if c:
            if n:
                runs.append(n)
            n = 1
        else:
            n += 1
    if n:
        runs.append(n)
    median_hold_samples = float(np.median(runs)) if runs else float("nan")
    y = np.asarray(d["y"], float)
    far = np.abs(y) > 0.35
    close = np.abs(y) < 0.08
    gate = {
        "pred_z_mean_far": float(np.nanmean(pred[far])) if np.any(far) else float("nan"),
        "pred_z_std_far": float(np.nanstd(pred[far])) if np.any(far) else float("nan"),
        "pred_z_mean_close": float(np.nanmean(pred[close])) if np.any(close) else float("nan"),
        "pred_z_std_close": float(np.nanstd(pred[close])) if np.any(close) else float("nan"),
    }
    dt_us = d.get("dt_us")
    dt_us_med = float(np.nanmedian(dt_us)) if dt_us is not None else float("nan")
    return {
        "file": path.name,
        "fs_hz": fs,
        "pred_change_rate_hz": change_rate_hz,
        "median_hold_samples_at_500hz": median_hold_samples,
        "implied_eval_hz_from_hold": float(fs / median_hold_samples) if median_hold_samples > 0 else float("nan"),
        "dt_us_median": dt_us_med,
        **gate,
    }


def fit_z_loop(rows: list[dict], *, thesis_only: bool = True) -> dict:
    """Linear fit dip_cm = b0 + b1*a_res_z; en=1 adds b2*pred_z (res_sign=+1). Predict res_sign=-1."""
    if thesis_only:
        rows = [r for r in rows if r.get("thesis_cohort")]
    en0 = [r for r in rows if r["rnn_en"] == 0]
    en1 = [r for r in rows if r["rnn_en"] == 1]
    out = {"n_en0": len(en0), "n_en1": len(en1)}
    if en0:
        dips0 = np.array([r["dip_cm"] for r in en0])
        a0 = np.array([r["a_res_z_mean"] for r in en0])
        out["en0_dip_mean_cm"] = float(np.mean(dips0))
        out["en0_dip_std_cm"] = float(np.std(dips0, ddof=1)) if len(dips0) > 1 else 0.0
        out["en0_a_res_mean"] = float(np.mean(a0))
    if en1:
        dips1 = np.array([r["dip_cm"] for r in en1])
        out["en1_dip_mean_cm"] = float(np.mean(dips1))
        out["en1_dip_std_cm"] = float(np.std(dips1, ddof=1)) if len(dips1) > 1 else 0.0
    if len(en0) >= 2 and len(en1) >= 2:
        d0 = [r["dip_cm"] for r in en0]
        d1 = [r["dip_cm"] for r in en1]
        tstat, pval = stats.ttest_ind(d0, d1, equal_var=False)
        out["welch_t"] = float(tstat)
        out["welch_p"] = float(pval)
        out["mannwhitney_u"], out["mannwhitney_p"] = stats.mannwhitneyu(d0, d1, alternative="two-sided")
        out["en0_dip_min_cm"] = float(np.min(d0))
        out["en0_dip_max_cm"] = float(np.max(d0))
        out["en1_dip_min_cm"] = float(np.min(d1))
        out["en1_dip_max_cm"] = float(np.max(d1))
        out["ranges_overlap"] = bool(
            out["en0_dip_max_cm"] >= out["en1_dip_min_cm"] and out["en1_dip_max_cm"] >= out["en0_dip_min_cm"]
        )

    # Fit on en=0: dip = b0 + b1 * a_res
    if len(en0) >= 2:
        A = np.column_stack([np.ones(len(en0)), [r["a_res_z_mean"] for r in en0]])
        b, _, _, _ = np.linalg.lstsq(A, [r["dip_cm"] for r in en0], rcond=None)
        b0, b1 = float(b[0]), float(b[1])
    else:
        b0, b1 = float("nan"), float("nan")

    # en=1 with res_sign=+1: dip = b0 + b1*a + b2*pred_z
    if len(en1) >= 2 and np.isfinite(b0):
        A1 = np.column_stack(
            [np.ones(len(en1)), [r["a_res_z_mean"] for r in en1], [r["pred_z_mean"] for r in en1]]
        )
        c, _, _, _ = np.linalg.lstsq(A1, [r["dip_cm"] for r in en1], rcond=None)
        b0_1, b1_1, b2 = float(c[0]), float(c[1]), float(c[2])
    else:
        b0_1, b1_1, b2 = b0, b1, float("nan")

    # Predict res_sign=-1 at typical crossing (mean a_res, pred from en1)
    if len(en1) and np.isfinite(b2):
        ar = float(np.mean([r["a_res_z_mean"] for r in en1]))
        pz = float(np.mean([r["pred_z_mean"] for r in en1]))
        dip_plus = b0_1 + b1_1 * ar + b2 * pz
        dip_minus = b0_1 + b1_1 * ar - b2 * pz
        # Incremental NN model: dip ≈ dip_en0_mean + k_nn * res_sign * pred_z (k_nn from cohort means).
        dip0_mean = float(np.mean([r["dip_cm"] for r in en0]))
        k_nn = (float(np.mean([r["dip_cm"] for r in en1])) - dip0_mean) / pz if abs(pz) > 1e-6 else float("nan")
        dip_rs_plus = dip0_mean + k_nn * (+1) * pz
        dip_rs_minus = dip0_mean + k_nn * (-1) * pz
        out.update(
            {
                "fit_en0_b0_cm": b0,
                "fit_en0_b1_cm_per_mps2": b1,
                "fit_en1_b2_cm_per_mps2_pred": b2,
                "predicted_dip_cm_joint_lstsq_res_sign_plus1": float(dip_plus),
                "predicted_dip_cm_joint_lstsq_res_sign_minus1": float(dip_minus),
                "k_nn_cm_per_mps2_from_cohort_means": float(k_nn),
                "predicted_dip_cm_incremental_res_sign_plus1": float(dip_rs_plus),
                "predicted_dip_cm_incremental_res_sign_minus1": float(dip_rs_minus),
                "typical_a_res_z_mps2": ar,
                "typical_pred_z_mps2": pz,
            }
        )
    return out


def neighbour_gate_merged(merged_paths: list[Path]) -> list[dict]:
    """Check pred_z vs horizontal separation from merged rel.* columns."""
    import csv

    out = []
    for mp in merged_paths:
        with mp.open(newline="") as f:
            lines = [ln for ln in f if not ln.startswith("#")]
        import io

        rdr = csv.DictReader(io.StringIO("".join(lines)))
        rows = list(rdr)
        if not rows:
            continue
        cols = list(rows[0].keys())
        pred_col = next(c for c in cols if c.endswith("rnn_pred_z"))
        rel_x = next(c for c in cols if c.startswith("rel.") and c.endswith(".x"))
        rel_y = rel_x[:-1] + "y"
        rel_z = rel_x[:-1] + "z"
        clamp_col = next((c for c in cols if c.endswith("rnn_clamped")), None)
        pred = np.array([float(r[pred_col]) for r in rows])
        rx = np.array([float(r[rel_x]) for r in rows])
        ry = np.array([float(r[rel_y]) for r in rows])
        rz = np.array([float(r[rel_z]) for r in rows])
        dist = np.sqrt(rx * rx + ry * ry + rz * rz)
        sep_y = np.abs(ry)
        far = sep_y > 0.45
        close = sep_y < 0.15
        clamped = (
            np.array([float(r.get(clamp_col, 0) or 0) for r in rows])
            if clamp_col
            else np.zeros(len(rows))
        )
        out.append(
            {
                "merged_file": mp.name,
                "pred_z_mean_far": float(np.mean(pred[far])) if np.any(far) else float("nan"),
                "pred_z_std_far": float(np.std(pred[far])) if np.any(far) else float("nan"),
                "pred_z_mean_close": float(np.mean(pred[close])) if np.any(close) else float("nan"),
                "pred_z_std_close": float(np.std(pred[close])) if np.any(close) else float("nan"),
                "frac_clamped": float(np.mean(clamped > 0.5)),
                "corr_sep_y_pred_z": float(np.corrcoef(sep_y, pred)[0, 1]) if len(pred) > 2 else float("nan"),
                "dist_min_m": float(np.min(dist)),
                "dist_max_m": float(np.max(dist)),
            }
        )
    return out


def _json_safe(obj):
    if isinstance(obj, float) and (np.isnan(obj) or np.isinf(obj)):
        return None
    if isinstance(obj, dict):
        return {k: _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_json_safe(x) for x in obj]
    return obj


def write_svg_dip(rows: list[dict], path: Path, *, thesis_only: bool = True) -> None:
    if thesis_only:
        rows = [r for r in rows if r.get("thesis_cohort")]
    en0 = [r["dip_cm"] for r in rows if r["rnn_en"] == 0]
    en1 = [r["dip_cm"] for r in rows if r["rnn_en"] == 1]
    groups = [("rnn.en=0", en0, "#48a"), ("rnn.en=1", en1, "#c44")]
    w, h = 420, 220
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">',
        '<rect width="100%" height="100%" fill="#fafafa"/>',
        '<text x="10" y="18" font-size="12">A8 cf5 crossing dip (min e_z, ±1 s) — thesis cohort 2026-10-05</text>',
        '<line x1="80" y1="30" x2="80" y2="200" stroke="#999"/>',
        '<text x="40" y="120" font-size="10" transform="rotate(-90 40,120)">dip [cm]</text>',
    ]
    x_centers = [140, 280]
    for i, (lab, vals, col) in enumerate(groups):
        if not vals:
            continue
        arr = np.array(vals)
        mean, med = float(np.mean(arr)), float(np.median(arr))
        q1, q3 = float(np.percentile(arr, 25)), float(np.percentile(arr, 75))
        xc = x_centers[i]
        # scale: 0 cm -> y=50, -14 cm -> y=190
        def ymap(v):
            return 50 + (-v / 14.0) * 140

        lines.append(f'<text x="{xc-50}" y="28" font-size="11">{lab} (n={len(vals)})</text>')
        lines.append(f'<text x="{xc-50}" y="210" font-size="10">mean {mean:.1f} cm</text>')
        for j, v in enumerate(vals):
            jitter = ((j % 5) - 2) * 6
            lines.append(
                f'<circle cx="{xc + jitter}" cy="{ymap(v):.1f}" r="4" fill="{col}" opacity="0.7"/>'
            )
        my, yq1, yq3 = ymap(med), ymap(q1), ymap(q3)
        lines.append(f'<line x1="{xc-20}" x2="{xc+20}" y1="{my:.1f}" y2="{my:.1f}" stroke="{col}" stroke-width="3"/>')
        lines.append(f'<line x1="{xc}" x2="{xc}" y1="{yq1:.1f}" y2="{yq3:.1f}" stroke="{col}" stroke-width="8" opacity="0.35"/>')
    lines.append("</svg>")
    path.write_text("\n".join(lines))


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    cf5_bins = sorted(USD.glob("cf5_A8*_2026-10-05_*.bin"))
    all_dips: list[dict] = []
    eval_rows = []
    for p in cf5_bins:
        if "A1" in p.name:
            continue
        try:
            all_dips.extend(crossing_dips_usd(p))
            eval_rows.append(eval_rate_and_gate(p))
        except Exception as e:
            eval_rows.append({"file": p.name, "error": str(e)})

    thesis_dips = [r for r in all_dips if r.get("thesis_cohort")]
    fit = fit_z_loop(all_dips, thesis_only=True)

    # CSV
    csv_lines = [
        "file,stamp,rnn_en,thesis_cohort,fresh_battery,t_cross_s,dip_cm,a_res_z_mean,pred_z_mean,pred_z_std"
    ]
    for r in all_dips:
        csv_lines.append(
            f"{r['file']},{r['stamp']},{r['rnn_en']},{r['thesis_cohort']},{r['fresh_battery']},{r['t_cross_s']:.3f},"
            f"{r['dip_cm']:.3f},{r['a_res_z_mean']:.4f},{r['pred_z_mean']:.4f},{r['pred_z_std']:.4f}"
        )
    (OUT / "crossing_dip_table.csv").write_text("\n".join(csv_lines) + "\n")

    merged_paths = sorted(LOG.glob("merged_A8*_2026-10-05_*.csv"))
    neighbour = neighbour_gate_merged(merged_paths)

    summary = _json_safe(
        {
            "crossings_total": len(all_dips),
            "thesis_cohort_crossings": len(thesis_dips),
            "fit_and_tests_thesis_cohort": fit,
            "eval_rate_by_file": eval_rows,
            "neighbour_gate_merged": neighbour,
        }
    )
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2))
    write_svg_dip(all_dips, OUT / "fig_crossing_dip.svg", thesis_only=True)

    # Correlation pred vs a_res per file (fresh)
    corr_rows = []
    for p in cf5_bins:
        if stamp_from_name(p.name) not in FRESH_BATTERY_STAMPS:
            continue
        d = load_usd(str(p))
        m = np.isfinite(d["rnn_pred_z"]) & np.isfinite(d["a_res_z"])
        if np.sum(m) < 50:
            continue
        c = float(np.corrcoef(d["rnn_pred_z"][m], d["a_res_z"][m])[0, 1])
        corr_rows.append({"file": p.name, "corr_pred_a_res": c})
    summary["fresh_corr_pred_a_res"] = _json_safe(corr_rows)
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2))

    print(
        json.dumps(
            _json_safe(
                {
                    "thesis_crossings": len(thesis_dips),
                    "fit": fit,
                    "out": str(OUT),
                }
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
