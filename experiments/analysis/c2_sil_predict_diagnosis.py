#!/usr/bin/env python3
"""Task 1: diagnose weak corr / high clamp in full-bank SIL predict log."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
RES = ROOT / "flying_drone_stack/tools/residual"
CSV = ROOT / "experiments/sim_validation/c2_fullbank_predict.csv"
STAGE_A = ROOT / "experiments/analysis/out/c2_e2e_2026-09-26/stage_a_full_bank.json"
OUT_CLAMP = 8.0
GATE_DXY = 0.2
GATE_DVX = 1.5

sys.path.insert(0, str(RES))
import dataset  # noqa: E402


def load_residual_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open() as f:
        header = f.readline().strip().split(",")
    a = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return {n: a[:, i] for i, n in enumerate(header)}


def corr(a: np.ndarray, b: np.ndarray) -> float:
    m = np.isfinite(a) & np.isfinite(b)
    if m.sum() < 50:
        return float("nan")
    return float(np.corrcoef(a[m], b[m])[0, 1])


def lag_sweep(a: np.ndarray, b: np.ndarray, max_lag: int = 10) -> list[dict]:
    """corr(a[t], b[t+lag]) — positive lag = prediction leads measurement."""
    rows = []
    n = min(len(a), len(b))
    a, b = a[:n], b[:n]
    for lag in range(-max_lag, max_lag + 1):
        if lag < 0:
            aa, bb = a[-lag:], b[: n + lag]
        elif lag > 0:
            aa, bb = a[: n - lag], b[lag:]
        else:
            aa, bb = a, b
        rows.append({"lag_samples": lag, "corr": corr(aa, bb), "n": int(len(aa))})
    return rows


def sil_relative_features(cols: dict, ego: str, peer: str) -> dict[str, np.ndarray]:
    ex, ey, ez = cols[f"{ego}.x"], cols[f"{ego}.y"], cols[f"{ego}.z"]
    evx, evy, evz = cols[f"{ego}.vx"], cols[f"{ego}.vy"], cols[f"{ego}.vz"]
    px, py, pz = cols[f"{peer}.x"], cols[f"{peer}.y"], cols[f"{peer}.z"]
    pvx, pvy, pvz = cols[f"{peer}.vx"], cols[f"{peer}.vy"], cols[f"{peer}.vz"]
    dx, dy, dz = px - ex, py - ey, pz - ez
    dvx, dvy, dvz = pvx - evx, pvy - evy, pvz - evz
    gated = (np.abs(dx) < GATE_DXY) & (np.abs(dy) < GATE_DXY) & (np.abs(dvx) < GATE_DVX)
    return dict(dx=dx, dy=dy, dz=dz, dvx=dvx, dvy=dvy, dvz=dvz, gated=gated)


def quantiles(x: np.ndarray, ps=(1, 5, 50, 95, 99)) -> dict:
    x = x[np.isfinite(x)]
    if len(x) == 0:
        return {}
    return {f"p{p}": float(np.percentile(x, p)) for p in ps}


def training_bank_stats(stage_a_path: Path) -> dict:
    sa = json.loads(stage_a_path.read_text())
    paths = [
        ROOT / v["path"]
        for v in sa["per_flight"].values()
        if v.get("rows_kept", 0) > 0
    ]
    rel, mask, _ground, y, stats = dataset.build([str(p) for p in paths], verbose=False)
    # First gated neighbour features (same gate as firmware)
    feats = {k: [] for k in ("dx", "dy", "dz", "dvx", "dvy", "dvz", "y")}
    for i in range(len(y)):
        for j in range(rel.shape[1]):
            if mask[i, j] < 0.5:
                continue
            r = rel[i, j]
            feats["dx"].append(float(r[0]))
            feats["dy"].append(float(r[1]))
            feats["dz"].append(float(r[2]))
            feats["dvx"].append(float(r[3]))
            feats["dvy"].append(float(r[4]))
            feats["dvz"].append(float(r[5]))
            feats["y"].append(float(y[i]))
            break  # one gated neighbour row per sample (primary)
    out = {"n_samples": int(len(y)), "n_gated_neighbour_rows": len(feats["y"]), "build_stats": stats}
    for k, arr in feats.items():
        a = np.array(arr, np.float64)
        out[k] = {
            "quantiles": quantiles(a),
            "abs_max": float(np.max(np.abs(a))) if len(a) else None,
        }
    return out


def main() -> int:
    cols = load_residual_csv(CSV)
    ego, peer = "cf231_active", "cf_second"
    a_res = cols[f"{ego}.a_res_z"]
    pred = cols[f"{ego}.rnn_pred_z"]
    t = cols["t"]

    # Meaningful flight: both airborne-ish
    m = (cols[f"{ego}.z"] > 0.05) & (cols[f"{peer}.z"] > 0.05)
    m &= np.isfinite(a_res) & np.isfinite(pred)

    lags = lag_sweep(a_res[m], pred[m], max_lag=10)
    best = max(lags, key=lambda r: abs(r["corr"]) if np.isfinite(r["corr"]) else -1)

    clamp_frac = float(np.mean(np.abs(pred[m]) >= OUT_CLAMP - 1e-6))
    zero_pred_frac = float(np.mean(np.abs(pred[m]) < 1e-9))

    rel = sil_relative_features(cols, ego, peer)
    sil_gated = rel["gated"] & m

    train = training_bank_stats(STAGE_A)

    # Compare SIL (gated ticks only) vs training gated-neighbour quantiles
    def sil_q(name: str) -> dict:
        return quantiles(rel[name][sil_gated])

    comparison = {}
    for k in ("dx", "dy", "dz", "dvx"):
        comparison[k] = {
            "training_gated": train.get(k, {}).get("quantiles", {}),
            "sil_gated_ticks": sil_q(k),
        }

    # Per-phase: horizontal separation tertiles
    horiz = np.sqrt(rel["dx"][m] ** 2 + rel["dy"][m] ** 2)
    dz_m = rel["dz"][m]
    pred_m, a_m = pred[m], a_res[m]
    edges = np.quantile(horiz, [0.33, 0.66])
    phases = []
    for label, sel in (
        ("close_horiz", horiz <= edges[0]),
        ("mid_horiz", (horiz > edges[0]) & (horiz <= edges[1])),
        ("far_horiz", horiz > edges[1]),
    ):
        phases.append(
            {
                "phase": label,
                "n": int(sel.sum()),
                "corr": corr(a_m[sel], pred_m[sel]),
                "clamp_fraction": float(np.mean(np.abs(pred_m[sel]) >= OUT_CLAMP - 1e-6)),
                "a_res_z_quantiles": quantiles(a_m[sel]),
            }
        )

    # Clamp vs gate
    phases.append(
        {
            "phase": "gated_true",
            "n": int(sil_gated.sum()),
            "corr": corr(a_res[sil_gated], pred[sil_gated]),
            "clamp_fraction": float(
                np.mean(np.abs(pred[sil_gated]) >= OUT_CLAMP - 1e-6)
            ),
        }
    )
    phases.append(
        {
            "phase": "gated_false",
            "n": int((m & ~rel["gated"]).sum()),
            "corr": corr(a_res[m & ~rel["gated"]], pred[m & ~rel["gated"]]),
            "clamp_fraction": float(
                np.mean(np.abs(pred[m & ~rel["gated"]]) >= OUT_CLAMP - 1e-6)
            ),
        }
    )

    report = {
        "csv": str(CSV),
        "n_rows": int(len(t)),
        "n_in_air": int(m.sum()),
        "corr_lag0_in_air": corr(a_res[m], pred[m]),
        "lag_sweep_in_air": lags,
        "best_lag_by_abs_corr": best,
        "rnn_pred_z_clamp_fraction_in_air": clamp_frac,
        "rnn_pred_z_zero_fraction_in_air": zero_pred_frac,
        "a_res_z_in_air": quantiles(a_res[m]),
        "training_bank": train,
        "feature_comparison_gated": comparison,
        "per_phase": phases,
    }

    out_path = ROOT / "experiments/analysis/out/c2_e2e_2026-09-26/sil_predict_diagnosis.json"
    out_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
