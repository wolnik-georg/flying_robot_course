#!/usr/bin/env python3
"""Evaluate exported residual weights on merged uSD logs (docs/27 P5).

    python3 eval_model.py weights/c1_geometric.npz merged_a1.csv merged_a7.csv \\
        -o experiments/analysis/out/residual_eval/

Reports R², RMSE, predict-zero and predict-mean baselines, error vs lateral offset and dz for
gated neighbours, and per-file (flight-wise) holdout metrics using the same weights — no
retraining per fold.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(_ROOT))

import dataset  # noqa: E402
from model import DEFAULT_MASS, firmware_forward  # noqa: E402


def _r2(y: np.ndarray, pred: np.ndarray) -> float:
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    return float(1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else float("nan")


def _rmse(y: np.ndarray, pred: np.ndarray) -> float:
    return float(np.sqrt(np.mean((y - pred) ** 2)))


def predict_batch(w: np.ndarray, rel: np.ndarray, mask: np.ndarray, ground: np.ndarray,
                  mass: float) -> np.ndarray:
    own_z = -ground[:, 0]
    own_vel = -ground[:, 1:]
    out = np.empty(len(ground), np.float64)
    n_neigh = rel.shape[1]
    for i in range(len(ground)):
        neigh = [(rel[i, k, :3].copy(), rel[i, k, 3:].copy())
                 for k in range(n_neigh) if mask[i, k] > 0]
        out[i] = firmware_forward(w, neigh, own_z[i], own_vel[i], mass=mass,
                                  apply_clamp=False)[0]
    return out


def neighbour_geometry(rel: np.ndarray, mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """First gated neighbour: horizontal distance |dy| proxy (|dx|,|dy|) and dz (peer - own, firmware sign)."""
    dy = np.full(len(rel), np.nan, np.float64)
    dz = np.full(len(rel), np.nan, np.float64)
    for i in range(len(rel)):
        for k in range(rel.shape[1]):
            if mask[i, k] <= 0:
                continue
            dp = rel[i, k, :3]
            dy[i] = float(np.linalg.norm(dp[:2]))
            dz[i] = float(dp[2])
            break
    return dy, dz


def eval_slice(y: np.ndarray, pred: np.ndarray) -> dict:
    base_zero = _rmse(y, np.zeros_like(y))
    base_mean = _rmse(y, np.full_like(y, np.mean(y)))
    rmse = _rmse(y, pred)
    return dict(
        n=int(len(y)),
        rmse_mps2=rmse,
        r2=_r2(y, pred),
        baseline_zero_rmse=base_zero,
        baseline_mean_rmse=base_mean,
        pct_reduction_vs_zero=(100.0 * (1.0 - rmse / base_zero) if base_zero > 0 else float("nan")),
    )


def binned_error(dy: np.ndarray, dz: np.ndarray, err: np.ndarray, axis: str,
                 edges: np.ndarray) -> list[dict]:
    coord = dy if axis == "dy" else dz
    valid = np.isfinite(coord) & np.isfinite(err)
    rows = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = valid & (coord >= lo) & (coord < hi)
        if sel.sum() < 10:
            continue
        e = err[sel]
        rows.append(dict(
            bin=f"{lo:.2f}-{hi:.2f}",
            axis=axis,
            n=int(sel.sum()),
            rmse_mps2=_rmse(e, np.zeros_like(e)),
        ))
    return rows


def maybe_plots(out: Path, y: np.ndarray, pred: np.ndarray, dy: np.ndarray, dz: np.ndarray):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[eval] matplotlib not available — skipping figures")
        return
    fig, ax = plt.subplots(figsize=(5, 5))
    ax.scatter(y, pred, s=1, alpha=0.15)
    lim = max(np.max(np.abs(y)), np.max(np.abs(pred)), 0.5)
    ax.plot([-lim, lim], [-lim, lim], "k--", lw=0.8)
    ax.set_xlabel("a_res_z measured [m/s²]")
    ax.set_ylabel("predicted [m/s²]")
    ax.set_title(f"Residual model (R²={_r2(y, pred):.3f})")
    fig.tight_layout()
    fig.savefig(out / "scatter_pred_vs_meas.png", dpi=150)
    plt.close(fig)

    err = pred - y
    for axis, coord in ("dy", dy), ("dz", dz):
        valid = np.isfinite(coord) & np.isfinite(err)
        if valid.sum() < 50:
            continue
        fig, ax = plt.subplots(figsize=(5, 3.5))
        ax.scatter(coord[valid], err[valid], s=1, alpha=0.12)
        ax.axhline(0, color="k", lw=0.6)
        ax.set_xlabel("horizontal |d| [m]" if axis == "dy" else "dz (neighbour relative) [m]")
        ax.set_ylabel("prediction error [m/s²]")
        ax.set_title(f"Error vs {axis} (gated 1st neighbour)")
        fig.tight_layout()
        fig.savefig(out / f"error_vs_{axis}.png", dpi=150)
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("weights", type=Path, help="exported .npz from train.py")
    ap.add_argument("logs", nargs="+", type=Path, help="merged uSD CSVs")
    ap.add_argument("-o", "--out", type=Path, required=True)
    ap.add_argument("--z-floor", type=float, default=0.0)
    ap.add_argument("--mass", type=float, default=DEFAULT_MASS)
    ap.add_argument("--seed", type=int, default=0, help="same as train.py for val-block split")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    npz = np.load(args.weights)
    w = np.asarray(npz["weights"], dtype=np.float64)
    meta_train = json.loads(str(npz.get("meta", "{}")))

    rel, mask, ground, y, stats = dataset.build([str(p) for p in args.logs],
                                                z_floor=args.z_floor)
    pred = predict_batch(w, rel, mask, ground, args.mass)
    tr, va = dataset.split(len(y), seed=args.seed)
    dy, dz = neighbour_geometry(rel, mask)

    report = dict(
        weights=str(args.weights.resolve()),
        train_meta=meta_train,
        build_stats=stats,
        mass_kg=args.mass,
        combined=eval_slice(y, pred),
        val_block_split=eval_slice(y[va], pred[va]),
        train_block_split=eval_slice(y[tr], pred[tr]),
        per_flight=[],
        binned_error=[],
    )

    for path in args.logs:
        r1, m1, g1, y1, s1 = dataset.build([str(path)], z_floor=args.z_floor)
        p1 = predict_batch(w, r1, m1, g1, args.mass)
        report["per_flight"].append(dict(
            path=str(path.resolve()),
            build_stats=s1,
            metrics=eval_slice(y1, p1),
        ))

    report["binned_error"] = (
        binned_error(dy[va], dz[va], (pred - y)[va], "dy", np.linspace(0, 0.25, 6))
        + binned_error(dy[va], dz[va], (pred - y)[va], "dz", np.linspace(-0.5, 1.2, 8))
    )

    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "eval_metrics.json").write_text(json.dumps(report, indent=2) + "\n")

    lines = [
        "# Residual model evaluation (P5)",
        "",
        f"**Weights:** `{args.weights}`",
        f"**Logs:** {len(args.logs)} merged file(s)",
        "",
        "## Combined (all samples)",
        "",
        f"- n = {report['combined']['n']}",
        f"- RMSE = {report['combined']['rmse_mps2']:.4f} m/s²",
        f"- R² = {report['combined']['r2']:.4f}",
        f"- Baseline predict-zero RMSE = {report['combined']['baseline_zero_rmse']:.4f} m/s² "
        f"({report['combined']['pct_reduction_vs_zero']:.1f}% reduction)",
        f"- Baseline predict-mean RMSE = {report['combined']['baseline_mean_rmse']:.4f} m/s²",
        "",
        "## Validation blocks (contiguous split, seed={})".format(args.seed),
        "",
        f"- val RMSE = {report['val_block_split']['rmse_mps2']:.4f} m/s², "
        f"R² = {report['val_block_split']['r2']:.4f}",
        "",
        "## Per flight (same weights, no retrain)",
        "",
        "| file | n | RMSE | R² | vs zero |",
        "|---|---:|---:|---:|---:|",
    ]
    for pf in report["per_flight"]:
        m = pf["metrics"]
        name = Path(pf["path"]).name
        lines.append(f"| `{name}` | {m['n']} | {m['rmse_mps2']:.4f} | {m['r2']:.3f} | "
                       f"{m['pct_reduction_vs_zero']:.1f}% |")
    lines.extend(["", "## Binned |error| on validation blocks", ""])
    for row in report["binned_error"]:
        lines.append(f"- {row['axis']} {row['bin']} m: n={row['n']}, RMSE={row['rmse_mps2']:.4f}")
    (args.out / "eval_report.md").write_text("\n".join(lines) + "\n")

    print(f"combined RMSE {report['combined']['rmse_mps2']:.4f} m/s²  "
          f"R² {report['combined']['r2']:.4f}")
    print(f"val-block RMSE {report['val_block_split']['rmse_mps2']:.4f} m/s²")
    if not args.no_plots:
        maybe_plots(args.out, y[va], pred[va], dy[va], dz[va])
    print(f"wrote {args.out}/eval_metrics.json and eval_report.md")


if __name__ == "__main__":
    main()
