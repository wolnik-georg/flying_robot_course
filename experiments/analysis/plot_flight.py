#!/usr/bin/env python3
"""Plot a single- or multi-drone flight against its commanded trajectory.

Run with the pyenv env (system python3's matplotlib is broken here — numpy
ABI mismatch, not a project convention choice):

    ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/plot_flight.py \\
        --scenario A1 --ctrl geometric \\
        --logs path/cf1.csv path/cf2.csv \\
        --dz-cmd 0.75 [--sidecar path/A1_<stamp>.meta.json] \\
        --source sim --out experiments/analysis/out/

Same loaders and same commanded-trajectory reconstruction as run_analysis.py
(imports metrics.py directly) so a plot and its numbers never disagree. One
PNG per call: a dashboard, not a slideshow. Panels that have no data (e.g.
e_R on a log from before 2026-09-08, or a_hat on a run without rnn.en) are
left blank with a "not in log" note instead of being silently skipped --
NaN is not the same as "nothing to show", and a missing panel that just
isn't there would look like an oversight rather than an absent signal.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import metrics as M  # noqa: E402

sys.path.insert(0, "/home/georg/Desktop/crazyswarm2/crazyflie_examples")

COLORS = ["#0E7C7B", "#C4501E", "#3F7D3B", "#6B4FA0", "#B08900"]


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--ctrl", required=True, choices=["geometric", "indi"])
    ap.add_argument("--logs", nargs="+", required=True)
    ap.add_argument("--dz-cmd", type=float, default=None)
    ap.add_argument("--sidecar", default=None)
    ap.add_argument("--source", choices=["sim", "hardware"], default="sim")
    ap.add_argument("--out", default="experiments/analysis/out/")
    return ap


def _load_sidecar_scenario(sidecar_path: str):
    from crazyflie_examples.formations import scenarios as S
    import json
    meta = json.load(open(sidecar_path))
    sc = S.build(meta["scenario"], **meta["params"])
    return sc, meta


def _plot_xy(ax, vehicles, des, names):
    for i, name in enumerate(names):
        v = vehicles[name]
        if v.pos is None or v.pos.shape[0] == 0:
            continue
        c = COLORS[i % len(COLORS)]
        ax.plot(v.pos[:, 0], v.pos[:, 1], color=c, lw=1.3, label=name)
        d = des.get(name)
        if d is not None:
            ax.plot(d[:, 0], d[:, 1], color=c, lw=1.0, ls="--", alpha=0.5)
        ax.plot(v.pos[0, 0], v.pos[0, 1], "o", color=c, ms=5)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("XY path (dashed = commanded)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(fontsize=7, loc="best")


def _plot_axis_vs_t(ax, vehicles, des, names, axis, label):
    for i, name in enumerate(names):
        v = vehicles[name]
        if v.pos is None or v.pos.shape[0] == 0:
            continue
        c = COLORS[i % len(COLORS)]
        ax.plot(v.t, v.pos[:, axis], color=c, lw=1.2, label=name)
        d = des.get(name)
        if d is not None:
            ax.plot(v.t, d[:, axis], color=c, lw=1.0, ls="--", alpha=0.5)
    ax.set_xlabel("t [s]"); ax.set_ylabel(f"{label} [m]")
    ax.set_title(f"{label} vs t (dashed = commanded)")


def _plot_pos_error(ax, vehicles, des, names):
    any_data = False
    for i, name in enumerate(names):
        v = vehicles[name]
        d = des.get(name)
        if v.pos is None or v.pos.shape[0] == 0 or d is None:
            continue
        err = np.linalg.norm(v.pos - d, axis=1)
        ax.plot(v.t, err, color=COLORS[i % len(COLORS)], lw=1.2, label=name)
        any_data = True
    if not any_data:
        ax.text(0.5, 0.5, "no commanded trajectory available", ha="center", va="center",
                 transform=ax.transAxes, fontsize=9, color="grey")
    ax.set_xlabel("t [s]"); ax.set_ylabel("|pos error| [m]")
    ax.set_title("Position tracking error")
    if any_data:
        ax.legend(fontsize=7)


def _plot_separation(ax, vehicles, names, dz_cmd):
    if len(names) < 2:
        ax.text(0.5, 0.5, "single vehicle -- no separation to plot", ha="center", va="center",
                 transform=ax.transAxes, fontsize=9, color="grey")
        ax.set_title("Vertical separation")
        return
    a, b = vehicles[names[0]], vehicles[names[1]]
    if a.pos is None or b.pos is None or a.pos.shape[0] == 0 or b.pos.shape[0] == 0:
        ax.text(0.5, 0.5, "no data", ha="center", va="center",
                 transform=ax.transAxes, fontsize=9, color="grey")
        ax.set_title("Vertical separation")
        return
    t_lo, t_hi = max(a.t[0], b.t[0]), min(a.t[-1], b.t[-1])
    if t_hi <= t_lo:
        ax.text(0.5, 0.5, "no overlapping window", ha="center", va="center",
                 transform=ax.transAxes, fontsize=9, color="grey")
        ax.set_title("Vertical separation")
        return
    grid = np.linspace(t_lo, t_hi, 200)
    za = np.interp(grid, a.t, a.pos[:, 2])
    zb = np.interp(grid, b.t, b.pos[:, 2])
    dz = np.abs(za - zb)
    ax.plot(grid, dz, color=COLORS[0], lw=1.3, label=f"|{names[0]} - {names[1]}| z")
    if dz_cmd is not None:
        ax.axhline(dz_cmd, color="grey", lw=1.0, ls="--", label="commanded")
    ax.set_xlabel("t [s]"); ax.set_ylabel("dz [m]")
    ax.set_title("Vertical separation")
    ax.legend(fontsize=7)


def _plot_vec_field(ax, vehicles, names, field, title, ylabel):
    any_data = False
    for i, name in enumerate(names):
        v = vehicles[name]
        arr = getattr(v, field)
        if arr is None or arr.shape[0] == 0:
            continue
        mag = np.linalg.norm(arr, axis=1)
        ax.plot(v.t, mag, color=COLORS[i % len(COLORS)], lw=1.2, label=f"{name} |{field}|")
        any_data = True
    if not any_data:
        ax.text(0.5, 0.5, f"{field} not in log", ha="center", va="center",
                 transform=ax.transAxes, fontsize=9, color="grey")
    ax.set_xlabel("t [s]"); ax.set_ylabel(ylabel)
    ax.set_title(title)
    if any_data:
        ax.legend(fontsize=7)


def _plot_a_hat_vs_a_res(ax, vehicles, names):
    any_data = False
    for i, name in enumerate(names):
        v = vehicles[name]
        if v.a_res is None or v.a_hat is None:
            continue
        m_res = np.linalg.norm(v.a_res, axis=1)
        m_hat = np.linalg.norm(v.a_hat, axis=1)
        c = COLORS[i % len(COLORS)]
        ax.plot(v.t, m_res, color=c, lw=1.2, label=f"{name} a_res")
        ax.plot(v.t, m_hat, color=c, lw=1.0, ls="--", label=f"{name} a_hat")
        any_data = True
    if not any_data:
        ax.text(0.5, 0.5, "a_hat (rnn.pred_*) not in log", ha="center", va="center",
                 transform=ax.transAxes, fontsize=9, color="grey")
    ax.set_xlabel("t [s]"); ax.set_ylabel("|a| [m/s^2]")
    ax.set_title("Predicted vs measured residual")
    if any_data:
        ax.legend(fontsize=7)


def _text_summary(ax, rows, source):
    ax.axis("off")
    lines = [f"SOURCE: {'SIM' if source == 'sim' else 'HARDWARE'}", ""]
    for r in rows:
        if r["vehicle_id"] == "__formation__":
            if r.get("dz_cmd_m") == r.get("dz_cmd_m"):  # not NaN
                lines.append(f"formation: dz_cmd={r['dz_cmd_m']:.3f}  "
                              f"dz_mean={r.get('dz_mean_m', float('nan')):.3f}  "
                              f"sag={r.get('dz_err_mean_m', float('nan')):.3f} m")
            continue
        lines.append(f"{r['vehicle_id']}: pos_rmse={r.get('pos_rmse_m', float('nan')):.3f} m  "
                      f"n={r.get('n_samples', 0)}/{r.get('n_raw', 0)}")
        if r.get("a_res_rms") == r.get("a_res_rms"):
            lines.append(f"   a_res_rms={r['a_res_rms']:.3f}  e_R_rmse={r.get('e_R_rmse', float('nan')):.4f}")
    ax.text(0.02, 0.95, "\n".join(lines), transform=ax.transAxes, fontsize=8,
            family="monospace", va="top")


def main():
    args = build_parser().parse_args()
    vehicles = M.load_any(args.logs)
    if not vehicles:
        sys.exit("no vehicles loaded from --logs")

    sc = meta = None
    needs_reconstruction = any(v.pos_des is None for v in vehicles.values())
    if needs_reconstruction and args.sidecar:
        sc, meta = _load_sidecar_scenario(args.sidecar)

    names = meta["names"] if meta else list(vehicles.keys())
    names = [n for n in names if n in vehicles]
    anchor = np.array(meta["anchor"]) if meta else None
    t0 = float(meta["t_start_sim"]) if meta else None
    timescale = float(meta.get("timescale", 1.0)) if meta else 1.0

    des = {}
    rows = []
    for i, name in enumerate(names):
        v = vehicles[name]
        d = v.pos_des
        if d is None and sc is not None and v.pos is not None and v.pos.shape[0] > 0:
            d = M.commanded_from_scenario(sc, i, anchor, t0, timescale, v.t)
        des[name] = d
        rows.append(M.vehicle_metrics(v, args.scenario, args.ctrl, len(names), d))
    rows.append(M.formation_row(args.scenario, args.ctrl,
                                 [vehicles[n] for n in names], args.dz_cmd))

    fig, axes = plt.subplots(3, 3, figsize=(16, 13))
    fig.suptitle(f"{args.scenario} / {args.ctrl} -- {'SIM' if args.source == 'sim' else 'HARDWARE'}",
                 fontsize=13, fontweight="bold")

    _plot_xy(axes[0, 0], vehicles, des, names)
    _plot_axis_vs_t(axes[0, 1], vehicles, des, names, 2, "z")
    _plot_pos_error(axes[0, 2], vehicles, des, names)
    _plot_separation(axes[1, 0], vehicles, names, args.dz_cmd)
    _plot_vec_field(axes[1, 1], vehicles, names, "a_res", "Residual acceleration", "|a_res| [m/s^2]")
    _plot_vec_field(axes[1, 2], vehicles, names, "e_r", "Geometric attitude error", "|e_R|")
    _plot_a_hat_vs_a_res(axes[2, 0], vehicles, names)
    _text_summary(axes[2, 1], rows, args.source)
    axes[2, 2].axis("off")

    fig.tight_layout(rect=(0, 0, 1, 0.96))

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    out_path = out_dir / f"{args.scenario}_{args.ctrl}_{stamp}_dashboard.png"
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
