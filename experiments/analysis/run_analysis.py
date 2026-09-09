#!/usr/bin/env python3
"""Turn flight logs into the numbers Chapter 5 already promised.

    python experiments/analysis/run_analysis.py \\
        --scenario A1 --ctrl geometric \\
        --logs path/cf1.csv path/cf2.csv \\
        [--dz-cmd 0.75] [--sidecar path/A1_<stamp>.meta.json] \\
        [--source {sim,hardware}] [--out experiments/analysis/out/]

`--logs` accepts either ONE merged CSV (all vehicles inside, `t,{name}.{field}...` --
see metrics.py's module docstring) or N per-vehicle files (ros-format or a decoded
uSD log each).

Commanded trajectory: used directly if the log already carries a setpoint
(`cmd_*` in a merged file, `ctrltarget_*` in a uSD log); otherwise reconstructed
from the scenario spec via `--sidecar` (the same anchor+slot+curve rule
`verify_formation_sim.py` uses). If neither exists, this refuses to run rather
than RMSE against nothing.

`--source` labels the report SIM or HARDWARE -- explicit, because a log NEVER
labels itself, and this must not default to claiming hardware. If the input
format can only exist in sim (record_states / `sim` format), `--source hardware`
is refused as a contradiction.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import metrics as M  # noqa: E402

sys.path.insert(0, "/home/georg/Desktop/crazyswarm2/crazyflie_examples")


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", required=True, help="scenario id, e.g. A1")
    ap.add_argument("--ctrl", required=True, choices=["geometric", "indi"],
                     help="controller under test (label only, not re-derived from the log)")
    ap.add_argument("--logs", nargs="+", required=True, help="one merged CSV, or N per-vehicle logs")
    ap.add_argument("--dz-cmd", type=float, default=None,
                     help="commanded vertical separation [m] for a stacked pair; "
                          "omit for a non-stacked scenario")
    ap.add_argument("--sidecar", default=None,
                     help="run_formation.py's <SID>_<stamp>.meta.json, needed to "
                          "reconstruct the commanded trajectory when the log has no setpoint")
    ap.add_argument("--source", choices=["sim", "hardware"], default="sim",
                     help="labels the report -- defaults to sim, never silently claims hardware")
    ap.add_argument("--out", default="experiments/analysis/out/")
    return ap


def _load_sidecar_scenario(sidecar_path: str):
    from crazyflie_examples.formations import scenarios as S
    meta = json.load(open(sidecar_path))
    sc = S.build(meta["scenario"], **meta["params"])
    return sc, meta


def main():
    args = build_parser().parse_args()
    vehicles = M.load_any(args.logs)
    if not vehicles:
        sys.exit("no vehicles loaded from --logs")

    any_sim_format = any(v.fmt == "sim" for v in vehicles.values())
    if any_sim_format and args.source == "hardware":
        sys.exit("--source hardware contradicts the input: 'sim' (record_states) format "
                  "only exists in the simulator")

    sc = meta = None
    needs_reconstruction = any(v.pos_des is None for v in vehicles.values())
    if needs_reconstruction and args.sidecar:
        sc, meta = _load_sidecar_scenario(args.sidecar)

    names_in_order = (meta["names"] if meta else list(vehicles.keys()))
    anchor = np.array(meta["anchor"]) if meta else None
    t0 = float(meta["t_start_sim"]) if meta else None
    timescale = float(meta.get("timescale", 1.0)) if meta else 1.0

    rows = []
    ts_columns = {}
    for i, name in enumerate(names_in_order):
        if name not in vehicles:
            continue
        v = vehicles[name]
        pos_des_override = None
        if v.pos_des is None and sc is not None and v.pos is not None and v.pos.shape[0] > 0:
            pos_des_override = M.commanded_from_scenario(sc, i, anchor, t0, timescale, v.t)
        row = M.vehicle_metrics(v, args.scenario, args.ctrl, len(names_in_order),
                                 pos_des_override)
        rows.append(row)
        des = pos_des_override if pos_des_override is not None else v.pos_des
        ts_columns[f"{name}.t"] = v.t
        if v.pos is not None:
            for k, ax in enumerate("xyz"):
                ts_columns[f"{name}.pos_{ax}"] = v.pos[:, k] if v.pos.shape[0] else np.array([])
        if des is not None:
            for k, ax in enumerate("xyz"):
                ts_columns[f"{name}.pos_des_{ax}"] = des[:, k]
        for field, arr in (("a_res", v.a_res), ("a_hat", v.a_hat), ("e_r", v.e_r)):
            if arr is not None:
                for k, ax in enumerate("xyz"):
                    ts_columns[f"{name}.{field}_{ax}"] = arr[:, k]

    ordered_vehicles = [vehicles[n] for n in names_in_order if n in vehicles]
    rows.append(M.formation_row(args.scenario, args.ctrl, ordered_vehicles, args.dz_cmd))

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    run_id = f"{args.scenario}_{args.ctrl}_{stamp}"

    metrics_path = out_dir / f"{run_id}_metrics.csv"
    fieldnames = sorted({k for r in rows for k in r})
    # Keep the natural reading order for the columns everyone will scan first.
    lead = ["scenario", "controller", "n_robots", "vehicle_id", "t_start", "t_end",
            "n_samples", "n_raw", "rate_hz_est", "nan_fraction",
            "pos_rmse_m", "pos_rmse_x", "pos_rmse_y", "pos_rmse_z",
            "pos_peak_m", "pos_peak_z",
            "dz_cmd_m", "dz_mean_m", "dz_err_mean_m", "dz_err_rms_m",
            "a_res_rms", "a_res_z_mean", "a_res_z_rms",
            "a_hat_res_rms", "a_hat_vs_a_res_rmse",
            "e_R_rmse", "e_R_peak", "notes"]
    fieldnames = [f for f in lead if f in fieldnames] + \
        [f for f in fieldnames if f not in lead]
    with open(metrics_path, "w") as fh:
        fh.write(",".join(fieldnames) + "\n")
        for r in rows:
            fh.write(",".join(str(r.get(f, "")) for f in fieldnames) + "\n")

    ts_path = out_dir / f"{run_id}_timeseries.csv"
    ts_cols = sorted(ts_columns)
    max_len = max((len(v) for v in ts_columns.values()), default=0)
    with open(ts_path, "w") as fh:
        fh.write(",".join(ts_cols) + "\n")
        for i in range(max_len):
            vals = []
            for c in ts_cols:
                arr = ts_columns[c]
                vals.append(f"{arr[i]:.6f}" if i < len(arr) else "")
            fh.write(",".join(vals) + "\n")

    report_path = out_dir / f"{run_id}_report.md"
    with open(report_path, "w") as fh:
        fh.write(f"# {args.scenario} / {args.ctrl} -- {stamp}\n\n")
        fh.write(f"**{'SIM' if args.source == 'sim' else 'HARDWARE'}** -- label taken "
                 f"from --source, never inferred or claimed beyond what was given.\n\n")
        fh.write("| " + " | ".join(fieldnames) + " |\n")
        fh.write("|" + "---|" * len(fieldnames) + "\n")
        for r in rows:
            fh.write("| " + " | ".join(str(r.get(f, "")) for f in fieldnames) + " |\n")

    print(f"wrote {metrics_path}\nwrote {ts_path}\nwrote {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
