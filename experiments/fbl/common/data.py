"""Pair dataset for the FBL residual: ego = bottom vehicle (cf5), neighbour = top vehicle (cf_second), from the merged NS2 banks.

Source: the manifest of the released NS2 v2 (73 flights = 26 relabelled v1 flights + 47 usable flights of lab 2026-10-16/17); the merged
CSVs under experiments/logs/bank_* are regenerable (merge_training_bank.py) and not committed. Label = a_res (world frame, m/s^2) =
filtered measured acceleration minus the acceleration implied by the motor RPM (Omar's definition, docs/ns2_model_registry.md); the top
vehicle's own label is zeroed by convention and is not used here.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[3]
MANIFEST = ROOT / "experiments/results/ns2_pilot2_2026-10-17/training_manifest_pilot2.json"


def _prefixes(cols):
    """Column prefix of the bottom (cf5) and top (cf_second) vehicle; older merged files use cf5, cf5_, cf5_pm, cf5_A1_dz050 ..."""
    ego = next(p for p in sorted({c.split(".")[0] for c in cols if "." in c}) if p.startswith("cf5"))
    return ego, ego.replace("cf5", "cf_second", 1)


def load_flight(entry: dict, step: int = 10) -> pd.DataFrame:
    """One flight -> DataFrame at 500/step Hz with ego/neighbour state, relative geometry (r0 horizontal, z0 vertical, neighbour above > 0) and label."""
    path = ROOT / entry["path"]
    hdr = pd.read_csv(path, comment="#", nrows=0).columns
    pe, pn = _prefixes(hdr)
    d = pd.read_csv(path, comment="#")
    d = d[d.t >= 0].iloc[::step].reset_index(drop=True)
    g = lambda p, c: d[f"{p}.{c}"].values
    out = pd.DataFrame({"t": d.t.values})
    for k, c in (("x", "x"), ("y", "y"), ("z", "z"), ("vx", "vx"), ("vy", "vy"), ("vz", "vz")):
        out[f"e_{k}"] = g(pe, c)
        out[f"n_{k}"] = g(pn, c)
    # attitude: not used by the FBL residual with the upper vehicle assumed upright (flow axis +z); identity quaternion [x, y, z, w]
    # (the older merged files carry no quaternion columns)
    for k, v in (("qx", 0.0), ("qy", 0.0), ("qz", 0.0), ("qw", 1.0)):
        out[f"e_{k}"] = v
        out[f"n_{k}"] = v
    for k in ("x", "y", "z"):
        out[f"a_res_{k}"] = g(pe, f"a_res_{k}")
    out["dx"], out["dy"], out["dz"] = out.n_x - out.e_x, out.n_y - out.e_y, out.n_z - out.e_z
    out["r0"] = np.hypot(out.dx, out.dy)
    out["z0"] = out.dz
    out["flight"] = entry["folder"]
    out["scenario"] = entry["scenario"]
    out["source"] = entry.get("source", "")
    return out


def load_bank(step: int = 10, scenarios=None) -> pd.DataFrame:
    man = json.load(open(MANIFEST))["flights"]
    parts = []
    for e in man:
        if scenarios and e["scenario"] not in scenarios:
            continue
        parts.append(load_flight(e, step))
    return pd.concat(parts, ignore_index=True)
