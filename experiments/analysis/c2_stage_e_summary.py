#!/usr/bin/env python3
"""Summarise Stage E SIL logs (predict vs compensate) for docs/40."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def load(path: Path) -> dict[str, np.ndarray]:
    with path.open() as f:
        header = f.readline().rstrip("\n").split(",")
    a = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if a.size == 0:
        raise SystemExit(f"{path}: no rows")
    return {n: a[:, i] for i, n in enumerate(header)}


def drone_names(cols: dict[str, np.ndarray]) -> list[str]:
    return sorted({k.split(".", 1)[0] for k in cols if k.endswith(".x") and "rnn_pred" not in k})


def summarise(label: str, cols: dict[str, np.ndarray]) -> dict:
    names = drone_names(cols)
    t = cols["t"]
    m = t >= t[0] + 0.15 * (t[-1] - t[0])
    out = {"label": label, "n_rows": int(len(t)), "drones": {}}
    for n in names:
        pred_z = f"{n}.rnn_pred_z"
        a_res_z = f"{n}.a_res_z"
        pos_z = f"{n}.z"
        cmd_z = f"{n}.cmd_z"
        z = cols[pos_z][m] if pos_z in cols else np.zeros(1)
        pred = cols[pred_z][m] if pred_z in cols else np.zeros(1)
        meas = cols[a_res_z][m] if a_res_z in cols else np.zeros(1)
        cmd = cols[cmd_z][m] if cmd_z in cols else np.zeros(1)
        rmse = float(np.sqrt(np.mean((pred - meas) ** 2))) if len(pred) else float("nan")
        out["drones"][n] = dict(
            z_min=float(np.min(z)),
            z_max=float(np.max(z)),
            cmd_z_max=float(np.max(np.abs(cmd))),
            rnn_pred_z_rms=float(np.sqrt(np.mean(pred ** 2))),
            rmse_pred_vs_a_res=rmse,
        )
    out["diverged"] = any(v["z_max"] > 3.0 or v["z_min"] < -0.5 for v in out["drones"].values())
    out["meaningful_flight"] = any(
        v["cmd_z_max"] > 0.2 or (v["z_max"] - v["z_min"]) > 0.15 for v in out["drones"].values()
    )
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--predict", type=Path, required=True)
    ap.add_argument("--compensate", type=Path, required=True)
    ap.add_argument("-o", type=Path, required=True)
    args = ap.parse_args()

    pred = summarise("predict_rnn_en_0", load(args.predict))
    comp = summarise("compensate_rnn_en_1", load(args.compensate))
    flew = pred["meaningful_flight"] and comp["meaningful_flight"]
    gate = {
        "note": "Desk/SIL go-no-fly gate — not evidence the model is good.",
        "weights": "train_without_A1_12-51-16.npz (LOO, trained on A3×3)",
        "scenario": "A3 dz=0.30 neuralswarm backend",
        "predict": pred,
        "compensate": comp,
        "meaningful_flight": flew,
        "pass": flew and not comp["diverged"],
        "inconclusive_reason": None if flew else (
            "EKF/mocap did not converge — logged state stayed at origin (see c2_e2e_client_*.log)."
        ),
    }
    args.o.write_text(json.dumps(gate, indent=2) + "\n")
    print(json.dumps(gate, indent=2))


if __name__ == "__main__":
    main()
