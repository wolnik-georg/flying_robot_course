#!/usr/bin/env python3
"""Host replay: rnn.div=1 vs 10 on SIL residual CSV states (1 kHz steps, 100 Hz peer holds).

CS2 SIL does not expose g_rnn_div in server yaml; this is the closest open-loop substitute:
same weights, same peer geometry as archived dry-run predict phase, stepping controllerOutOfTree
at 1 ms with peer timestamps advancing each CSV row (~10 ms).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HOST = ROOT / "flying_drone_stack/firmware_app/host"
sys.path.insert(0, str(HOST))
sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/build")

import cffirmware as fw  # noqa: E402
from test_residual_nn import Fw, N_WEIGHTS  # noqa: E402


def load_csv(path: Path) -> dict[str, np.ndarray]:
    with open(path) as f:
        header = f.readline().strip().split(",")
    a = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return {n: a[:, i] for i, n in enumerate(header)}


def _row_vec(cols: dict[str, np.ndarray], name: str, row: int, prefix: str) -> np.ndarray:
    return np.array(
        [cols[f"{name}.{prefix}x"][row], cols[f"{name}.{prefix}y"][row], cols[f"{name}.{prefix}z"][row]],
        np.float32,
    )


def run_replay(
    cols: dict[str, np.ndarray],
    weights: np.ndarray,
    div: int,
    ego: str,
    peer: str,
    steps_per_row: int = 10,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (t_s, pred_z, a_res_z) subsampled at CSV rate during in-air gate.

    Ego pose/velocity linearly interpolated at 1 kHz between 100 Hz CSV rows; peer position
    moves every 1 ms but peer *timestamp* advances only every 10 ms (sparse ~100 Hz packets).
    That mismatch is what makes div=1 and div=10 differ in production.
    """
    t = cols["t"]
    n_rows = len(t)
    fw.cvar.g_rnn_div = div
    h = Fw()
    assert h.upload(weights) == 1

    pred_z = []
    a_res_z = []
    t_out = []
    peer_ms = 10_000

    for row in range(n_rows - 1):
        t0, t1 = float(t[row]), float(t[row + 1])
        dt_row = max(t1 - t0, 1e-6)

        ego_p0 = _row_vec(cols, ego, row, "")
        ego_p1 = _row_vec(cols, ego, row + 1, "")
        ego_v0 = _row_vec(cols, ego, row, "v")
        ego_v1 = _row_vec(cols, ego, row + 1, "v")
        peer_p0 = _row_vec(cols, peer, row, "")
        peer_p1 = _row_vec(cols, peer, row + 1, "")

        for k in range(steps_per_row):
            alpha = (k + 1) / steps_per_row
            ex, ey, ez = (ego_p0 + alpha * (ego_p1 - ego_p0)).tolist()
            ev = ego_v0 + alpha * (ego_v1 - ego_v0)
            px, py, pz = (peer_p0 + alpha * (peer_p1 - peer_p0)).tolist()

            h.own_pos[:] = (ex, ey, ez)
            h.own_vel[:] = ev
            h.st.position.x, h.st.position.y, h.st.position.z = ex, ey, ez
            h.st.velocity.x, h.st.velocity.y, h.st.velocity.z = float(ev[0]), float(ev[1]), float(ev[2])
            h.sp.position.x, h.sp.position.y, h.sp.position.z = ex, ey, ez

            if k == steps_per_row - 1:
                peer_ms += 10
            h.peers([np.array([px, py, pz], np.float32)], peer_ms)
            h.step(1)

        if ez > 0.05 and cols[f"{peer}.z"][row] > 0.05:
            pred_z.append(float(h.pred()[2]))
            a_res_z.append(float(cols[f"{ego}.a_res_z"][row]))
            t_out.append(t0)

    return np.array(t_out), np.array(pred_z), np.array(a_res_z)


def metrics(pred: np.ndarray, a_res: np.ndarray) -> dict:
    base = float(np.sqrt(np.mean(a_res**2))) if len(a_res) else float("nan")
    err = float(np.sqrt(np.mean((pred - a_res) ** 2))) if len(a_res) else float("nan")
    red = 100.0 * (1.0 - err / base) if base > 1e-9 else float("nan")
    if len(pred) > 50 and np.std(pred) > 1e-12 and np.std(a_res) > 1e-12:
        corr = float(np.corrcoef(pred, a_res)[0, 1])
    else:
        corr = float("nan")
    return {"rmse_pred_minus_a_res": err, "rmse_a_res": base, "reduction_pct": red, "corr_z": corr}


def lag_samples(a: np.ndarray, b: np.ndarray, max_lag: int = 15) -> int:
    """Lag (in subsample steps) that maximizes correlation between a and b."""
    if len(a) < 100:
        return 0
    best_l, best_c = 0, -2.0
    for lag in range(-max_lag, max_lag + 1):
        if lag >= 0:
            x, y = a[lag:], b[: len(b) - lag]
        else:
            x, y = a[: len(a) + lag], b[-lag:]
        if len(x) < 50:
            continue
        c = np.corrcoef(x, y)[0, 1]
        if c > best_c:
            best_c, best_l = c, lag
    return int(best_l)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--csv",
        type=Path,
        default=ROOT / "experiments/sim_validation/residual_predict.csv",
    )
    ap.add_argument(
        "--weights",
        type=Path,
        default=ROOT / "experiments/analysis/out/c2_e2e_2026-10-01/full_bank_c1_complete.npz",
    )
    ap.add_argument("--ego", default="cf231_active")
    ap.add_argument("--peer", default="cf_second")
    ap.add_argument("-o", type=Path, default=None)
    args = ap.parse_args()

    w = np.load(args.weights)["weights"].astype(np.float32)
    if w.size != N_WEIGHTS:
        print(f"weights size {w.size} != N_WEIGHTS {N_WEIGHTS}", file=sys.stderr)
        return 1
    cols = load_csv(args.csv)

    _, p1, a1 = run_replay(cols, w, div=1, ego=args.ego, peer=args.peer)
    _, p10, a10 = run_replay(cols, w, div=10, ego=args.ego, peer=args.peer)

    diff = p10 - p1
    lag = lag_samples(p1, p10)
    report = {
        "method": "host_harness_replay_from_SIL_csv",
        "csv": str(args.csv),
        "weights": str(args.weights),
        "ego": args.ego,
        "n_in_air_samples": int(len(p1)),
        "div1_vs_div10": {
            "rms_pred_z_diff_mps2": float(np.sqrt(np.mean(diff**2))),
            "max_abs_pred_z_diff_mps2": float(np.max(np.abs(diff))),
            "lag_samples_at_100hz_subsample": lag,
            "lag_ms_est": lag * 10.0,
        },
        "pred_vs_a_res_div1": metrics(p1, a1),
        "pred_vs_a_res_div10": metrics(p10, a10),
        "note": "SIL 2-drone A8 live div sweep not run (no g_rnn_div in CS2 server); "
        "CSV is A3 dry-run predict phase at 100 Hz log rate.",
    }
    text = json.dumps(report, indent=2) + "\n"
    print(text)
    if args.o:
        args.o.write_text(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
