#!/usr/bin/env python3
"""A8 setpoint-path bisect: cmdFullState vs HLC; first 12 s after takeoff."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np

REPO = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent / "out" / "ns2_closed_loop_sil"
ANALYSIS = Path(__file__).resolve().parent

for _rn in ("rclpy", "rclpy.node", "rclpy.time", "rosgraph_msgs", "rosgraph_msgs.msg"):
    sys.modules.setdefault(_rn, MagicMock())
sys.path.insert(0, str(ANALYSIS))

from ns2_closed_loop_sil_sim import build_scenario, robot_world_kinematics, run_episode  # noqa: E402

TAKEOFF_S = 4.0
WINDOW_S = 12.0
ANCHOR = np.array([0.0, 0.0, 0.5])


def _window_metrics(r: dict) -> dict:
    t = np.asarray(r.get("_t") or [], float)
    if len(t) == 0:
        return {"error": "no trace"}
    m = (t >= TAKEOFF_S) & (t <= TAKEOFF_S + WINDOW_S)
    pb = np.asarray(r["_pos_bot"])[m]
    pt = np.asarray(r["_pos_top"])[m]
    sb = np.asarray(r["_sp_bot"])[m]
    st = np.asarray(r["_sp_top"])[m]
    eb = pb - sb
    et = pt - st
    lat_b = np.linalg.norm(eb[:, :2], axis=1)
    lat_t = np.linalg.norm(et[:, :2], axis=1)
    tw = t[m]
    idx_b = int(np.argmax(lat_b > 0.10)) if np.any(lat_b > 0.10) else -1
    first_lat_s = float(tw[idx_b]) if idx_b >= 0 else None
    detail = {}
    if first_lat_s is not None:
        k = idx_b
        t_sc = max(0.0, first_lat_s - TAKEOFF_S - float(r.get("_converge_s", 0)))
        sc = build_scenario("A8", r.get("_cfg", {}))
        _, vb, ab, _, _ = robot_world_kinematics(sc, 0, t_sc, ANCHOR, vel_mode="analytic")
        detail = {
            "t_s": first_lat_s,
            "t_sc_s": t_sc,
            "lat_err_bot_cm": float(lat_b[k] * 100),
            "cmd_vel_bot": vb.tolist(),
            "cmd_acc_bot": ab.tolist(),
        }
    return {
        "bot_lat_rms_cm": float(np.sqrt(np.mean(lat_b**2)) * 100),
        "top_lat_rms_cm": float(np.sqrt(np.mean(lat_t**2)) * 100),
        "bot_z_mean_cm": float(np.mean(pb[:, 2]) * 100),
        "top_z_mean_cm": float(np.mean(pt[:, 2]) * 100),
        "bot_z_err_mean_cm": float(np.mean(eb[:, 2]) * 100),
        "top_z_err_mean_cm": float(np.mean(et[:, 2]) * 100),
        "first_lat_gt_10cm_bot_s": first_lat_s,
        "first_lat_event": detail,
        "final_pos_bot": r.get("final_pos_bot"),
        "final_pos_top": r.get("final_pos_top"),
        "max_tilt_bot": r.get("tracking_bottom_cm", {}).get("max_tilt_deg"),
        "max_tilt_top": r.get("tracking_top_cm", {}).get("max_tilt_deg"),
    }


def run_mode(mode: str, *, converge_s: float) -> dict:
    cfg = {
        "label": f"a8_{mode}",
        "scenario": "A8",
        "downwash": False,
        "skip_rnn_upload": True,
        "duration_s": TAKEOFF_S + WINDOW_S + 2.0,
        "setpoint_mode": mode,
        "converge_s": converge_s,
        "record_gain_audit": mode == "cmd_fd",
        "n_crossings": 0,
    }
    r = run_episode(cfg)
    r["_t"] = r.pop("_trace_t", None)
    # inject dense trace via re-run with custom logging — use episode logs from run
    # Re-run with log_hz=200 for window
    cfg["log_hz"] = 200
    r2 = run_episode(cfg)
    # pull from subprocess-less: run_episode doesn't export raw trace; parse via second call internals
    # Store cfg for t_sc lookup
    r2["_cfg"] = cfg
    r2["_converge_s"] = converge_s
    # Attach trace by running lightweight duplicate — read from returned logs if we add them
    return {**_window_metrics_from_run(cfg), "mode": mode, "converge_s": converge_s}


def _window_metrics_from_run(cfg: dict) -> dict:
    """Run episode and extract 12 s window from log arrays."""
    from ns2_closed_loop_sil_sim import run_episode as run_ep

    r = run_ep(cfg)
    # run_episode doesn't return raw logs — add trace export
    cfg = dict(cfg)
    cfg["return_logs"] = True
    r = run_ep(cfg)
    return r


if __name__ == "__main__":
    print("use ns2_closed_loop_sil_probe.py driver", file=sys.stderr)
