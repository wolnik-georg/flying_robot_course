#!/usr/bin/env python3
"""One-off probes: gain audit, A8 setpoint bisect, hardware compare."""

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

from ns2_closed_loop_sil_metrics import _test_tilt_metric, max_tilt_deg, tilt_from_quat_deg  # noqa: E402
from ns2_closed_loop_sil_sim import build_scenario, robot_world_kinematics, run_episode  # noqa: E402

TAKEOFF_S = 4.0
WIN = 12.0
ANCHOR = np.array([0.0, 0.0, 0.5])


def window_stats(r: dict, converge_s: float) -> dict:
    logs = r["logs"]
    t = np.asarray(logs["t"], float)
    m = (t >= TAKEOFF_S) & (t <= TAKEOFF_S + WIN)
    pb = np.asarray(logs["pos_bot"], float)[m]
    pt = np.asarray(logs["pos_top"], float)[m]
    sb = np.asarray(logs["sp_bot"], float)[m]
    st = np.asarray(logs["sp_top"], float)[m]
    eb, et = pb - sb, pt - st
    lat_b = np.linalg.norm(eb[:, :2], axis=1)
    lat_t = np.linalg.norm(et[:, :2], axis=1)
    tw = t[m]
    first = None
    if np.any(lat_b > 0.10):
        k = int(np.where(lat_b > 0.10)[0][0])
        t_sc = max(0.0, float(tw[k]) - TAKEOFF_S - converge_s)
        sc = build_scenario("A8", {"passes": 4})
        _, vb, ab, _, _ = robot_world_kinematics(sc, 0, t_sc, ANCHOR, vel_mode="analytic")
        first = {
            "t_s": float(tw[k]),
            "t_sc_s": t_sc,
            "lat_err_bot_cm": float(lat_b[k] * 100),
            "cmd_vel_bot_m_s": vb.tolist(),
            "cmd_acc_bot_m_s2": ab.tolist(),
        }
    return {
        "bot_lat_rms_cm": float(np.sqrt(np.mean(lat_b**2)) * 100),
        "top_lat_rms_cm": float(np.sqrt(np.mean(lat_t**2)) * 100),
        "bot_mean_z_err_cm": float(np.mean(eb[:, 2]) * 100),
        "top_mean_z_err_cm": float(np.mean(et[:, 2]) * 100),
        "first_lat_gt_10cm_bot": first,
        "final_pos_bot": r.get("final_pos_bot"),
        "final_pos_top": r.get("final_pos_top"),
        "max_tilt_bot_deg": r.get("tracking_bottom_cm", {}).get("max_tilt_deg"),
        "max_tilt_top_deg": r.get("tracking_top_cm", {}).get("max_tilt_deg"),
    }


def run_a8(mode: str, converge_s: float) -> dict:
    cfg = {
        "scenario": "A8",
        "downwash": False,
        "skip_rnn_upload": True,
        "duration_s": TAKEOFF_S + WIN + 2.0,
        "setpoint_mode": mode,
        "converge_s": converge_s,
        "log_hz": 200,
        "return_logs": True,
        "n_crossings": 0,
        "record_gain_audit": False,
    }
    if mode == "cmd_fd":
        cfg["record_gain_audit"] = True
    r = run_episode(cfg)
    return {"mode": mode, "converge_s": converge_s, **window_stats(r, converge_s), "gain_audit": r.get("gain_audit")}


def run_a1() -> dict:
    r = run_episode(
        {
            "scenario": "A1",
            "downwash": False,
            "skip_rnn_upload": True,
            "duration_s": 22.0,
            "setpoint_mode": "cmd_fd",
            "converge_s": 0.0,
            "record_gain_audit": True,
            "return_logs": False,
        }
    )
    return r


def hw_a8_compare() -> dict:
    """Network-off A8 cf5: A8_2026-10-05_17-39-27."""
    csv = REPO / "experiments/logs/A8_cf5_2026-10-05_17-39-27.csv"
    if not csv.is_file():
        return {"error": f"missing {csv}"}
    sys.path.insert(0, str(REPO / "experiments/analysis"))
    from meeting_hw_common import load_controls_csv, z_command_for_vehicle, meta_json_for_csv

    meta = meta_json_for_csv(csv)
    _, cols = load_controls_csv(csv)
    from meeting_hw_common import controls_tilt_deg

    t = cols["time_s"]
    pz = cols["pos_z"]
    spz = z_command_for_vehicle(meta, "cf5", t)
    tilt = controls_tilt_deg(cols)
    roll = cols["roll"]
    pitch = cols["pitch"]
    ez = (pz - spz) * 100
    return {
        "flight": "A8_2026-10-05_17-39-27 (cf5, network-off cohort)",
        "max_tilt_deg": float(np.nanmax(tilt)),
        "max_abs_roll_deg": float(np.nanmax(np.abs(roll))),
        "max_abs_pitch_deg": float(np.nanmax(np.abs(pitch))),
        "z_err_mean_cm": float(np.nanmean(ez)),
        "z_err_min_cm": float(np.nanmin(ez)),
    }


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    _test_tilt_metric()
    report = {"tilt_unit_test_deg_30": tilt_from_quat_deg(__import__("rowan").from_euler(0.0, np.radians(30.0), 0.0))}

    a1 = run_a1()
    report["a1_after_init_reconfigure"] = {
        "tracking_bottom_cm": a1.get("tracking_bottom_cm"),
        "tracking_top_cm": a1.get("tracking_top_cm"),
        "final_pos_bot": a1.get("final_pos_bot"),
        "final_pos_top": a1.get("final_pos_top"),
        "gain_audit": a1.get("gain_audit"),
    }

    report["a8_bisect_12s"] = [
        run_a8("cmd_fd", 0.0),
        run_a8("cmd_analytic", 0.0),
        run_a8("hlc", 3.5),
    ]

    full_a8_hlc = run_episode(
        {
            "scenario": "A8",
            "downwash": False,
            "skip_rnn_upload": True,
            "duration_s": 33.0,
            "setpoint_mode": "hlc",
            "converge_s": 3.5,
            "log_hz": 100,
            "return_logs": False,
        }
    )
    report["a8_hlc_full_33s"] = {
        "tracking_bottom_cm": full_a8_hlc.get("tracking_bottom_cm"),
        "tracking_top_cm": full_a8_hlc.get("tracking_top_cm"),
        "partner_ok": full_a8_hlc.get("partner_ok"),
        "final_pos_bot": full_a8_hlc.get("final_pos_bot"),
        "final_pos_top": full_a8_hlc.get("final_pos_top"),
    }

    report["hardware_ref"] = hw_a8_compare()
    path = OUT / "probe_2026-10-07.json"
    path.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
