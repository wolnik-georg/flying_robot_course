#!/usr/bin/env python3
"""Rung 2 single-tick forensics and A1 segment metrics."""

from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
ANALYSIS = Path(__file__).resolve().parent
BUILD = Path("/home/georg/Desktop/crazyflie-firmware/build")
sys.path.insert(0, str(ANALYSIS))
sys.path.insert(0, str(BUILD))

from indi_replay_command import commanded_total_thrust_N, radio_csv_for_meta, supply_voltage_on_usd_timeline
from indi_replay_config import apply_ours_globals
from indi_replay_ladder import commanded_thrust_on_ticks, steady_indices, thrust_gate_metrics
from indi_replay_worker import WARMUP_TICKS

G = 9.81


def git_audit() -> dict:
    files = [
        "flying_drone_stack/firmware_app/src/lib.rs",
        "flying_drone_stack/firmware_app/traj_iface.c",
        "flying_drone_stack/firmware_app/src/residual_nn.rs",
    ]
    log = subprocess.check_output(
        ["git", "-C", str(REPO), "log", "--since=2026-10-02 19:00", "--oneline", "--", *files],
        text=True,
    ).strip()
    commits = [ln for ln in log.splitlines() if ln.strip()]
    return {
        "since": "2026-10-02 19:00",
        "commits": commits,
        "control_law_change_judgement": (
            "Commits are NS2/RNN peer-sync and 100 Hz hold (rnn.en=0 on these flights). "
            "No intentional change to geometric/INDI f_d/clamps in lib.rs for 2026-10-02 flights; "
            "host .so is current source — no firmware rebuild performed."
        ),
    }


def quat_to_rot(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    return np.array(
        [
            [qw * qw + qx * qx - qy * qy - qz * qz, 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy)],
            [2.0 * (qx * qy + qw * qz), qw * qw - qx * qx + qy * qy - qz * qz, 2.0 * (qy * qz - qw * qx)],
            [2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), qw * qw - qx * qx - qy * qy + qz * qz],
        ]
    )


def run_through_ticks(npz, flight_cfg, capture: dict[int, str]) -> dict[str, dict]:
    """One forward pass; capture ctl/sp/st at tick indices listed in capture."""
    import cffirmware as fw

    z = np.load(npz)
    apply_ours_globals(fw, flight_cfg, "L0")
    fw.controllerOutOfTreeInit()
    sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
    want = set(capture.keys())
    max_t = max(want)
    saved: dict[str, dict] = {}
    for tick in range(max_t + 1):
        i = tick
        p, v, q = z["pos"][i], z["vel"][i], z["quat"][i]
        spp = z["sp_pos"][i]
        sp.position.x, sp.position.y, sp.position.z = map(float, spp)
        sv = z["sp_vel"][i]
        sp.velocity.x, sp.velocity.y, sp.velocity.z = map(float, sv)
        sa = z["sp_acc"][i]
        sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = map(float, sa)
        sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs
        sp.mode.yaw = fw.modeAbs
        sp.attitude.yaw = math.degrees(float(z["yaw_d_rad"][i]))
        sens.gyro.x, sens.gyro.y, sens.gyro.z = map(float, z["gyro_deg_s"][i])
        ag = z["acc_g"][i]
        sens.acc.x, sens.acc.y, sens.acc.z = map(float, ag)
        st.position.x, st.position.y, st.position.z = map(float, p)
        st.velocity.x, st.velocity.y, st.velocity.z = map(float, v)
        st.attitudeQuaternion.w, st.attitudeQuaternion.x, st.attitudeQuaternion.y, st.attitudeQuaternion.z = map(float, q)
        rp = z["rpm"][i]
        fw.oot_set_rpm(int(rp[0]), int(rp[1]), int(rp[2]), int(rp[3]))
        fw.controllerOutOfTree(ctl, sp, sens, st, tick)
        if tick in want:
            saved[capture[tick]] = {
                "ctl": (float(ctl.thrustSi), float(ctl.torqueX), float(ctl.torqueY), float(ctl.torqueZ)),
                "a_res": [fw.oot_get_a_res(j) for j in range(3)],
                "tau": [fw.oot_get_tau(j) for j in range(3)],
                "e_r": [fw.oot_get_e_r(j) for j in range(3)],
                "i": i,
            }
    return saved, z


def hand_f_d_components(z, i, flight_cfg, fw_a_res, res_sign: int = 1):
    pos = z["pos"][i]
    vel = z["vel"][i]
    sp_p = z["sp_pos"][i]
    sp_v = z["sp_vel"][i]
    sp_a = z["sp_acc"][i]
    ep = sp_p - pos
    ev = sp_v - vel
    pos_g = flight_cfg["pos"]
    kp_xy, kp_z = float(pos_g["kp_xy"]), float(pos_g["kp_z"])
    kv_xy, kv_z = float(pos_g["kv_xy"]), float(pos_g["kv_z"])
    mass = float(flight_cfg["indi"]["mass"])
    q = z["quat"][i]
    R = quat_to_rot(*map(float, q))
    bz = R[:, 2]
    gz = G  # ENABLE_TILT_GRAVITY_COMP off in default build path for hand check
    pd = np.array([kp_xy * ep[0], kp_xy * ep[1], kp_z * ep[2]])
    vd = np.array([kv_xy * ev[0], kv_xy * ev[1], kv_z * ev[2]])
    a_indi = np.array(fw_a_res) * res_sign
    f_d = sp_a + pd + vd + np.array([0, 0, gz]) + a_indi
    thrust_vec = f_d * mass
    thrust_dot = float(np.dot(thrust_vec, bz))
    clamp_en = int(flight_cfg["indi"].get("clamp_en", 11))
    tmax = float(flight_cfg["indi"].get("thrust_max", 0.8))
    tilt_max = float(flight_cfg["indi"].get("tilt_max_deg", 30))
    active = []
    if clamp_en & 0b1000 and thrust_dot >= tmax - 1e-4:
        active.append(f"thrust_ceiling (bit3) at g_indi_thrust_max={tmax} N")
    if clamp_en & 0b0100 and thrust_vec[2] > 0:
        tan_max = math.tan(math.radians(tilt_max))
        h = math.hypot(thrust_vec[0], thrust_vec[1])
        if h > thrust_vec[2] * tan_max + 1e-9:
            active.append(f"tilt_clamp (bit2) tilt_max_deg={tilt_max}")
    return {
        "ep_m": ep.tolist(),
        "ev_mps": ev.tolist(),
        "f_d_terms_mps2": {
            "sp_acc": sp_a.tolist(),
            "Kp_ep": pd.tolist(),
            "Kv_ev": vd.tolist(),
            "g_z": gz,
            "a_indi_res_sign": a_indi.tolist(),
        },
        "f_d_sum_mps2": f_d.tolist(),
        "thrust_vec_N": thrust_vec.tolist(),
        "thrust_body_z_N_hand": thrust_dot,
        "clamp_en_bits": clamp_en,
        "clamp_active_hand": active,
    }


def pick_ticks(spec, z) -> dict:
    warm = int(z["warmup_start"]) + WARMUP_TICKS
    spz = z["sp_pos"][:, 2]
    steady = np.arange(warm, len(z["pos"]))
    steady = steady[spz[steady] >= 0.9 * spz.max()]
    label = spec["label"]
    out = {}
    if label.startswith("A8"):
        out["mid_hover"] = int(steady[len(steady) // 2])
        # crossing: min |x position error| during steady
        ex = np.abs(z["pos"][steady, 0] - z["sp_pos"][steady, 0])
        out["crossing"] = int(steady[int(np.argmin(ex))])
    if label.startswith("A1"):
        out["mid_hold"] = int(steady[len(steady) // 2])
    return out


def run_forensic_ticks(spec, npz: Path, flight_cfg: dict, run_dir: Path) -> list:
    z = np.load(npz)
    meta_path = spec["meta"]
    picks = pick_ticks(spec, z)
    cap = {tix: name for name, tix in picks.items()}
    snap, z = run_through_ticks(npz, flight_cfg, cap)
    radio = radio_csv_for_meta(meta_path)
    lag = float(z["lag_s"])
    vbat = supply_voltage_on_usd_timeline(z["t_usd"], lag, radio)
    rows = []
    for name, tix in picks.items():
        s = snap[name]
        i = s["i"]
        thrust_si, tx, ty, tz = s["ctl"]
        a_res = s["a_res"]
        hand = hand_f_d_components(z, i, flight_cfg, a_res, res_sign=int(flight_cfg.get("res_sign", 1)))
        cmd = float(commanded_total_thrust_N(z["motor_pwm"][i], vbat[i]))
        scen_z = float(z["sp_pos_scen"][i, 2]) if "sp_pos_scen" in z.files else float("nan")
        rows.append(
            {
                "tick_name": name,
                "tick": tix,
                "pos_z": float(z["pos"][i, 2]),
                "ctrltarget_z": float(z["sp_pos"][i, 2]),
                "scenario_sp_z": scen_z,
                "ctrltarget_minus_scenario_z_m": float(z["sp_pos"][i, 2] - scen_z) if np.isfinite(scen_z) else None,
                "flown_command_N": cmd,
                "replay_thrustSi_N": thrust_si,
                "oot_get_a_res": a_res,
                "oot_get_tau": s["tau"],
                "oot_get_e_r": s["e_r"],
                "hand_f_d": hand,
                "thrust_max_param_N": float(flight_cfg["indi"].get("thrust_max", 0.8)),
                "replay_at_thrust_max": thrust_si >= float(flight_cfg["indi"].get("thrust_max", 0.8)) - 0.02,
            }
        )
    out_path = run_dir / "forensics.json"
    out_path.write_text(json.dumps(rows, indent=2))
    return rows


def a1_segment_metrics(spec, z, rep, steady: np.ndarray) -> dict:
    idx_map = {int(t): i for i, t in enumerate(rep["ticks"])}
    sel = np.array([idx_map[t] for t in steady if t in idx_map])
    cmd = commanded_thrust_on_ticks(z, spec["meta"], steady)
    rep_th = rep["thrust_si"][sel]
    t0 = int(steady[0])
    first3 = steady[steady < t0 + 3000]
    hold = steady[steady >= t0 + 3000]
    def seg(ticks):
        if len(ticks) < 10:
            return {"n": len(ticks), "error": "too few samples"}
        sidx = [idx_map[t] for t in ticks if t in idx_map]
        c = commanded_thrust_on_ticks(z, spec["meta"], ticks)
        r = rep["thrust_si"][sidx]
        m = thrust_gate_metrics(c, r, ki_z=0.0, mass=0.041)
        return m
    return {"first_3s_after_steady": seg(first3), "hold_after_3s": seg(hold)}
