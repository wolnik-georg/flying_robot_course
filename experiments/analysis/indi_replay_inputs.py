"""Build 1 kHz replay input streams from uSD + scenario meta (500 Hz hold x2)."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
TOOLS = REPO / "flying_drone_stack/tools"
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(Path("/home/georg/Desktop/crazyswarm2/crazyflie_examples")))

from decode_usd_log import load as load_usd  # noqa: E402
from find_flight_window import commanded_trajectory, find_offset  # noqa: E402

SPIKE_DELTA_RPM = 2000.0
WARMUP_Z_M = 0.35


def _quat_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qw, qx, qy, qz


def _clean_rpm(rpm: np.ndarray, dshot: np.ndarray | None) -> np.ndarray:
    out = rpm.astype(float).copy()
    if dshot is not None and len(dshot) == len(rpm):
        bad = np.abs(dshot - rpm) > SPIKE_DELTA_RPM
        good = ~bad & np.isfinite(rpm)
        if good.sum() >= 2:
            idx = np.arange(len(rpm))
            out[bad] = np.interp(idx[bad], idx[good], rpm[good])
    else:
        d = np.abs(np.diff(out, prepend=out[0]))
        bad = d > SPIKE_DELTA_RPM
        good = ~bad & np.isfinite(out)
        if good.sum() >= 2:
            idx = np.arange(len(out))
            out[bad] = np.interp(idx[bad], idx[good], out[good])
    return out


def build_replay_npz(
    usd_bin: Path,
    meta_json: Path,
    role: str = "bottom",
    out_npz: Path | None = None,
) -> dict:
    meta = json.loads(meta_json.read_text())
    d = load_usd(str(usd_bin))
    t = np.asarray(d["t"], float)
    pos = np.stack([d["x"], d["y"], d["z"]], axis=1)
    vel = np.stack([d["vx"], d["vy"], d["vz"]], axis=1)
    rpy_deg = np.stack([d["roll_deg"], d["pitch_deg"], d["yaw_deg"]], axis=1)
    gyro = np.stack([d["gyro_x"], d["gyro_y"], d["gyro_z"]], axis=1)
    acc = np.stack([d["acc_x"], d["acc_y"], d["acc_z"]], axis=1)
    sp_pos = np.stack([d["ctrltarget_x"], d["ctrltarget_y"], d["ctrltarget_z"]], axis=1)
    rpm = np.stack(
        [
            _clean_rpm(d["motor_m1_rpm"], d.get("rpm_m1")),
            _clean_rpm(d["motor_m2_rpm"], d.get("rpm_m2")),
            _clean_rpm(d["motor_m3_rpm"], d.get("rpm_m3")),
            _clean_rpm(d["motor_m4_rpm"], d.get("rpm_m4")),
        ],
        axis=1,
    )
    motor_pwm = np.stack(
        [d["motor_m1"], d["motor_m2"], d["motor_m3"], d["motor_m4"]],
        axis=1,
    )

    ts_cmd, cmd = commanded_trajectory(meta, role)
    vel_cmd = np.gradient(cmd, ts_cmd, axis=0)
    acc_cmd = np.gradient(vel_cmd, ts_cmd, axis=0)

    total = float(meta["duration"])
    lag, mse, _ = find_offset(t, pos, ts_cmd, cmd, search_lo=t[0], search_hi=t[-1] - total)
    if lag is None:
        raise RuntimeError("find_offset failed — cannot align scenario to uSD")

    margin = 3.0
    lo, hi = lag - margin, lag + total + margin
    mask = (t >= lo) & (t <= hi)
    t = t[mask]
    pos, vel, rpy_deg, gyro, acc, sp_pos, rpm, motor_pwm = (
        pos[mask],
        vel[mask],
        rpy_deg[mask],
        gyro[mask],
        acc[mask],
        sp_pos[mask],
        rpm[mask],
        motor_pwm[mask],
    )

    t_scen = t - lag
    sp_vel = np.stack([np.interp(t_scen, ts_cmd, vel_cmd[:, i]) for i in range(3)], axis=1)
    sp_acc = np.stack([np.interp(t_scen, ts_cmd, acc_cmd[:, i]) for i in range(3)], axis=1)
    yaw_d = np.interp(t_scen, ts_cmd, np.zeros_like(ts_cmd))  # rotate_deg=0 in meta

    # Scenario-only position (Option B cross-check stored)
    sp_pos_scen = np.stack([np.interp(t_scen, ts_cmd, cmd[:, i]) for i in range(3)], axis=1)

    rpy = np.deg2rad(rpy_deg)
    quat = np.array([_quat_rpy(*r) for r in rpy])

    # 500 Hz -> 1 kHz: hold each sample twice
    def up2(x):
        return np.repeat(x, 2, axis=0)

    pos1k = up2(pos)
    sp_pos1k = up2(sp_pos)
    steady = pos1k[:, 2] > WARMUP_Z_M
    # Step 8: logged ctrltarget z is scenario height; vehicle held a steady offset above it
    # (pos − sp ≈ +3.4 cm on A8). Without trimming z, replay PD subtracts m·kp_z·Δz from thrust.
    z_tracking_trim_m = (
        float(np.mean(pos1k[steady, 2] - sp_pos1k[steady, 2])) if steady.any() else 0.0
    )

    pack = {
        "t_usd": up2(t),
        "lag_s": np.array(lag),
        "z_tracking_trim_m": np.array(z_tracking_trim_m),
        "pos": pos1k,
        "vel": up2(vel),
        "rpy": up2(rpy),
        "quat": up2(quat),
        "gyro_deg_s": up2(gyro),
        "acc_g": up2(acc),
        "sp_pos": sp_pos1k,
        "sp_vel": up2(sp_vel),
        "sp_acc": up2(sp_acc),
        "sp_pos_scen": up2(sp_pos_scen),
        "yaw_d_rad": up2(yaw_d),
        "rpm": up2(rpm),
        "motor_pwm": up2(motor_pwm),
        "fs_hz": np.array(1000.0),
        "meta_path": np.array(str(meta_json)),
        "usd_path": np.array(str(usd_bin)),
    }
    # Hover warm-up start index (first sample with z > WARMUP_Z_M sustained)
    z = pack["pos"][:, 2]
    air = np.where(z > WARMUP_Z_M)[0]
    pack["warmup_start"] = np.array(int(air[0]) if len(air) else 0)

    if out_npz:
        out_npz.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(out_npz, **pack)
    return pack
