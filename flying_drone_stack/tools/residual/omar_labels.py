#!/usr/bin/env python3
"""Offline NS2 training labels on the Omar C controller tree: a_res = a_imu_f - a_rpm_f, reconstructed from a uSD log.

Why: Omar's filters for a_rpm_f / a_imu_f are updated only when `indi & 1` (controller_omar_indi.c), so a flight with
indi 0 (S0, S2) logs zeros there. The label is the same physical quantity the INDI term uses (measured acceleration minus the
acceleration expected from the rotor speeds), so it can be rebuilt from the raw channels of ANY flight by running the real
Omar C controller code (host bindings, the same source that flies) over the log with indi = 3: state attitude, accelerometer,
gyro and deck RPM in, the controller's own filtered a_rpm_f / a_imu_f out. Omar's definition, as decided: mass 0.0427,
his MOTORRPM2FORCE, 30 Hz Butterworth at 500 Hz, DECK rpm (rpm.m1..m4, raw), world-frame state acceleration R*acc - [0,0,1].

  python3 flying_drone_stack/tools/residual/omar_labels.py --validate <flight.bin>      # offline vs logged ctrlOmarIndi.a_*_f
  from omar_labels import reconstruct_a_res      # d = decode_usd_log.load(bin) ; a_res (N,3) m/s^2

Run with the system python3 (3.10 bindings, PYTHONPATH=<crazyflie-firmware>/build). Output column convention of the merge
tool: `a_res_x/y/z` per drone (consumed by dataset.py)."""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))                       # decode_usd_log
sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/build")

G = 9.81


def _rot(qw, qx, qy, qz):
    return np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                     [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                     [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


def quaternions(d) -> np.ndarray:
    """(N,4) w,x,y,z. Prefers the logged estimator quaternion; falls back to roll/pitch/yaw (deg)."""
    if all(k in d for k in ("est_qw", "est_qx", "est_qy", "est_qz")) and np.any(np.abs(d["est_qw"]) > 0):
        q = np.stack([d["est_qw"], d["est_qx"], d["est_qy"], d["est_qz"]], axis=1)
        return q / np.linalg.norm(q, axis=1, keepdims=True)
    r, p, y = (np.deg2rad(d[k]) for k in ("roll_deg", "pitch_deg", "yaw_deg"))
    cr, sr, cp, sp, cy, sy = np.cos(r / 2), np.sin(r / 2), np.cos(p / 2), np.sin(p / 2), np.cos(y / 2), np.sin(y / 2)
    return np.stack([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy], axis=1)


def reconstruct_a_res(d: dict, rpm_source: str = "deck", warmup_s: float = 2.0):
    """-> (a_res (N,3) m/s^2 = a_imu_f - a_rpm_f, a_rpm_f (N,3), a_imu_f (N,3), valid (N,) bool).
    rpm_source: 'deck' (rpm_m1..4, Omar's definition, default) | 'dshot' (motor_m1_rpm..) | 'rpmf' (rpm-filter output)."""
    import cffirmware as fw
    key = {"deck": ("rpm_m1", "rpm_m2", "rpm_m3", "rpm_m4"), "dshot": ("motor_m1_rpm", "motor_m2_rpm", "motor_m3_rpm", "motor_m4_rpm"),
           "rpmf": ("rpmf_m1", "rpmf_m2", "rpmf_m3", "rpmf_m4")}[rpm_source]
    for k in key + ("acc_x", "acc_y", "acc_z", "gyro_x", "x", "vx"):
        if k not in d:
            raise KeyError(f"uSD log lacks channel {k!r}; cannot reconstruct the label")
    n = len(d["t"])
    q = quaternions(d)
    omar = fw.controllerOmarIndi_t()
    fw.controllerOmarIndiInit(omar)
    omar.indi = 3
    sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
    sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs
    sp.mode.yaw = fw.modeAbs
    a_res, a_rpm, a_imu = np.zeros((n, 3)), np.zeros((n, 3)), np.zeros((n, 3))
    for i in range(n):
        qw, qx, qy, qz = (float(v) for v in q[i])
        st.position.x, st.position.y, st.position.z = float(d["x"][i]), float(d["y"][i]), float(d["z"][i])
        st.velocity.x, st.velocity.y, st.velocity.z = float(d["vx"][i]), float(d["vy"][i]), float(d["vz"][i])
        st.attitudeQuaternion.w, st.attitudeQuaternion.x, st.attitudeQuaternion.y, st.attitudeQuaternion.z = qw, qx, qy, qz
        sp.position.x, sp.position.y, sp.position.z = st.position.x, st.position.y, st.position.z   # label does not depend on the setpoint
        sens.gyro.x, sens.gyro.y, sens.gyro.z = float(d["gyro_x"][i]), float(d["gyro_y"][i]), float(d["gyro_z"][i])
        a = np.array([d["acc_x"][i], d["acc_y"][i], d["acc_z"][i]], float)
        sens.acc.x, sens.acc.y, sens.acc.z = float(a[0]), float(a[1]), float(a[2])
        aw = _rot(qw, qx, qy, qz) @ a - np.array([0.0, 0.0, 1.0])       # state->acc: world frame, Gs, gravity removed
        st.acc.x, st.acc.y, st.acc.z = float(aw[0]), float(aw[1]), float(aw[2])
        fw.oot_set_rpm(*[int(max(0, round(float(d[k][i])))) for k in key])
        fw.controllerOmarIndi(omar, ctl, sp, sens, st, 2 * i)        # the controller executes on even main-loop ticks (500 Hz)
        ar, ai = omar.a_rpm_filtered, omar.a_imu_filtered
        a_rpm[i] = (ar.x, ar.y, ar.z)
        a_imu[i] = (ai.x, ai.y, ai.z)
    a_res = a_imu - a_rpm
    t = np.asarray(d["t"], float)
    valid = (t - t[0]) >= warmup_s
    return a_res, a_rpm, a_imu, valid


def validate(bin_path: str, rpm_source: str = "deck") -> int:
    import decode_usd_log
    d = decode_usd_log.load(bin_path)
    need = ("omar_a_rpm_fz", "omar_a_imu_fz")
    if not all(k in d for k in need):
        print("log has no ctrlOmarIndi.a_*_f channels; nothing to validate against"); return 2
    a_res, a_rpm, a_imu, valid = reconstruct_a_res(d, rpm_source)
    lg_rpm = np.stack([d["omar_a_rpm_fx"], d["omar_a_rpm_fy"], d["omar_a_rpm_fz"]], axis=1)
    lg_imu = np.stack([d["omar_a_imu_fx"], d["omar_a_imu_fy"], d["omar_a_imu_fz"]], axis=1)
    active = valid & (np.abs(lg_rpm).sum(axis=1) > 0)
    print(f"{Path(bin_path).name}: {len(valid)} samples, {int(active.sum())} with the controller's INDI filters active, rpm source {rpm_source}")
    if not active.any():
        print("controller filters were never active (indi 0?) -> cannot validate"); return 2
    lg = (lg_imu - lg_rpm)[active]
    rc = a_res[active]
    for ax, nm in enumerate("xyz"):
        e = rc[:, ax] - lg[:, ax]
        corr = float(np.corrcoef(rc[:, ax], lg[:, ax])[0, 1]) if np.std(lg[:, ax]) > 0 else float("nan")
        print(f"  a_res_{nm}: logged std {lg[:, ax].std():.3f}  offline-logged RMS {np.sqrt(np.mean(e**2)):.3f} m/s^2  max {np.abs(e).max():.3f}  corr {corr:.4f}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--validate", metavar="BIN")
    ap.add_argument("--rpm-source", default="deck", choices=("deck", "dshot", "rpmf"))
    a = ap.parse_args()
    if a.validate:
        return validate(a.validate, a.rpm_source)
    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
