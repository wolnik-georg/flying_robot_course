#!/usr/bin/env python
"""Decode a binary uSD-card log (from usd_indi_diagnostic_config.txt) into a numpy dict with
the SAME naming convention as our usual flight.py CSVs (gyro_x not gyro.x, etc.), and run the
three open-question checks from docs/investigation_indi_oscillation_2026-07-21.md directly.

Usage:
  ~/.pyenv/versions/flying_robots/bin/python decode_usd_log.py <path-to-log-file-from-sd-card>

The raw file is whatever name the config's line 3 produced (typically "log00" or similar) --
copy it off the SD card after the flight, no renaming needed.
"""
import sys
import numpy as np

sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/tools/usdlog")
import cfusdlog  # noqa: E402

# Any channel missing from this map keeps its firmware name, dots and all -- which reads as a
# harmless default but is not. Downstream code looks up `x`, `vx`, `a_res_x`; a key that arrives
# as `stateEstimate.x` is simply not found, and merge_usd_logs.py guards those lookups with
# `if k in out`, so an unmapped channel produces an EMPTY result rather than an error. Every
# channel in tools/usd_thesis_config.txt is therefore mapped here explicitly, and load() checks.
RENAME = {
    "stateEstimate.x": "x", "stateEstimate.y": "y", "stateEstimate.z": "z",
    "stateEstimate.vx": "vx", "stateEstimate.vy": "vy", "stateEstimate.vz": "vz",
    "gyro.x": "gyro_x", "gyro.y": "gyro_y", "gyro.z": "gyro_z",
    "acc.x": "acc_x", "acc.y": "acc_y", "acc.z": "acc_z",
    "indi.a_res_x": "a_res_x", "indi.a_res_y": "a_res_y", "indi.a_res_z": "a_res_z",
    "indi.tau_x": "tau_x", "indi.tau_y": "tau_y", "indi.tau_z": "tau_z",
    "indi.alp_x": "alp_x", "indi.alp_y": "alp_y", "indi.alp_z": "alp_z",
    "indi.alp_raw_x": "alp_raw_x", "indi.alp_raw_y": "alp_raw_y", "indi.alp_raw_z": "alp_raw_z",
    "indi.alp_notch_x": "alp_notch_x", "indi.alp_notch_y": "alp_notch_y",
    "indi.alp_notch_z": "alp_notch_z",
    "indi.e_r_x": "e_r_x", "indi.e_r_y": "e_r_y", "indi.e_r_z": "e_r_z",
    "indi.e_r_norm": "e_r_norm",
    "rnn.pred_x": "rnn_pred_x", "rnn.pred_y": "rnn_pred_y", "rnn.pred_z": "rnn_pred_z",
    "rnn.clamped": "rnn_clamped",
    "ctrltarget.x": "ctrltarget_x", "ctrltarget.y": "ctrltarget_y",
    "ctrltarget.z": "ctrltarget_z",
    "motor.m1": "motor_m1", "motor.m2": "motor_m2",
    "motor.m3": "motor_m3", "motor.m4": "motor_m4",
    "motor.m1_rpm": "motor_m1_rpm", "motor.m2_rpm": "motor_m2_rpm",
    "motor.m3_rpm": "motor_m3_rpm", "motor.m4_rpm": "motor_m4_rpm",
    "rpm.m1": "rpm_m1", "rpm.m2": "rpm_m2", "rpm.m3": "rpm_m3", "rpm.m4": "rpm_m4",
    "stabilizer.roll": "roll_deg", "stabilizer.pitch": "pitch_deg",
    "stabilizer.yaw": "yaw_deg",
    "indi.dt_us": "dt_us",
    "pm.vbat": "vbat",
    "usd.runTag": "run_tag",
    "indi.oot5_branch": "oot5_branch", "indi.oot5_thrust_si": "oot5_thrust_si", "indi.oot5_sp_mode_x": "oot5_sp_mode_x",
    "indi.oot5_sp_mode_y": "oot5_sp_mode_y", "indi.oot5_sp_mode_z": "oot5_sp_mode_z", "indi.oot5_sp_thrust": "oot5_sp_thrust",
    # --- usd_study_config.txt additions (2026-10-11): one superset for every controller ---
    "stateEstimate.qx": "est_qx", "stateEstimate.qy": "est_qy", "stateEstimate.qz": "est_qz", "stateEstimate.qw": "est_qw",
    "ctrltarget.vx": "ctrltarget_vx", "ctrltarget.vy": "ctrltarget_vy", "ctrltarget.vz": "ctrltarget_vz",
    "ctrltarget.ax": "ctrltarget_ax", "ctrltarget.ay": "ctrltarget_ay", "ctrltarget.az": "ctrltarget_az",
    "locSrv.x": "locsrv_x", "locSrv.y": "locsrv_y", "locSrv.z": "locsrv_z", "locSrvZ.tick": "locsrv_tick",
    "ctrlOmarIndi.a_rpm_fx": "omar_a_rpm_fx", "ctrlOmarIndi.a_rpm_fy": "omar_a_rpm_fy", "ctrlOmarIndi.a_rpm_fz": "omar_a_rpm_fz",
    "ctrlOmarIndi.a_imu_fx": "omar_a_imu_fx", "ctrlOmarIndi.a_imu_fy": "omar_a_imu_fy", "ctrlOmarIndi.a_imu_fz": "omar_a_imu_fz",
    "ctrlOmarIndi.torquex": "omar_tau_x", "ctrlOmarIndi.torquey": "omar_tau_y", "ctrlOmarIndi.torquez": "omar_tau_z",
    "ctrlOmarIndi.thrustSi": "omar_thrust_si", "ctrlOmarIndi.f_rpm": "omar_f_rpm",
    "ctrlOmarIndi.rpyx": "omar_rpy_x", "ctrlOmarIndi.rpyy": "omar_rpy_y", "ctrlOmarIndi.rpyz": "omar_rpy_z",
    "ctrlOmarIndi.rpydx": "omar_rpyd_x", "ctrlOmarIndi.rpydy": "omar_rpyd_y", "ctrlOmarIndi.rpydz": "omar_rpyd_z",
    "ctrlOmarIndi.error_posx": "omar_err_pos_x", "ctrlOmarIndi.error_posy": "omar_err_pos_y", "ctrlOmarIndi.error_posz": "omar_err_pos_z",
    "ctrlOmarIndi.error_velx": "omar_err_vel_x", "ctrlOmarIndi.error_vely": "omar_err_vel_y", "ctrlOmarIndi.error_velz": "omar_err_vel_z",
}


def load(path):
    """Returns a dict of numpy arrays, renamed to our usual CSV convention, plus 't' in seconds."""
    raw = cfusdlog.decode(path)["fixedFrequency"]
    out = {"t": raw["timestamp"] / 1000.0}
    unmapped = []
    for k, v in raw.items():
        if k == "timestamp":
            continue
        if k not in RENAME and "." in k:
            unmapped.append(k)
        out[RENAME.get(k, k)] = np.asarray(v, dtype=float)
    if unmapped:
        # Loud, because the alternative is a downstream lookup that quietly finds nothing.
        print(f"WARNING: {path}: unmapped channels kept as-is: {sorted(unmapped)}\n"
              f"         Add them to RENAME in decode_usd_log.py before using them.",
              file=sys.stderr)
    return out


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    d = load(sys.argv[1])
    fs = 1.0 / np.median(np.diff(d["t"]))
    print(f"Loaded {len(d['t'])} samples, fs={fs:.1f} Hz, duration={d['t'][-1]-d['t'][0]:.1f}s")
    print(f"Variables: {sorted(k for k in d if k != 't')}")
    print()
    print("True sample rate confirms whether this is genuinely 500 Hz (no radio decimation).")
    print("Next: see docs/investigation_indi_oscillation_2026-07-21.md for the three analyses")
    print("this log is meant to answer (filter aliasing, EKF attitude phase lag, motor->RPM")
    print("actuator confirmation in flight) -- run them against this dict.")


if __name__ == "__main__":
    main()
