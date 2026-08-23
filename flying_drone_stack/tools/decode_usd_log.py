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
    "rnn.pred_x": "rnn_pred_x", "rnn.pred_y": "rnn_pred_y", "rnn.pred_z": "rnn_pred_z",
    "rnn.clamped": "rnn_clamped",
    "ctrltarget.x": "ctrltarget_x", "ctrltarget.y": "ctrltarget_y",
    "ctrltarget.z": "ctrltarget_z",
    "motor.m1": "motor_m1", "motor.m2": "motor_m2",
    "motor.m3": "motor_m3", "motor.m4": "motor_m4",
    "motor.m1_rpm": "motor_m1_rpm",
    "rpm.m1": "rpm_m1", "rpm.m2": "rpm_m2", "rpm.m3": "rpm_m3", "rpm.m4": "rpm_m4",
    "stabilizer.roll": "roll_deg", "stabilizer.pitch": "pitch_deg",
    "stabilizer.yaw": "yaw_deg",
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
