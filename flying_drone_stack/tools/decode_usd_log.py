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

RENAME = {
    "gyro.x": "gyro_x", "gyro.y": "gyro_y", "gyro.z": "gyro_z",
    "indi.tau_x": "tau_x", "indi.tau_y": "tau_y",
    "indi.alp_x": "alp_x", "indi.alp_y": "alp_y",
    "indi.alp_raw_x": "alp_raw_x", "indi.alp_raw_y": "alp_raw_y",
    "motor.m1": "motor_m1", "motor.m2": "motor_m2",
    "motor.m3": "motor_m3", "motor.m4": "motor_m4",
    "motor.m1_rpm": "motor_m1_rpm",
    "rpm.m1": "rpm_m1", "rpm.m2": "rpm_m2", "rpm.m3": "rpm_m3", "rpm.m4": "rpm_m4",
    "stabilizer.roll": "roll_deg", "stabilizer.pitch": "pitch_deg",
}


def load(path):
    """Returns a dict of numpy arrays, renamed to our usual CSV convention, plus 't' in seconds."""
    raw = cfusdlog.decode(path)["fixedFrequency"]
    out = {"t": raw["timestamp"] / 1000.0}
    for k, v in raw.items():
        if k == "timestamp":
            continue
        out[RENAME.get(k, k)] = np.asarray(v, dtype=float)
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
