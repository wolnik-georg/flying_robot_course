#!/usr/bin/env python3
"""
Crazyflie bench logger helper.

Features:
- Connects to a Crazyflie using cflib (optional) and sends a simple step sequence of thrust
  commands (props OFF) for bench motor ID measurements.
- Logs CSV with columns: time_s, cmd (unitless), measured_thrust_N (optional, from load cell), imu_z (optional), height (optional)
- Supports a dry-run mode to generate a synthetic CSV using a simple first-order motor model (useful for testing the analysis script).

Usage examples:
  # Dry-run synthetic step (no hardware)
  python3 scripts/cf_bench_logger.py --dry-run --cmd0 0.2 --cmd1 0.6 --hold 1.0 --step 3.0 --out results/step_dry.csv

  # With Crazyflie (requires cflib) and optional load cell on serial port
  python3 scripts/cf_bench_logger.py --uri radio://0/80/2M --cmd0 0.2 --cmd1 0.6 --hold 1.0 --step 3.0 --out results/step_cf.csv --loadcell-port /dev/ttyUSB0

Notes:
- Props OFF for bench tests. If you must run with props, follow strict tether/kill-switch procedures.
- The script requires `cflib` if you want to talk to a Crazyflie, and `pyserial` if you want to read a load cell via serial.
"""

import argparse
import csv
import json
import math
import sys
import time
import os

try:
    import serial

    SERIAL_AVAILABLE = True
except Exception:
    SERIAL_AVAILABLE = False

try:
    import cflib
    from cflib.crazyflie import Crazyflie
    from cflib.utils import uri_helper
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    CFLIB_AVAILABLE = True
except Exception:
    CFLIB_AVAILABLE = False

import numpy as np

DEFAULT_URI = "radio://0/80/2M"


def synthetic_step_sequence(t0, cmd0, cmd1, hold, step, dt, motor_tau=0.02, K=1.0):
    # generate time vector and synthetic measured thrust (first-order response)
    t_end = hold + step + 1.0
    ts = np.arange(0.0, t_end, dt)
    cmds = np.zeros_like(ts)
    cmds[:] = cmd0
    start_idx = int(round(hold / dt))
    cmds[start_idx:] = cmd1
    # simulate first-order motor: y_dot = -(y - K*cmd)/tau
    y = np.zeros_like(ts)
    y[0] = K * cmd0
    alpha = dt / (motor_tau + dt)
    for i in range(1, len(ts)):
        y[i] = y[i - 1] + alpha * (K * cmds[i] - y[i - 1])
    return ts, cmds, y


def read_loadcell_stream(ser):
    # expects ASCII lines with one numeric value per line or timestamp,value
    line = ser.readline().decode("utf8", errors="ignore").strip()
    if not line:
        return None
    parts = [p.strip() for p in line.split(",") if p.strip()]
    try:
        if len(parts) == 1:
            val = float(parts[0])
            return (time.time(), val)
        else:
            t = float(parts[0])
            val = float(parts[1])
            return (t, val)
    except Exception:
        return None


def main():
    p = argparse.ArgumentParser(description="Crazyflie bench logger helper")
    p.add_argument("--uri", default=DEFAULT_URI, help="Crazyflie URI (for radio).")
    p.add_argument(
        "--cmd0", type=float, default=0.2, help="baseline command (unitless)"
    )
    p.add_argument("--cmd1", type=float, default=0.6, help="step command (unitless)")
    p.add_argument("--hold", type=float, default=1.0, help="hold time before step (s)")
    p.add_argument("--step", type=float, default=3.0, help="step duration (s)")
    p.add_argument("--dt", type=float, default=0.005, help="logging sample dt (s)")
    p.add_argument("--out", default="results/bench_step.csv", help="output CSV path")
    p.add_argument(
        "--dry-run", action="store_true", help="generate synthetic CSV (no hardware)"
    )
    p.add_argument("--loadcell-port", help="serial port for load cell (optional)")
    p.add_argument(
        "--spinup",
        type=float,
        default=0.0,
        help="spin-up ramp duration (s) from cmd0 to cmd1 before main step; 0=no ramp",
    )
    p.add_argument(
        "--confirm",
        action="store_true",
        help="ask for interactive confirmation before sending live commands",
    )
    p.add_argument(
        "--motor-tau",
        type=float,
        default=0.02,
        help="motor tau used for synthetic mode (s)",
    )
    p.add_argument(
        "--gain-K",
        type=float,
        default=1.0,
        help="gain used for synthetic mode (N/command)",
    )
    args = p.parse_args()

    out_dir = os.path.dirname(args.out) or "."
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    if args.dry_run:
        ts, cmds, y = synthetic_step_sequence(
            0.0,
            args.cmd0,
            args.cmd1,
            args.hold,
            args.step,
            args.dt,
            motor_tau=args.motor_tau,
            K=args.gain_K,
        )
        with open(args.out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "cmd", "measured_thrust_N"])
            t0 = ts[0]
            for i in range(len(ts)):
                w.writerow([f"{ts[i]:.6f}", f"{cmds[i]:.6f}", f"{y[i]:.6f}"])
        print("Wrote synthetic CSV to", args.out)
        return

    # Live mode
    loadcell = None
    if args.loadcell_port:
        if not SERIAL_AVAILABLE:
            print(
                "pyserial not available; cannot read load cell on serial. Install pyserial or use dry-run.",
                file=sys.stderr,
            )
            sys.exit(1)
        try:
            loadcell = serial.Serial(args.loadcell_port, baudrate=115200, timeout=0.1)
            # flush
            loadcell.reset_input_buffer()
            print("Opened load cell on", args.loadcell_port)
        except Exception as e:
            print("Could not open load cell port:", e, file=sys.stderr)
            sys.exit(1)

    if not CFLIB_AVAILABLE:
        print(
            "cflib not available. Install `cflib` if you want to connect to a Crazyflie radio.",
            file=sys.stderr,
        )
        print(
            "You can still provide --loadcell-port and run the test sequence manually while logging time and cmd externally.",
            file=sys.stderr,
        )
        sys.exit(1)

    # Prepare CSV writer
    f = open(args.out, "w", newline="")
    w = csv.writer(f)
    # columns: time_s, cmd, measured_thrust_N (if available), imu_z, height
    header = ["time_s", "cmd"]
    if args.loadcell_port:
        header.append("measured_thrust_N")
    header += ["imu_z", "height"]
    w.writerow(header)

    # Connect to Crazyflie and use commander (high-level) to send thrust commands.
    # We'll use SyncCrazyflie to ensure connection is managed.
    print("Connecting to Crazyflie at", args.uri)
    cflib.crtp.init_drivers()
    with SyncCrazyflie(args.uri, cf=Crazyflie()) as scf:
        cf = scf.cf

        # Attach loggers for imu and range if available. Use callbacks to store latest values.
        latest = {"imu_z": None, "height": None}

        def imu_callback(timestamp, data, logconf):
            # data fields depend on deck; try a few common names
            z = None
            if "acc.z" in data:
                z = data["acc.z"]
            elif "acc_z" in data:
                z = data["acc_z"]
            latest["imu_z"] = z

        def range_callback(timestamp, data, logconf):
            if "range.zrange" in data:
                latest["height"] = data["range.zrange"]
            elif "range" in data:
                latest["height"] = data["range"]

        # Setup log configs if available; tolerate if not supported on this firmware
        log_conf_imu = None
        log_conf_range = None
        try:
            from cflib.crazyflie.log import LogConfig

            log_conf_imu = LogConfig(name="IMU", period_in_ms=int(args.dt * 1000))
            # conservative fields - firmware dependent
            try:
                log_conf_imu.add_variable("acc.z", "float")
            except Exception:
                try:
                    log_conf_imu.add_variable("acc_z", "float")
                except Exception:
                    log_conf_imu = None
            if log_conf_imu is not None:
                cf.log.add_config(log_conf_imu)
                log_conf_imu.data_received_cb.add_callback(imu_callback)
                log_conf_imu.start()
        except Exception:
            log_conf_imu = None

        try:
            from cflib.crazyflie.log import LogConfig

            log_conf_range = LogConfig(name="RANGE", period_in_ms=int(args.dt * 1000))
            try:
                log_conf_range.add_variable("range.zrange", "float")
            except Exception:
                try:
                    log_conf_range.add_variable("range", "float")
                except Exception:
                    log_conf_range = None
            if log_conf_range is not None:
                cf.log.add_config(log_conf_range)
                log_conf_range.data_received_cb.add_callback(range_callback)
                log_conf_range.start()
        except Exception:
            log_conf_range = None

        # Send baseline hold
        sample_dt = args.dt
        total_time = args.hold + args.step + 1.0
        t0 = time.time()
        t_end = t0 + total_time
        in_step = False
        last_write = 0.0

        # Helper to send thrust via commander (platform-dependent API). We'll use set_param or high-level commander if available.
        # Try using the high level commander via parameters if available (this is firmware dependent). Otherwise, we just log times and ask user to run manual commands.
        try:
            from cflib.crazyflie.commander import Commander

            commander = Commander(cf)
            can_command = True
        except Exception:
            commander = None
            can_command = False

        print(
            "Starting sequence: hold %.3fs at cmd=%.3f, then step to %.3f for %.3fs (spinup=%.3fs)"
            % (args.hold, args.cmd0, args.cmd1, args.step, args.spinup)
        )

        if args.confirm:
            resp = input("CONFIRM live send to Crazyflie? Type YES to continue: ")
            if resp.strip() != "YES":
                print("Aborting live send.")
                f.close()
                return

        # Use a loop to sample and send commands
        while time.time() < t_end:
            now = time.time()
            elapsed = now - t0
            # default command
            cmd = args.cmd0
            if elapsed < args.hold:
                cmd = args.cmd0
            elif (
                args.spinup > 0.0
                and elapsed >= args.hold
                and elapsed < args.hold + args.spinup
            ):
                # linear ramp from cmd0 to cmd1
                frac = (elapsed - args.hold) / max(1e-6, args.spinup)
                cmd = args.cmd0 + frac * (args.cmd1 - args.cmd0)
            elif elapsed < args.hold + args.spinup + args.step:
                cmd = args.cmd1
            else:
                cmd = args.cmd0

            # send command if possible
            if can_command and commander is not None:
                try:
                    # commander takes roll,pitch,yawrate,thrust; we send zero attitude and thrust in [0,1]
                    commander.send_setpoint(
                        0, 0, 0, int(max(0, min(65535, cmd * 65535)))
                    )
                except Exception:
                    pass

            # read load cell if present
            measured = ""
            if loadcell is not None and loadcell.in_waiting:
                sample = read_loadcell_stream(loadcell)
                if sample is not None:
                    measured = f"{sample[1]:.6f}"

            # get latest telemetry
            imu_z = latest.get("imu_z")
            height = latest.get("height")

            # write row at sample rate
            if now - last_write >= sample_dt - 1e-6:
                row = [f"{elapsed:.6f}", f"{cmd:.6f}"]
                if args.loadcell_port:
                    row.append(measured)
                row.append(f"{imu_z:.6f}" if imu_z is not None else "")
                row.append(f"{height:.6f}" if height is not None else "")
                w.writerow(row)
                f.flush()
                last_write = now
            # tiny sleep to avoid busy loop
            time.sleep(min(0.001, sample_dt * 0.5))

        # cleanup log configs
        try:
            if log_conf_imu is not None:
                log_conf_imu.stop()
            if log_conf_range is not None:
                log_conf_range.stop()
        except Exception:
            pass

    f.close()
    print("Wrote CSV to", args.out)


if __name__ == "__main__":
    main()
