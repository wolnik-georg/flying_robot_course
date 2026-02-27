#!/usr/bin/env python3
"""
Safe offboard player for Crazyflie: play planned CSV (time_s, cmd) to the vehicle.
Features:
- preview, dry-run, and live modes
- interactive confirmation before live send
- spinup ramp, per-step thrust rate limiter (R), thrust_max
- telemetry CSV logging (IMU/flow/height/stateEstimate)
- conservative defaults and graceful fallback if cflib not installed

Usage examples:
  # preview
  python3 scripts/cf_offboard_player.py --preview --csv results/assignment5_hover_planned.csv

  # dry-run (no radio)
  python3 scripts/cf_offboard_player.py --dry-run --csv results/assignment5_hover_planned.csv --out results/hover_dry.csv

  # live (conservative)
  python3 scripts/cf_offboard_player.py --csv results/assignment5_hover_planned.csv --spinup 3.0 --limit-R 5.0 --confirm --out results/hover_live.csv

"""
import argparse
import csv
import os
import sys
import time

try:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.crazyflie.commander import Commander
    from cflib.crazyflie.log import LogConfig

    CFLIB = True
except Exception:
    CFLIB = False


def read_plan(path, time_col=None, cmd_col=None):
    import numpy as np

    # Try to detect header names (time/thrust) and pick columns accordingly.
    with open(path, "r") as fh:
        first = fh.readline().strip()
    headers = None
    if "," in first and any(c.isalpha() for c in first):
        headers = [h.strip() for h in first.split(",")]

    if headers is not None:
        # detect time column
        if time_col is None:
            for name in ("t", "time", "time_s"):
                if name in headers:
                    time_col = headers.index(name)
                    break
        # detect command/thrust column
        if cmd_col is None:
            for name in ("thrust", "measured_thrust_N", "cmd", "command"):
                if name in headers:
                    cmd_col = headers.index(name)
                    break
        # fallbacks
        if time_col is None:
            time_col = 0
        if cmd_col is None:
            cmd_col = min(1, len(headers) - 1)
        data = np.genfromtxt(path, delimiter=",", comments="#", skip_header=1)
    else:
        # no header: use provided defaults or 0/1
        if time_col is None:
            time_col = 0
        if cmd_col is None:
            cmd_col = 1
        data = np.genfromtxt(path, delimiter=",", comments="#")

    if data.ndim == 1:
        data = data.reshape(1, -1)
    t = data[:, time_col].astype(float)
    cmd = data[:, cmd_col].astype(float)
    # normalize to start at zero
    return t - t[0], cmd


def preview_plan(t, cmd, n=10):
    print("Plan preview (first rows):")
    print(" time_s, cmd")
    for i in range(min(n, len(t))):
        print(f" {t[i]:.3f}, {cmd[i]:.6f}")
    if len(t) > n:
        print(f" ... ({len(t)} rows total)")


def clamp_delta(next_cmd, last_cmd, max_delta):
    if max_delta is None:
        return next_cmd
    d = next_cmd - last_cmd
    if d > max_delta:
        return last_cmd + max_delta
    if d < -max_delta:
        return last_cmd - max_delta
    return next_cmd


def main():
    p = argparse.ArgumentParser(description="Safe offboard player for Crazyflie")
    p.add_argument("--csv", required=True, help="planned CSV path (time_s, cmd)")
    p.add_argument("--preview", action="store_true")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument(
        "--out", default=None, help="telemetry CSV output (for dry-run or live)"
    )
    p.add_argument("--spinup", type=float, default=0.0, help="spinup ramp (s)")
    p.add_argument(
        "--limit-R", type=float, default=None, help="thrust rate limit (units/sec)"
    )
    p.add_argument(
        "--wait-health",
        action="store_true",
        help="wait until estimator/telemetry looks healthy before sending",
    )
    p.add_argument(
        "--health-timeout",
        type=float,
        default=10.0,
        help="how many seconds to wait for health before aborting",
    )
    p.add_argument(
        "--auto-retries",
        type=int,
        default=0,
        help="number of automatic retries if motor outputs not observed (sign-check only)",
    )
    p.add_argument(
        "--retry-cmd-increment",
        type=float,
        default=0.02,
        help="amount to increase cmd1 on each retry (sign-check only)",
    )
    p.add_argument(
        "--verbose",
        action="store_true",
        help="print debug info during live playback",
    )
    p.add_argument(
        "--thrust-max", type=float, default=1.0, help="max thrust command (0..1)"
    )
    p.add_argument(
        "--confirm",
        action="store_true",
        help="require interactive YES before live send",
    )
    p.add_argument(
        "--sign-check",
        action="store_true",
        help="replace plan with a small baseline->cmd1 step to check sign",
    )
    p.add_argument(
        "--cmd1", type=float, default=0.05, help="cmd1 for sign-check (0..1)"
    )
    args = p.parse_args()

    if not os.path.exists(args.csv):
        print("CSV not found:", args.csv)
        sys.exit(1)

    t, cmd = read_plan(args.csv)

    if args.preview:
        preview_plan(t, cmd)
        if args.dry_run:
            print("Preview + dry-run requested: exiting")
            return

    # override plan for sign-check
    if args.sign_check:
        print("Running sign-check: replacing plan with baseline->cmd1 step")
        t0 = 0.0
        hold = 1.0
        step = 2.0
        dt = 0.01
        import numpy as _np

        ts = _np.arange(0.0, hold + args.spinup + step + 0.5, dt)
        cmds = _np.zeros_like(ts)
        cmds[:] = 0.0
        start_idx = int(round(hold / dt))
        if args.spinup > 0:
            ramp_steps = int(round(args.spinup / dt))
            for i in range(ramp_steps):
                frac = (i + 1) / max(1, ramp_steps)
                cmds[start_idx + i] = frac * args.cmd1
            cmds[start_idx + ramp_steps :] = args.cmd1
        else:
            cmds[start_idx:] = args.cmd1
        t, cmd = ts, cmds

    # compute sample dt from plan
    if len(t) > 1:
        dts = t[1:] - t[:-1]
        dt_median = float(max(1e-3, float(sorted(dts)[len(dts) // 2])))
    else:
        dt_median = 0.01

    # compute per-step max delta from R
    per_step_delta = None
    if args.limit_R is not None:
        per_step_delta = args.limit_R * dt_median

    if args.dry_run:
        # write out sampled plan to CSV and exit
        out = args.out or (os.path.splitext(args.csv)[0] + "_dry_play.csv")
        with open(out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "cmd"])
            for i in range(len(t)):
                w.writerow([f"{t[i]:.6f}", f"{cmd[i]:.6f}"])
        print("Wrote dry-run CSV to", out)
        return

    # live mode
    if not CFLIB:
        print(
            "cflib not available; cannot send to Crazyflie. Use --dry-run or install cflib."
        )
        sys.exit(1)

    # confirmation
    if args.confirm:
        resp = input("CONFIRM live send to Crazyflie? Type YES to continue: ")
        if resp.strip() != "YES":
            print("Aborting live send.")
            return

    # prepare telemetry CSV
    telem_file = None
    telem_writer = None
    if args.out:
        telem_file = open(args.out, "w", newline="")
        telem_writer = csv.writer(telem_file)
        telem_writer.writerow(
            [
                "time_s",
                "cmd",
                "imu_z",
                "height",
                "delta_x",
                "delta_y",
                "squal",
                "vx",
                "vy",
                "motor_m1",
                "motor_m2",
                "motor_m3",
                "motor_m4",
            ]
        )

    # connect and start log blocks
    cflib.crtp.init_drivers()
    with SyncCrazyflie("radio://0/80/2M", cf=Crazyflie()) as scf:
        cf = scf.cf
        # attach simple loggers
        latest = {
            "imu_z": None,
            "height": None,
            "delta_x": None,
            "delta_y": None,
            "squal": None,
            "vx": None,
            "vy": None,
            "motor_m1": None,
            "motor_m2": None,
            "motor_m3": None,
            "motor_m4": None,
        }

        try:
            log_ranges = LogConfig(name="RNG", period_in_ms=int(dt_median * 1000))
            log_ranges.add_variable("range.zrange", "float")
            cf.log.add_config(log_ranges)
            log_ranges.data_received_cb.add_callback(
                lambda ts, d, lc: latest.update({"height": d.get("range.zrange")})
            )
            log_ranges.start()
        except Exception:
            pass

        try:
            log_flow = LogConfig(name="FLOW", period_in_ms=int(dt_median * 1000))
            log_flow.add_variable("motion.deltaX", "int16_t")
            log_flow.add_variable("motion.deltaY", "int16_t")
            log_flow.add_variable("motion.squal", "uint8_t")
            log_flow.add_variable("stateEstimate.vx", "float")
            log_flow.add_variable("stateEstimate.vy", "float")
            cf.log.add_config(log_flow)

            def flow_cb(ts, data, lc):
                latest.update(
                    {
                        "delta_x": data.get("motion.deltaX"),
                        "delta_y": data.get("motion.deltaY"),
                        "squal": data.get("motion.squal"),
                        "vx": data.get("stateEstimate.vx"),
                        "vy": data.get("stateEstimate.vy"),
                    }
                )

            log_flow.data_received_cb.add_callback(flow_cb)
            log_flow.start()
        except Exception:
            pass

        # try to read motor outputs (probe several common variable names)
        try:
            motor_name_candidates = [
                "motor.m1",
                "motor.m2",
                "motor.m3",
                "motor.m4",
                "motors.m1",
                "motors.m2",
                "motors.m3",
                "motors.m4",
                "motor_power.m1",
                "motor_power.m2",
                "motor_power.m3",
                "motor_power.m4",
                "pwm.m1",
                "pwm.m2",
                "pwm.m3",
                "pwm.m4",
            ]
            log_mot = LogConfig(name="MOTORS", period_in_ms=int(dt_median * 1000))
            added = []
            for name in motor_name_candidates:
                try:
                    log_mot.add_variable(name, "float")
                    added.append(name)
                except Exception:
                    # variable not present on this firmware; skip
                    continue

            seen_mot_vars = set()
            if added:
                cf.log.add_config(log_mot)

                def mot_cb(ts, data, lc):
                    for var in added:
                        v = data.get(var)
                        if v is not None:
                            # normalize key to motor_mN
                            if "." in var:
                                key = var.split(".")[-1]
                            else:
                                key = var
                            # map common suffixes like m1 -> motor_m1
                            norm = f"motor_{key}"
                            latest[norm] = v
                            seen_mot_vars.add(norm)

                log_mot.data_received_cb.add_callback(mot_cb)
                log_mot.start()
        except Exception:
            # if anything goes wrong, continue without motor logs
            pass

        commander = Commander(cf)
        print("Starting live playback: Ctrl+C to abort")

        # optional wait for estimator/telemetry health
        if args.wait_health:
            print("Waiting for estimator/telemetry to become healthy...")
            t0 = time.time()
            consecutive_ok = 0
            required_ok = 5
            poll_dt = 0.1
            while True:
                # consider good if we have flow/stateEstimate or range data
                vx = latest.get("vx")
                vy = latest.get("vy")
                height = latest.get("height")
                squal = latest.get("squal")
                ok = False
                if vx is not None and vy is not None:
                    try:
                        # finite numbers
                        ok = all([float(vx) == float(vx), float(vy) == float(vy)])
                    except Exception:
                        ok = False
                if not ok and height is not None:
                    try:
                        ok = float(height) > 0
                    except Exception:
                        ok = False
                if ok:
                    consecutive_ok += 1
                else:
                    consecutive_ok = 0
                if consecutive_ok >= required_ok:
                    print("Estimator/telemetry looks healthy — proceeding")
                    break
                if time.time() - t0 > args.health_timeout:
                    print(
                        f"Health wait timed out after {args.health_timeout:.1f}s — aborting live send"
                    )
                    return
                time.sleep(poll_dt)
        start_time = time.time()
        last_cmd = cmd[0] if len(cmd) > 0 else 0.0
        try:
            for i in range(len(t)):
                target_time = start_time + t[i]
                now = time.time()
                sleep = target_time - now
                if sleep > 0:
                    time.sleep(sleep)
                desired = float(cmd[i])
                # enforce thrust_max
                desired = max(0.0, min(args.thrust_max, desired))
                # enforce per-step delta
                desired = clamp_delta(desired, last_cmd, per_step_delta)
                last_cmd = desired
                # send setpoint (roll, pitch, yawrate, thrust)
                try:
                    thrust_val = int(max(0, min(65535, desired * 65535)))
                    if args.verbose:
                        print(f"send_setpoint: thrust={desired:.6f} -> {thrust_val}")
                    commander.send_setpoint(0, 0, 0, thrust_val)
                except Exception:
                    # don't crash on transient radio errors
                    if args.verbose:
                        print("send_setpoint failed (transient)")
                    pass
                # log telemetry row
                if telem_writer is not None:
                    elapsed = time.time() - start_time

                    def fmt(v, fmt_str="{:.6f}"):
                        if v is None:
                            return ""
                        try:
                            return fmt_str.format(float(v))
                        except Exception:
                            return str(v)

                    row = [
                        f"{elapsed:.6f}",
                        f"{desired:.6f}",
                        fmt(latest.get("imu_z"), "{:.6f}"),
                        fmt(latest.get("height"), "{:.6f}"),
                        fmt(latest.get("delta_x"), "{}"),
                        fmt(latest.get("delta_y"), "{}"),
                        fmt(latest.get("squal"), "{}"),
                        fmt(latest.get("vx"), "{:.6f}"),
                        fmt(latest.get("vy"), "{:.6f}"),
                        fmt(latest.get("motor_m1"), "{:.6f}"),
                        fmt(latest.get("motor_m2"), "{:.6f}"),
                        fmt(latest.get("motor_m3"), "{:.6f}"),
                        fmt(latest.get("motor_m4"), "{:.6f}"),
                    ]
                    telem_writer.writerow(row)
        except KeyboardInterrupt:
            print("\nAborted by user")
        finally:
            try:
                if "log_ranges" in locals() and log_ranges is not None:
                    log_ranges.stop()
                    log_ranges.delete()
            except Exception:
                pass
            try:
                if "log_flow" in locals() and log_flow is not None:
                    log_flow.stop()
                    log_flow.delete()
            except Exception:
                pass
            try:
                if "log_mot" in locals() and log_mot is not None:
                    log_mot.stop()
                    log_mot.delete()
            except Exception:
                pass
            if telem_file is not None:
                telem_file.close()
            # report motor-variable observation summary
            try:
                if "seen_mot_vars" in locals() and seen_mot_vars:
                    print(
                        "Observed motor output variables:",
                        ",".join(sorted(list(seen_mot_vars))),
                    )
                    if len(seen_mot_vars) < 4:
                        print(
                            "Warning: fewer than 4 motor output variables observed. "
                            "This may indicate firmware does not expose all motor logs or the controller did not command all motors."
                        )
                else:
                    print("No motor output variables were observed in the log TOC.")
            except Exception:
                pass
            print("Playback finished")


if __name__ == "__main__":
    main()
