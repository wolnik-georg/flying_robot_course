#!/usr/bin/env python3
"""
Safe offboard player for Crazyflie: play planned CSV (time_s, cmd) to the vehicle.
Features:
- preview, dry-run, and live modes
- interactive confirmation before live send
- spinup ramp, per-step thrust rate limiter (R), thrust_max
- telemetry CSV logging (IMU/flow/height/stateEstimate/motors)
- conservative defaults and graceful fallback if cflib not installed
- automatic thrust unit conversion: simulation outputs thrust in Newtons;
  use --thrust-max-N to specify the measured max thrust so the player can
  correctly scale to the 0-65535 PWM range that send_setpoint() expects.

Thrust unit notes
-----------------
The simulation CSVs (assignment5_closedloop_*.csv, assignment5_planned_*.csv)
store thrust in Newtons (hover ≈ 0.265 N for a 27 g Crazyflie).
Crazyflie send_setpoint() expects an integer in 0..65535 where 65535 maps to
maximum motor PWM.  Without --thrust-max-N the player would send
0.265 * 65535 ≈ 17 000 which is far below the ~36 000–44 000 needed for hover.

Recommended workflow
--------------------
1. Bench-identify max thrust with bench_motor_id.py -> get thrust_max_N
2. Run sign-check to verify motor response:
     python3 scripts/cf_offboard_player.py \\
       --sign-check --cmd1 0.35 --spinup 1.0 \\
       --thrust-max-N 0.56 --uri radio://0/80/2M \\
       --csv results/data/assignment5_closedloop_hover.csv \\
       --out results/sign_check_live.csv
3. Run hover:
     python3 scripts/cf_offboard_player.py \\
       --csv results/data/assignment5_closedloop_hover.csv \\
       --thrust-max-N 0.56 --spinup 2.0 --limit-R 0.5 \\
       --confirm --uri radio://0/80/2M \\
       --out results/hover_live.csv
4. Run circle / figure-8 (after successful hover):
     python3 scripts/cf_offboard_player.py \\
       --csv results/data/assignment5_closedloop_circle.csv \\
       --thrust-max-N 0.56 --spinup 2.0 --limit-R 0.5 \\
       --confirm --uri radio://0/80/2M \\
       --out results/circle_live.csv

Arming notes
------------
The Crazyflie firmware supervisor blocks all motor output when the drone is not
"armed".  Before sending any setpoints, this script calls:
  cf.platform.send_arming_request(True)
which sends a PLATFORM_REQUEST_ARMING CRTP packet.  The supervisor then sets its
armed flag and the power distribution stage passes thrust commands to motors.
After playback finishes (or on Ctrl+C) the script sends:
  send_setpoint(0,0,0,0) × 10 at 50 Hz   — ramps to zero, keeps watchdog alive
  cf.platform.send_arming_request(False)  — disarms cleanly (no lock)
  cf.commander.send_notify_setpoint_stop() — releases setpoint priority

WARNING: do NOT call send_stop_setpoint() (TYPE_STOP).  It zeros the setpoint
timestamp, which makes the commander watchdog age instantly → supervisor enters
ExceptFreeFall → permanently Locked (requires battery replug to recover).

Thrust range notes (cflib commander)
-------------------------------------
send_setpoint() documents the effective thrust range as 10001–60000 (integers).
Values below 10001 are treated as "no power" by the firmware.  This maps to
normalised fractions 0.153–0.916.  The default --cmd1=0.35 (PWM≈22938) is
safely above the deadband but well below liftoff.

Typical Crazyflie 2.x thrust_max_N (4 motors, 7×19 mm props, 3.7 V battery):
  ~0.50–0.65 N total; measure with bench_motor_id.py for your specific build.
  If no load-cell data is available, start conservatively with 0.50 N.

Usage examples:
  # preview (no radio)
  python3 scripts/cf_offboard_player.py --preview \\
    --csv results/data/assignment5_closedloop_hover.csv

  # dry-run (no radio)
  python3 scripts/cf_offboard_player.py --dry-run \\
    --csv results/data/assignment5_closedloop_hover.csv \\
    --thrust-max-N 0.56 --out results/hover_dry.csv

  # live hover (conservative)
  python3 scripts/cf_offboard_player.py \\
    --csv results/data/assignment5_closedloop_hover.csv \\
    --thrust-max-N 0.56 --spinup 2.0 --limit-R 0.5 \\
    --confirm --uri radio://0/80/2M \\
    --out results/hover_live.csv
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
        # NOTE: assignment5 closedloop CSVs use 'cmd_thrust'; planned CSVs use 'thrust'.
        # 'ref_thrust' also exists in closedloop but is the un-rate-limited value —
        # prefer 'cmd_thrust' (rate-limited, actuator-applied) when both are present.
        if cmd_col is None:
            for name in (
                "cmd_thrust",
                "thrust",
                "measured_thrust_N",
                "cmd",
                "command",
                "ref_thrust",
            ):
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


def preview_plan(t, cmd, n=10, thrust_max_N=None):
    print("Plan preview (first rows):")
    if thrust_max_N is not None:
        print(
            f" thrust_max_N={thrust_max_N:.4f} N  (fractions and PWM are after normalisation)"
        )
        print(" time_s,  cmd_raw_N,  fraction,   PWM")
        for i in range(min(n, len(t))):
            raw = float(cmd[i])
            frac = max(0.0, min(1.0, raw / thrust_max_N))
            pwm = int(frac * 65535)
            print(f" {t[i]:.3f},  {raw:.6f},  {frac:.4f},  {pwm}")
    else:
        print(" time_s,  cmd (0..1 fraction),  PWM")
        for i in range(min(n, len(t))):
            raw = float(cmd[i])
            pwm = int(max(0, min(65535, raw * 65535)))
            print(f" {t[i]:.3f},  {raw:.6f},  {pwm}")
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
        "--uri",
        type=str,
        default="radio://0/80/2M",
        help="Crazyflie radio URI (default: radio://0/80/2M)",
    )
    p.add_argument(
        "--thrust-max-N",
        type=float,
        default=None,
        help=(
            "Measured maximum total thrust [N] for your Crazyflie build. "
            "When provided, cmd values (in Newtons, as output by the simulator) "
            "are divided by this value to get a 0..1 fraction before scaling to "
            "the 0-65535 PWM range.  Typical value for CF2.x: 0.50-0.65 N. "
            "If omitted the cmd column is used as-is (must already be 0..1)."
        ),
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
        "--thrust-max",
        type=float,
        default=1.0,
        help="max thrust command (0..1, applied after --thrust-max-N normalisation)",
    )
    p.add_argument(
        "--confirm",
        action="store_true",
        help="require interactive YES before live send",
    )
    p.add_argument(
        "--sign-check",
        action="store_true",
        help="replace plan with a small baseline->cmd1 step to check sign. "
        "--cmd1 is always treated as a 0..1 normalised fraction (independent "
        "of --thrust-max-N) so that you can probe at a known fraction of full "
        "thrust without needing a calibrated thrust_max_N.",
    )
    p.add_argument(
        "--toc-only",
        action="store_true",
        help="connect, dump CF log TOC to a sidecar file, then exit (no sends)",
    )
    p.add_argument(
        "--list-all",
        action="store_true",
        help="enumerate all TOC entries and search for motor-related names",
    )
    p.add_argument(
        "--cmd1",
        type=float,
        default=0.35,
        help=(
            "Normalised (0..1) thrust step for --sign-check (default: 0.35). "
            "0.35 → PWM ≈ 22938. "
            "The cflib commander effective range is 10001–60000 (≈0.153–0.916); "
            "0.35 is safely above the deadband but not enough for liftoff. "
            "For a robust sign-check that clearly spins all motors use 0.30–0.40."
        ),
    )
    args = p.parse_args()

    if not os.path.exists(args.csv):
        print("CSV not found:", args.csv)
        sys.exit(1)

    t, cmd = read_plan(args.csv)

    if args.preview:
        preview_plan(t, cmd, thrust_max_N=args.thrust_max_N)
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

    # compute per-step max delta from R.
    # --limit-R is always in the *normalised* 0..1 fraction domain so that users
    # can reason about it independently of whether --thrust-max-N is set.
    # For sign-check mode cmd is already in 0..1 so this is consistent.
    per_step_delta = None
    if args.limit_R is not None:
        per_step_delta = args.limit_R * dt_median

    if args.dry_run:
        # write out sampled plan to CSV and exit
        out = args.out or (os.path.splitext(args.csv)[0] + "_dry_play.csv")
        with open(out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "cmd_raw", "cmd_pwm_fraction"])
            for i in range(len(t)):
                raw = float(cmd[i])
                if args.thrust_max_N is not None and args.thrust_max_N > 0:
                    frac = raw / args.thrust_max_N
                else:
                    frac = raw
                frac = max(0.0, min(1.0, frac))
                w.writerow([f"{t[i]:.6f}", f"{raw:.6f}", f"{frac:.6f}"])
        print("Wrote dry-run CSV to", out)
        if args.thrust_max_N is not None:
            import numpy as _np

            raw_arr = _np.array([float(c) for c in cmd])
            frac_arr = _np.clip(raw_arr / args.thrust_max_N, 0.0, 1.0)
            print(
                f"  Thrust normalization: max_raw={raw_arr.max():.4f} N, "
                f"thrust_max_N={args.thrust_max_N:.4f} N, "
                f"max_pwm_fraction={frac_arr.max():.4f} -> PWM={int(frac_arr.max()*65535)}"
            )
        else:
            import numpy as _np

            raw_arr = _np.array([float(c) for c in cmd])
            print(
                f"  No --thrust-max-N given: cmd used as-is. "
                f"max_cmd={raw_arr.max():.4f} -> PWM={int(raw_arr.max()*65535)}"
            )
            if raw_arr.max() < 0.3:
                print(
                    "  WARNING: max cmd < 0.3 -> max PWM < 19661. "
                    "This is likely below hover threshold (~36 000-44 000)."
                )
                print(
                    "  Did you forget --thrust-max-N?  "
                    "Typical CF2.x value: 0.50-0.65 N"
                )
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
                "cmd_fraction",
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
                "motor_m1req",
                "motor_m2req",
                "motor_m3req",
                "motor_m4req",
                "supervisor_info",
                "stab_thrust",
            ]
        )

    # connect and start log blocks
    cflib.crtp.init_drivers()
    radio_uri = args.uri
    print(f"Connecting to Crazyflie at {radio_uri} ...")
    # Print the effective thrust scaling so the user can verify before flight
    if args.thrust_max_N is not None and args.thrust_max_N > 0:
        import numpy as _np

        raw_arr = _np.array([float(c) for c in cmd])
        frac_arr = _np.clip(raw_arr / args.thrust_max_N, 0.0, 1.0)
        print(
            f"Thrust normalisation: thrust_max_N={args.thrust_max_N:.4f} N  "
            f"max_cmd={raw_arr.max():.4f} N  "
            f"max_fraction={frac_arr.max():.4f}  "
            f"max_PWM={int(frac_arr.max() * 65535)}"
        )
    else:
        import numpy as _np

        raw_arr = _np.array([float(c) for c in cmd])
        print(
            f"No --thrust-max-N given: cmd used as-is.  "
            f"max_cmd={raw_arr.max():.4f} -> max_PWM={int(raw_arr.max() * 65535)}"
        )
        if raw_arr.max() < 0.3:
            print(
                "  WARNING: max cmd < 0.3 -> max PWM < 19 661.  "
                "This is likely BELOW hover threshold (~36 000–44 000).\n"
                "  Did you forget --thrust-max-N?  Typical CF2.x value: 0.50–0.65 N"
            )
    with SyncCrazyflie(radio_uri, cf=Crazyflie()) as scf:
        cf = scf.cf
        # Dump log TOC to a sidecar file so we record exactly which
        # log variables the firmware exposes for this run. This helps
        # debugging when motor variables are missing or renamed.
        try:
            if args.out:
                toc_dump_path = args.out + ".toc.txt"
            else:
                toc_dump_path = os.path.join(
                    "results", f"cf_log_toc_{int(time.time())}.txt"
                )
            try:
                with open(toc_dump_path, "w") as tf:
                    tf.write("Crazyflie log TOC dump\n")
                    tf.write(f"time: {time.asctime()}\n\n")
                    try:
                        # Common place for the TOC object
                        if hasattr(cf, "log") and hasattr(cf.log, "toc"):
                            tf.write("cf.log.toc repr:\n")
                            tf.write(repr(cf.log.toc) + "\n\n")
                            try:
                                tf.write("cf.log.toc dir():\n")
                                tf.write(", ".join(dir(cf.log.toc)) + "\n\n")
                            except Exception as e:
                                tf.write("dir() failed: " + repr(e) + "\n\n")

                        tf.write("cf.log dir():\n")
                        tf.write(", ".join(dir(cf.log)) + "\n\n")

                        # Inspect a few likely internal attributes
                        for attr in (
                            "_log_toc",
                            "toc",
                            "get_toc",
                            "entries",
                            "_entries",
                        ):
                            if hasattr(cf.log, attr):
                                val = getattr(cf.log, attr)
                                tf.write(f"cf.log.{attr} repr:\n")
                                try:
                                    tf.write(repr(val) + "\n\n")
                                except Exception:
                                    tf.write("<repr failed>\n\n")

                        # cf.log.toc.toc is a dict: { group_name: { var_name: TocElement } }
                        # Iterating toc.toc directly yields only the group-name strings.
                        # We must call .items() to get group→vars mappings.
                        try:
                            toc = getattr(cf.log, "toc", None)
                            if toc is not None and hasattr(toc, "toc"):
                                toc_dict = toc.toc
                                if isinstance(toc_dict, dict):
                                    tf.write("TOC variables (group.name):\n")
                                    count = 0
                                    for group, vars_dict in toc_dict.items():
                                        if isinstance(vars_dict, dict):
                                            for var_name, elem in vars_dict.items():
                                                tf.write(
                                                    f"  {group}.{var_name}  {repr(elem)}\n"
                                                )
                                                count += 1
                                                if count > 2000:
                                                    tf.write(
                                                        "  ...truncated after 2000 entries...\n"
                                                    )
                                                    break
                                        else:
                                            tf.write(
                                                f"  {group} -> {repr(vars_dict)}\n"
                                            )
                                        if count > 2000:
                                            break
                                    tf.write(f"\nTotal variables listed: {count}\n")
                                else:
                                    # Fallback: stringify whatever it is
                                    tf.write("cf.log.toc.toc (not a dict):\n")
                                    tf.write(repr(toc_dict)[:4000] + "\n")
                            else:
                                tf.write("cf.log.toc not available.\n")
                        except Exception as e:
                            tf.write("TOC iteration failed: " + repr(e) + "\n")
                    except Exception as e:
                        tf.write("TOC dump introspection error: " + repr(e) + "\n")
                print("Wrote CF log TOC dump to", toc_dump_path)
            except Exception as e:
                print("Could not write TOC dump:", e)
        except Exception:
            pass
        # If requested, also search the TOC for motor-like variable names and print matches
        try:
            if args.list_all:
                matches = []
                try:
                    toc = getattr(cf.log, "toc", None)
                    if toc is not None and hasattr(toc, "toc"):
                        toc_dict = toc.toc
                        if isinstance(toc_dict, dict):
                            for group, vars_dict in toc_dict.items():
                                if isinstance(vars_dict, dict):
                                    for var_name, elem in vars_dict.items():
                                        full_name = f"{group}.{var_name}"
                                        if any(
                                            k in full_name.lower()
                                            for k in (
                                                "motor",
                                                "motors",
                                                "pwm",
                                                "esc",
                                                "power",
                                                "thrust",
                                            )
                                        ):
                                            matches.append(f"{full_name}  {repr(elem)}")
                                else:
                                    if any(
                                        k in str(group).lower()
                                        for k in (
                                            "motor",
                                            "motors",
                                            "pwm",
                                            "esc",
                                            "power",
                                            "thrust",
                                        )
                                    ):
                                        matches.append(f"{group} -> {repr(vars_dict)}")
                except Exception:
                    pass
                print("TOC search matches (motor-like names):")
                if matches:
                    for m in matches:
                        print(m)
                else:
                    print("<no matches found>")
        except Exception:
            pass
        # If the user requested only a TOC dump, exit cleanly now before any sends
        if args.toc_only:
            print("TOC-only requested: exiting before any setpoints were sent.")
            return
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
            "supervisor_info": None,
            "stab_thrust": None,
        }

        # Crazyflie log block period minimum is 10 ms (100 Hz maximum).
        # The simulation dt may be 5 ms, which would request a 5 ms period and
        # cause the firmware to silently reject the log block.
        log_period_ms = max(10, int(dt_median * 1000))

        try:
            log_ranges = LogConfig(name="RNG", period_in_ms=log_period_ms)
            log_ranges.add_variable("range.zrange", "float")
            cf.log.add_config(log_ranges)
            log_ranges.data_received_cb.add_callback(
                lambda ts, d, lc: latest.update({"height": d.get("range.zrange")})
            )
            log_ranges.start()
        except Exception:
            pass

        try:
            log_flow = LogConfig(name="FLOW", period_in_ms=log_period_ms)
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

        # Log supervisor.info (bitmask) and stabilizer.thrust to diagnose arming.
        # supervisor.info bits:  0=canFly  1=isFlying  2=isTumbled  3=armed
        # stabilizer.thrust shows the thrust the firmware is actually commanding.
        try:
            log_supervisor = LogConfig(name="SUP", period_in_ms=log_period_ms)
            log_supervisor.add_variable("supervisor.info", "uint16_t")
            log_supervisor.add_variable("stabilizer.thrust", "float")
            cf.log.add_config(log_supervisor)

            def sup_cb(ts, data, lc):
                latest.update(
                    {
                        "supervisor_info": data.get("supervisor.info"),
                        "stab_thrust": data.get("stabilizer.thrust"),
                    }
                )

            log_supervisor.data_received_cb.add_callback(sup_cb)
            log_supervisor.start()
        except Exception as _e:
            if args.verbose:
                print(f"supervisor/stabilizer log unavailable: {_e}")
            log_supervisor = None
        # TOC confirmed: firmware exposes group "motor" with variables m1..m4 (uint32_t).
        # Also log m1req..m4req (requested PWM before slew-rate limiting, if present).
        #
        # IMPORTANT: cflib marks an entire LogConfig block as unusable the moment any
        # single add_variable() call fails ("not in TOC").  We therefore build one
        # separate LogConfig per motor variable so a missing variable cannot poison
        # the others.  All successfully started sub-blocks feed into the same `latest`
        # dict and `seen_mot_vars` set.
        seen_mot_vars = set()
        _motor_log_blocks = []  # keep references alive for cleanup in finally

        motor_var_candidates = [
            # group "motor" — confirmed in this firmware's TOC (uint32_t)
            ("motor.m1", "uint32_t", "motor_m1"),
            ("motor.m2", "uint32_t", "motor_m2"),
            ("motor.m3", "uint32_t", "motor_m3"),
            ("motor.m4", "uint32_t", "motor_m4"),
            # requested PWM (before internal slew limiting) — useful for diagnosis
            ("motor.m1req", "uint32_t", "motor_m1req"),
            ("motor.m2req", "uint32_t", "motor_m2req"),
            ("motor.m3req", "uint32_t", "motor_m3req"),
            ("motor.m4req", "uint32_t", "motor_m4req"),
        ]

        for mot_var, mot_type, mot_key in motor_var_candidates:
            try:
                blk = LogConfig(name=f"MOT_{mot_key}", period_in_ms=log_period_ms)
                blk.add_variable(mot_var, mot_type)
                cf.log.add_config(blk)

                # Capture loop variables by default-argument binding to avoid
                # the classic Python closure-in-loop bug.
                def _make_mot_cb(var_name, latest_key):
                    def _cb(ts, data, lc):
                        v = data.get(var_name)
                        if v is not None:
                            latest[latest_key] = v
                            seen_mot_vars.add(latest_key)

                    return _cb

                blk.data_received_cb.add_callback(_make_mot_cb(mot_var, mot_key))
                blk.start()
                _motor_log_blocks.append(blk)
                if args.verbose:
                    print(f"Motor log started: {mot_var} ({mot_type}) -> {mot_key}")
            except Exception as _e:
                if args.verbose:
                    print(f"Motor log unavailable: {mot_var} ({_e})")
                continue

        # Update latest dict keys to include the req variants
        for _, _, mk in motor_var_candidates:
            if mk not in latest:
                latest[mk] = None

        commander = Commander(cf)

        # ------------------------------------------------------------------ #
        # ARM the Crazyflie via the platform service.                         #
        #                                                                      #
        # IMPORTANT — LOCKED STATE:                                           #
        # The supervisor enters the LOCKED state (bit 6 of supervisor.info)   #
        # after a commander watchdog timeout (2 s without a setpoint while    #
        # armed/flying) or after an emergency stop.  LOCKED is permanent      #
        # until the Crazyflie is physically power-cycled (battery replug).    #
        # We check for this condition immediately and abort if locked.         #
        #                                                                      #
        # COMMANDER WATCHDOG:                                                  #
        # The firmware fires COMMANDER_WDT_TIMEOUT 500 ms after the last      #
        # received setpoint (warning), then LOCKS after 2000 ms.  We must     #
        # send setpoints continuously — never pause for > 500 ms.             #
        # The arm wait below uses a keep-alive loop instead of time.sleep().  #
        # ------------------------------------------------------------------ #

        # --- Check if supervisor is already locked (requires reboot) ---
        # We can read supervisor_info from the log that started above.
        # Wait briefly for the first log packet to arrive.
        time.sleep(0.15)
        sup_info = latest.get("supervisor_info")
        if sup_info is not None:
            sup_int = int(sup_info)
            is_locked = bool(sup_int & (1 << 6))
            is_armed = bool(sup_int & (1 << 1))
            can_fly = bool(sup_int & (1 << 3))
            print(
                f"supervisor.info = {sup_int} (0x{sup_int:04X})  "
                f"locked={is_locked}  armed={is_armed}  canFly={can_fly}"
            )
            if is_locked:
                print()
                print("ERROR: Crazyflie supervisor is LOCKED (bit 6 set).")
                print(
                    "  The firmware locks itself permanently after a watchdog timeout"
                )
                print("  or emergency stop.  Motors are completely blocked.")
                print()
                print(
                    "  FIX: Power-cycle the Crazyflie (unplug and replug the battery),"
                )
                print("       then re-run this command.")
                print()
                return
        else:
            print(
                "supervisor.info not yet available — proceeding (will check again on first step)"
            )

        # --- Send arm request, keep-alive loop replaces time.sleep() ---
        try:
            cf.platform.send_arming_request(True)
            print(
                "ARM request sent — waiting for firmware (keep-alive setpoints running) …"
            )
        except Exception as _arm_err:
            print(
                f"Warning: arming request failed ({_arm_err}). "
                "Proceeding anyway — motors may not spin if firmware is not armed."
            )

        # Send keep-alive zero-thrust setpoints for 0.5 s.
        # This prevents the commander watchdog from firing during the arm wait.
        _arm_wait_end = time.time() + 0.5
        while time.time() < _arm_wait_end:
            try:
                commander.send_setpoint(0, 0, 0, 0)
            except Exception:
                pass
            time.sleep(0.02)  # 50 Hz keep-alive

        print("Arm wait done — starting playback.")

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
                # Keep-alive: send zero setpoint so watchdog doesn't fire during health wait
                try:
                    commander.send_setpoint(0, 0, 0, 0)
                except Exception:
                    pass
                time.sleep(poll_dt)
        start_time = time.time()
        # Initialise last_cmd in the same normalised 0..1 domain used inside the loop.
        _first_raw = float(cmd[0]) if len(cmd) > 0 else 0.0
        if args.thrust_max_N is not None and args.thrust_max_N > 0:
            last_cmd = max(0.0, min(1.0, _first_raw / args.thrust_max_N))
        else:
            last_cmd = max(0.0, min(1.0, _first_raw))
        try:
            for i in range(len(t)):
                # Check for locked state every 50 steps (~0.5 s at 100 Hz)
                # and abort immediately to avoid burning time with blocked motors.
                if i % 50 == 0 and i > 0:
                    _si = latest.get("supervisor_info")
                    if _si is not None and (int(_si) & (1 << 6)):
                        print(f"\nERROR at step {i}: supervisor entered LOCKED state!")
                        print(
                            "  Motors will not spin. Power-cycle the Crazyflie and retry."
                        )
                        break

                target_time = start_time + t[i]
                now = time.time()
                sleep = target_time - now
                if sleep > 0:
                    time.sleep(sleep)
                desired = float(cmd[i])
                # --- Thrust unit conversion ---
                # If the CSV stores thrust in Newtons (as produced by the Rust
                # simulator) we must normalise to 0..1 before mapping to PWM.
                # --thrust-max-N provides the measured total max thrust [N].
                # Without it the cmd is assumed to already be in 0..1.
                if args.thrust_max_N is not None and args.thrust_max_N > 0:
                    desired = desired / args.thrust_max_N
                # enforce thrust_max (0..1 normalised fraction)
                desired = max(0.0, min(args.thrust_max, desired))
                # enforce per-step delta (in normalised fraction units)
                desired = clamp_delta(desired, last_cmd, per_step_delta)
                last_cmd = desired
                # send setpoint (roll=0, pitch=0, yawrate=0, thrust 0..65535)
                try:
                    thrust_val = int(max(0, min(65535, desired * 65535)))
                    if args.verbose:
                        print(
                            f"send_setpoint: fraction={desired:.4f} -> PWM={thrust_val}"
                        )
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
                        fmt(latest.get("motor_m1"), "{}"),
                        fmt(latest.get("motor_m2"), "{}"),
                        fmt(latest.get("motor_m3"), "{}"),
                        fmt(latest.get("motor_m4"), "{}"),
                        fmt(latest.get("motor_m1req"), "{}"),
                        fmt(latest.get("motor_m2req"), "{}"),
                        fmt(latest.get("motor_m3req"), "{}"),
                        fmt(latest.get("motor_m4req"), "{}"),
                        fmt(latest.get("supervisor_info"), "{}"),
                        fmt(latest.get("stab_thrust"), "{:.2f}"),
                    ]
                    telem_writer.writerow(row)
        except KeyboardInterrupt:
            print("\nAborted by user")
        finally:
            # Safe shutdown sequence — must NOT leave the supervisor in a state
            # that causes a watchdog-timeout lock on the next connection.
            #
            # What MUST NOT be done:
            #   send_stop_setpoint() (TYPE_STOP): zeros the setpoint AND its
            #   timestamp, making the watchdog age instantly > 2 s → supervisor
            #   enters ExceptFreeFall → immediately Locked (requires reboot).
            #
            # What MUST be done:
            #   1. Send a few zero-thrust send_setpoint(0,0,0,0) packets — these
            #      update the setpoint timestamp so the watchdog stays alive.
            #   2. Disarm via send_arming_request(False) — transitions the supervisor
            #      from ReadyToFly/Flying/Landed back to PreFlChecksPassed, which is
            #      a safe idle state.  No lock occurs on the next connect.
            #   3. send_notify_setpoint_stop() is fine as a final courtesy packet.
            try:
                # Ramp down over ~200 ms at 50 Hz to avoid a sudden thrust cut
                for _ in range(10):
                    commander.send_setpoint(0, 0, 0, 0)
                    time.sleep(0.02)
            except Exception:
                pass
            # Disarm first (while setpoint is still fresh).
            try:
                cf.platform.send_arming_request(False)
                print("Disarm request sent.")
            except Exception:
                pass
            # Notify setpoint stop as a courtesy.
            try:
                cf.commander.send_notify_setpoint_stop(0)
            except Exception:
                pass
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
                if "log_supervisor" in locals() and log_supervisor is not None:
                    log_supervisor.stop()
                    log_supervisor.delete()
            except Exception:
                pass
            try:
                for _blk in _motor_log_blocks if "_motor_log_blocks" in dir() else []:
                    try:
                        _blk.stop()
                        _blk.delete()
                    except Exception:
                        pass
            except Exception:
                pass
            if telem_file is not None:
                telem_file.close()
            # report motor-variable observation summary
            try:
                actual_pwm_vars = (
                    {v for v in seen_mot_vars if not v.endswith("req")}
                    if "seen_mot_vars" in dir()
                    else set()
                )
                if actual_pwm_vars:
                    print(
                        "Observed motor PWM variables:",
                        ", ".join(sorted(actual_pwm_vars)),
                    )
                    if len(actual_pwm_vars) < 4:
                        print(
                            f"WARNING: only {len(actual_pwm_vars)}/4 motor PWM variables received data. "
                            "Check that motors are armed and the log block started correctly."
                        )
                    else:
                        print("All 4 motor PWM variables received data — motor log OK.")
                    req_vars = {v for v in seen_mot_vars if v.endswith("req")}
                    if req_vars:
                        print(
                            "Also observed requested-PWM variables:",
                            ", ".join(sorted(req_vars)),
                        )
                else:
                    print(
                        "No motor PWM data received in the log. "
                        "Possible causes: drone not armed, log block rejected, or PWM=0 throughout."
                    )
            except Exception:
                pass
            print("Playback finished")


if __name__ == "__main__":
    main()
