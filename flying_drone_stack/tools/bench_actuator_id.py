#!/usr/bin/env python
"""Open-loop brushless actuator ID — NO FLIGHT. Drone must be SECURED to the bench
(strapped/taped down, props ON for realistic load, area clear).

Drives the motors directly via motorPowerSet (bypasses the controller but passes through
the SAME battery-compensation + DShot path as flight, motors.c:723) and logs the optical
RPM response at a true 100 Hz over the radio. This measures the commanded->RPM transfer
OPEN LOOP — the one quantity the in-flight logs cannot give (closed-loop bias: any
in-flight tau->alpha estimate converges to the controller inverse -1/C, not the plant).

What it answers (investigation doc §15):
  1. Small-signal vs large-signal gain+phase at 1-15 Hz  -> is the actuator
     amplitude-dependent (ESC slew/accel limiting) or linear (plain pole)?
  2. Step response up vs down                            -> spool asymmetry
     (brushless spool-down is drag/brake-limited; brushed is not).
  3. Static ratio->RPM curve                             -> verifies the bat-comp
     linearization slope at the hover operating point.

Sequence (~3 min total, all four motors driven identically so the frame does not
try to roll while strapped down):
  A. staircase 8 levels up/down around hover ratio      (static curve)
  B. steps: +/-10% and +/-30% square waves at 0.5 Hz    (asymmetry, slew)
  C. chirps 1->15 Hz at +/-5% and +/-25% amplitude      (freq response, 2 amplitudes)

Usage:  ~/.pyenv/versions/flying_robots/bin/python bench_actuator_id.py
Output: Controls/logs/bench_actuator_<timestamp>.csv
Abort:  Ctrl-C (motors are always ramped to 0 and motorPowerSet.enable cleared).
"""
import math
import signal as _signal
import sys
import time
from datetime import datetime

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = "radio://0/80/2M/E7E7E7E7E7"

# Hover operating point (override with --hover N). Brushless CF21BL: 0.1 N/motor of
# THRUST_MAX=0.2 N -> ratio 0.5*65535 = 32700. Brushed drones sit higher: standard
# CF2.1 ~48000, upgraded ~40000 (check hover PWM in cfclient / a recent log first).
# motorPowerSet feeds motorsCompensateBatteryVoltage as iThrust, same as flight.
HOVER = 32700
if "--hover" in sys.argv:
    HOVER = int(sys.argv[sys.argv.index("--hover") + 1])
RATE_HZ = 100          # command update rate (limited by radio; fine for <=15 Hz content)
OUT = None

rows = []


def log_cb(ts, data, logconf):
    rows.append((time.time(), data["rpm.m1"], data["rpm.m2"], data["rpm.m3"],
                 data["rpm.m4"], data["motor.m1"], data.get("pm.vbat", 0.0)))


def set_all(cf, val):
    v = int(max(0, min(65535, val)))
    for m in ("m1", "m2", "m3", "m4"):
        cf.param.set_value(f"motorPowerSet.{m}", str(v))


def ramp(cf, frm, to, dur=1.5):
    n = int(dur * 50)
    for i in range(n + 1):
        set_all(cf, frm + (to - frm) * i / n)
        time.sleep(dur / n)


def segment_marker(name):
    rows.append((time.time(), -1, -1, -1, -1, -1, name))
    print(f"  -> {name}")


def run_sequence(cf):
    dt = 1.0 / RATE_HZ

    segment_marker("staircase")
    for lvl in (0.35, 0.42, 0.5, 0.58, 0.65, 0.58, 0.5, 0.42, 0.35):
        set_all(cf, lvl * 65535)
        time.sleep(2.0)

    ramp(cf, 0.35 * 65535, HOVER, 1.0)

    for amp_frac, name in ((0.10, "steps_10pct"), (0.30, "steps_30pct")):
        segment_marker(name)
        amp = amp_frac * HOVER
        for _ in range(4):                       # 4 periods of 0.5 Hz square
            set_all(cf, HOVER + amp); time.sleep(1.0)
            set_all(cf, HOVER - amp); time.sleep(1.0)
        set_all(cf, HOVER); time.sleep(1.0)

    # Single-motor chirps modulate ONLY m1 (m2-m4 hold hover) so the CRTP param channel
    # sees a single write per tick at 100 Hz. The analysis reads rpm_m1 only.
    for amp_frac, name in ((0.05, "chirp_5pct"), (0.25, "chirp_25pct")):
        segment_marker(name)
        amp = amp_frac * HOVER
        T = 40.0                                  # 1 -> 15 Hz log-sweep
        k = math.log(15.0 / 1.0) / T
        t0 = time.time()
        while True:
            t = time.time() - t0
            if t > T:
                break
            # phase = integral of 2*pi*f dt for the exponential sweep
            phase = 2 * math.pi * 1.0 * ((math.exp(k * t) - 1) / k)
            v = int(max(0, min(65535, HOVER + amp * math.sin(phase))))
            cf.param.set_value("motorPowerSet.m1", str(v))
            time.sleep(dt)
        set_all(cf, HOVER); time.sleep(1.0)

    # Differential chirps (2026-07-22, doc §16.2 follow-up): mimic an actual roll command —
    # m1,m2 = hover+chirp, m3,m4 = hover-chirp SIMULTANEOUSLY, instead of wiggling one motor
    # alone. Tests whether prop-to-prop aerodynamic interaction under a real differential
    # command adds lag beyond the single-motor result (candidate explanation for why the
    # single-motor bench doesn't reproduce the brushless low-kr shake). Two writes/tick.
    for amp_frac, name in ((0.05, "diff_chirp_5pct"), (0.25, "diff_chirp_25pct")):
        segment_marker(name)
        amp = amp_frac * HOVER
        T = 40.0
        k = math.log(15.0 / 1.0) / T
        t0 = time.time()
        while True:
            t = time.time() - t0
            if t > T:
                break
            phase = 2 * math.pi * 1.0 * ((math.exp(k * t) - 1) / k)
            d = amp * math.sin(phase)
            v_up = int(max(0, min(65535, HOVER + d)))
            v_dn = int(max(0, min(65535, HOVER - d)))
            cf.param.set_value("motorPowerSet.m1", str(v_up))
            cf.param.set_value("motorPowerSet.m2", str(v_up))
            cf.param.set_value("motorPowerSet.m3", str(v_dn))
            cf.param.set_value("motorPowerSet.m4", str(v_dn))
            time.sleep(dt)
        set_all(cf, HOVER); time.sleep(1.0)


def main():
    print("SAFETY CHECK: drone strapped down, props clear of everything, battery fresh?")
    if input("type 'yes' to spin motors: ").strip().lower() != "yes":
        sys.exit(1)

    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf

        lc = LogConfig(name="bench", period_in_ms=10)
        for var in ("rpm.m1", "rpm.m2", "rpm.m3", "rpm.m4"):
            lc.add_variable(var, "uint16_t")
        lc.add_variable("motor.m1", "uint16_t")   # actual ratio after bat comp
        cf.log.add_config(lc)
        lc.data_received_cb.add_callback(log_cb)

        def shutdown(*_):
            try:
                set_all(cf, 0)
                cf.param.set_value("motorPowerSet.enable", "0")
            finally:
                sys.exit(1)
        _signal.signal(_signal.SIGINT, shutdown)

        # Build has CONFIG_MOTORS_REQUIRE_ARMING=y — arm first so the motors may spin.
        try:
            cf.platform.send_arming_request(True)
            time.sleep(0.5)
        except AttributeError:
            print("(cflib without arming API — if motors stay silent, arm via cfclient first)")

        cf.param.set_value("motorPowerSet.enable", "1")
        time.sleep(0.3)
        lc.start()

        print("spin-up...")
        ramp(cf, 0, HOVER, 3.0)
        time.sleep(1.0)
        run_sequence(cf)
        print("spin-down...")
        ramp(cf, HOVER, 0, 2.0)

        lc.stop()
        set_all(cf, 0)
        cf.param.set_value("motorPowerSet.enable", "0")

    ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out = f"/home/georg/Desktop/flying_robot_course/Controls/logs/bench_actuator_{ts}.csv"
    with open(out, "w") as f:
        f.write("t_unix,rpm_m1,rpm_m2,rpm_m3,rpm_m4,motor_m1,marker\n")
        for r in rows:
            f.write(",".join(str(x) for x in r) + "\n")
    print(f"saved {len(rows)} rows -> {out}")


if __name__ == "__main__":
    main()
