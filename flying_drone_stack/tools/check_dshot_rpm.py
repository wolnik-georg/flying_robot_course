"""DShot RPM check for S1 / S3 (Omar C INDI reads motor.m1_rpm..m4_rpm since the DShot image, 2026-11-22). CS2 must be stopped.
Usage: python3 flying_drone_stack/tools/check_dshot_rpm.py [radio uri] [--require-spin]
Default (prep, motors idle): the four log variables motor.m1_rpm..m4_rpm must exist in the log TOC (the controller's own availability test
is exactly this), and they are read for about 1 s and printed (idle: 0 or 65535 = no value is normal). Exit 0 if the variables exist.
--require-spin (ground / hover test, motors turning): additionally every motor must report 0 < rpm < 28000 in the last 10 samples.
Read-only: nothing is written to the drone. Not tested on hardware when written."""
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

args = [a for a in sys.argv[1:] if not a.startswith("--")]
SPIN = "--require-spin" in sys.argv
URI = args[0] if args else "radio://0/80/2M/E7E7E7BB02"
NAMES = [f"motor.m{i}_rpm" for i in (1, 2, 3, 4)]
cflib.crtp.init_drivers()
with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
    group = scf.cf.log.toc.toc.get("motor", {})
    missing = [n for n in NAMES if n.split(".")[1] not in group]
    if missing:
        print("MISSING in the log TOC:", ", ".join(missing), ": STOP, S1/S3 INDI would be silently off")
        sys.exit(2)
    lg = LogConfig(name="dshot_rpm", period_in_ms=20)
    for n in NAMES:
        lg.add_variable(n, "uint16_t")
    rows = []
    t0 = time.time()
    with SyncLogger(scf, lg) as logger:
        for _, data, _ in logger:
            rows.append([data[n] for n in NAMES])
            if time.time() - t0 > 1.0 and len(rows) >= 10:
                break
    last = rows[-10:]
    print("motor.m1_rpm..m4_rpm present; last sample:", rows[-1], f"({len(rows)} samples in {time.time() - t0:.1f} s)")
    if SPIN:
        bad = [i + 1 for i in range(4) if not all(0 < r[i] < 28000 for r in last)]
        if bad:
            print("NOT PLAUSIBLE for motors", bad, ": need 0 < rpm < 28000 while spinning: STOP")
            sys.exit(1)
        print("DShot RPM plausible on all four motors: OK")
    else:
        print("OK (log variables exist; idle values 0 / 65535 are normal)")
