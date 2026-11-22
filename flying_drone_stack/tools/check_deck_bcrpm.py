"""Read deck.bcRpm from the drone (S1 / S3 INDI needs deck.bcRpm == 1). CS2 must be stopped.
Usage: python3 flying_drone_stack/tools/check_deck_bcrpm.py [radio uri]   (default cf5)
Exit code 0 only if deck.bcRpm == 1. Read-only: nothing is written to the drone."""
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = sys.argv[1] if len(sys.argv) > 1 else "radio://0/80/2M/E7E7E7BB02"
cflib.crtp.init_drivers()
with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
    cf = scf.cf
    if "bcRpm" not in cf.param.toc.toc.get("deck", {}):
        print("MISSING  deck.bcRpm in the param TOC: STOP, do not fly S1/S3")
        sys.exit(2)
    got = {}
    cf.param.add_update_callback(group="deck", name="bcRpm", cb=lambda n, v: got.__setitem__("v", v))
    cf.param.request_param_update("deck.bcRpm")   # fresh read from the drone, not the cache
    t0 = time.time()
    while "v" not in got and time.time() - t0 < 5:
        time.sleep(0.1)
    v = got.get("v")
    print(f"deck.bcRpm = {v}  {'OK' if str(v) == '1' else 'NOT 1: STOP, do not fly S1/S3'}")
    sys.exit(0 if str(v) == "1" else 1)
