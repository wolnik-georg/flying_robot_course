import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import time

URI = "radio://0/80/2M/E7E7E7E7E7"

cflib.crtp.init_drivers()
print(f"Connecting to {URI} ...")
with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
    cf = scf.cf
    toc = cf.param.toc.toc
    usd_group = toc.get("usd", {})
    if not usd_group:
        print("MISSING  usd.* group entirely -- deck NOT detected by firmware (param TOC has no usd group)")
    else:
        print("usd param group found:", list(usd_group.keys()))
        elem = usd_group.get("logging")
        if elem is None:
            print("MISSING  usd.logging specifically")
        else:
            val = cf.param.get_value("usd.logging")
            print(f"usd.logging current value = {val}")
            print("Setting usd.logging = 1 ...")
            cf.param.set_value("usd.logging", "1")
            time.sleep(1.0)
            val2 = cf.param.get_value("usd.logging")
            print(f"usd.logging after set = {val2}")
            print("Setting usd.logging = 0 ...")
            cf.param.set_value("usd.logging", "0")
    # also check deck detection log var if present
    log_toc = cf.log.toc.toc
    print()
    print("gyro group present in log TOC:", "gyro" in log_toc)
