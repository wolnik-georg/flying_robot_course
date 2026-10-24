#!/usr/bin/env python3
"""Bench check (ground only, CS2 stopped) that the FBL configuration survives the controller init (image study_tree_v2_fbl_initfix_candidate.bin and later).
Reproduces the CS2 launch order on a freshly powered drone: write ctrlFbl.cd 0.55 and ctrlFbl.en 1 while the drone is still on controller 6, THEN switch to controller 9
(the init). On a fresh connection the values must still be 0.55 and 1. Then it restores en 0, cd 0.43 and controller 6. No setpoint is sent, motors stay off.
Run it right after a power cycle (the old image fails here: cd 0.43, en 0 after the switch).
  python3 flying_drone_stack/tools/fbl_init_survival_check.py radio://0/80/2M/E7E7E7BB02"""
import sys, time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

uri = sys.argv[1]
cflib.crtp.init_drivers()
def rd(names):
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        return {n: float(scf.cf.param.get_value(n)) for n in names}
N = ["stabilizer.controller", "ctrlFbl.en", "ctrlFbl.cd", "ctrlFbl.preset", "ctrlFbl.kp_xy", "ctrlFbl.z_on"]
v0 = rd(N); print("fresh state          ", v0)
with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
    cf = scf.cf
    if int(v0["stabilizer.controller"]) != 6: cf.param.set_value("stabilizer.controller", "6"); time.sleep(1.5)
    cf.param.set_value("ctrlFbl.cd", "0.55"); cf.param.set_value("ctrlFbl.en", "1"); time.sleep(0.5)
    cf.param.set_value("stabilizer.controller", "9"); time.sleep(2.5)       # the controller init runs here
v1 = rd(N); print("after yaml-like order", v1)
with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
    scf.cf.param.set_value("ctrlFbl.en", "0"); scf.cf.param.set_value("ctrlFbl.cd", "0.43"); time.sleep(0.5)
    scf.cf.param.set_value("stabilizer.controller", "6"); time.sleep(1.0)
ok = (abs(v1["ctrlFbl.cd"] - 0.55) < 1e-3 and v1["ctrlFbl.en"] == 1 and v1["stabilizer.controller"] == 9 and v1["ctrlFbl.preset"] == 1 and abs(v1["ctrlFbl.kp_xy"] - 14.0) < 1e-3)
print("PASS: ctrlFbl.en 1 and cd 0.55 survived the controller init (restored en 0, cd 0.43, controller 6)" if ok else "FAIL: the init still wipes the FBL config; do not launch for FBL flights")
sys.exit(0 if ok else 1)
