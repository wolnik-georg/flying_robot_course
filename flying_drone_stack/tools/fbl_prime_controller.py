#!/usr/bin/env python3
"""Put the bottom drone on controller 9 BEFORE the CS2 launch, so that the launch does not trigger the controller init that wipes the ctrlFbl.* yaml values.
Why (found 2026-10-24): the Omar controller init (run when stabilizer.controller is switched to 9) calls fbl_defaults(), which resets ctrlFbl.en / cd / gains.
At a fresh boot the ctrlFbl.* values are zero until that first init, and CS2 applies the yaml params in alphabetical order (std::map): ctrlFbl.* first,
stabilizer.controller LAST -> the init would overwrite ctrlFbl.en 1 / cd 0.55 with the defaults (FBL silently off, flight = plain S0).
With the drone already on controller 9 the switch is a no-op and the yaml values stay. Needed after EVERY power cycle (battery swap) of cf5, CS2 stopped.
cflib's get_value returns the value cached at connect time, so the check reconnects and reads fresh values.
  python3 flying_drone_stack/tools/fbl_prime_controller.py radio://0/80/2M/E7E7E7BB02
Exit 0 only if, on a fresh connection, the controller is 9 and ctrlFbl.preset 1, kp_xy 14, cd 0.43, z_on 0.15, en 0 (defaults in place, FBL off until the yaml turns it on)."""
import sys, time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

uri = sys.argv[1]
cflib.crtp.init_drivers()
with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
    scf.cf.param.set_value("stabilizer.controller", "9"); time.sleep(2.0)
time.sleep(1.0)
with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
    g = lambda n: float(scf.cf.param.get_value(n))
    v = {n: g(n) for n in ("stabilizer.controller", "ctrlFbl.en", "ctrlFbl.preset", "ctrlFbl.kp_xy", "ctrlFbl.ki_z", "ctrlFbl.cd", "ctrlFbl.z_on")}
for n, x in v.items(): print(f"  {n:24s} {x}")
ok = (v["stabilizer.controller"] == 9 and v["ctrlFbl.en"] == 0 and v["ctrlFbl.preset"] == 1 and abs(v["ctrlFbl.kp_xy"] - 14.0) < 1e-3
      and abs(v["ctrlFbl.cd"] - 0.43) < 1e-3 and abs(v["ctrlFbl.z_on"] - 0.15) < 1e-3)
print("PASS: cf5 is on controller 9 with the FBL defaults in place; start CS2 now" if ok else "FAIL: do not launch CS2; tell Claude")
sys.exit(0 if ok else 1)
