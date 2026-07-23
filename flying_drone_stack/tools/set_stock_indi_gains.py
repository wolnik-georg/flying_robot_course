#!/usr/bin/env python
"""Push matched-aggressiveness gains into the STOCK firmware INDI controller (ctrlINDI param
group, controller_indi.c) for the apples-to-apples comparison test (doc §18.4).

Stock's inner loop: alpha_ref = ref_rate*(ref_err*eR - e_omega)
Ours:                alpha_ref = alpha_des - kr*eR - kw*e_omega
=> ref_rate plays the role of our kw; ref_err = kr/kw sets the same PD-zero.
Default values: ref_err_p/q=5.0, ref_rate_p/q=24.0 (gentle, generic CF2.1).
This script sets them to match our flown kr=2400/kw=170 -> ref_err=14.12, ref_rate=170.

g1 (effectiveness), g2 (yaw coupling), act_dyn (actuator model) are DELIBERATELY left untouched
-- this test isolates aggressiveness/bandwidth only, per the doc's next-step plan.

IMPORTANT: run this BEFORE starting the CS2 server for the test flight (only one client can
hold the radio at a time). Values set here persist in the firmware's RAM regardless of which
client connects next -- CS2 will not overwrite them (crazyflies.yaml has no ctrlINDI block).
They reset to firmware defaults on power-cycle/reflash, so re-run this after either.

Usage: ~/.pyenv/versions/flying_robots/bin/python set_stock_indi_gains.py
"""
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = "radio://0/80/2M/E7E7E7E7E7"

KR, KW = 2400.0, 170.0
REF_RATE = KW
REF_ERR = KR / KW

SETS = [
    ("ctrlINDI", "ref_err_p", REF_ERR),
    ("ctrlINDI", "ref_err_q", REF_ERR),
    ("ctrlINDI", "ref_rate_p", REF_RATE),
    ("ctrlINDI", "ref_rate_q", REF_RATE),
]


def main():
    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        print(f"Setting stock INDI gains to match kr={KR}/kw={KW} "
              f"(ref_err={REF_ERR:.2f}, ref_rate={REF_RATE:.1f}):")
        for group, name, val in SETS:
            cf.param.set_value(f"{group}.{name}", str(val))
            readback = cf.param.get_value(f"{group}.{name}")
            print(f"  {group}.{name:12s} -> {readback}")
        print("\nDone. g1/g2/act_dyn left at stock defaults (not part of this test).")
        print("Now start the CS2 server and fly with stabilizer.controller=3, e.g.:")
        print("  ros2 run crazyflie_examples flight -- --trajectory oval --mode 1 --kt 0.3 --brushless")


if __name__ == "__main__":
    main()
