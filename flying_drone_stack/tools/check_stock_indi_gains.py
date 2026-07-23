#!/usr/bin/env python
"""Read-only check of the LIVE stock-INDI gains currently on the drone (ctrlINDI param group).
No motors, no writes -- safe to run any time the radio is free (CS2 stopped).

Usage: ~/.pyenv/versions/flying_robots/bin/python check_stock_indi_gains.py
"""
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = "radio://0/80/2M/E7E7E7E7E7"

VARS = [
    ("ctrlINDI", "ref_err_p"), ("ctrlINDI", "ref_err_q"), ("ctrlINDI", "ref_err_r"),
    ("ctrlINDI", "ref_rate_p"), ("ctrlINDI", "ref_rate_q"), ("ctrlINDI", "ref_rate_r"),
    ("ctrlINDI", "g1_p"), ("ctrlINDI", "g1_q"), ("ctrlINDI", "g1_r"), ("ctrlINDI", "g2"),
    ("ctrlINDI", "act_dyn_p"), ("ctrlINDI", "act_dyn_q"), ("ctrlINDI", "act_dyn_r"),
    ("stabilizer", "controller"),
]

# expected values after the full-match reflash (controller_indi.h edits, docs §18.4c):
# ref_err/ref_rate match kr=2400/kw=170 on all 3 axes; g1/g2 re-derived through the legacy
# mixer for this airframe; act_dyn set to our bench-measured 44ms actuator; filt_cutoff=60Hz.
RAISED = {
    "ref_err_p": 14.12, "ref_err_q": 14.12, "ref_err_r": 14.12,
    "ref_rate_p": 170.0, "ref_rate_q": 170.0, "ref_rate_r": 170.0,
    "g1_p": 0.01997, "g1_q": 0.01997, "g1_r": 0.00463, "g2": 0.000134,
    "act_dyn_p": 0.04348, "act_dyn_q": 0.04348, "act_dyn_r": 0.04348,
}


def main():
    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        for group, name in VARS:
            val = cf.param.get_value(f"{group}.{name}")
            flag = ""
            if name in RAISED:
                flag = "  <- MATCHES raised value" if abs(float(val) - RAISED[name]) < 0.01 \
                       else f"  <- expected {RAISED[name]} (still at default/other value!)"
            print(f"  {group}.{name:12s} = {val}{flag}")
        print("\nIf stabilizer.controller != 3, stock INDI isn't even the active controller.")


if __name__ == "__main__":
    main()
