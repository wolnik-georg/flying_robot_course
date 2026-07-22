#!/usr/bin/env python
"""Post-reflash sanity check — confirms the new clamp/timeline/act_tau params exist on the
flashed firmware and reads their current (yaml-pushed) values. NO motors spin; read-only
param TOC check. Run this after `make cload` and BEFORE any test flight, to catch a stale
flash or a yaml/firmware mismatch cheaply.

Usage: ~/.pyenv/versions/flying_robots/bin/python check_new_params.py
"""
import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = "radio://0/80/2M/E7E7E7E7E7"

EXPECTED = [
    ("traj", "n_entry"), ("traj", "n_exit"), ("traj", "ci"),
    ("indi_gains", "clamp_en"), ("indi_gains", "tau_xy_max"), ("indi_gains", "tau_z_max"),
    ("indi_gains", "tilt_max_deg"), ("indi_gains", "thrust_max"), ("indi_gains", "act_tau"),
]


def main():
    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        toc = cf.param.toc.toc
        ok = True
        for group, name in EXPECTED:
            elem = toc.get(group, {}).get(name)
            if elem is None:
                print(f"  MISSING  {group}.{name}  <-- reflash did not take, or yaml/firmware mismatch")
                ok = False
                continue
            val = cf.param.get_value(f"{group}.{name}")
            print(f"  OK       {group}.{name:14s} = {val:>10}  (type {elem.ctype})")
        print()
        if ok:
            print("All new params present. Values above should match what you expect from")
            print("crazyflies.yaml (clamp_en=15, tau_xy_max=0.014, act_tau=0.0 unless set, etc).")
            print("If any value looks wrong, restart the CS2 server so it re-pushes crazyflies.yaml.")
        else:
            sys.exit(1)


if __name__ == "__main__":
    main()
