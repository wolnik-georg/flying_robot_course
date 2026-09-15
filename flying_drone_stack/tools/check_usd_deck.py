"""Confirm the uSD deck is detected and ready to log, WITHOUT touching usd.logging.

2026-09-15: the previous version of this script toggled usd.logging 1 -> 0 to prove the deck
would accept the command. That toggle writes into whatever file the deck currently has open --
usddeck.c does not start a new file on each logging start, so every deck-check run before a
flight becomes part of the SAME uSD log the flight itself will write, contaminating it with
unrelated data. This cost real analysis time on 2026-09-15 (the actual ~18s A8 flight had to be
found by correlation inside a ~200s recording that also contained several of these checks).

`usd.canLog` and `usd.bcUSD` are read-only params (`PARAM_RONLY`, usddeck.c) that directly
reflect whether the card mounted and the deck initialised successfully -- checking them touches
`usd.logging` not at all, so running this as many times as you like before a flight is safe.

Run once per drone before flying, with the URI edited below (or pass one as argv[1]).
"""

import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = sys.argv[1] if len(sys.argv) > 1 else "radio://0/80/2M/E7E7E7E7E7"

cflib.crtp.init_drivers()
print(f"Connecting to {URI} ...")
with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
    cf = scf.cf
    toc = cf.param.toc.toc
    usd_group = toc.get("usd", {})
    if not usd_group:
        print("MISSING  usd.* group entirely -- deck NOT detected by firmware "
              "(param TOC has no usd group)")
    else:
        print("usd param group found:", list(usd_group.keys()))

        bc = usd_group.get("bcUSD")
        if bc is None:
            print("MISSING  usd.bcUSD (deck-initialised flag) -- firmware build predates it?")
        else:
            val = cf.param.get_value("usd.bcUSD")
            print(f"usd.bcUSD (deck initialised)   = {val}  {'OK' if val == '1' else 'NOT READY'}")

        can = usd_group.get("canLog")
        if can is None:
            print("MISSING  usd.canLog (card-mounted-and-ready flag)")
        else:
            val = cf.param.get_value("usd.canLog")
            print(f"usd.canLog (card ready to write) = {val}  {'OK' if val == '1' else 'NOT READY'}")

        cur = usd_group.get("logging")
        if cur is None:
            print("MISSING  usd.logging specifically")
        else:
            # read-only glance, never written -- see the module docstring for why
            print(f"usd.logging current value (not touched by this check) = "
                  f"{cf.param.get_value('usd.logging')}")

    log_toc = cf.log.toc.toc
    print()
    print("gyro group present in log TOC:", "gyro" in log_toc)
