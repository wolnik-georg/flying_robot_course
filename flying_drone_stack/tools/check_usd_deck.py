"""Confirm the uSD deck is detected and ready to log, WITHOUT touching usd.logging.

2026-09-15: the previous version of this script toggled usd.logging 1 -> 0 to prove the deck
would accept the command. Reading usddeck.c's usdWriteTask properly (later the same evening)
shows what that actually did: every 0 -> 1 CREATES A NEW thesisNN file, and the 1 -> 0 closes
it. So each deck-check burned a file-counter slot and left a short junk recording on the card,
cluttering it with files that look like flights but are not. (An earlier note here claimed the
toggle appended into the flight's own file -- that was wrong, but the conclusion to stop
toggling was right.)

usd.canLog and usd.bcUSD are read-only params (PARAM_RONLY, usddeck.c) that directly reflect
whether the card mounted and the deck initialised successfully -- checking them creates no
file and consumes no counter slot, so running this as many times as you like before a flight
is safe.

One real trade-off to know about: the old toggle had a side effect people relied on, because a
clean 1 -> 0 is the ONLY thing that runs f_close and finalises a file. If a previous flight
died without its stop command (crash, power loss, card pulled while logging), it leaves a
0-byte file behind, and the toggle would "fix" that by forcing a clean open/close cycle. This
version does not do that -- but it also is not needed: the next flight's own logging start
simply creates the next file. A 0-byte file is a record of a session that never stopped
cleanly, not a fault to be repaired.

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
