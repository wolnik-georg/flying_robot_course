#!/usr/bin/env python3
"""Read each drone's radio address from its own EEPROM and compare with the yaml URI.

Why: with backend=cpp the PC tags every position with (yaml address & 0xFF); the drone only
uses the packet whose tag equals (EEPROM address & 0xFF). If they differ, the drone uses another
drone's position (2026-10-03: cf5's estimate was identical to cf_second's, both A8 flights crashed).

Read-only. STOP the CS2 launch first (only one program can talk to a drone).

Usage (lab PC):
    ~/.pyenv/versions/flying_robots/bin/python flying_drone_stack/tools/read_stored_radio_address.py
    ... read_stored_radio_address.py radio://0/80/2M/E7E7E7BB02 radio://0/80/2M/E7E7E7E7E9
Output is also appended to debug/lab_logs/addr_check.log (git add/push it to share).
"""
import sys
import time
import datetime
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

DEFAULT_URIS = ["radio://0/80/2M/E7E7E7BB02",   # cf5
                "radio://0/80/2M/E7E7E7E7E9"]   # cf_second
LOG = Path(__file__).resolve().parents[2] / "debug" / "lab_logs" / "addr_check.log"


def out(line, fh):
    print(line)
    fh.write(line + "\n")
    fh.flush()


def read_one(uri, fh):
    yaml_addr = int(uri.rsplit("/", 1)[1], 16)
    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="/tmp/cfcache_addr")) as s:
            mems = s.cf.mem.get_mems(MemoryElement.TYPE_I2C)
            if not mems:
                out(f"{uri}: no I2C/EEPROM memory found", fh)
                return
            done = []
            mems[0].update(lambda m: done.append(1))
            t0 = time.time()
            while not done and time.time() - t0 < 15:
                time.sleep(0.1)
            if not done:
                out(f"{uri}: EEPROM read timed out", fh)
                return
            stored = mems[0].elements["radio_address"]
    except Exception as e:  # noqa: BLE001
        out(f"{uri}: FAILED to read ({type(e).__name__}: {e})", fh)
        return
    ok = (stored & 0xFF) == (yaml_addr & 0xFF)
    out(f"{uri}: yaml address 0x{yaml_addr:010X} (id 0x{yaml_addr & 0xFF:02X}) | "
        f"EEPROM address 0x{stored:010X} (id 0x{stored & 0xFF:02X}) -> "
        f"{'MATCH' if ok else '*** MISMATCH ***'}", fh)


def main():
    uris = sys.argv[1:] or DEFAULT_URIS
    cflib.crtp.init_drivers()
    LOG.parent.mkdir(parents=True, exist_ok=True)
    with open(LOG, "a") as fh:
        out(f"--- {datetime.datetime.now().isoformat(timespec='seconds')} ---", fh)
        for u in uris:
            read_one(u, fh)


if __name__ == "__main__":
    main()
