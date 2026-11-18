#!/usr/bin/env python3
"""Ground test of the CS2 BROADCAST path to usd.logging (the call run_formation makes), no motors, no takeoff.

Run on the lab PC with CS2 launched (drones on the pad), same shell as run_formation:
    python3 flying_drone_stack/tools/usd_broadcast_test.py
Watch the CS2 launch terminal: each drone should print `uSD: Logging to: thesisNN` after the start
and `uSD: Wrote ... B to: thesisNN` after the stop. Afterwards one test file (usd.runTag 888) is on each card.
It sends NO usec.reset, no takeoff, no setpoints.
"""
from crazyflie_py import Crazyswarm


def main():
    swarm = Crazyswarm()
    th = swarm.timeHelper
    allcfs = swarm.allcfs
    print('[usd_test] broadcast usd.runTag=888, usd.logging=1')
    allcfs.setParam('usd.runTag', 888)
    th.sleep(0.05)
    allcfs.setParam('usd.logging', 1)
    th.sleep(5.0)
    print('[usd_test] broadcast usd.logging=0')
    allcfs.setParam('usd.logging', 0)
    th.sleep(3.0)
    print('[usd_test] done: look for `uSD: Logging to:` and `uSD: Wrote` lines of BOTH drones in the CS2 output')


if __name__ == '__main__':
    main()
