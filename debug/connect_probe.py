#!/usr/bin/env python3
"""Reproduce the robots.<name>.initial_position/uri IndexError WITHOUT flying.

Crazyswarm()'s constructor is where flight.py/simple_flight.py crash -- it happens
before takeoff() is ever called, so this script is safe to loop many times: it never
arms, never takes off, never sends a single flight command. It just measures how often
the crash actually happens and how long __init__ took on the runs that succeeded.

Run this on flightcontrol1, in the same sourced ROS2 terminal you'd normally run
flight.py from, with the crazyflie_server already launched and the drone already
connected (however you judge "connected" today).

Usage:
  python3 connect_probe.py --attempts 10 --pause 3

Output: one line per attempt (PASS/FAIL + timing), a summary at the end, and every
attempt appended to connect_probe_<timestamp>.log in the current directory.
"""

import argparse
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

PROBE_ONE = r"""
import sys, time
t0 = time.monotonic()
try:
    from crazyflie_py import Crazyswarm
    swarm = Crazyswarm()
    dt = time.monotonic() - t0
    print(f"PASS dt={dt:.3f}s n_drones={len(swarm.allcfs.crazyflies)}")
    sys.exit(0)
except IndexError as e:
    dt = time.monotonic() - t0
    print(f"FAIL dt={dt:.3f}s IndexError: {e}")
    sys.exit(1)
except Exception as e:
    dt = time.monotonic() - t0
    print(f"FAIL dt={dt:.3f}s {type(e).__name__}: {e}")
    sys.exit(1)
"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--attempts', type=int, default=10)
    parser.add_argument('--pause', type=float, default=3.0,
                         help='seconds between attempts (each attempt spins up a fresh '
                              'rclpy node, give the DDS layer time to clean up)')
    args = parser.parse_args()

    log_path = Path(f'connect_probe_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
    results = []

    print(f'[connect_probe] {args.attempts} attempts, {args.pause}s apart, log -> {log_path}')
    print('[connect_probe] each attempt only builds Crazyswarm() -- no arm, no takeoff, safe to loop.\n')

    with open(log_path, 'w') as logf:
        for i in range(1, args.attempts + 1):
            stamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            proc = subprocess.run(
                [sys.executable, '-c', PROBE_ONE],
                capture_output=True, text=True, timeout=30,
            )
            out = (proc.stdout.strip() + ' ' + proc.stderr.strip()).strip()
            line = f'[{stamp}] attempt {i}/{args.attempts}: {out}'
            print(line)
            logf.write(line + '\n')
            logf.flush()
            results.append(out.startswith('PASS'))
            if i < args.attempts:
                time.sleep(args.pause)

    n_pass = sum(results)
    n_fail = len(results) - n_pass
    print(f'\n[connect_probe] {n_pass}/{len(results)} passed, {n_fail}/{len(results)} failed. '
          f'Full log: {log_path}')


if __name__ == '__main__':
    main()
