#!/usr/bin/env python3
"""Time the three server-startup signals against each other to test the race theory.

The hypothesis: crazyflie_server.cpp creates a robot's `/start_trajectory` service
BEFORE it declares `robots.<name>.initial_position`/`.uri` as parameters (there's a
blocking radio call, cf_.logReset(), in between). crazyflie_py's Crazyswarm() discovers
a robot by that service existing, then immediately queries the params -- if the gap is
real and big enough, that query can lose the race.

This script polls three things every ~0.2s from the moment you start it and timestamps
the first time each becomes true:
  1. /crazyflie_server node visible
  2. {cfname}/start_trajectory service exists  (how Crazyswarm() finds the robot)
  3. get_parameters(robots.<cfname>.initial_position, .uri) returns 2/2 values
     (the EXACT call that crashes in crazyflie_py/crazyflie.py:156-172)

It never crashes on a short response, just logs it and keeps polling -- so instead of
one IndexError you get a full timeline showing whether #2 and #3 really are separated
by a gap, and how big.

Run this in a sourced ROS2 terminal on flightcontrol1, starting it just BEFORE (or at
the same moment as) you launch the crazyflie_server -- earlier is fine, it just waits.
Let it run through steady state (all three signals true, printed a few times in a row),
then Ctrl+C. It never touches the drone -- no arm, no takeoff.

Usage:
  python3 param_race_timeline.py --cfname cf231_active
"""

import argparse
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters


def ts():
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cfname', default='cf231_active')
    parser.add_argument('--poll-hz', type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node('param_race_timeline_probe')

    events = []
    t0 = time.monotonic()

    def log(label):
        elapsed = time.monotonic() - t0
        events.append((elapsed, label))
        print(f'[{ts()}] t+{elapsed:6.2f}s  {label}')

    log('probe started -- waiting for signals (this never arms or flies anything)')

    server_seen = False
    service_seen = False
    params_seen = False
    steady_prints = 0

    get_client = node.create_client(GetParameters, '/crazyflie_server/get_parameters')
    period = 1.0 / args.poll_hz

    try:
        while rclpy.ok() and steady_prints < 5:
            rclpy.spin_once(node, timeout_sec=0.0)

            if not server_seen:
                names_and_ns = node.get_node_names_and_namespaces()
                if any(n == 'crazyflie_server' for n, ns in names_and_ns):
                    server_seen = True
                    log('/crazyflie_server node visible')

            if not service_seen:
                svc = node.get_service_names_and_types()
                target = f'/{args.cfname}/start_trajectory'
                if any(name == target for name, types in svc):
                    service_seen = True
                    log(f'{target} exists (this is how Crazyswarm() discovers the robot)')

            if not params_seen and get_client.service_is_ready():
                req = GetParameters.Request()
                req.names = [
                    f'robots.{args.cfname}.initial_position',
                    f'robots.{args.cfname}.uri',
                ]
                future = get_client.call_async(req)
                rclpy.spin_until_future_complete(node, future, timeout_sec=0.5)
                if future.done():
                    resp = future.result()
                    n = 0 if resp is None else len(resp.values)
                    if n >= 2:
                        params_seen = True
                        log(f'get_parameters returned {n}/2 values -- SUCCESS '
                            f'(the exact call crazyflie_py makes)')
                    else:
                        log(f'get_parameters returned {n}/2 values -- '
                            f'THIS WOULD CRASH crazyflie_py right now')

            if server_seen and service_seen and params_seen:
                steady_prints += 1
                if steady_prints <= 5:
                    log(f'steady state ({steady_prints}/5) -- all three signals hold true')

            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        print('\n=== summary ===')
        for elapsed, label in events:
            print(f't+{elapsed:6.2f}s  {label}')

        svc_times = [e for e, lbl in events if 'discovers the robot' in lbl]
        par_times = [e for e, lbl in events if 'SUCCESS' in lbl]
        if svc_times and par_times:
            gap = par_times[0] - svc_times[0]
            print(f'\nGap between service-discoverable and params-gettable: {gap:.3f}s')
            if gap > 0.02:
                print('-> Non-trivial gap confirmed: Crazyswarm() CAN query in this '
                      'window and crash. This supports the race theory.')
            else:
                print('-> Gap is negligible (<20ms). Race theory looks unlikely to be '
                      'the whole story -- look elsewhere.')
        else:
            print('\nDid not observe all signals before Ctrl+C -- re-run and let it '
                  'reach steady state.')

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
