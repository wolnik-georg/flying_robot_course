#!/usr/bin/env python3
"""Minimal ROS 2 check, independent of CS2: does a node receive its own /parameter_events
through a /parameter_events subscription (what rclcpp::ParameterEventHandler does, what crazyflie_server relies on for runtime sets)?

Run with ROS sourced, CS2 NOT required:
  python3 param_event_selftest.py                       # default RMW (Fast DDS)
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 param_event_selftest.py   # if installed
Prints RESULT lines; exit 0 = both events received, 1 = something missing.
"""
import os
import subprocess
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterEvent
from rclpy.qos import qos_profile_parameter_events

NODE = 'pe_selftest'
seen = {'own': False, 'cli': False}


def main():
    rclpy.init()
    node = rclpy.create_node(NODE)
    node.declare_parameter('x', 0)

    def cb(event):
        if event.node != '/' + NODE:
            return
        for p in list(event.changed_parameters) + list(event.new_parameters):
            if p.name == 'x':
                v = p.value.integer_value
                if v == 1:
                    seen['own'] = True
                if v == 2:
                    seen['cli'] = True
                print(f'  event received: x={v}', flush=True)

    # same topic + QoS that rclcpp::ParameterEventHandler subscribes with
    _sub = node.create_subscription(ParameterEvent, '/parameter_events', cb, qos_profile_parameter_events)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    time.sleep(1.5)

    print('RMW_IMPLEMENTATION =', os.environ.get('RMW_IMPLEMENTATION', '(unset, default)'), flush=True)
    print('set x=1 from inside the node (like the server cache update) ...', flush=True)
    node.set_parameters([Parameter('x', value=1)])
    time.sleep(2.0)
    print('set x=2 via ros2 param set (like the CLI / crazyflie_py) ...', flush=True)
    r = subprocess.run(['ros2', 'param', 'set', '/' + NODE, 'x', '2'], capture_output=True, text=True, timeout=20)
    print('  ros2 param set:', (r.stdout + r.stderr).strip(), flush=True)
    time.sleep(2.0)

    print(f"RESULT own-set event received: {'yes' if seen['own'] else 'NO'}")
    print(f"RESULT cli-set event received: {'yes' if seen['cli'] else 'NO'}", flush=True)
    return 0 if all(seen.values()) else 1


if __name__ == '__main__':
    rc = main()
    os._exit(rc)  # skip executor teardown (avoids a harmless abort at exit)
