#!/usr/bin/env python3
"""Ground test of the uSD start gate (crazyflie_examples/usd_gate.py): CS2 path, no motors, no takeoff, no usec.reset.

Run on the lab PC with CS2 launched (drones on the pad), in the shell where `ros2 run` works, with the crazyswarm2
build that contains usd_gate.py and the yaml topic `usd_status`:

    python3 flying_drone_stack/tools/usd_gate_ground_test.py --mode pass       # must PASS on both drones
    python3 flying_drone_stack/tools/usd_gate_ground_test.py --mode negative   # start only cf5: the gate must FAIL cf_second

Exit code 0 = the test behaved as required (pass mode: gate passed; negative mode: gate failed on cf_second only).
Each run writes one test file per card (usd.runTag 888 = pass, 889 = negative): archive them off the cards before
flying (see experiments/diffcotune/lab/USD_GATE_LAB_STEPS.md). Always stops logging on both drones at the end.
"""
import argparse
import json
import sys

from crazyflie_examples import usd_gate as g
from crazyflie_interfaces.msg import LogDataGeneric
from crazyflie_py import Crazyswarm


class _DeadFuture:
    """Negative test: a start that never reaches the drone."""

    def done(self):
        return True

    def result(self):
        class R:
            successful = False
            reason = 'negative test: start not sent to this drone'
        return type('Res', (), {'results': [R()]})()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=('pass', 'negative'), required=True)
    args, _ = ap.parse_known_args()

    swarm = Crazyswarm()
    th, allcfs = swarm.timeHelper, swarm.allcfs
    cfs = list(allcfs.crazyflies)
    names = [c.prefix.lstrip('/') for c in cfs]
    tag = 888 if args.mode == 'pass' else 889
    print(f'[gate-test] mode={args.mode} drones={names} runTag={tag}')
    print(f'[gate-test] usd params in the server list (allcfs.paramTypeDict): '
          f'{ {p: (p in allcfs.paramTypeDict) for p in g.PARAMS} }')
    for c in cfs:
        print(f'[gate-test]   {c.prefix}: ' + str({p: (p in c.paramTypeDict) for p in g.PARAMS}))

    taps = g.make_taps(allcfs, names, LogDataGeneric)
    th.sleep(2.0)
    pre = g.precheck(allcfs, names, taps, th.sleep, th.time)
    print('[gate-test] precheck: ' + json.dumps(pre, sort_keys=True))
    if not pre['passed']:
        print('[gate-test] RESULT: precheck FAILED (topic silent / already writing / params missing): not starting')
        return 1

    keep = 'cf5' if 'cf5' in names else names[0]
    set_fn = g.set_param_future
    if args.mode == 'negative':
        target = cfs[names.index(keep)]

        def set_fn(t, name, value, broadcast=False):      # noqa: F811 - only the kept drone ever gets the start
            if broadcast:
                return g.set_param_future(target, name, value, broadcast=False)
            if t is target:
                return g.set_param_future(t, name, value, broadcast=False)
            return _DeadFuture()
        print(f'[gate-test] NEGATIVE: the start is sent ONLY to {keep}')

    res = g.start_and_verify(allcfs, cfs, names, taps, tag, th.sleep, th.time, lambda: th.time(),
                             set_fn=set_fn)
    print('[gate-test] start: ' + json.dumps(res, sort_keys=True))
    th.sleep(2.0 if res['passed'] else 0.5)
    stop = g.stop_and_verify(allcfs, cfs, names, taps, th.sleep, th.time)
    print('[gate-test] stop: ' + json.dumps(stop, sort_keys=True))
    print('[gate-test] test files (runTag %d) are now on the cards: ARCHIVE them before any flight' % tag)

    if args.mode == 'pass':
        ok = res['passed'] and stop['passed']
    else:
        d = res['drones']
        ok = (not res['passed']) and d[keep]['passed'] and all(not v['passed'] for n, v in d.items() if n != keep)
    print(f'[gate-test] RESULT: {"OK" if ok else "UNEXPECTED"} (mode {args.mode})')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
