#!/usr/bin/env python3
"""Stage 5 A2: apply_gains.py readback and set against a MOCK cflib (no radio, no drone, no ROS): a fake parameter store behind fake `cflib` modules.
Why a mock: the crazyswarm2 sim backend (crazyflie_sim) has no CRTP / cflib link (it writes yaml firmware_params into the compiled controller through python bindings,
`crazyflie_server.py::_apply_oot_firmware_params`), and cflib's UDP driver needs a firmware SITL binary, which does not exist on this PC (building one is a firmware rebuild).
Covered: parameter names, the fresh-connection read (a fresh connection sees the store; a get_value after set_value on the SAME connection returns the cache), the float32 tolerance,
exit codes (0 match, 1 mismatch), `set` followed by the readback. NOT covered: the radio, real cflib timing, the real firmware's parameter table, CS2 re-pushing the yaml."""
import sys, types, io, contextlib, argparse
from pathlib import Path
import numpy as np
sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / 'dryrun')); sys.path.insert(0, str(HERE.parent))
import round as RD, apply_gains as AG  # noqa
import loop_lib as L  # noqa

STORE = {}
f32 = lambda v: float(np.float32(v))


class _Param:
    def __init__(self, cache): self.cache = cache
    def get_value(self, n): return str(self.cache[n])                  # connect-time cache, like cflib
    def set_value(self, n, v): STORE[n] = f32(float(v))               # firmware stores float32; the cache of THIS connection is not updated


class _CF:
    def __init__(self, rw_cache=None): self.param = None


class _Sync:
    def __init__(self, uri, cf=None): self.cf = cf; self.cf.param = _Param(dict(STORE))
    def __enter__(self): return self
    def __exit__(self, *a): return False


def install():
    for n in ('cflib', 'cflib.crtp', 'cflib.crazyflie', 'cflib.crazyflie.syncCrazyflie'): sys.modules[n] = types.ModuleType(n)
    sys.modules['cflib'].crtp = sys.modules['cflib.crtp']; sys.modules['cflib'].crazyflie = sys.modules['cflib.crazyflie']
    sys.modules['cflib.crtp'].init_drivers = lambda: None; sys.modules['cflib.crazyflie'].Crazyflie = _CF; sys.modules['cflib.crazyflie.syncCrazyflie'].SyncCrazyflie = _Sync


def run(fn, a):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf): rc = fn(a)
    return rc, buf.getvalue()


if __name__ == '__main__':
    install(); import tempfile
    with tempfile.TemporaryDirectory() as td:
        frag = Path(td) / 'gains.yaml'; phi = np.log(np.array([1.1, 0.9, 1.2, 1.0, 1.15, 0.8, 1.0, 1.05, 0.95, 1.25])); RD.gains_to_yaml(phi, frag)
        gains = AG.load_fragment(frag); a = argparse.Namespace(uri='radio://0/80/2M/E7E7E7BB02', fragment=str(frag))
        STORE.clear(); STORE.update({'%s.%s' % (AG.GROUP, k): 0.0 for k in gains}); STORE.update({'stabilizer.controller': 9.0, '%s.indi' % AG.GROUP: 0.0, 'rnn.en': 0.0})
        rc, out = run(AG.cmd_readback, a); print('1 readback on default store (expected FAIL, exit 1):', rc, out.splitlines()[-1]); assert rc == 1 and 'MISMATCH' in out
        rc, out = run(AG.cmd_set, a); print('2 set + readback (expected PASS, exit 0, float32 tolerance):', rc, out.splitlines()[-1]); assert rc == 0
        STORE['%s.Kpos_Dz' % AG.GROUP] = f32(STORE['%s.Kpos_Dz' % AG.GROUP] * 1.01)
        rc, out = run(AG.cmd_readback, a); print('3 one gain changed by 1 % in the firmware (expected FAIL):', rc, [l for l in out.splitlines() if 'MISMATCH' in l]); assert rc == 1
        n_names = sum(1 for k in STORE if k.startswith(AG.GROUP + '.') and not k.endswith('.indi')); print('4 parameters in the store:', n_names, '(15 gains)'); assert n_names == 15
        # ---- Stage 5: strategy switches in the read-back (S0 .. S3) and the merge on the yaml of each crazyswarm2 study branch (read with `git show`, nothing checked out)
        run(AG.cmd_set, a)                               # restore the 15 gains in the fake firmware (step 3 had changed one)
        for strat, (indi, en) in AG.SWITCHES.items():
            a2 = argparse.Namespace(uri=a.uri, fragment=a.fragment, strategy=strat)
            STORE.update({'stabilizer.controller': 9.0, '%s.indi' % AG.GROUP: float(indi), 'rnn.en': float(en)})
            rc, out = run(AG.cmd_readback, a2); assert rc == 0, out; print('5 %s read-back with the switches of %s: PASS (exit 0)' % (strat, strat))
            wrong = AG.SWITCHES['S3' if strat != 'S3' else 'S0']; STORE.update({'%s.indi' % AG.GROUP: float(wrong[0]), 'rnn.en': float(wrong[1])})
            rc, out = run(AG.cmd_readback, a2); assert rc == 1 and 'MISMATCH' in out; print('5 %s read-back with the switches of another strategy: FAIL as wanted (exit 1)' % strat)
        import subprocess
        repo = Path.home() / 'Desktop' / 'crazyswarm2'; branches = {'S0': 'main', 'S1': 'study_S1', 'S2': 'study_S2', 'S3': 'study_S3'}
        if (repo / '.git').exists():
            for strat, br in branches.items():
                y = Path(td) / ('crazyflies_%s.yaml' % strat); y.write_text(subprocess.run(['git', '-C', str(repo), 'show', '%s:crazyflie/config/crazyflies.yaml' % br], capture_output=True, text=True, check=True).stdout)
                for use, expect_ok in ((strat, True), ('S3' if strat != 'S3' else 'S0', False)):
                    ns = argparse.Namespace(fragment=str(frag), yaml=str(y), out=str(Path(td) / 'm.yaml'), strategy=use)
                    try:
                        buf = io.StringIO()
                        with contextlib.redirect_stdout(buf): AG.cmd_merge(ns)
                        ok = True
                    except SystemExit as e:
                        ok = False
                    assert ok == expect_ok, (strat, use, ok)
                    if expect_ok:
                        import yaml as Y; got = Y.safe_load(open(ns.out)); assert tuple(int(v) for v in AG.yaml_switches(got)) == AG.SWITCHES[strat]
                print('6 merge on the yaml of branch %s with --strategy %s: PASS, switches unchanged; with --strategy %s: refused' % (br, strat, 'S3' if strat != 'S3' else 'S0'))
        else:
            print('6 crazyswarm2 repo not found: merge-on-branch test skipped')
    print('MOCK TEST PASSED')
