#!/usr/bin/env python3
"""Stage 7 mock test of preflight item 7 (NS2 weights hash and yaml switches), desk only, no pool, no radio: right values pass, one wrong hash and one wrong yaml value fail (also S0 / S1 with NS2 on).
  /usr/bin/python3 test_preflight_ns2.py  -> prints PASS / FAIL per case and 'ALL CASES AS EXPECTED'"""
import argparse, subprocess, sys, tempfile
from pathlib import Path
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent))
import preflight as P  # noqa
repo = Path.home() / 'Desktop' / 'crazyswarm2'
def branch_yaml(br): return subprocess.run(['git', '-C', str(repo), 'show', '%s:crazyflie/config/crazyflies.yaml' % br], capture_output=True, text=True, check=True).stdout
def run(ctrl, yaml_text=None, ns2_file=None):
    P.FAILS.clear(); a = argparse.Namespace(yaml=None, ns2_file=ns2_file, ns2_yaml=None)
    td = tempfile.mkdtemp()
    if yaml_text is not None: (Path(td) / 'y.yaml').write_text(yaml_text); a.ns2_yaml = str(Path(td) / 'y.yaml')
    lines = []; orig = P.res
    def res(ok, name, msg='', note=False):
        lines.append((ok, name, msg, note)); orig(ok, name, msg, note)
    P.ns2_checks(ctrl, a, res); fails = [f for f in P.FAILS]; P.FAILS.clear(); return fails
import shutil
W = Path(__file__).resolve().parents[3] / 'simulator' / 'frsim' / 'data' / 'ns2_v2.npz'
bad = Path(tempfile.mkdtemp()) / 'ns2_v2.npz'; b = bytearray(W.read_bytes()); b[100] ^= 1; bad.write_bytes(bytes(b))          # one flipped bit: wrong hash
exp = []; ok_all = True
def case(name, ctrl, want_fail, **kw):
    global ok_all
    f = run(ctrl, **kw); good = bool(f) == want_fail; ok_all &= good; print('CASE %-58s -> %s (%s)' % (name, 'FAILS as expected' if (f and want_fail) else ('passes as expected' if good else 'UNEXPECTED'), f)); return good
for ctrl, br in (('S2', 'study_S2'), ('S3', 'study_S3')):
    y = branch_yaml(br)
    case('%s right hash, yaml of branch %s' % (ctrl, br), ctrl, False, yaml_text=y)
    case('%s WRONG hash (one bit flipped), right yaml' % ctrl, ctrl, True, yaml_text=y, ns2_file=str(bad))
    case('%s right hash, WRONG yaml (cf5 rnn.en flipped)' % ctrl, ctrl, True, yaml_text=y.replace('en: 1           # %s' % ctrl, 'en: 0           # %s' % ctrl, 1))
for ctrl, br in (('S0', 'main'), ('S1', 'study_S1')):
    y = branch_yaml(br)
    case('%s yaml of branch %s (NS2 off)' % (ctrl, br), ctrl, False, yaml_text=y)
y1 = branch_yaml('study_S1'); case('S1 with the yaml of study_S2 (NS2 on, wrong)', 'S1', True, yaml_text=branch_yaml('study_S2'))
print('ALL CASES AS EXPECTED' if ok_all else 'SOME CASE UNEXPECTED'); sys.exit(0 if ok_all else 1)
