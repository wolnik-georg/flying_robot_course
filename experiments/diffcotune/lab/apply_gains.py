#!/usr/bin/env python3
"""Stage 4c, P2 (proposal, NOT tested on hardware): apply the gain file of a round (round_<l>/gains.yaml, written by round.py) to cf5 and read the values back from the firmware.
  merge     python3 apply_gains.py merge --fragment S0/round_1/gains.yaml --yaml <crazyflies.yaml> [--out <file>]
            Option A: merges the 15 `ctrlOmarIndi` values into the cf5 block of a crazyflies.yaml (text edit, comments kept; existing keys are replaced, missing keys inserted),
            then parses before and after with PyYAML and fails if anything other than those keys changed. Works on a path you give it (default: in place on a copy named <yaml>.new when --out is not given).
  readback  python3 apply_gains.py readback --uri radio://0/80/2M/E7E7E7BB02 --fragment S0/round_1/gains.yaml
            Fresh cflib connection (the values cflib reads at connect time come from the firmware; a get_value after a set on the SAME connection is a cache), compares the 15 gains with the fragment
            (float32 tolerance), prints stabilizer.controller, ctrlOmarIndi.indi, rnn.en; exit 0 only if all match. CS2 must be stopped (the radio is shared).
  set       python3 apply_gains.py set --uri ... --fragment ...   Option B: writes the 15 values over the radio, disconnects, then runs the readback. CS2 stopped. A later CS2 launch applies the
            yaml's own firmware_params again (it sets ctrlOmarIndi.Kpos_Iz and indi), so `set` is only valid if the yaml has no Kpos_Iz line for cf5 or holds the same value.
Nothing here touches crazyswarm2 unless you pass its yaml path."""
import argparse, re, sys, time
from pathlib import Path
import yaml

ROBOT = 'cf5'; GROUP = 'ctrlOmarIndi'
# Stage 5: the strategy switches of the study branches (crazyswarm2 study_S0 = main, study_S1, study_S2, study_S3 differ ONLY in these two yaml values; flying_drone_stack/tools/set_tree_strategy.py):
# ctrlOmarIndi.indi (0 geometric / 3 INDI) and rnn.en (NS2 off / on); stabilizer.controller is 9 (Omar tree) in all of them. cf_second is always indi 0, rnn.en 0.
SWITCHES = {'S0': (0, 0), 'S1': (3, 0), 'S2': (0, 1), 'S3': (3, 1)}; CONTROLLER_SLOT = 9


def yaml_switches(doc, robot=ROBOT):
    fp = doc['robots'][robot]['firmware_params']
    return fp.get(GROUP, {}).get('indi'), fp.get('rnn', {}).get('en')


def load_fragment(path, robot=ROBOT):
    d = yaml.safe_load(open(path))
    g = d['robots'][robot]['firmware_params'][GROUP]
    return {k: float(v) for k, v in g.items()}


def merge_text(text, gains, robot=ROBOT):
    lines = text.split('\n')
    i0 = next((i for i, l in enumerate(lines) if re.match(r'^  %s:\s*(#.*)?$' % robot, l)), None)
    if i0 is None: raise SystemExit('block "  %s:" not found' % robot)
    end = next((i for i in range(i0 + 1, len(lines)) if re.match(r'^  \S', lines[i]) or re.match(r'^\S', lines[i])), len(lines))
    h = next((i for i in range(i0, end) if re.match(r'^      %s:\s*(#.*)?$' % GROUP, lines[i])), None)
    if h is None: raise SystemExit('block "      %s:" not found inside %s' % (GROUP, robot))
    e = next((i for i in range(h + 1, end) if lines[i].strip() and not lines[i].startswith('        ') and not lines[i].strip().startswith('#')), end)
    for k, v in gains.items():
        idx = next((i for i in range(h + 1, e) if re.match(r'^        %s:\s' % re.escape(k), lines[i])), None)
        val = repr(float(v))
        if idx is not None:
            m = re.match(r'^(        %s:\s*)\S+(\s*(#.*)?)$' % re.escape(k), lines[idx])
            lines[idx] = (m.group(1) + val + (m.group(2) or '')) if m else '        %s: %s' % (k, val)
        else:
            lines.insert(h + 1, '        %s: %s' % (k, val)); e += 1
    return '\n'.join(lines)


def deep_diff(a, b, path=''):
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(set(a) | set(b), key=str):
            out += deep_diff(a.get(k, '<missing>'), b.get(k, '<missing>'), path + '/' + str(k))
        return out
    return [] if a == b else [(path, a, b)]


def cmd_merge(a):
    gains = load_fragment(a.fragment); src = Path(a.yaml); text = src.read_text(); new = merge_text(text, gains)
    before, after = yaml.safe_load(text), yaml.safe_load(new)
    if a.strategy:                                       # Stage 5: the yaml must be the branch of the strategy; the merge never changes the switches (deep_diff below allows only the 15 gains)
        for lab_, doc in (('before', before), ('after', after)):
            got = yaml_switches(doc)
            if got[0] is None or got[1] is None or tuple(int(x) for x in got) != SWITCHES[a.strategy]: raise SystemExit('FAIL: yaml %s the merge has indi / rnn.en = %s, strategy %s needs %s (wrong crazyswarm2 branch?)' % (lab_, got, a.strategy, SWITCHES[a.strategy]))
        print('strategy %s: cf5 indi / rnn.en = %s in the yaml, unchanged by the merge' % (a.strategy, SWITCHES[a.strategy]))
    diffs = deep_diff(before, after); allowed = '/robots/%s/firmware_params/%s/' % (ROBOT, GROUP)
    bad = [d for d in diffs if not d[0].startswith(allowed)]
    got = after['robots'][ROBOT]['firmware_params'][GROUP]
    miss = {k: (v, got.get(k)) for k, v in gains.items() if abs(float(got.get(k, 1e99)) - v) > 1e-12}
    if bad or miss: raise SystemExit('FAIL: unexpected changes %s / missing %s' % (bad[:3], miss))
    out = Path(a.out) if a.out else src.with_name(src.name + '.new'); out.write_text(new)
    print('wrote %s: %d values merged into %s.firmware_params.%s (%d changed vs the input, nothing else differs)' % (out, len(gains), ROBOT, GROUP, len(diffs)))
    for p, x, y in diffs: print('   %-50s %s -> %s' % (p.split('/')[-1], x, y))


def compare(gains, values, rel=1e-6):
    rows = []
    for k, v in gains.items():
        x = values.get('%s.%s' % (GROUP, k)); ok = x is not None and abs(x - v) <= rel * max(1.0, abs(v)); rows.append((k, v, x, ok))
    return rows


def read_fresh(uri, names):
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        return {n: float(scf.cf.param.get_value(n)) for n in names}


def cmd_readback(a, gains=None):
    gains = gains or load_fragment(a.fragment)
    names = ['%s.%s' % (GROUP, k) for k in gains] + ['stabilizer.controller', '%s.indi' % GROUP, 'rnn.en']
    vals = read_fresh(a.uri, names); rows = compare(gains, vals)
    for k, v, x, ok in rows: print('  %-10s fragment %-14.8g firmware %-14s %s' % (k, v, x, 'ok' if ok else 'MISMATCH'))
    for n in ('stabilizer.controller', '%s.indi' % GROUP, 'rnn.en'): print('  %-26s %s' % (n, vals[n]))
    ok = all(r[3] for r in rows)
    if getattr(a, 'strategy', None):                      # Stage 5: the switches of the strategy are part of the read-back
        want = {'stabilizer.controller': CONTROLLER_SLOT, '%s.indi' % GROUP: SWITCHES[a.strategy][0], 'rnn.en': SWITCHES[a.strategy][1]}
        bad = {n: (vals[n], w) for n, w in want.items() if int(round(vals[n])) != w}
        for n, w in want.items(): print('  switch %-24s firmware %-6g wanted %-3d %s' % (n, vals[n], w, 'ok' if n not in bad else 'MISMATCH'))
        ok = ok and not bad
    print('PASS: 15 gains%s read back from the firmware equal the fragment' % (' and the switches of ' + a.strategy if getattr(a, 'strategy', None) else '') if ok else 'FAIL: do not fly this round; tell Claude'); return 0 if ok else 1


def cmd_set(a):
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    gains = load_fragment(a.fragment); cflib.crtp.init_drivers()
    with SyncCrazyflie(a.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        for k, v in gains.items(): scf.cf.param.set_value('%s.%s' % (GROUP, k), repr(float(v))); time.sleep(0.1)
        time.sleep(1.0)
    time.sleep(1.0); return cmd_readback(a, gains)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest='cmd', required=True)
    m = sub.add_parser('merge'); m.add_argument('--fragment', required=True); m.add_argument('--yaml', required=True); m.add_argument('--out'); m.add_argument('--strategy', choices=sorted(SWITCHES))
    for n in ('readback', 'set'):
        s = sub.add_parser(n); s.add_argument('--uri', required=True); s.add_argument('--fragment', required=True); s.add_argument('--strategy', choices=sorted(SWITCHES))
    a = ap.parse_args()
    sys.exit({'merge': lambda x: cmd_merge(x) or 0, 'readback': cmd_readback, 'set': cmd_set}[a.cmd](a))
