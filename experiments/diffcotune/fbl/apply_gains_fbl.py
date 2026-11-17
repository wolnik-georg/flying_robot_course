#!/usr/bin/env python3
"""Stage F2.5: the FBL counterpart of lab/apply_gains.py (which is hard-wired to the ctrlOmarIndi group and is not changed). merge / readback for the `ctrlFbl` group of round.py's FBL fragment
(lab/FBL<variant>/round_<l>/gains.yaml: ctrlFbl en 1, inner 1, cd 0.55, dw_en, nn_en and the 6 gains kp_xy kv_xy ki_xy kp_z kv_z ki_z; kr_xy / kr_z are not written).
  merge     python3 apply_gains_fbl.py merge --fragment <fragment> --yaml <crazyflies.yaml> [--out FILE]     same text-merge and 'nothing else changed' check as apply_gains.py, group ctrlFbl, cf5 block
            (the yaml base is the crazyswarm2 branch study_FBL_stock, which already has ctrlFbl en / inner / cd; dw_en and nn_en are inserted)
  readback  python3 apply_gains_fbl.py readback --uri radio://0/80/2M/E7E7E7BB02 --fragment <fragment>      fresh cflib connection (CS2 stopped): the fragment values, stabilizer.controller (9) from the firmware
No hardware is touched by `merge`; `readback` connects to the drone only when you run it (not run in Stage F2)."""
import argparse, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'lab'))
import apply_gains as AG  # noqa: E402

AG.GROUP = 'ctrlFbl'


def cmd_readback(a):
    gains = AG.load_fragment(a.fragment)
    names = ['ctrlFbl.%s' % k for k in gains] + ['stabilizer.controller']
    vals = AG.read_fresh(a.uri, names); rows = AG.compare(gains, vals)
    for k, v, x, ok in rows: print('  %-8s fragment %-14.8g firmware %-14s %s' % (k, v, x, 'ok' if ok else 'MISMATCH'))
    print('  stabilizer.controller      %s (wanted 9)' % vals['stabilizer.controller'])
    ok = all(r[3] for r in rows) and int(round(vals['stabilizer.controller'])) == 9
    print('PASS: the FBL values read back from the firmware equal the fragment' if ok else 'FAIL: do not fly this round; tell Claude')
    return 0 if ok else 1


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest='cmd', required=True)
    m = sub.add_parser('merge'); m.add_argument('--fragment', required=True); m.add_argument('--yaml', required=True); m.add_argument('--out'); m.add_argument('--strategy', default=None)
    r = sub.add_parser('readback'); r.add_argument('--uri', required=True); r.add_argument('--fragment', required=True)
    a = ap.parse_args()
    sys.exit({'merge': lambda x: AG.cmd_merge(x) or 0, 'readback': cmd_readback}[a.cmd](a))
