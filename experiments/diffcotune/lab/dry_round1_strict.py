#!/usr/bin/env python3
"""Stage 4b: round 1 of the dry test once more with strict=True (a crashed or floor flight is not used). Needs lab/dry/ from dry_test.py (round-1 flights and the round-1 state).
Works on a copy (lab/dry_strict/); output lab/results/dry_round1_strict.json: is the result the same as with strict=False?"""
import json, shutil, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun'))
import round as RD  # noqa
import dry_test as DT  # noqa

if __name__ == '__main__':
    DRY = HERE / 'dry'; SD = HERE / 'dry_strict'; shutil.rmtree(SD, ignore_errors=True); (SD / 'S0' / 'round_1').mkdir(parents=True)
    shutil.copy(DRY / 'S0' / 'round_1' / 'state.json', SD / 'S0' / 'round_1' / 'state.json')
    rep = RD.run_round(1, DRY / 'round1_logs', SD / 'S0', top='planned', strict=True)
    ref = json.load(open(DT.REF)); c = DT.compare(rep, 1, ref)
    nonstrict = json.load(open(HERE / 'results' / 'dry_test.json'))['round_1']   # written by dry_test.py --from-round 1 (run before, never at the same time)
    c['same_as_strict_false_phi_next'] = float(np.abs(np.array(c['phi_next']) - np.array(nonstrict['phi_next'])).max()); c['same_as_strict_false_psi'] = None
    json.dump(c, open(HERE / 'results' / 'dry_round1_strict.json', 'w'), indent=1); print(json.dumps({k: v for k, v in c.items() if not k.startswith('phi_next')}, indent=1))
