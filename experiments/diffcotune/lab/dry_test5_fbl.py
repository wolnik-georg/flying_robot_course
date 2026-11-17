#!/usr/bin/env python3
"""Stage F2.6 (COPY of dry_test5.py for FBL: variant from --variant / DIFFCOTUNE_FBL_VARIANT, state folder lab/dry5_FBL_F<variant>, always strict-dry mode semantics when --strict-dry): Stage 5 B4: round-0 dry test of `round.py` for one controller (and the S0 regression check): 4 frsim flights of the dry-run plant (P1; S1 / S3: P1RD) with the default gains and flight seeds 100 + i
are written as fake merged-uSD files (lab/fake_usd.py) and `round.py --controller <C>` runs on them (strict = False as in Stage 4b: loop_lib fits a floor flight of theta_0 on A1 too, top path planned).
The result must equal the loop_lib dry run with the same seeds: theta_1 (phi after round 0) and beta after the fit of round 0 (max abs difference).
  python3 dry_test5.py --controller S2|S1|S3   reference = dryrun/stage5/results/loop_<C>.json (rounds[1].phi, rounds[0].psi_after_fit)
  python3 dry_test5.py --controller S0         reference = lab/results/dry_test.json round_0 (Stage 4b run of round.py, phi_next_loop_lib) and dryrun/stage3e/results/loop_P1.json
Output lab/results/dry_test5_<C>.json, files in lab/dry5_<C>/. Run with nice -n 19, not while another pool runs."""
import json, os, shutil, sys, time
from pathlib import Path
import numpy as np
if '--controller' in sys.argv: os.environ['DIFFCOTUNE_CONTROLLER'] = sys.argv[sys.argv.index('--controller') + 1]
if '--variant' in sys.argv: os.environ['DIFFCOTUNE_FBL_VARIANT'] = sys.argv[sys.argv.index('--variant') + 1]          # Stage F2
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun'))
import round as RD  # noqa  (first: it sets DIFFCOTUNE_GRAD_MODE from its per-controller table before loop_lib is imported; Stage 5c)
import loop_lib as L  # noqa
import fake_usd as F  # noqa
from frsim.scenarios import tuning_cells, study_cells  # noqa

C = L.CTRL; PLANT = L.PLANTS['P1RD' if L.N_BETA == 5 else 'P1']
STRICT_DRY = '--strict-dry' in sys.argv          # Stage 8d: the round test against a strict dry run (loop_lib DIFFCOTUNE_DRY_STRICT=1): flights are repeated like the lab does (floor / crash, attempt <= 3, seed + 1000 x tries) and round.py runs with strict = True; reference = loop_<C>_ours_strict.json; outputs dry_test5_<C>_8s.json / dry5_<C>_8s
TAG = os.environ.get('DIFFCOTUNE_DRY_TAG', '')          # Stage 5c: '_5c' = reference = dryrun/stage5c/results/loop_<C>.json (the rerun with the adopted gradient mode), outputs dry_test5_<C>_5c.json / dry5_<C>_5c
def old_ref(C):
    return 'stage3e/results/loop_P1.json' if C == 'S0' else ('stage5c_rdfix' if TAG == '_5cfix' else ('stage5c' if TAG == '_5c' else 'stage5')) + '/results/loop_%s.json' % C

VAR = os.environ.get('DIFFCOTUNE_FBL_VARIANT', ''); TAG = '_F' + VAR          # Stage F2: reference = dryrun/stage8/results/loop_FBL_FBL<variant>_ours_strict.json
REF = HERE.parent / 'dryrun' / ('stage8/results/loop_FBL_FBL%s_ours_strict.json' % VAR if TAG.startswith('_F') else ('stage8/results/loop_%s_ours_strict.json' % C if TAG == '_8s' else 'stage8/results/loop_%s_ours.json' % C if TAG == '_8o' else 'stage8/results/loop_%s_paper.json' % C if TAG == '_8' else ('stage7/results/loop_%s.json' % C if TAG == '_7' else old_ref(C))))


def main():
    DRY = HERE / ('dry5_%s%s' % (C, TAG)); shutil.rmtree(DRY, ignore_errors=True); DRY.mkdir(); RD.init_state(DRY / C)
    ref = json.load(open(REF)); gains = L.gains_of(np.zeros(L.NG)); logs = DRY / 'round0_logs'; logs.mkdir(); man = []; t0 = time.time()
    for i, (s, c) in enumerate(L.TUNING):          # Stage 7: A4 cell 1 is a tuning scenario (study cell 0 of A4)
        idx = [k for k, cc in enumerate(study_cells(s) if s == 'A4' else tuning_cells(s)) if cc == c][0]
        for att in range(1, 4 if STRICT_DRY else 2):
            tr, stable = F.fly_500hz(s, c, gains, PLANT, 100 + i + 1000 * (att - 1)); name = '%s_dry_merged_usd.csv' % s if att == 1 else '%s_dry_a%d_merged_usd.csv' % (s, att); F.write_merged(logs / name, tr, s, c)
            man.append(dict(file=name, scenario=s, cell=idx, role='tuning', attempt=att, rotate_deg=0, lag_s=0)); print('flew + wrote', name, 'attempt', att, 'stable', stable, flush=True)
            if not STRICT_DRY or (stable and not L.floor_failed(tr, L.R.build(s, c).scenario_s)): break          # the lab repeats a crashed or floor flight (attempt <= 3)
    json.dump(man, open(logs / 'flights.json', 'w'), indent=1); t_fly = time.time() - t0
    RD.SETTINGS['n_rounds'] = 6 if TAG in ('_8', '_8o', '_8s') or TAG.startswith('_F') else 5          # Stage 8: 6 rounds (decision 8); the earlier tags keep their 5
    rep = RD.run_round(0, logs, DRY / C, top='planned', strict=STRICT_DRY)
    phi_ref = np.array(ref['rounds'][1]['phi']); psi_ref = np.array(ref['rounds'][0]['psi_after_fit'])
    out = dict(controller=C, reference=str(REF.relative_to(HERE.parent)), real_cm=rep['real']['rms3d_cm'], real_cm_loop_lib=ref['rounds'][0]['real']['rms3d_cm'] if not STRICT_DRY else ref['rounds'][0].get('real_used', ref['rounds'][0]['real'])['rms3d_cm'], strict_dry=STRICT_DRY, r3=rep.get('r3'),
               phi_next_max_abs_diff=float(np.abs(np.array(rep['phi_next']) - phi_ref).max()), psi_max_abs_diff=float(np.abs(np.array(rep['psi_after_fit']) - psi_ref).max()),
               hits=rep['tune']['hits'], hits_loop_lib=ref['rounds'][0]['tune']['hits'], seconds_fly_write=t_fly, seconds_ingest=rep['seconds_ingest'], seconds_fit=rep['seconds_fit'], seconds_tune=rep['seconds_tune'], seconds_round=rep['seconds_total'],
               phi_next=rep['phi_next'], psi_after_fit=rep['psi_after_fit'], n_beta=L.N_BETA, yaml=str((DRY / C / 'round_1' / 'gains.yaml').relative_to(HERE)))
    ph = RD.yaml_to_phi(DRY / C / 'round_1' / 'gains.yaml'); out['yaml_roundtrip_max_abs_phi_diff'] = float(np.abs(ph - np.array(rep['phi_next'])).max())
    json.dump(out, open(HERE / 'results' / ('dry_test5_%s%s.json' % (C, TAG)), 'w'), indent=1); print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
