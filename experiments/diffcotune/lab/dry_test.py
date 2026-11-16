#!/usr/bin/env python3
"""Stage 4a / 4b C2 dry test, rounds 0 and 1, start to end (no --reuse): frsim flights of the P1 plant are written as fake merged-uSD files and `round.py` runs on them.
Round 0: default gains, flight seeds 100 + i. Round 1: theta_1 from the round-0 state file, flight seeds 110 + i (as in Stage 3e B1). The results must equal dryrun/stage3e/results/loop_P1.json:
round 0 -> phi of round 1 and psi_after_fit of round 0; round 1 -> phi of round 2 and psi_after_fit of round 1 (gates of Stage 4b: max difference <= 1e-12, real cost 9.71 cm +- 0.01).
strict=False (as in Stage 3e): loop_lib fits a floor flight of theta_0 on A1 too; round 1 has none. `dry_round1_strict.py` repeats round 1 with strict=True.
Output: lab/results/dry_test.json (+ lab/dry/). Run with nice -n 19, not while another pool runs."""
import json, shutil, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun'))
import loop_lib as L  # noqa
import fake_usd as F  # noqa
import round as RD  # noqa

REF = HERE.parent / 'dryrun' / 'stage3e' / 'results' / 'loop_P1.json'


def fly_round(DRY, l, phi):
    """Fly the 4 tuning scenarios with the gains phi (flight seeds 100 + 10 l + i) and write them as fake merged-uSD files with a manifest. Returns (folder, seconds)."""
    logs = DRY / ('round%d_logs' % l); logs.mkdir(parents=True)
    t0 = time.time(); gains = L.gains_of(np.asarray(phi)); man = []
    for i, (s, c) in enumerate(L.TUNING):
        tr, stable = F.fly_500hz(s, c, gains, L.P1, 100 + 10 * l + i)
        name = '%s_dry_merged_usd.csv' % s; F.write_merged(logs / name, tr, s, c)
        tun = __import__('frsim.scenarios', fromlist=['x']).tuning_cells(s); idx = [k for k, cc in enumerate(tun) if cc == c][0]
        man.append(dict(file=name, scenario=s, cell=idx, role='tuning', attempt=1, rotate_deg=0, lag_s=0)); print('round %d flew + wrote' % l, name, 'stable', stable, flush=True)
    json.dump(man, open(logs / 'flights.json', 'w'), indent=1)
    return logs, time.time() - t0


def compare(rep, l, ref):
    phi_ref = np.array(ref['rounds'][l + 1]['phi']); psi_ref = np.array(ref['rounds'][l]['psi_after_fit'])
    phi = np.array(rep['phi_next']); psi = np.array(rep['psi_after_fit'])
    return dict(real_cm=rep['real']['rms3d_cm'], real_cm_loop_lib=ref['rounds'][l]['real']['rms3d_cm'], phi_next_max_abs_diff=float(np.abs(phi - phi_ref).max()),
                psi_max_abs_diff=float(np.abs(psi - psi_ref).max()), phi_next=phi.tolist(), phi_next_loop_lib=phi_ref.tolist(), keep_best=rep['keep_best'],
                hits=rep['tune']['hits'], hits_loop_lib=ref['rounds'][l]['tune']['hits'],
                seconds_ingest=rep['seconds_ingest'], seconds_fit=rep.get('seconds_fit'), seconds_tune=rep['seconds_tune'], seconds_round=rep['seconds_total'])


if __name__ == '__main__':
    # Stage 4b: --from-round 1 keeps lab/dry/ (round 0 as written by the first run, round-1 flights and round_1/state.json) and runs only round 1.
    first = 1 if '--from-round' in sys.argv and sys.argv[sys.argv.index('--from-round') + 1] == '1' else 0
    DRY = HERE / 'dry'; ref = json.load(open(REF)); out = {}
    if first == 0:
        shutil.rmtree(DRY, ignore_errors=True); DRY.mkdir(); RD.init_state(DRY / 'S0')
    else:
        rep0 = json.load(open(DRY / 'S0' / 'round_0' / 'report.json'))
        out['round_0'] = dict(compare(rep0, 0, ref), note='from lab/dry/S0/round_0/report.json (round 0 run 14:29-14:50, not redone)')
        shutil.rmtree(DRY / 'S0' / 'round_2', ignore_errors=True)
    for l in range(first, 2):
        phi = json.load(open(DRY / 'S0' / ('round_%d' % l) / 'state.json'))['phi']
        if first == 0 or l > first:
            logs, t_fly = fly_round(DRY, l, phi)
        else:
            logs, t_fly = DRY / ('round%d_logs' % l), None     # existing flights
        rep = RD.run_round(l, logs, DRY / 'S0', top='planned', strict=False)
        out['round_%d' % l] = dict(compare(rep, l, ref), seconds_fly_write=t_fly)
    rt = {}
    for f in sorted((DRY / 'S0').glob('round_*/gains*.yaml')):
        ph = RD.yaml_to_phi(f); src = np.array(json.load(open(f.parent / 'state.json'))['phi']) if f.name == 'gains.yaml' else None
        rt[str(f.relative_to(DRY))] = dict(max_abs_phi=float(np.abs(ph).max()), vs_state=None if src is None else float(np.abs(ph - src).max()))
    out['yaml_roundtrip'] = rt
    g64 = np.array(list(L.gains_of(np.array(ref['rounds'][1]['phi'])).values())); out['float32_rel_error_of_gains'] = float(np.abs(g64.astype(np.float32).astype(np.float64) / g64 - 1).max())
    out['state_round_2_keep'] = json.load(open(DRY / 'S0' / 'round_2' / 'state.json'))['keep']
    json.dump(out, open(HERE / 'results' / 'dry_test.json', 'w'), indent=1); print(json.dumps({k: v for k, v in out.items() if not k.startswith('phi')}, indent=1)[:3000])
