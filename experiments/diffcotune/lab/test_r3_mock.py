#!/usr/bin/env python3
"""Stage 8d T2 tests of rule R3 in round.py (fake uSD files from lab/dry5_S0_8o/round0_logs; epochs reduced to 2 and ONE worker so that it is a light single-process test; logic only, the numbers are not method results).
  python3 test_r3_mock.py            (a) A1 floor x3 + fallback -> round completes, 4 flights fitted, set not eligible, keep-best unchanged; A1 floor x3 without fallback -> clear message, nothing written as the next round;
                                         round 0 with 3 failed attempts -> no fallback needed, round completes, no eligible set
  python3 test_r3_mock.py --regress  (b) all flights usable (lenient, as the old dry test): round.py now against round.py.before_stage8d on the same folder: theta_1 and beta after the fit identical (diff 0.0)
Scratch folder: argument --work <dir> (default lab/results/r3_mock_work). nice -n 19, MemAvailable >= 6 GB."""
import importlib.util, json, os, shutil, sys, time
from pathlib import Path
os.environ.setdefault('DIFFCOTUNE_CONTROLLER', 'S0'); os.environ.setdefault('DIFFCOTUNE_METHOD', 'ours'); os.environ.setdefault('OMP_NUM_THREADS', '2'); os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
WORK = Path(sys.argv[sys.argv.index('--work') + 1]) if '--work' in sys.argv else None
sys.argv = [sys.argv[0], '--controller', 'S0', '--no-preview'] + [a for a in sys.argv[1:] if a in ('--regress', '--ext', '--rs', '--t3b')]
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun'))
import numpy as np
import round as RD  # noqa
SRC = HERE / 'dry5_S0_8o' / 'round0_logs'; WORK = WORK or HERE / 'results' / 'r3_mock_work'
Orig = RD.LP.PoolTop; RD.LP.PoolTop = lambda n=None, **kw: Orig(1, **kw)          # one worker: light
RD.SETTINGS['epochs'] = 2; RD.SETTINGS['n_rounds'] = 6
ok = True
def check(name, cond, info=''):
    global ok; ok &= bool(cond); print('PASS' if cond else 'FAIL', name, info, flush=True)
def folder(name, a1_attempts, fallback):
    f = WORK / name / 'logs'; shutil.rmtree(WORK / name, ignore_errors=True); f.mkdir(parents=True); man = []
    for s in ('A8', 'A4', 'CIRCLE', 'FIG8'): shutil.copy(SRC / ('%s_dry_merged_usd.csv' % s), f / ('%s_dry_merged_usd.csv' % s))
    src_man = {m['scenario']: m for m in json.load(open(SRC / 'flights.json'))}
    for s in ('A8', 'A4', 'CIRCLE', 'FIG8'): man.append(dict(src_man[s]))
    for a in range(1, a1_attempts + 1):
        shutil.copy(SRC / 'A1_dry_merged_usd.csv', f / ('A1_a%d.csv' % a)); man.append(dict(src_man['A1'], file='A1_a%d.csv' % a, attempt=a))
    if fallback: shutil.copy(SRC / 'A1_dry_merged_usd.csv', f / 'A1_fallback.csv'); man.append(dict(src_man['A1'], file='A1_fallback.csv', attempt=4, role='fallback'))
    json.dump(man, open(f / 'flights.json', 'w'), indent=1); return f, WORK / name / 'S0'
def folder_all(name, fallback):          # Stage 8e3: all 5 scenarios fail 3 attempts (the A1 file, a floor flight, as every scenario's file) + optional fallback flights for all 5
    f = WORK / name / 'logs'; shutil.rmtree(WORK / name, ignore_errors=True); f.mkdir(parents=True); man = []; src_man = {m['scenario']: m for m in json.load(open(SRC / 'flights.json'))}
    for s in ('A1', 'A8', 'A4', 'CIRCLE', 'FIG8'):
        for a in range(1, 4):
            fn = '%s_a%d.csv' % (s, a); shutil.copy(SRC / 'A1_dry_merged_usd.csv', f / fn); man.append(dict(src_man[s], file=fn, attempt=a))
        if fallback: fn = '%s_fb.csv' % s; shutil.copy(SRC / 'A1_dry_merged_usd.csv', f / fn); man.append(dict(src_man[s], file=fn, attempt=4, role='fallback'))
    json.dump(man, open(f / 'flights.json', 'w'), indent=1); return f, WORK / name / 'S0'
def state_round(sd, l, keep, ext=0):
    RD.init_state(sd); st = json.load(open(sd / 'round_0' / 'state.json'))
    if l > 0:
        (sd / ('round_%d' % l)).mkdir(exist_ok=True); st.update(round=l, keep=keep, ext=ext, history=[dict(round=0, rms3d_cm=10.0, phi=[0.0] * 10, psi_after_fit=st['psi'])]); json.dump(st, open(sd / ('round_%d' % l) / 'state.json', 'w'), indent=1)
        RD.gains_to_yaml(np.zeros(10), sd / ('round_%d' % l) / 'gains.yaml', 'mock')
        RD.gains_to_yaml(np.zeros(10), sd / 'round_0' / 'gains.yaml', 'mock')
def main():
    global ok
    KEEP0 = dict(cost=0.01, rms3d_cm=10.0, round=0, phi=[0.0] * 10)
    if '--regress' in sys.argv:
        import importlib.machinery; ld = importlib.machinery.SourceFileLoader('round_old', str(HERE / 'round.py.before_stage8d')); spec = importlib.util.spec_from_loader('round_old', ld); old = importlib.util.module_from_spec(spec); ld.exec_module(old)
        old.LP.PoolTop = RD.LP.PoolTop; old.SETTINGS['epochs'] = 2; old.SETTINGS['n_rounds'] = 6; res = {}
        for nm, mod in (('old', old), ('new', RD)):
            d = WORK / ('regress_' + nm); shutil.rmtree(d, ignore_errors=True); shutil.copytree(SRC, d / 'logs'); mod.init_state(d / 'S0'); t0 = time.time()
            res[nm] = mod.run_round(0, d / 'logs', d / 'S0', top='planned', strict=False); print(nm, 'round 0 done in %.0f s' % (time.time() - t0), flush=True)
        d1 = float(np.abs(np.array(res['old']['phi_next']) - np.array(res['new']['phi_next'])).max()); d2 = float(np.abs(np.array(res['old']['psi_after_fit']) - np.array(res['new']['psi_after_fit'])).max())
        check('b regression, all flights usable: theta_1 diff %.1e, beta diff %.1e, real cost equal' % (d1, d2), d1 == 0.0 and d2 == 0.0 and res['old']['real']['cost'] == res['new']['real']['cost'])
        check('b new fields only added (eligible True, no r3 block)', res['new']['eligible'] is True and 'r3' not in res['new'])
    elif '--t3b' in sys.argv:          # Stage 8e3 (T3b): all 5 scenarios R3
        starts = []; orig_adam = RD.L.adam_inner; fits = []
        def rec_adam(eval_fn, x0, idx, lr, **kw):
            if kw.get('label', '').startswith('tune'): starts.append(np.array(x0, float)[:10].copy())
            if kw.get('label', '').startswith('fit'): fits.append(1)
            return orig_adam(eval_fn, x0, idx, lr, **kw)
        RD.L.adam_inner = rec_adam
        KB = dict(cost=0.01, rms3d_cm=10.0, round=0, phi=[0.05] * 10)
        f, sd = folder_all('t3b_a', True); state_round(sd, 1, KB); st0 = json.load(open(sd / 'round_1' / 'state.json')); rep = RD.run_round(1, f, sd, top='planned', strict=True)
        nxt = json.load(open(sd / 'round_2' / 'state.json')) if (sd / 'round_2' / 'state.json').exists() else {}
        check('a all 5 R3 + fallbacks, previous round clean: round completes, no beta fit, tuned from theta_best, not eligible, next round written', len(rep['r3']['scenarios']) == 5 and not fits and len(starts) == 1 and np.allclose(starts[0], 0.05) and rep['eligible'] is False and rep['restart_from_best'] is True and rep['rs_stop'] is False and 'phi_next' in rep and np.allclose(rep['psi_after_fit'], st0['psi']) and np.allclose(nxt.get('psi', [9]), st0['psi']) and nxt.get('keep', {}).get('round') == 0 and nxt['history'][-1].get('eligible') is False, rep.get('status'))
        f, sd = folder_all('t3b_b', True); state_round(sd, 2, KB); st = json.load(open(sd / 'round_2' / 'state.json')); st['history'] = [dict(round=0, rms3d_cm=10.0, phi=[0.0] * 10, psi_after_fit=st['psi']), dict(round=1, rms3d_cm=float('nan'), phi=[0.0] * 10, psi_after_fit=st['psi'], eligible=False, r3=['A1', 'A8', 'A4', 'CIRCLE', 'FIG8'], restart_from_best=True)]; json.dump(st, open(sd / 'round_2' / 'state.json', 'w'), indent=1)
        starts.clear(); fits.clear(); rep = RD.run_round(2, f, sd, top='planned', strict=True)
        check('b all 5 R3 + previous round R3: "Tuning stopped: two R3 rounds in a row", gains_theta_best.yaml = theta_best, no tuning, no next round', 'Tuning stopped: two R3 rounds in a row' in rep['status'] and rep['rs_stop'] is True and (sd / 'round_2' / 'gains_theta_best.yaml').exists() and np.allclose(RD.yaml_to_phi(sd / 'round_2' / 'gains_theta_best.yaml'), 0.05, atol=1e-6) and not starts and not fits and not (sd / 'round_3').exists(), rep.get('status'))
        f, sd = folder_all('t3b_c', False); state_round(sd, 0, None); rep = RD.run_round(0, f, sd, top='planned', strict=True)
        check('c round 0, all 5 R3: stop message, nothing written as next round', 'theta_0 failed on every scenario: stop and ask the user (check drone and setup)' in rep['status'] and not (sd / 'round_1').exists() and 'phi_next' not in rep, rep.get('status'))
        f, sd = folder_all('t3b_d', False); state_round(sd, 1, KB); rep = RD.run_round(1, f, sd, top='planned', strict=True)
        check('d all 5 R3, fallbacks missing: R3 message (fly the fallbacks), nothing fitted, no next round', 'R3' in rep['status'] and 'fallback' in rep['status'] and not (sd / 'round_2').exists() and 'phi_next' not in rep, rep.get('status', '')[:120])
    elif '--rs' in sys.argv:          # Stage 8e: rule RS in round.py
        starts = []; orig_adam = RD.L.adam_inner
        def rec_adam(eval_fn, x0, idx, lr, **kw):
            if kw.get('label', '').startswith('tune'): starts.append(np.array(x0, float)[:10].copy())
            return orig_adam(eval_fn, x0, idx, lr, **kw)
        RD.L.adam_inner = rec_adam
        KB = dict(cost=0.01, rms3d_cm=10.0, round=0, phi=[0.05] * 10)          # theta_best = 0.05 in every gain, theta_l of the mock state = 0
        f, sd = folder('rs1', 3, True); state_round(sd, 1, KB); starts.clear(); rep = RD.run_round(1, f, sd, top='planned', strict=True)
        nxt = json.load(open(sd / 'round_2' / 'state.json'))
        check('r1 R3 + fallback with theta_best: tuning step starts from theta_best (0.05), not theta_1 (0)', len(starts) == 1 and np.allclose(starts[0], 0.05) and rep['restart_from_best'] is True and rep['rs_version'] == 'RS-2026-11-17' and nxt['history'][-1].get('restart_from_best') is True and 'started from theta_best' in (sd / 'round_1' / 'report.md').read_text())
        f, sd = folder('rs2', 3, True); state_round(sd, 2, KB); st = json.load(open(sd / 'round_2' / 'state.json')); st['history'] = [dict(round=0, rms3d_cm=10.0, phi=[0.0] * 10, psi_after_fit=st['psi']), dict(round=1, rms3d_cm=9.0, phi=[0.0] * 10, psi_after_fit=st['psi'], eligible=False, r3=['A1'], restart_from_best=True)]; json.dump(st, open(sd / 'round_2' / 'state.json', 'w'), indent=1)
        starts.clear(); rep = RD.run_round(2, f, sd, top='planned', strict=True)
        check('r2 two R3 rounds in a row: stop message, gains_theta_best.yaml = theta_best, no next round, no tuning step', 'Tuning stopped: two R3 rounds in a row' in rep['status'] and 'theta_best (round 0)' in rep['status'] and rep['last_round'] is True and rep['rs_stop'] is True and (sd / 'round_2' / 'gains_theta_best.yaml').exists() and not (sd / 'round_3').exists() and not starts and rep.get('phi_next') is None, rep.get('status'))
        check('r2b the final gains file holds theta_best (0.05 in every gain)', np.allclose(RD.yaml_to_phi(sd / 'round_2' / 'gains_theta_best.yaml'), 0.05, atol=1e-6))
        f, sd = folder('rs3', 3, False); state_round(sd, 0, None); starts.clear(); rep = RD.run_round(0, f, sd, top='planned', strict=True)
        check('r3 R3 in round 0 (no theta_best): no restart, tuning from theta_0', rep['restart_from_best'] is False and rep['rs_stop'] is False and len(starts) == 1 and np.allclose(starts[0], 0.0))
        f, sd = folder('rs4', 3, True); state_round(sd, 2, KB); st = json.load(open(sd / 'round_2' / 'state.json')); st['history'] = [dict(round=0, rms3d_cm=10.0, phi=[0.0] * 10, psi_after_fit=st['psi']), dict(round=1, rms3d_cm=9.0, phi=[0.0] * 10, psi_after_fit=st['psi'])]; json.dump(st, open(sd / 'round_2' / 'state.json', 'w'), indent=1)
        starts.clear(); rep = RD.run_round(2, f, sd, top='planned', strict=True)
        check('r4 R3 in round 2 after an eligible round 1: restart, no stop, next round written', rep['restart_from_best'] is True and rep['rs_stop'] is False and (sd / 'round_3').exists() and len(starts) == 1 and np.allclose(starts[0], 0.05))
        f, sd = folder('rs5', 1, False); state_round(sd, 1, KB); starts.clear(); rep = RD.run_round(1, f, sd, top='planned', strict=False)
        check('r5 no R3 (all flights usable): no restart, tuning from theta_l (0)', rep['eligible'] is True and rep['restart_from_best'] is False and len(starts) == 1 and np.allclose(starts[0], 0.0))
    elif '--ext' in sys.argv:          # Stage 8d2: extension rounds + fallback flight in session.py manifest
        import subprocess
        # (c) round 5 = last planned round, no eligible set so far, A1 failed 3x (round 5 not eligible): extension round 1 is added, next state written, no final gains file
        f, sd = folder('ext1', 3, False); state_round(sd, 5, None); rep = RD.run_round(5, f, sd, top='planned', strict=True)
        st6 = json.load(open(sd / 'round_6' / 'state.json')) if (sd / 'round_6' / 'state.json').exists() else {}
        check('c1 no eligible set after the last planned round: extension round 1 (state round 6 with ext 1, gains.yaml, no state_after, no final gains)', rep.get('extension_round_added') == 1 and rep['last_round'] is False and st6.get('ext') == 1 and (sd / 'round_6' / 'gains.yaml').exists() and not (sd / 'round_5' / 'state_after.json').exists() and not (sd / 'round_5' / 'gains_theta_best.yaml').exists())
        # (d) round 7 (extension 2 used): still no eligible set -> "ask the user", no final gains file, theta_0 not written
        f, sd = folder('ext3', 3, False); state_round(sd, 7, None, ext=2); rep = RD.run_round(7, f, sd, top='planned', strict=True)
        check('d no eligible set after extension: message "ask the user", NO_ELIGIBLE_SET.md, no gains_theta_best.yaml, no next round', 'no eligible set after extension: ask the user' in rep['status'] and (sd / 'round_7' / 'NO_ELIGIBLE_SET.md').exists() and not (sd / 'round_7' / 'gains_theta_best.yaml').exists() and not (sd / 'round_8').exists() and rep['last_round'] is True, rep.get('status'))
        # (e) round 6 (extension 1) is eligible: last round, gains_theta_best = this set, extension rounds used 1
        f, sd = folder('ext_ok', 1, False); state_round(sd, 6, None, ext=1); rep = RD.run_round(6, f, sd, top='planned', strict=False)          # lenient: the fake A1 file is a floor flight, lenient fits it, so all 5 flights are usable
        gb = sd / 'round_6' / 'gains_theta_best.yaml'
        check('e extension round eligible: it is the last round, gains_theta_best.yaml written, extension rounds used 1', rep['eligible'] is True and rep['last_round'] is True and gb.exists() and rep['extension_rounds_used'] == 1 and rep['keep_best']['round'] == 6)
        # (f) round 5 with an eligible keep-best set: no extension, final gains file
        f, sd = folder('noext', 3, True); state_round(sd, 5, dict(cost=0.01, rms3d_cm=10.0, round=2, phi=[0.0] * 10)); rep = RD.run_round(5, f, sd, top='planned', strict=True)
        check('f eligible keep-best exists: no extension, last round writes gains_theta_best.yaml', rep['last_round'] is True and 'extension_round_added' not in rep and rep['extension_rounds_used'] == 0 and (sd / 'round_5' / 'gains_theta_best.yaml').exists())
        # (g) the R3 stop message contains the fallback-failure rule
        f, sd = folder('msg', 3, False); state_round(sd, 1, KEEP0); rep = RD.run_round(1, f, sd, top='planned', strict=True)
        check('g R3 message: fallback failure rule + resume sentence', 'pause, check and fix the drone and setup (battery, props, mocap, EKF toggle)' in rep['status'] and 'resumes next session' in rep['status'] and 'session.py manifest' in rep['status'])
        # (h) session.py manifest writes the fallback entry (role fallback, attempt 4, gains = theta_best file), --append keeps the round's tuning entries
        w = WORK / 'sess'; shutil.rmtree(w, ignore_errors=True); (w / 'inbox').mkdir(parents=True)
        for n in ('A1', 'A8', 'FB'): shutil.copy(SRC / 'A1_dry_merged_usd.csv', w / 'inbox' / (n + '.csv'))
        base = [sys.executable, str(HERE / 'session.py'), 'manifest', '--controller', 'S0', '--round', '1', '--inbox', str(w / 'inbox'), '--out', str(w / 'flights.json'), '--root', str(w), '--fake-frame']
        r1 = subprocess.run(base + ['--assign', 'A1=A1.csv', 'A8=A8.csv'], capture_output=True, text=True)
        r2 = subprocess.run(base + ['--append', '--fallback', 'A1=FB.csv', '--fallback-gains', str(w / 'round_0_gains.yaml')], capture_output=True, text=True)
        man = json.load(open(w / 'flights.json')); fb_ = [m for m in man if m['role'] == 'fallback']
        check('h1 manifest --fallback --append: tuning entries kept, one fallback entry (A1, attempt 4, gains file)', r1.returncode == 0 and r2.returncode == 0 and len(man) == 3 and len(fb_) == 1 and fb_[0]['scenario'] == 'A1' and fb_[0]['attempt'] == 4 and fb_[0]['gains'].endswith('round_0_gains.yaml') and [m['role'] for m in man].count('tuning') == 2, (r1.stderr + r2.stderr)[-200:])
        r3 = subprocess.run(base + ['--append', '--fallback', 'A1=FB.csv', 'A1=FB.csv', '--fallback-gains', 'x.yaml'], capture_output=True, text=True)
        check('h2 two fallback flights for one scenario refused', r3.returncode != 0 and 'ONE fallback' in (r3.stderr + r3.stdout))
        r4 = subprocess.run(base + ['--fallback', 'A1=FB.csv'], capture_output=True, text=True)
        check('h3 --fallback without --fallback-gains refused', r4.returncode != 0 and 'fallback-gains' in (r4.stderr + r4.stdout))
        # (i) the round.py ingest reads the written manifest entry (role fallback is recognised like a tuning flight by cell_of)
        check('i cell_of accepts role fallback', RD.cell_of('A1', 'fallback', 0) == RD.cell_of('A1', 'tuning', 0))
    else:
        f, sd = folder('with_fallback', 3, True); state_round(sd, 1, KEEP0); rep = RD.run_round(1, f, sd, top='planned', strict=True)
        a1 = [x for x in rep['flights'] if x['scenario'] == 'A1' and x['role'] == 'tuning']
        check('a1 the A1 attempts are floor flights in the ingest (test premise)', len(a1) == 3 and all(x['floor'] for x in a1), [x['floor'] for x in a1])
        check('a2 A1 x3 + fallback: round completes, 4 flights fitted, set not eligible', rep.get('eligible') is False and len(rep['real']['tries']) == 4 and rep['r3']['scenarios'] == ['A1'] and 'phi_next' in rep and rep['r3']['theta_best_round'] == 0)
        check('a3 fallback flight not used', [x for x in rep['flights'] if x['role'] == 'fallback'][0]['used'] is False and rep['per_scenario_cm']['A1'] is None)
        nxt = json.load(open(sd / 'round_2' / 'state.json')); check('a4 keep-best of the next state unchanged (round 0), history marks not eligible', nxt['keep']['round'] == 0 and nxt['history'][-1].get('eligible') is False)
        f, sd = folder('no_fallback', 3, False); state_round(sd, 1, KEEP0); rep = RD.run_round(1, f, sd, top='planned', strict=True)
        check('a5 A1 x3 without fallback: clear message, nothing fitted, no next round', 'R3' in rep['status'] and "['A1']" in rep['status'] and 'round_0' in rep['status'] and not (sd / 'round_2').exists() and 'phi_next' not in rep, rep['status'][:200])
        f, sd = folder('round0', 3, False); state_round(sd, 0, None); rep = RD.run_round(0, f, sd, top='planned', strict=True)
        st1 = json.load(open(sd / 'round_1' / 'state.json'))
        check('a6 round 0 with 3 failed attempts: no fallback needed, round completes, no eligible set yet', rep['eligible'] is False and st1['keep'] is None and len(rep['real']['tries']) == 4)
        f, sd = folder('two_attempts', 2, False); state_round(sd, 1, KEEP0); rep = RD.run_round(1, f, sd, top='planned', strict=True)
        check('a7 only 2 failed attempts: old behaviour (repeat needed, nothing fitted)', 'repeat' in rep['status'] and not (sd / 'round_2').exists())

    print('ALL PASS' if ok else 'SOME FAILED'); return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
