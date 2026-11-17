#!/usr/bin/env python3
"""Stage 8b: writes dryrun/stage8/LAB_TOMORROW.md from the result files (works with partial data: a missing file is written as "not yet"). Per controller: "ours" (frozen lab variant) and paper (tested, not adopted) side by side.
Usage: ~/.pyenv/versions/flying_robots/bin/python make_lab_tomorrow.py  (or /usr/bin/python3; numpy only)"""
import json, os, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent; R = HERE / 'results'; LAB = HERE.parents[1] / 'lab'
J = lambda p: json.load(open(p)) if Path(p).exists() else None
CTRLS = ['S0', 'S2', 'S1', 'S3']


def analyse(C, tag):
    rec = J(R / ('loop_%s_%s.json' % (C, tag))); ev = J(R / ('eval_%s_%s.json' % (C, tag)))
    if rec is None: return None
    rd = rec['rounds']; n = len(rd); out = dict(n=n, ext=rec.get('extension_rounds_used', 0), complete=(n == 6 + rec.get('extension_rounds_used', 0) and rec.get('halt') is None and ev is not None))          # Stage 8d2: extension rounds (strict mode, up to 2) make the loop 7 or 8 rounds
    out['real'] = [r['real']['rms3d_cm'] for r in rd]
    out['crash'] = [sum(bool(f['crashed']) for f in r['flights']) for r in rd]; out['floor'] = [sum(bool(f['floor']) for f in r['flights']) for r in rd]
    out['floor_scen'] = [[f['scenario'] for f in r['flights'] if f['floor']] for r in rd]
    out['minutes'] = [r['seconds_round'] / 60 for r in rd]
    out['jump'] = rec.get('max_gain_change_per_round')
    out['halt'] = rec.get('halt'); out['best_round'] = rec['best']['round'] if rec.get('best') else None
    r_ = out['real']
    out['a'] = (all(r_[l] <= r_[1] + 1.0 for l in range(2, n)) if n >= 5 else None)
    fl = [{f['scenario']: bool(f['crashed'] or f['floor']) for f in r['flights']} for r in rd]
    out['strict'] = bool(rec.get('strict'))
    if out['strict']:          # Stage 8d: strict dry run (lab flight rule + R3): a scenario counts as failed in round l if no usable attempt exists; real cost as flown (last attempt) stays 'real'
        fl = [{f['scenario']: bool(f['failed_all']) for f in r['flights']} for r in rd]
        out['real_used'] = [r['real_used']['rms3d_cm'] if r.get('real_used') else None for r in rd]; out['eligible'] = [bool(r.get('eligible')) for r in rd]; out['r3'] = [r.get('r3_scenarios') or [] for r in rd]
        out['attempts'] = [sum(len(f['attempts']) for f in r['flights']) for r in rd]; out['fallbacks'] = [sum(1 for f in r['flights'] if f.get('fallback')) for r in rd]
        out['best_eligible'] = (rec['best'] or {}).get('eligible', True); out['best_note'] = (rec['best'] or {}).get('note')
    out['b'] = (not [1 for l in range(1, n) for s in fl[l] if fl[l][s] and not (fl[0][s] and fl[1][s])]) if n >= 2 else None
    if ev:
        e0, eb = ev['eval']['theta_0'], ev['eval']['theta_best']
        m = lambda e_, f: float(np.mean([x['rms3d_cm'] for x in e_ if f(x)]))
        tun = lambda x: x['role'] == 'tuning'; held = lambda x: x['role'] == 'test' and x['scenario'] == 'A4'; ins = lambda x: x['role'] == 'tuning' and x['scenario'] == 'A4'
        out.update(tun0=m(e0, tun), tunb=m(eb, tun), held0=m(e0, held), heldb=m(eb, held), ins0=m(e0, ins), insb=m(eb, ins))
        out['c'] = bool(out['tunb'] < out['tun0'] and out['heldb'] < out['held0'])
        out['crash_eval'] = (sum(bool(x['crashed']) for x in eb), len(eb))
    else:
        out['c'] = None
    pv = J(R / ('preview_%s_%s.json' % (C, tag)))
    out['preview'] = pv
    return out


def yn(x): return 'not yet' if x is None else ('yes' if x else 'no')
def row_real(a): return ' → '.join('%.2f' % v for v in a['real']) + ('' if a['complete'] else '  (%d of 6 rounds%s)' % (a['n'], ', halted: ' + str(a['halt']) if a['halt'] else ''))


def main():
    L = ['# LAB_TOMORROW (Stage 8b, written %s from the result files; the user picks and commits)' % time.strftime('%d %b %Y %H:%M'), '',
         'Lab variant (decided by the user, 15 Nov evening): **"ours", frozen, the same for S0–S3** = DiffCoTune Split-Alternate loop (Krishna et al. 2025) with DiffTune\'s quadrotor settings (Cheng et al. 2024); 5 tuning scenarios (A1, A8, A4 cell 1, CIRCLE, FIG8), test A4 cell 2 (held out) and A4 cell 1 (in-sample), 6 rounds, keep-best, position cost, J_sysId position + velocity + attitude, 50 + 50 epochs best iterate, lr 0.01, ±25 % per round, noise 2 seeds, forward gradient, RPM delay fixed (S1 / S3). The paper variant is shown for comparison only (not adopted).', '']
    for C in CTRLS:
        o = analyse(C, 'ours'); p = analyse(C, 'paper'); L += ['## %s' % C, '']
        L += ['| | "ours" (lab variant) | paper (not adopted) |', '|---|---|---|']
        g = lambda a, f, d='not yet': d if a is None else f(a)
        L.append('| real 3-D cost per round (cm, mean of 5 flights) | %s | %s |' % (g(o, row_real), g(p, row_real)))
        L.append('| criteria a / b / c | %s | %s |' % (g(o, lambda a: '%s / %s / %s' % (yn(a['a']), yn(a['b']), yn(a['c']))), g(p, lambda a: '%s / %s / %s' % (yn(a['a']), yn(a['b']), yn(a['c'])))))
        L.append('| crashes per round | %s | %s |' % (g(o, lambda a: ' '.join(map(str, a['crash']))), g(p, lambda a: ' '.join(map(str, a['crash'])))))
        L.append('| floor flights per round (A1 is a floor flight at the default gains in the dry-run plant) | %s | %s |' % (g(o, lambda a: ' '.join(map(str, a['floor']))), g(p, lambda a: ' '.join(map(str, a['floor'])))))
        L.append('| largest gain change to the next flown set | %s | %s |' % (g(o, lambda a: ', '.join('%.0f %%' % (100 * x) for x in (a['jump'] or []))), g(p, lambda a: ', '.join('%.0f %%' % (100 * x) for x in (a['jump'] or [])))))
        fm = lambda a: ('tuning %.2f → %.2f; held-out A4 cell 2 %.2f → %.2f; in-sample A4 cell 1 %.2f → %.2f (θ_0 → θ_best, round %s)' % (a['tun0'], a['tunb'], a['held0'], a['heldb'], a['ins0'], a['insb'], a['best_round'])) if 'tun0' in a else 'not yet'
        L.append('| means (cm, evaluation seeds 2000–2004) | %s | %s |' % (g(o, fm), g(p, fm)))
        L.append('| desk minutes per round (round 0 … 4) | %s | %s |' % (g(o, lambda a: ' '.join('%.0f' % x for x in a['minutes'][:5])), g(p, lambda a: ' '.join('%.0f' % x for x in a['minutes'][:5]))))
        rt = J(LAB / 'results' / ('dry_test5_%s_8o.json' % C)); gt = J(R / ('grad_gate9_%s_ours.json' % C))
        L.append('| round test (round.py round 0 against the dry run) | %s | |' % ('not yet' if rt is None else 'θ₁ difference %.1e, β difference %.1e, %.1f min' % (rt['phi_next_max_abs_diff'], rt['psi_max_abs_diff'], rt['seconds_round'] / 60)))
        L.append('| gradient check at the fitted β of every round | %s | |' % ('not yet' if gt is None else ('passed' if gt.get('passed') else 'see %s (sections 1–2 are judged on the used components in the fixed version)' % ('results/grad_gate9_%s_ours.json' % C))))
        st_ = analyse(C, 'ours_strict')
        if st_ is not None:
            L.append('| **strict dry run (lab flight rule + R3), tag ours_strict** | |')
            L.append('| real cost as flown per round (cm, last attempt of each scenario) | %s |' % row_real(st_))
            L.append('| real cost over the used flights per round (cm) | %s |' % ' → '.join('n/a' if v is None else '%.2f' % v for v in st_['real_used']))
            L.append('| attempts per round (all scenarios) / R3 cases (scenario) / fallback flights | %s / %s / %s |' % (' '.join(map(str, st_['attempts'])), ' '.join(','.join(x) if x else '-' for x in st_['r3']), ' '.join(map(str, st_['fallbacks']))))
            L.append('| eligible for keep-best per round | %s ; keep-best round %s%s |' % (' '.join('yes' if e else 'no' for e in st_['eligible']), st_['best_round'], '' if st_['best_eligible'] else ' (NO ELIGIBLE SET: default gains)'))
            L.append('| criteria a / b / c | %s / %s / %s |' % (yn(st_['a']), yn(st_['b']), yn(st_['c'])))
            L.append('| means (cm) | %s |' % ('not yet' if 'tun0' not in st_ else 'tuning %.2f → %.2f; held-out A4 cell 2 %.2f → %.2f; in-sample A4 cell 1 %.2f → %.2f' % (st_['tun0'], st_['tunb'], st_['held0'], st_['heldb'], st_['ins0'], st_['insb'])))
            L.append('| desk minutes per round (round 0 … 4) | %s |' % ' '.join('%.0f' % x for x in st_['minutes'][:5]))
            rts = J(LAB / 'results' / ('dry_test5_%s_8s.json' % C)); gts = J(R / ('grad_gate9_%s_ours_strict.json' % C))
            L.append('| round test (strict) | %s |' % ('not yet' if rts is None else 'θ₁ difference %.1e, β difference %.1e' % (rts['phi_next_max_abs_diff'], rts['psi_max_abs_diff'])))
            L.append('| gradient check (strict run) | %s |' % ('not yet' if gts is None else ('passed' if gts.get('passed') or gts.get('passed_posthoc') else 'see results/grad_gate9_%s_ours_strict.json' % C)))
        pv = o['preview'] if o else None
        if pv: L.append('| safety preview of each round\'s θ (replay) | %s | |' % '; '.join('round %d %s (real %.1f cm)' % (x['round'], 'PASSED' if x['preview']['passed'] else 'FAILED', x['real_cm']) for x in pv))
        verdict = 'not yet' if (o is None or not o['complete']) else ('yes' if (o['a'] and o['b'] and o['c']) else 'no')
        L += ['', '**%s lab-ready ("ours"): %s**%s' % (C, verdict, '' if verdict != 'no' else ' (criteria a–c not all met)'), '']
    L += ['## Session 1 (S0 + S2 interleaved)', '', 'Cards: `lab/INTERLEAVED_LAB_CARD.md` (+ `lab/S0_LAB_CARD.md` items per flight). Command per round: `python3 lab/round.py --controller <C> --round <l> --logs <folder> --method ours` (default method is ours), then read `lab/<C>/round_<l+1>/PREVIEW.txt`; on `PREVIEW FAILED` do not fly the set without asking; default: fly the best flown set again. Preflight: `python3 lab/preflight.py --controller <C> --yaml <lab crazyflies.yaml>` must print PREFLIGHT PASSED for S0 and S2 (results: see `logs/pre_S0.log`, `logs/pre_S2.log`). Estimated hours (Stage 6 timeline model, now 6 rounds × 5 flights): see `lab/results/session_timeline.md` (not yet rerun with 6 rounds if missing). uSD cards: empty both before, archive once after round 2 of both controllers, never ≥ 85 files; check on the first flight that it is one file per flight.', '',
          '## Open items (who decides)', '', '* Which PC runs `round.py` in the lab (user); whether two desk steps at once are acceptable there (the desk PC runs out of memory with two "ours" pools: a pool worker grows to about 2.6 GB over two hours; one "ours" desk step at a time is safe).', '* Flights per battery pack, uSD files per flight and card (assumed 1), the strategy-switch overhead (measured in the first block): not stated.', '* The preview rule (a check that never changes the gains): the abort rules are the protocol\'s; the "NOTE worse" thresholds (1.5 × and 2 cm) are a proposal. Replay results: `dryrun/stage8/report_stage8.md` §6.', '* Supervisor agreement: the user.']
    (HERE / 'LAB_TOMORROW.md').write_text('\n'.join(L) + '\n'); print('\n'.join(L))


if __name__ == '__main__':
    main()
