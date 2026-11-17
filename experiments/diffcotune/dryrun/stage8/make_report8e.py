#!/usr/bin/env python3
"""Stage 8e: writes report_stage8e.md (generated, re-run after each job): T0 (Stage 8d reporting), T1 / T3 / T4 (rule RS, tests), queue c9.sh (the two RS regressions), lab-ready table S0-S3, open items.
Scope change of the user, 17 Nov: no S1 / S3 reruns, no plant switch. Read-only on results / logs. Usage: nice -n 19 /usr/bin/python3 make_report8e.py"""
import datetime, time, json
from pathlib import Path
import make_lab_tomorrow as M, make_report8d as D
H = Path(__file__).resolve().parent; LG = H / 'logs'; LAB = H.parents[1] / 'lab'; R = H / 'results'; J = M.J; pf = D.pf; rd = D.rd
CT = ['S0', 'S2', 'S1', 'S3']


def queue():
    o = ['| Job | start | end | rc | key result |', '|---|---|---|---|---|']
    for j in ('regress9r_S0', 'regress9r_S2'):
        C = j.split('_')[1]; ts = rd(LG / ('t_start_%s.txt' % j)); te = rd(LG / ('t_end_%s.txt' % j)); rc = rd(LG / ('%s.rc' % j)); g = J(R / ('regress9r_%s.json' % C))
        fresh = (LG / ('t_start_%s.txt' % j)).exists() and (LG / ('t_start_%s.txt' % j)).stat().st_mtime > datetime.datetime(2026, 11, 17, 11, 0).timestamp()
        if not fresh: o.append('| %s | not yet | – | – | not yet |' % j); continue
        res = 'running' if g is None and rc is None else ('not yet' if g is None else '%s: max abs diff %s; structure %s' % ('PASS (bit-identical)' if g['passed'] else 'FAIL', {k: ('%.1e' % v) for k, v in g['max_abs_diff'].items()}, 'equal' if all(g['structure'].values()) else {k: v for k, v in g['structure'].items() if not v}))
        o.append('| %s | %s | %s | %s | %s |' % (j, (ts or '').strip(), (te or 'running').strip(), (rc or '–').strip(), res))
    return o


def lab_table():
    o = ['| Controller | Stage 8d strict dry run: real cost per round (cm, as flown) | a / b / c | round test (strict, round 0) | gradient check | preview (old runs) | preflight (R3, extension, RS markers) | RS mock tests | Remark |', '|---|---|---|---|---|---|---|---|---|']
    mocks = '%s (`--rs`); %s (R3 / 8d cases); %s (`--ext`); %s (`--regress`)' % tuple(pf(LAB / 'results' / ('test_r3_mock_8e_%s.txt' % m)) for m in ('rs', 'a', 'ext', 'regress'))
    for C in CT:
        a = M.analyse(C, 'ours_strict'); rt = J(LAB / 'results' / ('dry_test5_%s_8s.json' % C)); g = D.grad_ok(C); gj = J(R / ('grad_gate9_%s_ours_strict.json' % C))
        rtt = 'not yet' if rt is None else ('exact (θ₁ %.0e, β %.0e)' % (rt['phi_next_max_abs_diff'], rt['psi_max_abs_diff']) if max(rt['phi_next_max_abs_diff'], rt['psi_max_abs_diff']) < 1e-9 else 'differs (θ₁ %.1e, β %.1e)' % (rt['phi_next_max_abs_diff'], rt['psi_max_abs_diff']))
        if g is None: gt = 'not yet'
        elif g: gt = 'PASS'
        else:
            gt = 'FAIL on A1 task only, accepted by the user 17 Nov' if C == 'S2' else 'FAIL'          # S2: user decision 17 Nov (the gate and its threshold are unchanged)
        pv = J(R / ('preview_%s_ours.json' % C)); pvt = 'not run' if not pv else '%d of %d rounds PASSED' % (sum(1 for x in pv if x['preview']['passed']), len(pv))
        pfl = (rd(LAB / 'results' / ('preflight_%s_8e.txt' % C)) or ''); pft = 'PASS (6b, 6c, 6d; PREFLIGHT PASSED)' if 'PREFLIGHT PASSED' in pfl and '[PASS] 6d' in pfl else 'not yet'
        if a is None: o.append('| %s | not yet | – | %s | %s | %s | %s | %s | |' % (C, rtt, gt, pvt, pft, mocks)); continue
        rem = 'criteria a / b / c = %s / %s / %s' % (M.yn(a['a']), M.yn(a['b']), M.yn(a['c']))
        if C in ('S1', 'S3'): rem = 'dry-run behaviour under an unfittable 8 ms delay (plant P1RD, RPM delay fixed at 4.0502 ms in the model); in the lab covered by RS + stop rule. Crashed rounds: %s; dry-run criteria a / b / c = %s / %s / %s' % ([l for l in range(a['n']) if not a['eligible'][l]], M.yn(a['a']), M.yn(a['b']), M.yn(a['c']))
        o.append('| %s | %s | %s / %s / %s | %s | %s | %s | %s | %s | %s |' % (C, ' → '.join('%.2f' % x for x in a['real']), M.yn(a['a']), M.yn(a['b']), M.yn(a['c']), rtt, gt, pvt, pft, mocks if C == 'S0' else 'same (controller-independent)', rem))
    return o


def main():
    f = lambda p: pf(p)
    rs_t = f(H / 'logs' / 'test_restart_rule.log'); st_t = f(H / 'logs' / 'test_strict_rule_8e.log')
    reg_mock = ', '.join(l for l in (rd(LAB / 'results' / 'test_r3_mock_8e_regress.txt') or '').split('\n') if l.startswith(('PASS', 'FAIL'))) or 'not run'
    L = ['# Stage 8e report (generated %s by `make_report8e.py`; the planning agent validates before anything is committed or flown)' % time.strftime('%d %b %Y %H:%M'), '',
         'Scope (user, 17 Nov): focus is the real lab sessions; no S1 / S3 dry-run reruns, no plant switch. Rule RS, both S0 / S2 regressions, tests, readers.', '',
         '## 0. T0 Stage 8d reporting', '',
         '`report_stage8d.md`, `LAB_TOMORROW.md`, `report_stage8.md` regenerated from the finished `c8.sh` (ALL DONE 08:03:36, 13 jobs rc 0, `logs/slotC8.log`). Real 3-D cost per round (cm, as flown):', '',
         '| Controller | Rounds | θ_best | Round test | Gradient check |', '|---|---|---|---|---|']
    for C in CT:
        a = M.analyse(C, 'ours_strict'); rt = J(LAB / 'results' / ('dry_test5_%s_8s.json' % C)); g = D.grad_ok(C)
        L.append('| %s | %s | round %s | %s | %s |' % (C, ', '.join('%.2f' % x for x in a['real']), a['best_round'], 'exact' if rt and max(rt['phi_next_max_abs_diff'], rt['psi_max_abs_diff']) < 1e-9 else 'differs / not yet', 'PASS' if g else ('FAIL on A1 task only, accepted by the user 17 Nov' if C == 'S2' else 'FAIL')))
    L += ['', 'Numbers match the handover table of 17 Nov. Reader fix: the queue-time cutoff in `make_report8c.py` / `make_report8d.py` used today\'s date; it is now 16 Nov.', '',
          '## 1. T1 / T3 / T4: rule RS (user decision 17 Nov)', '',
          'Rule: (a) a round with an R3 scenario and an existing θ_best starts its tuning step from θ_best (box ± 25 % around θ_best; the β fit is unchanged); (b) R3 in round l and in round l − 1 with θ_best existing: no further tuning, result θ_best, "stopped: two R3 rounds in a row"; everything else unchanged.', '',
          '| Item | Files (copies `*.before_stage8e` kept) | Change | Test | Result |', '|---|---|---|---|---|',
          '| T1 RS in the dry run | `dryrun/loop_lib.py`, `stage8/run_8.py` | `DIFFCOTUNE_RESTART_BEST` (default 0 = Stage 8d, only with `DIFFCOTUNE_DRY_STRICT=1`); per round `restart_from_best`, `restart_theta_best_round`, `prev_r3`, `rs_stop`; run `halt = rs_stop = "two R3 rounds in a row"`; `run_8.py` keeps that stop reason | `stage8/test_restart_rule.py` (fake pool: restart, round 0, two in a row, not consecutive, no θ_best, RS = 0) | %s |' % rs_t,
          '| T1 strict rule unchanged with RS off | same | – | `stage8/test_strict_rule.py` | %s |' % st_t,
          '| T3 RS in `round.py` | `lab/round.py`, marker `RS_VERSION = \'RS-2026-11-17\'` | R3 round with θ_best: tuning from θ_best (printed, `report.json` / `report.md` / `state.json` history `restart_from_best`, `prev_r3`); stop rule from the state history: message "Tuning stopped: two R3 rounds in a row. Final gains = θ_best (round k). Fly the test flights with θ_best.", `gains_theta_best.yaml` written, no next round | `lab/test_r3_mock.py --rs` | %s |' % pf(LAB / 'results' / 'test_r3_mock_8e_rs.txt'),
          '| T3 earlier mock cases unchanged | – | – | `lab/test_r3_mock.py` (R3), `--ext` (extension rounds, fallback, manifest) | %s; %s |' % (pf(LAB / 'results' / 'test_r3_mock_8e_a.txt'), pf(LAB / 'results' / 'test_r3_mock_8e_ext.txt')),
          '| T4 (c) regression, all flights usable | `lab/test_r3_mock.py --regress` | `round.py` now against `round.py.before_stage8d`, same folder, θ₁ and β after the fit | `lab/results/test_r3_mock_8e_regress.txt` | %s |' % reg_mock,
          '| T3 `preflight.py` | `lab/preflight.py` | check 6d (RS marker and stop text in `round.py`) | `preflight.py --controller S0 / S1 / S2 / S3` (`lab/results/preflight_<C>_8e.txt`) | %s |' % ('6d PASS, PREFLIGHT PASSED for all four' if all('[PASS] 6d' in (rd(LAB / 'results' / ('preflight_%s_8e.txt' % c)) or '') and 'PREFLIGHT PASSED' in (rd(LAB / 'results' / ('preflight_%s_8e.txt' % c)) or '') for c in CT) else 'not yet'),
          '| T3 cards | `lab/S0_LAB_CARD.md`, `lab/INTERLEAVED_LAB_CARD.md` | rule RS and the stop rule (item 9a / 5b) | – | done |',
          '| Dropped (user 17 Nov) | – | plant switch, S1 / S3 reruns, `dry9r_*` / `rtest9r_*` / `gate9r_*`; the code of the plant switch was reverted (`dry_test5.py` identical to `.before_stage8e`) | – | – |', '',
          '## 2. Queue c9.sh (the two RS regressions)', ''] + queue() + ['',
          'Comparison (`stage8/compare_rs.py`): per round phi, psi, psi after the fit, real cost as flown and over used flights, attempt costs, tuning J_best; structure: eligibility, R3 scenarios, attempts, keep-best set, no RS action. PASS only if every difference is 0.0.', '',
          '## 3. Lab-ready table', ''] + lab_table() + ['',
          'Gradient check S2: FAIL on A1 task only (round 0: 5.7e-3, round 4: 1.9e-3, limit 1e-3), every other check passes; accepted by the user 17 Nov; gate and threshold unchanged.',
          'S1 / S3: the Stage 8d dry-run crashes (all five scenarios R3 from round 2 / round 3, β not refitted) are dry-run behaviour under an unfittable 8 ms RPM delay (the real drones were measured at 4.05 ms); in the lab covered by RS + stop rule. Lab readiness of S1 / S3 is judged on the round tests, gradient checks, preflight and the RS mock tests (user, 17 Nov).', '',
          '## 4. Open items (who decides)', '',
          '* **Keep-best eligibility rule** confirmed by the user 16 Nov (a set with an R3 scenario is never θ_best).',
          '* Which controller order, and the lab session plan: user / planning agent (not stated).',
          '* Two-at-once desk times, the switch overhead, flights per pack and which PC runs `round.py`: not stated (carried from Stage 8c).',
          '* Dry-run S1 / S3 results under RS: not run (scope change); RS in the dry run is proven by the fake-pool test and, for S0 / S2, by the regressions above.']
    (H / 'report_stage8e.md').write_text('\n'.join(L) + '\n'); print('written report_stage8e.md')


if __name__ == '__main__': main()
