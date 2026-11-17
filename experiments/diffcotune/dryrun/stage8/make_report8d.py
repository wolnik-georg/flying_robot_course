#!/usr/bin/env python3
"""Stage 8d: writes report_stage8d.md (generated, re-run after every job): changes + tests, c8.sh queue table, answer to the user's question per controller (old "ours" against "ours_strict"), lab-ready table (strict runs), open items.
Read-only on results / logs. Usage: nice -n 19 /usr/bin/python3 make_report8d.py"""
import datetime, time, json, subprocess
from pathlib import Path
import make_lab_tomorrow as M, make_report8 as RP
H = Path(__file__).resolve().parent; LG = H / 'logs'; LAB = H.parents[1] / 'lab'; R = H / 'results'; J = M.J
CUT = datetime.datetime.combine(datetime.date(2026, 11, 16), datetime.time(13, 18, 0)).timestamp()      # c8.sh started 16 Nov 13:18:47
rdf = lambda p: Path(p).read_text().strip() if Path(p).exists() and Path(p).stat().st_mtime >= CUT else None
rd = lambda p: Path(p).read_text() if Path(p).exists() else None
JOBS = 'regress8s_S0 dry8s_S0 rtest8s_S0 dry8s_S2 rtest8s_S2 gate9s_S0 gate9s_S2 dry8s_S1 rtest8s_S1 gate9s_S1 dry8s_S3 rtest8s_S3 gate9s_S3'.split()
CT = ['S0', 'S2', 'S1', 'S3']


def pf(path, ids=None):
    """PASS / FAIL count of a test log (optionally only the checks whose id (first word after PASS/FAIL) is in ids)"""
    t = rd(path)
    if t is None: return 'not run'
    ln = [l.split() for l in t.splitlines() if l.startswith(('PASS', 'FAIL')) and len(l.split()) > 1]
    if ids: ln = [l for l in ln if l[1] in ids]
    n = len(ln); f = sum(1 for l in ln if l[0] == 'FAIL')
    return 'not run' if n == 0 else ('%d checks, all PASS' % n if f == 0 else '%d of %d checks FAIL' % (f, n))


def key(j):
    kind, C = j.split('_')
    if kind == 'regress8s':
        g = J(R / 'regress8s_S0.json'); return 'not yet' if g is None else '%s: θ₁ diff %.1e, β diff %.1e, real cost diff %.1e' % ('PASS' if g['passed'] else 'FAIL', g['theta1_max_abs_diff'], g['psi_after_fit_max_abs_diff'], g['real_cm_diff'])
    if kind == 'dry8s':
        a = M.analyse(C, 'ours_strict'); return 'not yet' if a is None else 'cost as flown %s; used %s%s' % (' → '.join('%.2f' % x for x in a['real']), ' → '.join('n/a' if x is None else '%.2f' % x for x in a['real_used']), '' if a['complete'] else ' (%d of 6 rounds)' % a['n'])
    if kind == 'rtest8s':
        t = J(LAB / 'results' / ('dry_test5_%s_8s.json' % C)); return 'not yet' if t is None else 'θ₁ diff %.1e, β diff %.1e (R3 in round 0: %s)' % (t['phi_next_max_abs_diff'], t['psi_max_abs_diff'], t.get('r3') and t['r3']['scenarios'])
    if kind == 'gate9s':
        g = J(R / ('grad_gate9_%s_ours_strict.json' % C))
        if g is None: return 'not yet'
        rows = g.get('guard_fitted_beta', []); mx = max([s['adopted_vs_fd_rel'] for e in rows for p in ('task', 'sysid') for s in e.get(p, [])] or [0]); bad = [e['round'] for e in rows if not e['passed']]
        return '%s at the fitted β of %d rounds (max adopted-vs-FD %.1e%s)' % ('PASS' if rows and not bad else 'FAIL', len(rows), mx, '' if not bad else '; failing rounds %s' % bad)


def grad_ok(C):
    g = J(R / ('grad_gate9_%s_ours_strict.json' % C))
    if g is None: return None
    rows = g.get('guard_fitted_beta', []); return bool(rows) and all(e['passed'] for e in rows)


def lab_table():
    o = ['| Controller | strict dry run | a / b / c | round test (strict) | gradient check | eligible sets / keep-best | Lab-ready | Reason |', '|---|---|---|---|---|---|---|---|']
    for C in CT:
        a = M.analyse(C, 'ours_strict')
        if a is None: o.append('| %s | not yet | – | – | – | – | **not yet** | strict dry run not yet |' % C); continue
        rt = J(LAB / 'results' / ('dry_test5_%s_8s.json' % C)); g = grad_ok(C); y = M.yn
        rtt = 'not yet' if rt is None else ('exact' if max(rt['phi_next_max_abs_diff'], rt['psi_max_abs_diff']) < 1e-9 else 'differs (θ₁ %.1e, β %.1e)' % (rt['phi_next_max_abs_diff'], rt['psi_max_abs_diff']))
        why = []
        if not a['complete']: why.append('dry run incomplete (%d of 6 rounds%s)' % (a['n'], '' if 'tun0' in a else ', evaluation not yet'))
        for k in 'abc':
            if a[k] is False: why.append('criterion %s not met' % k)
            elif a[k] is None: why.append('criterion %s not yet' % k)
        if not a['best_eligible']: why.append('no eligible set')
        if rt is None: why.append('round test not yet')
        elif rtt != 'exact': why.append('round test differs')
        if g is None: why.append('gradient check not yet')
        elif not g: why.append('gradient check FAIL')
        ok = a['complete'] and a['a'] and a['b'] and a['c'] and a['best_eligible'] and rtt == 'exact' and g
        pend = (not a['complete']) or rt is None or g is None
        o.append('| %s | %s | %s / %s / %s | %s | %s | %s / %s | **%s** | %s |' % (C, 'complete' if a['complete'] else '%d of 6 rounds' % a['n'], y(a['a']), y(a['b']), y(a['c']), rtt, 'not yet' if g is None else ('PASS' if g else 'FAIL'), [l for l in range(a['n']) if a['eligible'][l]], a['best_round'], 'yes' if ok else ('not yet' if pend and not any(a[k] is False for k in 'abc') else 'no'), '; '.join(why) or 'all conditions met'))
    return o


def main():
    t = lambda f: (rd(f) or 'not run').strip().split('\n')[-1]
    L = ['# Stage 8d report (generated %s by `make_report8d.py`; the planning agent validates before anything is committed or flown)' % time.strftime('%d %b %Y %H:%M'), '',
         '## 1. Changes (T1–T3) and tests', '',
         '| Item | Files (copies `*.before_stage8d` kept) | Change | Test | Result |', '|---|---|---|---|---|',
         '| T1 strict switch | `dryrun/loop_lib.py` | `DIFFCOTUNE_DRY_STRICT` (default 0 = old path): `fly_set_strict` (repeat crash and floor flights, ≤ 3 attempts, seed + 1000 × tries, all attempts returned); `run_loop`: fit on usable flights only (scenario subset), R3 fallback flight with θ_best (not used), keep-best over eligible sets with cost over used flights, "no eligible set" → θ_0 marked; as-flown cost kept as `real`, `real_used`, `attempts`, `used_attempt`, `failed_all`, `fallback`, `eligible`, `r3_scenarios` added | `dryrun/stage8/test_strict_rule.py` (fake pool): all-3-fail → 3 attempts + fallback with θ_best + 4-scenario fit + not eligible; floor on attempt 1 only → attempt 2 used; round 0 with 3 fails → no fallback; no eligible set → θ_0 marked; fallback uses θ_best of the eligible sets | %s |' % t(LG / 'test_strict_rule.log'),
         '| T1 regression strict = 0 | `stage8/regress8s.sh`, `compare_regress8s.py` | round 0 of S0 "ours" rerun against `loop_S0_ours.json` (θ₁ from `tune.log[best_epoch].x`, β after the fit, real cost) | queue job `regress8s_S0` | %s |' % key('regress8s_S0'),
         '| T2 R3 in `round.py` | `lab/round.py` | stop at "no usable flight" replaced by R3 for a scenario with attempts 1, 2, 3 all failed (manifest role `fallback`, attempt 4, flown with θ_best); missing fallback → exact message with the θ_best gains file, nothing written; fit on the usable flights, set not eligible (state.json keep unchanged, history `eligible: false`), `report.json` fields `eligible`, `r3`, `r3_version`; `cell_of` treats role `fallback` like `tuning`; version marker `R3-2026-11-16` | `lab/test_r3_mock.py` (fake uSD files, epochs 2, one worker): `%s` | %s |' % ('lab/results/test_r3_mock_a.txt', 'a: ' + ((rd(LAB / 'results' / 'test_r3_mock_a.txt') or '').count('PASS ') and '%d PASS lines, %s' % ((rd(LAB / 'results' / 'test_r3_mock_a.txt') or '').count('\nPASS ') + 1, 'ALL PASS' if 'ALL PASS' in (rd(LAB / 'results' / 'test_r3_mock_a.txt') or '') else 'check file') or 'not run')),
         '| T2 regression (all flights usable) | `lab/test_r3_mock.py --regress` | `round.py` now against `round.py.before_stage8d`, same folder (`dry5_S0_8o/round0_logs`), lenient as the old dry test, epochs 2 | `lab/results/test_r3_mock_b.txt` | %s |' % ', '.join(l for l in (rd(LAB / 'results' / 'test_r3_mock_b.txt') or '').split('\n') if l.startswith('PASS b regression') or l.startswith('ALL PASS'))[:300],
         '| T2 `dry_test5.py` | `lab/dry_test5.py` | `--strict-dry`: flights repeated like the lab (≤ 3 attempts), `round.py` strict; reference `loop_<C>_ours_strict.json`; outputs `dry_test5_<C>_8s.json`; default path unchanged | round tests `rtest8s_<C>` (queue) | see §2 |',
         '| T2 `preflight.py` | `lab/preflight.py` | new check 6b (R3 version marker in `round.py`) | `preflight.py --controller S0` | 6b PASS, PREFLIGHT PASSED (13:18) |',
         '| T2 cards | `lab/S0_LAB_CARD.md`, `lab/INTERLEAVED_LAB_CARD.md` | item 9a / 5b (R3 step, +1 flight per R3 case), rule rows | – | done |',
         '| **8d2** extension rounds, dry run | `dryrun/loop_lib.py` (`*.before_stage8d2`), `stage8/run_8.py` (no evaluation when there is no eligible set) | strict mode only: no eligible set after round 5 → extension rounds l = 6, 7 (same method, fit and tuning step after round 5 as in any round); record `extension_rounds_used`; still none → `best` has no gains, note "no eligible set after extension: ask the user", never θ_0; default path bit-identical (strict = 0) | `stage8/test_strict_rule.py` (fake pool) | %s |' % pf(LG / 'test_strict_rule.log'),
         '| **8d2** extension rounds, lab | `lab/round.py` (`*.before_stage8d2`), `lab/session.py` status, `lab/preflight.py` 6c | last planned round without eligible set: no final gains file, `round_<l+1>/` written, `extension_round_added`; after round 7 without eligible set: `NO_ELIGIBLE_SET.md`, message "no eligible set after extension: ask the user", no gains file | `lab/test_r3_mock.py --ext` (c–f, 2 epochs, one worker) | %s |' % pf(LAB / 'results' / 'test_r3_mock_ext.txt', ('c1', 'c', 'd', 'e', 'f')),
         '| **8d2** fallback-flight failure rule + `session.py manifest --fallback` | `lab/round.py` R3 message, `lab/session.py` (`--fallback SCEN=file --fallback-gains <θ_best gains.yaml> --append`: role fallback, attempt 4, one per scenario), both lab cards | message and cards: if the fallback flight fails, pause, fix drone and setup (battery, props, mocap, EKF toggle), fly it once more, the round continues either way (`report.json` r3.fallback_flags); a stopped lab day resumes from the saved round state | `lab/test_r3_mock.py --ext` (g, h1–h3, i) | %s |' % pf(LAB / 'results' / 'test_r3_mock_ext.txt', ('g', 'h1', 'h2', 'h3', 'i')),
         '| T3 readers | `make_lab_tomorrow.py`, `make_report8.py`, `make_report8c.py` unchanged, new `make_report8d.py` | tag `ours_strict` rendered next to old "ours" (cost as flown / used, attempts, R3 cases, eligibility, a/b/c); old files render as before | – | in use |', '',
         '## 2. Queue c8.sh (started 13:18:47; waits for `logs/dry8o_S2.rc`)', '', 'Old-mode S2 run `dry8o_S2` (before run): %s' % ('rc %s, ended %s' % (rd(LG / 'dry8o_S2.rc').strip(), rd(LG / 't_end_dry8o_S2.txt').strip()) if rd(LG / 'dry8o_S2.rc') else 'running (started 11:56:00)'), '',
         '| Job | start | end | rc | key result |', '|---|---|---|---|---|']
    for j in JOBS:
        s = rdf(LG / ('t_start_%s.txt' % j)); e = rdf(LG / ('t_end_%s.txt' % j)); rc = rdf(LG / ('%s.rc' % j))
        L.append('| %s | %s | %s | %s | %s |' % (j, s or 'not yet', e or ('running' if s else 'not yet'), rc if rc is not None else '–', key(j) if e else ('running' if s else 'not yet')))
    L += ['', 'Dropped (superseded by the strict runs): `rtest8o_S2`, `gate9o_S2`, `gate9o_S0`, `dry8o_S1`, `rtest8o_S1`, `gate9o_S1`, `dry8o_S3`, `rtest8o_S3`, `gate9o_S3` and the paper gate `gate9_S1`. `c7.sh` was stopped at 12:42:50.', '', '```', rd(LG / 'slotC8.log') or '', '```', '',
          '## 3. Answer: does the bad round still happen with the lab\'s rule?', '', 'Controllers S0 and S2 first. Cost in cm, 3-D RMS, mean of the 5 tuning flights of a round; "as flown" = last attempt of each scenario (the old definition), "used" = mean over the usable flights.', '']
    for C in CT: L += RP.strict_block(C, '###')
    L += ['## 4. Lab-ready table (strict runs)', ''] + lab_table() + ['', '## 5. Open items', '',
          '* **Keep-best eligibility rule = consequence of R3, user to confirm** (a set with an R3 scenario is not eligible; cost over the used flights; no eligible set after round 5 → up to 2 extension rounds (l = 6, 7, same method), then "no eligible set after extension: ask the user", never θ_0). **Rule confirmed by the user 16 Nov** (Stage 8d2).',
          '* S0 criteria a / b / c are evaluated on the strict runs; no criterion was changed (user).',
          '* `session.py manifest` does not write the `fallback` entry: it is added to `flights.json` by hand (card item 9a / 5b); whether `session.py` should do it: user / planning agent.',
          '* In the dry run a fallback flight with θ_best is itself recorded but not judged (no rule stated for a failed fallback flight): user.',
          '* Preview, session timeline (T6) with the strict desk minutes, and the S2 preview replay (T4) follow when the runs are complete.']
    (H / 'report_stage8d.md').write_text('\n'.join(L) + '\n'); print('written report_stage8d.md')


if __name__ == '__main__': main()
