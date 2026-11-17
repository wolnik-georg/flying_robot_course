#!/usr/bin/env python3
"""Stage 8c: writes dryrun/stage8/report_stage8.md = parts 1-4 (report_stage8_part1..4.md, by section number) + generated sections 5 (dry runs "ours" S0-S3 with the paper columns) and 7 (lab readiness)
+ a generated table in section 6 (preview vs real cost). Read-only on results/, logs/ and lab/results/. Re-runnable after every job. Usage: nice -n 19 /usr/bin/python3 make_report8.py
Numbers only from the result files; a missing file is written as "not yet"."""
import json, re, time
from pathlib import Path
import numpy as np
import make_lab_tomorrow as M
H = Path(__file__).resolve().parent; R = H / 'results'; LAB = H.parents[1] / 'lab'; J = M.J
CT = ['S0', 'S2', 'S1', 'S3']; yn = M.yn


def f2(x): return '%.2f' % x


def stale(C):
    # loop_<C>_ours.json byte-identical to a kept *_stopped2104* copy = still the file of the stopped (crashed) run, the queue run has not written its first round yet
    a = R / ('loop_%s_ours.json' % C); b = R / ('loop_%s_ours_stopped2104.json' % C)
    return a.exists() and b.exists() and a.read_bytes() == b.read_bytes()


def per_round(C, tag):
    rec = J(R / ('loop_%s_%s.json' % (C, tag)))
    return rec


def grad_summary(C):
    g = J(R / ('grad_gate9_%s_ours.json' % C))
    if g is None: return 'not yet', None
    rows = g.get('guard_fitted_beta', []); bad = []; mx = 0.0
    for e in rows:
        for part in ('task', 'sysid'):
            for s in e.get(part, []):
                mx = max(mx, s['adopted_vs_fd_rel'])
                if not s['passed']: bad.append('round %d %s %s' % (e['round'], part, s['scenario']))
    allp = len(rows) > 0 and not bad
    txt = '%s at the fitted β of %d rounds (max adopted-vs-finite-difference relative error %.1e%s); sections 1–2 (θ / β rows on the used components): %s' % ('PASS' if allp else 'FAIL', len(rows), mx, '' if allp else '; failing: ' + ', '.join(bad), 'passed' if g.get('passed') or g.get('passed_posthoc') else 'not passed')
    return txt, allp


def strict_block(C, head='####'):
    """Stage 8d: old "ours" next to "ours_strict" (lab flight rule + R3) for one controller; empty list if the strict run does not exist yet."""
    st = M.analyse(C, 'ours_strict'); old = M.analyse(C, 'ours')
    if st is None: return [head + ' %s strict dry run (tag ours_strict): not yet' % C, '']
    rec = J(R / ('loop_%s_ours_strict.json' % C)); n = st['n']; r_ = st['real']
    o = [head + ' %s: old "ours" against "ours_strict" (lab flight rule + R3)' % C, '']
    if old is not None and not stale(C): o.append('Old "ours" (as flown): ' + M.row_real(old) + '; a / b / c = %s / %s / %s; keep-best round %s.' % (yn(old['a']), yn(old['b']), yn(old['c']), old['best_round']))
    o += ['', '| Round | cost as flown (cm) | cost over used flights (cm) | attempts (5 scenarios) | R3 cases | fallback flights | eligible | floor / crash flags of the last attempts | desk min |', '|---|---|---|---|---|---|---|---|---|']
    for l in range(n):
        o.append('| %d | %s | %s | %d | %s | %d | %s | %d floor / %d crash | %.0f |' % (l, f2(r_[l]), 'n/a' if st['real_used'][l] is None else f2(st['real_used'][l]), st['attempts'][l], ', '.join(st['r3'][l]) or '–', st['fallbacks'][l], 'yes' if st['eligible'][l] else 'no', st['floor'][l], st['crash'][l], st['minutes'][l]))
    o += ['', 'Per scenario, all attempts (cm; F = floor, C = crash; * = used; fallback in brackets):', '', '| Round | ' + ' | '.join(f['scenario'] for f in rec['rounds'][0]['flights']) + ' |', '|---|' + '---|' * len(rec['rounds'][0]['flights'])]
    for l, r in enumerate(rec['rounds']):
        cells = []
        for f in r['flights']:
            t = ' / '.join('%.1f%s%s%s' % (a['rms3d_cm'], 'F' if a['floor'] else '', 'C' if a['crashed'] else '', '*' if a['used'] else '') for a in f['attempts'])
            if f.get('fallback'): t += ' [fallback θ_best round %s: %.1f%s]' % (f['fallback']['theta_round'], 100 * f['fallback']['cost'] ** 0.5, 'F' if f['fallback']['floor'] else '')
            cells.append(t)
        o.append('| %d | %s |' % (l, ' | '.join(cells)))
    va = [l for l in range(2, n) if r_[l] > r_[1] + 1.0] if n > 1 else []
    o += ['', 'Status: %s.' % ('complete' if st['complete'] else '%d of 6 rounds%s' % (n, '' if 'tun0' in st else ', evaluation not yet')),
          '* Keep-best: round %s%s; eligible sets: rounds %s.' % (st['best_round'], '' if st['best_eligible'] else ' (NO ELIGIBLE SET: %s)' % st['best_note'], [l for l in range(n) if st['eligible'][l]]),
          '* Criteria (definitions of make_lab_tomorrow.py:23–25; b on "no usable attempt"): a **%s**, b **%s**, c **%s**.' % (yn(st['a']), yn(st['b']), yn(st['c'])),
          '* A round worse than round 1 + 1 cm (%s cm as flown): %s.' % (f2(r_[1] + 1.0) if n > 1 else 'n/a', ', '.join('round %d (%s)' % (l, f2(r_[l])) for l in va) if va else 'none so far'),
          '* Means θ₀ → θ_best (cm, seeds 2000–2004): %s.' % ('not yet' if 'tun0' not in st else 'tuning %s → %s; held-out A4 cell 2 %s → %s; in-sample A4 cell 1 %s → %s' % (f2(st['tun0']), f2(st['tunb']), f2(st['held0']), f2(st['heldb']), f2(st['ins0']), f2(st['insb']))),
          '* Round test (strict): %s.' % (lambda t: 'not yet' if t is None else 'θ₁ difference %.1e, β difference %.1e, R3 in round 0: %s' % (t['phi_next_max_abs_diff'], t['psi_max_abs_diff'], t.get('r3')))(J(LAB / 'results' / ('dry_test5_%s_8s.json' % C))),
          '* Gradient check: %s.' % (lambda g: 'not yet' if g is None else ('PASS' if g.get('passed') or g.get('passed_posthoc') else 'FAIL, see results/grad_gate9_%s_ours_strict.json' % C))(J(R / ('grad_gate9_%s_ours_strict.json' % C))), '']
    return o


def sect5():
    o = ['## 5. Dry runs "ours" S0–S3 (frozen lab variant) with the paper variant next to them (not adopted)', '',
         'Plant P1 (thrust and lag wrong on purpose); cost = real 3-D RMS in cm, mean of the 5 tuning flights of the round (one flight per scenario). Criteria (`make_lab_tomorrow.py:23–25`): **a** every round l ≥ 2 ≤ round 1 + 1.0 cm; **b** no crash or floor flight in round l ≥ 1 on a scenario that did not already fail in rounds 0 and 1; **c** θ_best better than θ₀ on the tuning mean and on held-out A4 cell 2 (evaluation seeds 2000–2004). A1 floors at the default gains (plant P1). Generated %s.' % time.strftime('%d %b %Y %H:%M'), '']
    for C in CT:
        a = M.analyse(C, 'ours'); p = M.analyse(C, 'paper'); rec = per_round(C, 'ours'); recp = per_round(C, 'paper')
        o += ['### 5.%d %s' % (CT.index(C) + 1, C), '']
        if a is None:
            o += ['"ours" dry run: not yet (`results/loop_%s_ours.json` missing).' % C, '']
        else:
            o += ['| Round | real 3-D cost (cm) | crashes | floor flights (scenario) | largest gain change to the next set | desk minutes |', '|---|---|---|---|---|---|']
            for l in range(a['n']):
                jmp = a['jump'][l] if a['jump'] and l < len(a['jump']) else None
                o.append('| %d | %s | %d | %d%s | %s | %.0f |' % (l, f2(a['real'][l]), a['crash'][l], a['floor'][l], (' (' + ', '.join(a['floor_scen'][l]) + ')') if a['floor'][l] else '', ('%.0f %%' % (100 * jmp)) if jmp is not None else '–', a['minutes'][l]))
            if stale(C): o += ['', '**Caution: `results/loop_%s_ours.json` is still byte-identical to `loop_%s_ours_stopped2104.json` (the file of the run stopped on 15 Nov 21:04); the queue run has not written its first round yet, so the rows above are those of the stopped run, not of the queue run.**' % (C, C)]
            r_ = a['real']; n = a['n']
            fl = [{f['scenario']: bool(f['crashed'] or f['floor']) for f in x['flights']} for x in rec['rounds']]
            va = [l for l in range(2, n) if r_[l] > r_[1] + 1.0]
            vb = [(l, s) for l in range(1, n) for s in fl[l] if fl[l][s] and not (fl[0][s] and fl[1][s])]
            o += ['', 'Status: %s%s.' % ('complete (6 rounds + evaluation)' if a['complete'] else '%d of 6 rounds%s' % (n, ', evaluation not yet' if 'tun0' not in a else ''), ', halted: %s' % a['halt'] if a['halt'] else ''), '',
                  '* a: **%s**. Threshold round 1 + 1.0 = %s cm; %s.' % (yn(a['a']), f2(r_[1] + 1.0) if n > 1 else 'n/a', ('violations: ' + ', '.join('round %d (%s cm)' % (l, f2(r_[l])) for l in va)) if va else 'no violation so far'),
                  '* b: **%s**. %s.' % (yn(a['b']), ('violations (round, scenario): %s' % vb) if vb else 'no violation so far'),
                  '* c: **%s**. %s.' % (yn(a['c']), M.analyse and ('tuning mean %s → %s cm; held-out A4 cell 2 %s → %s cm' % (f2(a['tun0']), f2(a['tunb']), f2(a['held0']), f2(a['heldb']))) if 'tun0' in a else 'evaluation not yet'),
                  '* Means (θ₀ → θ_best, round %s): tuning %s; held-out A4 cell 2 %s; in-sample A4 cell 1 %s.' % (a['best_round'], ('%s → %s' % (f2(a['tun0']), f2(a['tunb']))) if 'tun0' in a else 'not yet', ('%s → %s' % (f2(a['held0']), f2(a['heldb']))) if 'tun0' in a else 'not yet', ('%s → %s' % (f2(a['ins0']), f2(a['insb']))) if 'tun0' in a else 'not yet'),
                  '* Evaluation crashes at θ_best: %s.' % (('%d of %d' % a['crash_eval']) if 'crash_eval' in a else 'not yet')]
            rt = J(LAB / 'results' / ('dry_test5_%s_8o.json' % C))
            o.append('* Round test (`lab/round.py` round 0 against the dry run): %s.' % ('not yet' if rt is None else 'θ₁ difference %.1e, β difference %.1e, %.1f min (`lab/results/dry_test5_%s_8o.json`)' % (rt['phi_next_max_abs_diff'], rt['psi_max_abs_diff'], rt['seconds_round'] / 60, C)))
            o.append('* Gradient check: %s.' % grad_summary(C)[0])
            pv = a['preview']
            o.append('* Safety preview: %s.' % ('not yet' if not pv else '; '.join('round %d %s (real %.2f cm)' % (x['round'], 'PASSED' if x['preview']['passed'] else 'FAILED', x['real_cm']) for x in pv)))
            o.append('')
        o += strict_block(C)
        if p is not None:
            o.append('Paper variant (evidence, not adopted): real cost %s; a / b / c = %s / %s / %s; crashes per round %s; floor per round %s; largest gain change %s; means tuning %s, held-out A4 cell 2 %s (θ₀ → θ_best, round %s); desk minutes %s.' % (
                M.row_real(p), yn(p['a']), yn(p['b']), yn(p['c']), ' '.join(map(str, p['crash'])), ' '.join(map(str, p['floor'])), ', '.join('%.0f %%' % (100 * x) for x in (p['jump'] or [])),
                ('%s → %s' % (f2(p['tun0']), f2(p['tunb']))) if 'tun0' in p else 'not yet', ('%s → %s' % (f2(p['held0']), f2(p['heldb']))) if 'tun0' in p else 'not yet', p['best_round'], ' '.join('%.0f' % x for x in p['minutes'][:5])))
            gp = J(R / ('grad_gate9_%s_paper.json' % C))
            o.append('Paper gradient gate: %s.' % ('not yet' if gp is None else ('passed (posthoc, used components)' if gp.get('passed_posthoc') or gp.get('passed') else 'not passed')))
            o.append('')
        else:
            o += ['Paper variant: not yet.', '']
    return '\n'.join(o)


def sect6_add():
    o = ['', '### 6.1 Safety preview against the real cost of each round (generated, replay of the stored θ; the preview never changes gains)', '', '| Run | Round | Preview | Real cost of that round (cm) | Best flown before (cm) | Failed events |', '|---|---|---|---|---|---|']
    any_ = False
    for C in CT:
        for tag in ('ours',):
            pv = J(R / ('preview_%s_%s.json' % (C, tag)))
            if pv: any_ = True
            for x in (pv or []):
                o.append('| %s %s | %d | %s | %s | %s | %s |' % (C, tag, x['round'], 'PASSED' if x['preview']['passed'] else 'FAILED', f2(x['real_cm']), f2(x['real_best_cm']), x['preview']['fails'] or '–'))
            if not pv: o.append('| %s %s | not yet | | | | |' % (C, tag))
    o += ['', 'S2 partial replay (kept): `results/preview_S2_ours_partial.json`.' if (R / 'preview_S2_ours_partial.json').exists() else '', '']
    return '\n'.join(o)


def sect7():
    o = ['## 7. Lab readiness per controller ("ours")', '', 'Rule used here (as in `make_lab_tomorrow.py`): yes only if the dry run is complete (6 rounds + evaluation) and a, b and c are met; and a round test and a gradient check are present and passed. Anything missing is "not yet". Criteria are not changed.', '',
         '| Controller | dry run | a / b / c | round test | gradient check | preview | Lab-ready | Reason |', '|---|---|---|---|---|---|---|---|']
    for C in CT:
        a = M.analyse(C, 'ours')
        if a is not None and stale(C):
            o.append('| %s | queue run not yet written (file = stopped run) | – | – | – | – | **not yet** | dry run running |' % C); continue
        rt = J(LAB / 'results' / ('dry_test5_%s_8o.json' % C)); gt, gp = grad_summary(C)
        if a is None:
            o.append('| %s | not yet | – | %s | %s | not yet | **not yet** | dry run missing |' % (C, 'not yet' if rt is None else 'present', gt.split(' at ')[0])); continue
        rtt = 'not yet' if rt is None else ('exact (θ₁ %.0e, β %.0e)' % (rt['phi_next_max_abs_diff'], rt['psi_max_abs_diff']) if max(rt['phi_next_max_abs_diff'], rt['psi_max_abs_diff']) < 1e-9 else 'differs (θ₁ %.1e, β %.1e)' % (rt['phi_next_max_abs_diff'], rt['psi_max_abs_diff']))
        why = []
        if not a['complete']: why.append('dry run incomplete (%d of 6 rounds%s)' % (a['n'], ', evaluation not yet' if 'tun0' not in a else ''))
        for k in 'abc':
            if a[k] is False: why.append('criterion %s not met' % k)
            elif a[k] is None: why.append('criterion %s not yet' % k)
        if rt is None: why.append('round test not yet')
        elif 'differs' in rtt: why.append('round test differs')
        if gp is None: why.append('gradient check not yet')
        elif not gp: why.append('gradient check FAIL')
        pv = a['preview']; pvt = 'not yet' if not pv else ' / '.join('r%d %s' % (x['round'], 'P' if x['preview']['passed'] else 'F') for x in pv) + ' (P = passed, F = failed)'
        ok = a['complete'] and a['a'] and a['b'] and a['c'] and rt is not None and 'differs' not in rtt and gp
        o.append('| %s | %s | %s / %s / %s | %s | %s | %s | **%s** | %s |' % (C, 'complete' if a['complete'] else '%d of 6 rounds' % a['n'], yn(a['a']), yn(a['b']), yn(a['c']), rtt, ('PASS' if gp else ('FAIL' if gp is False else 'not yet')), pvt, 'yes' if ok else ('not yet' if (not a['complete'] or rt is None or gp is None) and not any(a[k] is False for k in 'abc') else 'no'), '; '.join(why) or 'all conditions met'))
    o += ['', 'S0 criteria a and b: see `S0_ROUND3_DECISION.md` (decision of the user). Preflight (`logs/pre_<C>.log`): see the Stage 8c report.', '']
    return '\n'.join(o)


def main():
    parts = {}
    for i in range(1, 5):
        t = (H / ('report_stage8_part%d.md' % i)).read_text()
        for blk in re.split(r'(?m)^(?=## \d+\. )', t):
            m = re.match(r'## (\d+)\. ', blk)
            if m: parts[int(m.group(1))] = blk.rstrip() + '\n'
    parts[5] = sect5() + '\n'; parts[6] = parts.get(6, '') + sect6_add(); parts[7] = sect7()
    out = ['# Stage 8 report (DiffCoTune "ours" and paper variants; generated by `make_report8.py` %s from the result files; parts 1–4 are `report_stage8_part*.md`)' % time.strftime('%d %b %Y %H:%M'), '']
    for k in sorted(parts): out += [parts[k], '']
    (H / 'report_stage8.md').write_text('\n'.join(out)); print('written report_stage8.md', sum(len(x) for x in out), 'chars')


if __name__ == '__main__':
    main()
