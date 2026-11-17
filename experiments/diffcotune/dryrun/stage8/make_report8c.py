#!/usr/bin/env python3
"""Stage 8c: writes report_stage8c.md (short): queue table from logs (t_start / t_end / rc), task status, lab-ready table, open items. Read-only on results and logs. Re-run after every job."""
import time, json, re
from pathlib import Path
import make_lab_tomorrow as M, make_report8 as RP
H = Path(__file__).resolve().parent; LG = H / 'logs'
JOBS = 'dry8o_S2 rtest8o_S2 gate9o_S2 gate9o_S0 dry8o_S1 rtest8o_S1 gate9o_S1 dry8o_S3 rtest8o_S3 gate9o_S3 gate9_S1'.split()
import datetime
CUT = datetime.datetime.combine(datetime.date(2026, 11, 16), datetime.time(11, 56, 0)).timestamp()      # queue c7.sh started 16 Nov 11:56:00 (logs/slotC7.log); older t_start / t_end / rc files are from the crashed queues
rd = lambda p: Path(p).read_text().strip() if Path(p).exists() else None
rdf = lambda p: Path(p).read_text().strip() if Path(p).exists() and Path(p).stat().st_mtime >= CUT else None
KEY = {}   # job -> key result text, filled below from the result files
def key(j):
    kind, C = j.split('_')[0], j.split('_')[1]
    if kind == 'dry8o':
        a = M.analyse(C, 'ours')
        if a is None or RP.stale(C): return 'not yet'
        return 'real cost %s%s' % (' → '.join('%.2f' % x for x in a['real']), '' if a['complete'] else ' (%d of 6 rounds)' % a['n'])
    if kind == 'rtest8o':
        t = M.J(H.parents[1] / 'lab' / 'results' / ('dry_test5_%s_8o.json' % C)); return 'not yet' if t is None else 'θ₁ diff %.1e, β diff %.1e' % (t['phi_next_max_abs_diff'], t['psi_max_abs_diff'])
    if kind in ('gate9o', 'gate9'):
        return RP.grad_summary(C)[0] if kind == 'gate9o' else (('see results/grad_gate9_%s_paper.json' % C) if (H / 'results' / ('grad_gate9_%s_paper.json' % C)).exists() else 'not yet')
def main():
    L = ['# Stage 8c report (generated %s by `make_report8c.py`; the planning agent validates before anything is committed or flown)' % time.strftime('%d %b %Y %H:%M'), '', '## 1. Queue (`c7.sh`, PID 3110006)', '', '| Job | start | end | rc | key result |', '|---|---|---|---|---|']
    for j in JOBS:
        s = rdf(LG / ('t_start_%s.txt' % j)); e = rdf(LG / ('t_end_%s.txt' % j)); rc = rdf(LG / ('%s.rc' % j))
        L.append('| %s | %s | %s | %s | %s |' % (j, s or 'not yet', e or ('running' if s else 'not yet'), rc if rc is not None else '–', key(j) if e else ('running' if s else 'not yet')))
    L += ['', 'Slot log (`logs/slotC7.log`):', '```', rd(LG / 'slotC7.log') or '', '```', '']
    L += ['## 2. Tasks', '', '| Task | Status | Path |', '|---|---|---|']
    for row in TASKS: L.append('| ' + ' | '.join(row) + ' |')
    L += ['', '## 3. Lab-ready table ("ours")', '']
    L += RP.sect7().split('\n')[4:]
    L += ['## 4. Open items', ''] + ['* %s' % x for x in OPEN]
    (H / 'report_stage8c.md').write_text('\n'.join(L) + '\n'); print('written')
TASKS = [
 ('T1 queue watch', 'running; see §1', '`logs/slotC7.log`'),
 ('T2 `report_stage8.md`', 'script done, re-run after every job (§5 and §7 are filled as results arrive)', '`make_report8.py`, `report_stage8.md`'),
 ('T3 S0 round-3 decision sheet', 'done', '`S0_ROUND3_DECISION.md` (`make_s0_round3_sheet.py`)'),
 ('T4 S2 preview replay', 'not yet (waits for `dry8o_S2`)', '`results/preview_S2_ours_partial.json` kept'),
 ('T5 preview mock S1 / S3', 'PASSED for S1 and S3 (mock covers both; A8 only, same set → PASSED, crashing set → FAILED; rc 0 both)', '`logs/prevmock_S1S3_stage8c.log`; `rtest8o_S1` / `rtest8o_S3` remain the full test'),
 ('T6 session timeline', 'not yet (needs 6 rounds of `loop_S2_ours.json`; old files kept as `lab/results/session_timeline.before_stage8c.*`)', '`session_timeline8c.py`'),
]
OPEN = ['S0 criteria a and b (round 3 A1 floor flight): user (`S0_ROUND3_DECISION.md`).',
        '`loop_S2_ours.json` / `LAB_TOMORROW.md` showed 2 S2 rounds until the queue run wrote its own: that was the stopped run\'s file (byte-identical copy `loop_S2_ours_stopped2104.json`); flagged in the generated reports: planning agent.',
        'Two-at-once desk times for "ours" are not measured (session timeline uses cases A / B): user.',
        'Switch and `--solo` relaunch overhead, flights per pack, which PC runs `round.py`: not stated (user).']
if __name__ == '__main__': main()
