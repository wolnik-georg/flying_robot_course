#!/usr/bin/env python3
"""Stage 6 lever d: wall time of usd_ingest.ingest_flight (parse, gate, frame fit, lag search, reference check, flags, cost) on the 10 real S0 flights of 23 Oct (experiments/logs/bank_unseen_S0_2026-10-23), file by file,
cold (first file includes the import and first-call overhead) and with top_from_log False and True. Output results/ingest_timing.json. One process; run through job_runner."""
import csv, json, sys, time
from pathlib import Path
HERE = Path(__file__).resolve().parent; ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun'))
import usd_ingest as U  # noqa
from test_ingest_oct23 import cell_of, BANK, PF  # noqa
rows = [r for r in csv.DictReader(open(PF)) if r['bank'] == 'bank_unseen_S0_2026-10-23']; out = []
for r in rows:
    f = BANK / r['folder'] / (r['folder'] + '_merged_usd.csv'); cell = cell_of(r['scenario'], r)
    for top in (False, True):
        t0 = time.time(); g = U.ingest_flight(f, r['scenario'], cell, top_from_log=top); dt = time.time() - t0
        out.append(dict(folder=r['folder'], scenario=r['scenario'], top_from_log=top, seconds=dt, file_mb=f.stat().st_size / 1e6, rows=g['gate']['rows'])); print('%-26s %-6s top_log %-5s %.2f s  (%.1f MB, %d rows)' % (r['folder'], r['scenario'], top, dt, f.stat().st_size / 1e6, g['gate']['rows']), flush=True)
tot = [o['seconds'] for o in out if not o['top_from_log']]
res = dict(per_file=out, mean_s=sum(tot) / len(tot), max_s=max(tot), total_10_flights_s=sum(tot)); json.dump(res, open(HERE / 'results' / 'ingest_timing.json', 'w'), indent=1); print('mean %.2f s, max %.2f s, sum of 10 flights %.1f s' % (res['mean_s'], res['max_s'], res['total_10_flights_s']))
