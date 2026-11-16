#!/usr/bin/env python3
"""Stage 6 mock test of the interleaved session tooling (no hardware, no radio): a whole two-controller session (6 rounds each since Stage 8) with fake uSD files from frsim flights (the Stage 5 dry-run setup: plant P1 / P1RD,
seeds 100 + 10 l + i), in two modes with the SAME inputs: sequential (controller after controller) and interleaved (round l of A, switch, round l of B, the desk steps in the background, session.py desk / manifest / status).
The switch steps are printed. EPOCHS (default 2) replaces the 50 + 50 epochs of the method ONLY for this tooling test (written into round_0/state.json settings), so that it finishes in minutes: the test is about the files,
state dirs and the identity of the two modes, not about the numbers of the method.
  python3 test_session_mock.py --pair S0 S2 [--epochs 2]      -> results/test_session_mock_<pair>.json (and .txt)"""
import argparse, json, os, subprocess, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent; PY = sys.executable
ap = argparse.ArgumentParser(); ap.add_argument('--pair', nargs=2, default=['S0', 'S2']); ap.add_argument('--epochs', type=int, default=2); ap.add_argument('--rounds', type=int, default=6); ap.add_argument('--parallel-desks', action='store_true', help='allow the desk steps of the two controllers to overlap (two worker pools at once); default off: the next desk step waits for a running one (one pool at a time, desk-side rule)'); a = ap.parse_args()
A, B = a.pair; LOG = []
def say(*x):
    s = time.strftime('%H:%M:%S') + ' ' + ' '.join(str(y) for y in x); print(s, flush=True); LOG.append(s)
def run(cmd, **kw): return subprocess.run(cmd, check=True, capture_output=True, text=True, **kw)
def init(root, c):
    sd = root / c
    run([PY, str(HERE / 'round.py'), '--init', '--controller', c, '--state-dir', str(sd), '--n-rounds', str(a.rounds)])
    f = sd / 'round_0' / 'state.json'; st = json.load(open(f)); st['settings']['epochs'] = a.epochs; json.dump(st, open(f, 'w'), indent=1)
def fly(root, c, l):
    out = root / 'session_logs' / c / ('round_%d' % l); r = run([PY, str(HERE / 'fake_round_flights.py'), '--controller', c, '--round', str(l), '--state-dir', str(root / c), '--out', str(out)])
    files = sorted(out.glob('*_merged_usd.csv'), key=lambda p: p.stat().st_mtime)
    assign = ['%s=%s' % (f.name.split('_r')[0], f.name) for f in files]
    r = run([PY, str(HERE / 'session.py'), 'manifest', '--controller', c, '--round', str(l), '--inbox', str(out), '--out', str(out / 'flights.json'), '--root', str(root), '--fake-frame', '--assign'] + assign); say('manifest', c, l, r.stdout.strip().replace('\n', ' | ')); return out
def desk(root, c, l, out):
    if not a.parallel_desks:                                    # one worker pool at a time: wait for a desk step still running
        for f in root.glob('*/desk_*.json'):
            try: pid = json.load(open(f))['pid']; os.kill(pid, 0)
            except (OSError, KeyError, ValueError): continue
            while True:
                try: os.kill(pid, 0)
                except OSError: break
                time.sleep(2)
    r = run([PY, str(HERE / 'session.py'), 'desk', '--controller', c, '--round', str(l), '--logs', str(out), '--root', str(root), '--n-rounds', str(a.rounds), '--top', 'planned', '--lenient']); say(r.stdout.strip())
def switch(root, c, l):
    r = run([PY, str(HERE / 'session.py'), 'switch', '--to', c, '--fragment', '%s/%s/round_%d/gains.yaml' % (root.name, c, l)]); say('SWITCH to %s (round %d):\n   ' % (c, l) + r.stdout.strip().replace('\n', '\n   '))
def wait(root, c, l):
    f = root / c / ('round_%d' % (l + 1)) / 'state.json'; g = root / c / ('round_%d' % l) / 'report.json'
    while not (f.exists() if l < a.rounds - 1 else g.exists()):
        time.sleep(3)
        dk = json.load(open(root / c / ('desk_%d.json' % l))); 
        try: os.kill(dk['pid'], 0)
        except OSError:
            if not (f.exists() if l < a.rounds - 1 else g.exists()): sys.exit('desk step of %s round %d ended without a result: %s' % (c, l, dk['log']))
            break
    time.sleep(1)
def final(root, c):
    d = root / c / ('round_%d' % (a.rounds - 1)); st = json.load(open(d / 'state_after.json')); return dict(keep=st['keep'], phi_last=st['phi'], psi_last=st['psi'], best_yaml=(d / 'gains_theta_best.yaml').read_text())
res = {}; T = {}
for mode in ('sequential', 'interleaved'):
    root = HERE / 'mock6' / ('%s_%s%s' % (mode, A, B)); import shutil; shutil.rmtree(root, ignore_errors=True); root.mkdir(parents=True); t0 = time.time(); say('=== mode', mode, 'root', root)
    for c in (A, B): init(root, c)
    if mode == 'sequential':
        for c in (A, B):
            switch(root, c, 0)
            for l in range(a.rounds):
                out = fly(root, c, l); desk(root, c, l, out); wait(root, c, l)
    else:
        outs = {}
        for l in range(a.rounds):
            for c in (A, B):
                if l > 0: wait(root, c, l - 1)                      # the desk step of the previous round of this controller must be done before its gains are flown
                switch(root, c, l); outs[(c, l)] = fly(root, c, l); desk(root, c, l, outs[(c, l)])
                r = run([PY, str(HERE / 'session.py'), 'status', '--pair', A, B, '--root', str(root), '--logs', str(root / 'session_logs'), '--loaded', c, '--n-rounds', str(a.rounds)]); say('STATUS after %s round %d:\n   ' % (c, l) + r.stdout.strip().replace('\n', '\n   '))
        for c in (A, B): wait(root, c, a.rounds - 1)
    T[mode] = time.time() - t0; res[mode] = {c: final(root, c) for c in (A, B)}; say(mode, 'done in %.0f s' % T[mode])
    r = run([PY, str(HERE / 'session.py'), 'cards', '--start-count', '10', '--pair', A, B]); say(r.stdout.strip())
same = {c: dict(keep_identical=res['sequential'][c]['keep'] == res['interleaved'][c]['keep'], phi_last_identical=res['sequential'][c]['phi_last'] == res['interleaved'][c]['phi_last'], psi_last_identical=res['sequential'][c]['psi_last'] == res['interleaved'][c]['psi_last'],
                best_yaml_identical=res['sequential'][c]['best_yaml'] == res['interleaved'][c]['best_yaml'], max_abs_phi_diff=float(np.abs(np.array(res['sequential'][c]['phi_last']) - np.array(res['interleaved'][c]['phi_last'])).max())) for c in (A, B)}
say('IDENTICAL sequential vs interleaved:', json.dumps(same)); (HERE / 'results').mkdir(exist_ok=True)
json.dump(dict(pair=[A, B], epochs=a.epochs, rounds=a.rounds, seconds=T, identical=same, theta_best_round={c: res['interleaved'][c]['keep']['round'] for c in (A, B)}), open(HERE / 'results' / ('test_session_mock_%s%s.json' % (A, B)), 'w'), indent=1)
(HERE / 'results' / ('test_session_mock_%s%s.txt' % (A, B))).write_text('\n'.join(LOG) + '\n'); sys.exit(0 if all(all(v for k, v in d.items() if k != 'max_abs_phi_diff') for d in same.values()) else 1)
