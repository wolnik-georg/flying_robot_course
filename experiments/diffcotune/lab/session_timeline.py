#!/usr/bin/env python3
"""Stage 6, step 3: timeline of a DiffCoTune lab session, sequential against interleaved (two controllers at once). Desk only, a model of the schedule, not a measurement of a session.

Inputs (all parameters, nothing measured in the lab):
  86.6 s per flight (mean spacing of the 10 flight stamps of the S0 block of 23 Oct, includes launch and landing; protocol draft P5), 4 tuning flights per round (A1, A8 two drones, then CIRCLE, FIG8 solo),
  5 rounds (l = 0 ... 4), 12 test flights per controller (6 with the default gains, flown any time the controller is loaded, 6 with theta_best, only after the last desk step),
  desk minutes per round of each controller (file results/desk_times.json: alone and with two loops at once; default = the Stage 5 / 5c dry-run times, last round 0.1 min),
  strategy-switch overhead (CS2 stop, yaml merge, set_tree_strategy.py, relaunch, readback): parameter, 3 / 5 / 10 min (NOT measured; not stated anywhere),
  the extra --solo relaunch per round: same parameter value (not measured).
Policy: sequential = controller after controller (each starts with one switch, drones wait for the PC; the default-gain test flights are flown in the wait of round 0, the theta_best tests after the last desk step).
Interleaved pair (A, B) = the drones fly round l of A, switch, round l of B, switch back to A if its desk step is ready (else the wait is filled with test flights of the loaded controller, then idle), and so on; the desk steps run
on the PC in the background (one loop per controller; durations 'two at once' from the measurement). Flights per hour = all flights / session hours.
Usage: python3 session_timeline.py  -> results/session_timeline.json, session_timeline.md, figs/session_timeline.png (needs matplotlib: run with the pyenv python for the figure)."""
import json, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
FLY_S = 86.6; N_TUNE = 4; N_ROUNDS = 5; N_TEST = 12
STAGE5_DESK = {'S0': [20.1, 20.5, 20.3, 20.3, 0.1], 'S1': [25.1, 22.4, 21.2, 21.3, 0.1], 'S2': [24.7, 23.5, 23.5, 23.5, 0.1], 'S3': [28.4, 25.5, 25.7, 25.6, 0.1]}


def desk_times(path=HERE / 'results' / 'desk_times.json'):
    try:
        d = json.load(open(path)); return d['alone'], d.get('two_at_once', d['alone']), d.get('source', str(path))
    except Exception:
        return STAGE5_DESK, STAGE5_DESK, 'Stage 5 dry-run times (default; two at once = alone: not yet measured)'


def sequential(ctrls, desk, sw, solo):
    """list of segments (start_min, end_min, kind, controller) for the controllers one after the other."""
    t = 0.0; seg = []
    def add(d, kind, c):
        nonlocal t; seg.append((t, t + d, kind, c)); t += d
    for c in ctrls:
        add(sw, 'switch', c); tests_default = N_TEST // 2
        for l in range(N_ROUNDS):
            add(2 * FLY_S / 60, 'fly', c); add(solo, 'solo relaunch', c); add(2 * FLY_S / 60, 'fly', c)
            wait = desk[c][l]
            if l == 0 and tests_default:                                            # default-gain tests in the wait of round 0
                d = min(wait, tests_default * FLY_S / 60); add(d, 'test', c); wait -= d
                if wait > 0: add(wait, 'idle', c)
            elif wait > 0: add(wait, 'idle', c)
        add(N_TEST // 2 * FLY_S / 60, 'test', c)                                    # theta_best tests
    return seg


def interleaved(pair, desk, sw, solo):
    A, B = pair; t = 0.0; seg = []; nxt = {A: 0, B: 0}; ready = {A: 0.0, B: 0.0}; loaded = None; tests_left = {A: N_TEST // 2, B: N_TEST // 2}; best_tests = {A: N_TEST // 2, B: N_TEST // 2}; done_desk = {A: False, B: False}
    def add(d, kind, c):
        nonlocal t; seg.append((t, t + d, kind, c)); t += d
    order = [A, B]; i = 0
    while nxt[A] < N_ROUNDS or nxt[B] < N_ROUNDS:
        c = order[i % 2]; i += 1
        if nxt[c] >= N_ROUNDS:
            continue
        if loaded != c:
            add(sw, 'switch', c); loaded = c
        while ready[c] > t and tests_left[c] > 0 and (ready[c] - t) >= FLY_S / 60:       # wait for the desk step: fill with default-gain tests of the loaded controller
            add(FLY_S / 60, 'test', c); tests_left[c] -= 1
        if ready[c] > t: add(ready[c] - t, 'idle', c)
        l = nxt[c]; add(2 * FLY_S / 60, 'fly', c); add(solo, 'solo relaunch', c); add(2 * FLY_S / 60, 'fly', c)
        ready[c] = t + desk[c][l]; nxt[c] += 1
        if nxt[c] >= N_ROUNDS: done_desk[c] = True
    for c in (A, B):                                                               # theta_best tests (the controller whose last desk step is ready first goes first)
        pass
    for c in sorted((A, B), key=lambda x: ready[x]):
        if loaded != c: add(sw, 'switch', c); loaded = c
        while tests_left[c] > 0 and ready[c] > t: add(FLY_S / 60, 'test', c); tests_left[c] -= 1
        if ready[c] > t: add(ready[c] - t, 'idle', c)
        for _ in range(tests_left[c] + best_tests[c]): add(FLY_S / 60, 'test', c)
        tests_left[c] = 0
    return seg


def summarise(seg):
    tot = seg[-1][1]; by = {}
    for a, b, k, c in seg: by[k] = by.get(k, 0.0) + (b - a)
    n_fl = round((by.get('fly', 0) + by.get('test', 0)) * 60 / FLY_S)
    return dict(hours=tot / 60, minutes=tot, flying_min=by.get('fly', 0) + by.get('test', 0), idle_min=by.get('idle', 0), switch_min=by.get('switch', 0) + by.get('solo relaunch', 0), by_kind=by, flights=n_fl, flights_per_hour=n_fl / (tot / 60))


def main():
    alone, two, src = desk_times(); out = dict(desk_source=src, params=dict(fly_s=FLY_S, tuning_flights=N_TUNE, rounds=N_ROUNDS, test_flights=N_TEST), desk_alone=alone, desk_two_at_once=two, results={}, segments={})
    rows = ['| overhead (switch and --solo relaunch) | mode | session | hours | flying min | idle min | switch + relaunch min | flights per hour |', '|---|---|---|---|---|---|---|---|']
    for ov in (3, 5, 10):
        res = {}
        for pair in (('S0', 'S2'), ('S1', 'S3')):
            seq = sequential(pair, alone, ov, ov); itl = interleaved(pair, two, ov, ov); name = '%s + %s' % pair
            res[name] = dict(sequential=summarise(seq), interleaved=summarise(itl)); out['segments']['%d_%s_sequential' % (ov, name)] = seq; out['segments']['%d_%s_interleaved' % (ov, name)] = itl
            for mode, s in (('sequential', res[name]['sequential']), ('interleaved', res[name]['interleaved'])):
                rows.append('| %d min | %s | %s | %.2f | %.0f | %.0f | %.0f | %.1f |' % (ov, mode, name, s['hours'], s['flying_min'], s['idle_min'], s['switch_min'], s['flights_per_hour']))
        tot = {m: sum(res[n][m]['hours'] for n in res) for m in ('sequential', 'interleaved')}; res['all four'] = tot
        rows.append('| %d min | sequential | all four | %.2f | | | | |' % (ov, tot['sequential'])); rows.append('| %d min | interleaved | all four | %.2f | | | | |' % (ov, tot['interleaved']))
        out['results'][str(ov)] = res
    md = '\n'.join(['Desk times: ' + src, ''] + rows) + '\n\nNot stated (not in the repository, not measured): the switch overhead (3 / 5 / 10 min is a parameter), the extra --solo relaunch time (same parameter), flights per battery pack and packs per hour, which PC runs the loops in the lab.\n'
    (HERE / 'results').mkdir(exist_ok=True); json.dump(out, open(HERE / 'results' / 'session_timeline.json', 'w'), indent=1); (HERE / 'results' / 'session_timeline.md').write_text(md); print(md)
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    except Exception as e:
        print('no matplotlib (%s): table only; run with ~/.pyenv/versions/flying_robots/bin/python for the figure' % e); return
    col = {'fly': '#0072B2', 'test': '#56B4E9', 'switch': '#D55E00', 'solo relaunch': '#E69F00', 'idle': '#D9D9D9'}
    ov = 5; fig, axs = plt.subplots(4, 1, figsize=(11, 5.2), sharex=True)
    for ax, (key, title) in zip(axs, [('S0 + S2_sequential', 'S0 + S2 sequential'), ('S0 + S2_interleaved', 'S0 + S2 interleaved'), ('S1 + S3_sequential', 'S1 + S3 sequential'), ('S1 + S3_interleaved', 'S1 + S3 interleaved')]):
        for a, b, k, c in out['segments']['%d_%s' % (ov, key)]:
            ax.barh(0, (b - a) / 60, left=a / 60, color=col[k], edgecolor='white', linewidth=0.4, height=0.6)
        end = out['segments']['%d_%s' % (ov, key)][-1][1] / 60; ax.set_yticks([]); ax.set_ylabel(title, rotation=0, ha='right', va='center', fontsize=8); ax.text(end + 0.05, 0, '%.2f h' % end, va='center', fontsize=8)
        for s_ in ('top', 'right', 'left'): ax.spines[s_].set_visible(False)
    axs[-1].set_xlabel('hours from the start of the session (switch and --solo relaunch %d min each, not measured)' % ov)
    axs[0].legend(handles=[plt.Rectangle((0, 0), 1, 1, color=col[k]) for k in col], labels=['flying a tuning round', 'test flights', 'strategy switch', '--solo relaunch', 'drones idle'], ncol=5, fontsize=7, frameon=False, loc='lower left', bbox_to_anchor=(0, 1.0))
    (HERE / 'figs').mkdir(exist_ok=True); plt.tight_layout(); plt.savefig(HERE / 'figs' / 'session_timeline.png', dpi=140); print('figure written')


if __name__ == '__main__':
    main()
