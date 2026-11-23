#!/usr/bin/env python3
"""Stage 6, step 4 (PROPOSED, no hardware): session tool for an INTERLEAVED lab session of two DiffCoTune controllers (e.g. S0 + S2). Desk side only: it reads files and starts `round.py` in the background;
it never connects to a drone or a radio and never writes to crazyswarm2.

  session.py status   --pair S0 S2 [--root lab] [--logs <root of the round log folders>] [--timeline-min S0=20,S2=24]
        reads lab/<C>/ (round_<l>/state.json = the round to fly, round_<l+1>/state.json = desk step of round l done) and <logs>/<C>/round_<l>/flights.json (flights of the round registered),
        prints what to do next: "fly S2 round 1: A1, A8, then --solo CIRCLE, FIG8", "S0 round 1 desk ready at hh:mm", the strategy switch commands, and the uSD card count.
  session.py desk     --controller S0 --round 1 --logs <folder> [--now]   starts `round.py` for the controller in the background (nice -n 19) once the round's 4 tuning flights are registered; writes lab/<C>/desk_<l>.json
  session.py manifest --controller S0 --round 1 --inbox <folder of merged files> --out <folder>/flights.json --assign A1=<file> A8=<file> CIRCLE=<file> FIG8=<file> [test entries as A4c1=<file> ...]
        explicit assignment of every file to scenario / round / controller (never "the files that arrived together"), refuses a file that is already registered for another controller or round (registry lab/session_registry.json),
        checks that the files of one round are in the card order by modification time (A1, A8, CIRCLE, FIG8) and writes flights.json for round.py.
  session.py switch   --to S2 [--solo]    prints the exact command sequence of a strategy switch (from the existing tools; nothing is executed)
  session.py cards    --start-count <files on the card now> [--pair S0 S2] prints the uSD file budget of the session (archive rule >= 85 files) and when to archive

Everything below is copied from the S0 lab card / the existing tools; what is not stated anywhere is marked "not stated"."""
import argparse, json, os, subprocess, sys, time
from pathlib import Path
HERE = Path(__file__).resolve().parent
TUNING_ORDER = [('A1', 'two drones'), ('A8', 'two drones'), ('A4', 'two drones'), ('CIRCLE', '--solo'), ('FIG8', '--solo')]          # Stage 7: A4 cell 1 is the third tuning flight (user decision 15 Nov)
SWITCH = {'S0': (0, 0), 'S1': (3, 0), 'S2': (0, 1), 'S3': (3, 1)}
ARCHIVE_AT = 85; HARD_LIMIT = 100          # memory feedback_usd_card_file_counter_100_2026-10-19: archive at >= 85, never reach 100
FILES_PER_FLIGHT_PER_CARD = 1             # ASSUMPTION, not stated in the repository (one log file per flight and drone); check at the first block
TEST_FLIGHTS = 12; TUNING_FLIGHTS = 30     # per controller (6 rounds x 5 since Stage 8; test: A4 cell 2 3 + 3, A4 cell 1 3 + 3)


def rounds_state(root, c):
    """(round to fly next, rounds with desk step done) from lab/<C>/round_<l>/state.json and report.json."""
    sd = Path(root) / c
    st = sorted(int(p.name.split('_')[1]) for p in sd.glob('round_*') if (p / 'state.json').exists()) if sd.exists() else []
    return (max(st) if st else None), st


def desk_info(root, c, l):
    p = Path(root) / c / ('desk_%d.json' % l)
    if not p.exists(): return None
    d = json.load(open(p))
    try: os.kill(d['pid'], 0); d['alive'] = True
    except OSError: d['alive'] = False
    return d


def desk_done(root, c, l, n_rounds=5):
    sd = Path(root) / c
    return (sd / ('round_%d' % (l + 1)) / 'state.json').exists() or (sd / ('round_%d' % l) / 'state_after.json').exists()          # Stage 8d2: state_after.json = the last round (also after extension rounds)


def flights_registered(logs, c, l):
    f = Path(logs) / c / ('round_%d' % l) / 'flights.json'
    if not f.exists(): return None
    m = json.load(open(f)); tun = {x['scenario'] for x in m if x['role'] == 'tuning'}
    return m if all(s in tun for s, _ in TUNING_ORDER) else None


def _method_args(state_json):
    """Protocol v2 (23 Nov): the method of the round is the one written in its state.json (settings.method); v1 states say 'ours' = the round.py default, so nothing is added for them."""
    try: m = json.load(open(state_json)).get('settings', {}).get('method', 'ours')
    except OSError: return []
    return [] if m == 'ours' else ['--method', m]


def hhmm(t): return time.strftime('%H:%M', time.localtime(t))


def switch_commands(to, solo=False, root='lab', yaml='<crazyflies.yaml>', uri='<cf5 uri>', frag=None):
    indi, en = SWITCH[to]
    seq = ['stop CS2 (the radio is shared)',
           'python3 flying_drone_stack/tools/set_tree_strategy.py %s --yaml %s%s     # cf5 indi %d, rnn.en %d%s' % (to, yaml, ' --solo' if solo else '', indi, en, '; cf_second enabled: false' if solo else '; cf_second enabled')]
    if frag: seq.append('python3 experiments/diffcotune/lab/apply_gains.py merge --fragment %s --yaml %s     # the 15 gains of this controller\'s round; put the .new file in place' % (frag, yaml))
    seq += ['python3 flying_drone_stack/tools/lab_check_strategy.py     # read-only: the yaml CS2 will read describes strategy %s' % to,
            'launch CS2 (applies the yaml firmware_params to the drones)',
            'stop CS2 again; python3 experiments/diffcotune/lab/apply_gains.py readback --uri %s --fragment %s     # 15 gains + stabilizer.controller 9, indi %d, rnn.en %d from the firmware, fresh connection (S0 lab card, per-round items 1 and 2)' % (uri, frag or '<fragment>', indi, en),
            'relaunch CS2; read the `Update parameter "rnn.en"` lines of cf5 in the launch output']
    if to in ('S2', 'S3'):
        seq.append('S2 / S3 beyond S0: rnn.en 1 on cf5 (NS2 in the controller); a neighbour (cf_second) enabled for A1 / A8 (two-drone flights), disabled with --solo for CIRCLE / FIG8. NOT STATED in the tools: NS2 weights '
                   'are part of the flashed image (study_tree_v2.bin per the docs; check_active_controller.py only after a flash), `res_sign` and `rnn.ready` need no action here')
    if to in ('S1', 'S3'): seq.append('S1 / S3: INDI reads DShot RPM (motor.mN_rpm) since the DShot image (2026-11-22), needs no deck.bcRpm; lab_run.sh prep checks the log variables')
    return seq


def cmd_status(a):
    now = time.time(); tl = dict(kv.split('=') for kv in a.timeline_min.split(',')) if a.timeline_min else {}
    lines = []; nxt = {}
    for c in a.pair:
        l, done = rounds_state(a.root, c)
        if l is None: lines.append('%s: not initialised (python3 lab/round.py --init --controller %s --state-dir %s/%s)' % (c, c, a.root, c)); continue
        reg = flights_registered(a.logs, c, l); dk = desk_info(a.root, c, l)
        if (Path(a.root) / c / ('round_%d' % l) / 'state_after.json').exists():          # Stage 8d2: the last round (planned or extension) has its desk step done
            if (Path(a.root) / c / ('round_%d' % l) / 'gains_theta_best.yaml').exists(): lines.append('%s: all %d rounds done (extension rounds: %d); gains_theta_best.yaml in %s/%s/round_%d/ ; next: test flights with theta_best' % (c, l + 1, max(0, l + 1 - a.n_rounds), a.root, c, l))
            else: lines.append('%s: no eligible set after extension: ask the user (round_%d/NO_ELIGIBLE_SET.md); no final gains file' % (c, l))
            continue
        if dk and dk['alive']: eta = dk['start'] + 60 * dk['expected_min']; lines.append('%s round %d desk RUNNING (pid %d), ready at about %s (expected %.0f min, estimate)' % (c, l, dk['pid'], hhmm(eta), dk['expected_min']))
        elif reg is not None and not desk_done(a.root, c, l, a.n_rounds): lines.append('%s round %d: flights registered, desk step not started: run `session.py desk --controller %s --round %d --logs %s/%s/round_%d`' % (c, l, c, l, a.logs, c, l))
        elif reg is None: nxt[c] = l; lines.append('fly %s round %d: A1, A8, then --solo CIRCLE, FIG8' % (c, l))
    out = ['== session status (%s) ==' % hhmm(now)] + lines
    if nxt:
        fly = [x for x in a.pair if x in nxt]; pick = [x for x in fly if x != a.loaded] or fly
        out.append('NEXT for the drones: %s round %d (loaded now: %s)' % (pick[0], nxt[pick[0]], a.loaded or 'unknown'))
        if pick[0] != a.loaded:
            out.append('switch:'); out += ['   ' + s for s in switch_commands(pick[0], solo=False, root=a.root, frag='%s/%s/round_%d/gains.yaml' % (a.root, pick[0], nxt[pick[0]]))]
    elif all(rounds_state(a.root, c)[0] == a.n_rounds for c in a.pair if rounds_state(a.root, c)[0] is not None):
        out.append('NEXT: test flights with theta_best (A4 cell 1 and 2, 3 flights each) for each controller')
    else: out.append('NEXT: wait for the desk steps above (drones: test flights with the default gains of the loaded controller if some are left)')
    print('\n'.join(out)); return out


def cmd_desk(a):
    l = a.round; reg = flights_registered(Path(a.logs).parent.parent, a.controller, l) if a.logs else None
    need = Path(a.logs) / 'flights.json'
    if not need.exists(): sys.exit('no flights.json in %s' % a.logs)
    if not a.now and not flights_registered(Path(a.logs).parent.parent, a.controller, l): sys.exit('the tuning flights of round %d are not registered (manifest); use --now to start anyway' % l)
    cmd = ['nice', '-n', '19', sys.executable, str(HERE / 'round.py'), '--controller', a.controller, '--round', str(l), '--logs', a.logs, '--state-dir', str(Path(a.root) / a.controller), '--n-rounds', str(a.n_rounds)] + (['--top', a.top] if a.top else []) + (['--lenient'] if getattr(a, 'lenient', False) else []) + _method_args(Path(a.root) / a.controller / ('round_%d' % l) / 'state.json')
    logf = Path(a.root) / a.controller / ('desk_%d.log' % l); logf.parent.mkdir(parents=True, exist_ok=True)
    if a.print_only: print(' '.join(cmd)); return
    p = subprocess.Popen(cmd, stdout=open(logf, 'w'), stderr=subprocess.STDOUT, start_new_session=True)
    json.dump(dict(controller=a.controller, round=l, pid=p.pid, start=time.time(), expected_min=a.expected_min, log=str(logf), cmd=cmd), open(Path(a.root) / a.controller / ('desk_%d.json' % l), 'w'), indent=1)
    print('started desk step of %s round %d (pid %d, log %s); expected about %.0f min (estimate)' % (a.controller, l, p.pid, logf, a.expected_min))


def cmd_manifest(a):
    reg_f = Path(a.root) / 'session_registry.json'; reg = json.load(open(reg_f)) if reg_f.exists() else {}
    inbox = Path(a.inbox); man = []; mt = []
    for kv in a.assign:
        key, f = kv.split('=', 1); p = (inbox / f) if not Path(f).is_absolute() else Path(f)
        if not p.exists(): sys.exit('file not found: %s' % p)
        rk = str(p.resolve()); prev = reg.get(rk)
        if prev and (prev['controller'], prev['round']) != (a.controller, a.round): sys.exit('%s is already registered for %s round %d: refused' % (p.name, prev['controller'], prev['round']))
        if key.startswith('A4c'): scen, cell, role = 'A4', int(key[3:]) - 1, 'test'
        else: scen, cell, role = key, 0, 'tuning'
        att = 1 + sum(1 for m in man if m['scenario'] == scen and m['cell'] == cell and m['role'] == role)
        rot = {'CIRCLE': -90.0, 'FIG8': -90.0, 'A4': -90.0}.get(scen)
        if getattr(a, 'fake_frame', False): rot = 0.0          # mock test only: the fake files are written in the model frame, already in time
        man.append(dict(file=str(p), scenario=scen, cell=cell, role=role, attempt=att, rotate_deg=rot, lag_s=(0 if getattr(a, 'fake_frame', False) else None))); mt.append((scen, role, p.stat().st_mtime, p.name))
    for kv in (getattr(a, 'fallback', None) or []):          # Stage 8d2 (user decision 16 Nov): rule R3 fallback flight = ONE flight per R3 scenario with theta_best, role fallback, attempt 4, not used for the fit
        key, f = kv.split('=', 1); p = (inbox / f) if not Path(f).is_absolute() else Path(f)
        if not p.exists(): sys.exit('file not found: %s' % p)
        if not a.fallback_gains: sys.exit('--fallback needs --fallback-gains <the gains.yaml of theta_best, e.g. lab/<C>/round_K/gains.yaml>')
        rk = str(p.resolve()); prev = reg.get(rk)
        if prev and (prev['controller'], prev['round']) != (a.controller, a.round): sys.exit('%s is already registered for %s round %d: refused' % (p.name, prev['controller'], prev['round']))
        if any(m['scenario'] == key and m['role'] == 'fallback' for m in man): sys.exit('only ONE fallback flight per scenario (%s)' % key)
        man.append(dict(file=str(p), scenario=key, cell=0, role='fallback', attempt=4, rotate_deg={'CIRCLE': -90.0, 'FIG8': -90.0, 'A4': -90.0}.get(key) if not getattr(a, 'fake_frame', False) else 0.0, lag_s=(0 if getattr(a, 'fake_frame', False) else None), gains=str(a.fallback_gains)))
        mt.append((key, 'fallback', p.stat().st_mtime, p.name))
    order = [s for s, _ in TUNING_ORDER]; tun = [x for x in mt if x[1] == 'tuning' and x[0] in order]
    first = {}
    for s, r, t, n in tun: first.setdefault(s, t)
    seq = sorted(first, key=lambda s: first[s]); warn = [] if seq == [s for s in order if s in first] else ['WARNING: modification times are not in the card order %s (got %s): check the assignment' % (order, seq)]
    if getattr(a, 'append', False) and Path(a.out).exists(): man = [m for m in json.load(open(a.out)) if not (m['role'] == 'fallback' and m['scenario'] in {x['scenario'] for x in man if x['role'] == 'fallback'})] + man          # --append: add the fallback entries to the flights.json of the round (the round's tuning flights are already in it)
    for m in man: reg[str(Path(m['file']).resolve())] = dict(controller=a.controller, round=a.round, scenario=m['scenario'], role=m['role'], time=time.time())
    reg_f.parent.mkdir(parents=True, exist_ok=True); json.dump(reg, open(reg_f, 'w'), indent=1)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True); json.dump(man, open(a.out, 'w'), indent=1); print('wrote %s (%d flights)' % (a.out, len(man))); [print(w) for w in warn]


def cmd_cards(a):
    per_ctrl = TUNING_FLIGHTS + TEST_FLIGHTS; n_ctrl = len(a.pair) if a.pair else 1; flights = per_ctrl * n_ctrl * FILES_PER_FLIGHT_PER_CARD; rep = int(0.0)
    print('uSD file budget (assumption: %d file per flight and card, not stated in the repository; repeats not counted):' % FILES_PER_FLIGHT_PER_CARD)
    print('  per controller: %d tuning + %d test flights = %d files per card; %d controller(s) in the session = %d files' % (TUNING_FLIGHTS, TEST_FLIGHTS, per_ctrl, n_ctrl, flights))
    print('  start count %d + %d = %d at the end (archive rule: at >= %d, never reach %d)' % (a.start_count, flights, a.start_count + flights, ARCHIVE_AT, HARD_LIMIT))
    if a.start_count + flights >= ARCHIVE_AT:
        k = ARCHIVE_AT - a.start_count
        print('  ARCHIVE the cards after flight %d of the session (count %d), i.e. before it crosses %d; an interleaved pair doubles the files per session' % (max(k, 0), a.start_count + max(k, 0), ARCHIVE_AT))
    else: print('  no archive needed during the session')
    print('  files only count; the file-to-flight assignment is explicit (manifest + registry), never by arrival time')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0]); sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('status'); p.add_argument('--pair', nargs='+', default=['S0', 'S2']); p.add_argument('--root', default=str(HERE)); p.add_argument('--logs', default=str(HERE / 'session_logs')); p.add_argument('--loaded', default=None)
    p.add_argument('--n-rounds', type=int, default=6); p.add_argument('--timeline-min', default=None)
    p = sub.add_parser('desk'); p.add_argument('--lenient', action='store_true', help='mock test only (round.py --lenient)'); p.add_argument('--controller', required=True); p.add_argument('--round', type=int, required=True); p.add_argument('--logs', required=True); p.add_argument('--root', default=str(HERE))
    p.add_argument('--n-rounds', type=int, default=6); p.add_argument('--top', default=None); p.add_argument('--expected-min', type=float, default=23.0); p.add_argument('--now', action='store_true'); p.add_argument('--print-only', action='store_true')
    p = sub.add_parser('manifest'); p.add_argument('--fake-frame', action='store_true', help='mock test only: rotate_deg 0 and lag_s 0 (files of fake_round_flights.py)'); p.add_argument('--controller', required=True); p.add_argument('--round', type=int, required=True); p.add_argument('--inbox', required=True); p.add_argument('--out', required=True); p.add_argument('--root', default=str(HERE)); p.add_argument('--assign', nargs='*', default=[]); p.add_argument('--fallback', nargs='+', default=None, metavar='SCEN=file', help='rule R3: the ONE fallback flight per failed scenario (theta_best), role fallback, attempt 4'); p.add_argument('--fallback-gains', default=None, help='gains.yaml of theta_best used for the fallback flight'); p.add_argument('--append', action='store_true', help='add the fallback entries to the existing --out flights.json')
    p = sub.add_parser('switch'); p.add_argument('--to', required=True, choices=list(SWITCH)); p.add_argument('--solo', action='store_true'); p.add_argument('--yaml', default='<crazyflies.yaml>'); p.add_argument('--uri', default='<cf5 uri>'); p.add_argument('--fragment', default=None)
    p = sub.add_parser('cards'); p.add_argument('--start-count', type=int, required=True); p.add_argument('--pair', nargs='+', default=None)
    a = ap.parse_args()
    if a.cmd == 'status': cmd_status(a)
    elif a.cmd == 'desk': cmd_desk(a)
    elif a.cmd == 'manifest': cmd_manifest(a)
    elif a.cmd == 'switch': print('\n'.join(switch_commands(a.to, a.solo, yaml=a.yaml, uri=a.uri, frag=a.fragment)))
    elif a.cmd == 'cards': cmd_cards(a)


if __name__ == '__main__':
    main()
