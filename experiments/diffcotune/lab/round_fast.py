#!/usr/bin/env python3
"""Stage 4a C2: one command per DiffCoTune round on the real drone (desk side; nothing is flown, flashed or written to crazyswarm2).

  round.py --init                        --controller S0 [--state-dir lab/S0]        writes round_0 (default gains yaml + state)
  round.py --controller S0 --round l --logs <folder>  [--state-dir lab/S0] [--n-rounds 5] [--grad-mode reverse|forward]

Folder layout (state-dir = experiments/diffcotune/lab/S0):
  round_<l>/gains.yaml        gains FLOWN in round l (written by round l-1; defaults for l = 0), crazyswarm2 yaml fragment, see GAIN_PARAMS
  round_<l>/state.json        state before round l: theta_l (phi, log-factors of the 10 gains), beta (psi, log-factors), keep-best, settings, seeds, history
  round_<l>/report.md|json    written by this command (real cost per scenario, flags, beta, theta, keep-best, desk time)
  round_<l+1>/gains.yaml, state.json   written by this command (not after the last round)
--logs <folder> holds the merged uSD files of the round and `flights.json`: a list of {file, scenario, cell (0-based index into the tuning / study cells), role (tuning | test),
attempt (1 = first flight, 2 / 3 = repeat after a crash or floor flight)}. Tuning scenarios of S0: A1, A8, CIRCLE, FIG8 (loop_lib.TUNING); test: A4 (2 cells), OVAL.

One round = ingest + quality gate -> real cost -> fit beta (50 + 1 epochs, noise-free J_sysId = position + velocity + attitude, best iterate; skipped in the last round) -> tune theta
(method version "A + B": J_task with 2 fixed noise seeds per round, +-25 % gain limit around theta_l, 50 + 1 epochs, best iterate, lr 0.01, Adam reset per inner loop) -> next gains.
A flight that fails the quality gate, tilts over (tilt >= 70 deg) or stays on the floor (floor rule) is not used; the scenario then needs a repeat (at most 2 repeats, attempt <= 3).
"""
import argparse, json, os, sys, time
from pathlib import Path
import numpy as np
os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
if '--controller' in sys.argv:            # Stage 5: the controller selects the model of loop_lib BEFORE it is imported (default S0 = Stage 4 behaviour)
    os.environ['DIFFCOTUNE_CONTROLLER'] = sys.argv[sys.argv.index('--controller') + 1]
# Stage 5b: gradient mode of the model (loop_lib.GRAD_MODE; same autodiff gradient, other differentiation mode). Default per controller, written here: S0 'reverse'; S1, S2, S3 'mixed' since Stage 5c (eager forward mode is exact but about 20-60 x slower, not adopted: dryrun/stage5b/report_stage5b.md; the compiled forward sensitivity of dryrun/stage5c/report_stage5c.md is as fast as reverse). --grad-mode overrides. Set BEFORE loop_lib is imported (spawned workers inherit it).
GRAD_MODE_DEFAULT = dict(S0='reverse', S1='mixed', S2='mixed', S3='mixed')          # Stage 5c: mixed = exact forward sensitivity (compiled) for the noise-free beta fit on A1, compiled reverse elsewhere (S0 unchanged)
_ctrl = sys.argv[sys.argv.index('--controller') + 1] if '--controller' in sys.argv else 'S0'
os.environ['DIFFCOTUNE_GRAD_MODE'] = sys.argv[sys.argv.index('--grad-mode') + 1] if '--grad-mode' in sys.argv else GRAD_MODE_DEFAULT.get(_ctrl, 'reverse')
# Stage 5c, RPM delay fixed (user decision, 14 Nov 2026): S1 / S3 keep the RPM delay at the calibration (4.0502 ms) and fit the other four beta (loop_lib.FIX_RD). The table is True for S1 and S3 (flipped 15 Nov after the Stage 5c round tests with the delay fitted had run; switch: --fix-rd / --fit-rd). Set BEFORE loop_lib is imported.
FIX_RD_DEFAULT = dict(S0=False, S1=True, S2=False, S3=True)
os.environ['DIFFCOTUNE_FIX_RD'] = '1' if '--fix-rd' in sys.argv else ('0' if '--fit-rd' in sys.argv else ('1' if FIX_RD_DEFAULT.get(_ctrl, False) else '0'))
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun'))
import loop_lib as L  # noqa
import lab_pool_fast as LP  # noqa
import usd_ingest as U  # noqa
import reference as R  # noqa
from frsim.scenarios import study_cells, tuning_cells  # noqa

# gain of the model -> firmware parameter(s) of the ctrlOmarIndi group (controller_omar_indi.c PARAM_GROUP ctrlOmarIndi, lines 679-717 of crazyflie-firmware); x and y tied
GAIN_PARAMS = [('Pxy', ['Kpos_Px', 'Kpos_Py']), ('Pz', ['Kpos_Pz']), ('Dxy', ['Kpos_Dx', 'Kpos_Dy']), ('Dz', ['Kpos_Dz']), ('Iz', ['Kpos_Iz']),
               ('KRxy', ['KR_x', 'KR_y']), ('KRz', ['KR_z']), ('Kwxy', ['Kw_x', 'Kw_y']), ('Kwz', ['Kw_z']), ('KIxy', ['KI_x', 'KI_y'])]
ROBOT = 'cf5'          # the study drone (crazyflies.yaml: cf5 = bottom, scored)
# the peer input of the S0 model: the LOGGED top-drone path for the two-drone tuning scenarios (decision 13 Nov: on 23 Oct A1 it moves the model from 10.1 to 20.4-20.7 cm against the real 20.5-22.1 cm)
TOP_FROM_LOG = ('A1', 'A8')
SETTINGS = dict(lr=0.01, epochs=50, best_iterate=True, states=['p', 'v', 'q'], gain_limit=0.25, noise_seed_base=6000, n_noise_seeds=2, skip_last_fit=True, n_rounds=5, grad_mode=os.environ.get('DIFFCOTUNE_GRAD_MODE', 'reverse'), fix_rd=os.environ.get('DIFFCOTUNE_FIX_RD', '0') == '1')


def say(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


# ------------------------------------------------------------------------------------------------------------- yaml
def gains_to_yaml(phi, path, header=''):
    """theta -> crazyswarm2 yaml fragment (robots: cf5: firmware_params: ctrlOmarIndi: ...). Values written with repr (float64); the firmware stores float32."""
    g = L.gains_of(np.asarray(phi, float))
    lines = ['# DiffCoTune %s gains to fly (Stage 4a / 5). Merge into the cf5 block of the study_%s crazyflies.yaml; written by lab/round.py, not applied by it.' % (L.CTRL, L.CTRL)]
    lines += ['# ' + h for h in header.splitlines() if h]
    lines += ['robots:', '  %s:' % ROBOT, '    firmware_params:', '      ctrlOmarIndi:']
    for name, params in GAIN_PARAMS:
        for p in params:
            lines.append('        %s: %r' % (p, float(g[name])))
    Path(path).write_text('\n'.join(lines) + '\n')


def yaml_to_phi(path):
    """yaml fragment -> theta (phi = log(gain / default)); the x / y pairs must be equal."""
    import yaml
    d = yaml.safe_load(open(path))['robots'][ROBOT]['firmware_params']['ctrlOmarIndi']
    phi = np.zeros(10)
    for i, (name, params) in enumerate(GAIN_PARAMS):
        vals = [float(d[p]) for p in params]
        assert max(vals) == min(vals), 'tied gains differ: %s' % params
        phi[i] = np.log(vals[0] / L.GAIN_DEFAULT[i])
    return phi


# ------------------------------------------------------------------------------------------------------------- state
def init_state(sd):
    rd = Path(sd) / 'round_0'; rd.mkdir(parents=True, exist_ok=True)
    st = dict(round=0, phi=[0.0] * 10, psi=[0.0] * L.N_BETA, keep=None, history=[], settings=SETTINGS, controller=L.CTRL)
    json.dump(st, open(rd / 'state.json', 'w'), indent=1); gains_to_yaml(np.zeros(10), rd / 'gains.yaml', 'round 0: firmware default gains')
    return st


def cell_of(scen, role, idx):
    return (tuning_cells(scen) if role == 'tuning' else study_cells(scen))[idx]


# ------------------------------------------------------------------------------------------------------------- the round
PERF_DEFAULT = dict(fit_workers=4, fit_threads=2, tune_workers=4, tune_threads=2, warm=False, warm_pool=None)      # Stage 6 levers; defaults = the Stage 5c behaviour


def run_round(l, folder, sd, top='auto', strict=True, perf=None, pool_pre=None):
    perf = dict(PERF_DEFAULT, **(perf or {})); t_start = time.time(); sd = Path(sd); folder = Path(folder); rd = sd / ('round_%d' % l)
    st = json.load(open(rd / 'state.json')); S = st['settings']; assert st['round'] == l, 'state is for round %d' % st['round']
    phi_l = np.array(st['phi']); psi = np.array(st['psi']); last = l == S['n_rounds'] - 1
    man = json.load(open(folder / 'flights.json'))
    # 1. ingest + gate + selection
    t0 = time.time(); ing = []
    for m in man:
        cell = cell_of(m['scenario'], m['role'], m['cell'])
        use_top = (top == 'logged') or (top == 'auto' and m['scenario'] in TOP_FROM_LOG)
        g = U.ingest_flight(folder / m['file'], m['scenario'], cell, top_from_log=use_top, rotate_deg=m.get('rotate_deg'), lag_s=m.get('lag_s'))
        g.update(m=m, cell=cell); ing.append(g)
        say('ingest %-40s %s attempt %d: gate %s, crashed %s floor %s, real %.2f cm' % (m['file'], m['scenario'], m['attempt'], 'ok' if g['gate']['ok'] else g['gate']['reasons'],
            g.get('flags', {}).get('crashed'), g.get('flags', {}).get('floor'), g.get('rms3d_cm', float('nan'))))
    def good(g):
        if not strict: return g.get('usable', False)      # strict=False only for the dry test: loop_lib fits a floor flight of theta_0 (A1) too
        return g.get('usable', False) and not g['flags']['crashed'] and not g['flags']['floor']
    pick = {}
    for s, c in L.TUNING:
        cands = sorted([g for g in ing if g['m']['role'] == 'tuning' and g['scenario'] == s and g['m']['attempt'] <= 3], key=lambda g: g['m']['attempt'])
        ok = [g for g in cands if good(g)]
        pick[s] = ok[0] if ok else None
    missing = [s for s, g in pick.items() if g is None]
    seconds_ingest = time.time() - t0
    flights = [dict(scenario=g['scenario'], file=g['m']['file'], attempt=g['m']['attempt'], role=g['m']['role'], gate_ok=g['gate']['ok'], gate_reasons=g['gate']['reasons'],
                    rows=g['gate']['rows'], rate_hz=g['gate']['rate_hz'], gap_fraction=g['gate']['gap_fraction'], frame=g.get('frame'), reference_check=g.get('reference_check'),
                    crashed=g.get('flags', {}).get('crashed'), floor=g.get('flags', {}).get('floor'), tilt_max_deg=g.get('flags', {}).get('tilt_max_deg'),
                    rms3d_cm=g.get('rms3d_cm'), used=any(g is p for p in pick.values())) for g in ing]
    rep = dict(round=l, controller=L.CTRL, phi=phi_l.tolist(), psi_before=psi.tolist(), flights=flights, missing=missing, seconds_ingest=seconds_ingest)
    if missing:
        rep['status'] = 'no usable flight for %s: repeat (attempt <= 3), nothing fitted' % missing
        rd.mkdir(exist_ok=True); json.dump(rep, open(rd / 'report.json', 'w'), indent=1); (rd / 'report.md').write_text('# Round %d: %s\n' % (l, rep['status'])); say(rep['status'])
        return rep
    fl = [dict(cost=pick[s]['cost_m2'], z_cost=pick[s]['z_cost_m2'], crashed=False, floor=False, tries=pick[s]['m']['attempt'] - 1) for s, _ in L.TUNING]
    rc = L.round_cost(fl); rep['real'] = rc
    rep['per_scenario_cm'] = {s: pick[s]['rms3d_cm'] for s, _ in L.TUNING}
    rep['test_cm'] = {'%s[%d]' % (g['scenario'], g['m']['cell']): g['rms3d_cm'] for g in ing if g['m']['role'] == 'test' and g.get('usable')}
    keep = st['keep']
    if keep is None or rc['cost'] <= keep['cost']:
        keep = dict(cost=rc['cost'], rms3d_cm=rc['rms3d_cm'], round=l, phi=phi_l.tolist())
    rep['keep_best'] = keep; say('real 3-D RMS %.2f cm (mean of the 4 tuning flights); keep-best round %d (%.2f cm)' % (rc['rms3d_cm'], keep['round'], keep['rms3d_cm']))
    x = np.r_[phi_l, psi]
    traces = {i: [pick[s]['trace']] for i, (s, _) in enumerate(L.TUNING)}
    tops = {i: (pick[s_]['ref'].top_pos, pick[s_]['ref'].top_vel) for i, (s_, _) in enumerate(L.TUNING) if pick[s_]['ref'].has_top and (top == 'logged' or (top == 'auto' and s_ in TOP_FROM_LOG))}
    rep['top_path'] = {s_: ('logged' if i in tops else 'planned') for i, (s_, _) in enumerate(L.TUNING)}
    same = (perf['fit_workers'], perf['fit_threads']) == (perf['tune_workers'], perf['tune_threads'])
    nseed = S['n_noise_seeds']; epoch_s = {'fit': [], 'tune': []}
    def timed(kind, fn):
        def g(xx):
            t_ = time.time(); r = fn(xx); epoch_s[kind].append(time.time() - t_); return r
        return g
    def make_pool(workers, threads):
        if pool_pre is not None and pool_pre['key'] == (workers, threads): p_ = pool_pre['pool']; p_.tops = tops; return p_
        return LP.PoolTop(workers, states=tuple(S['states']), tops=tops, threads=threads, split_seeds=(workers >= len(L.TUNING) * nseed), warm=perf['warm'], n_seeds=nseed)
    t_pool = time.time(); pool = make_pool(perf['fit_workers'], perf['fit_threads']); rep['seconds_pool_start'] = time.time() - t_pool
    try:
        t1 = time.time()
        if not (last and S['skip_last_fit']):
            x, rf = L.adam_inner(timed('fit', lambda xx: pool.rollouts('sysid', xx, traces)), x, L.PSI_FIT_IDX, S['lr'], K2=S['epochs'], label='fit beta, round %d' % l, stop=False, best=True)
            rep['fit'] = dict(epochs=rf['epochs'], J0=rf['J0'], J_best=rf['J_best'], best_epoch=rf['best_epoch']); say('fit beta: J0 %.4g -> %.4g (best epoch %d), psi %s' % (rf['J0'], rf['J_best'], rf['best_epoch'], np.round(x[10:], 3).tolist()))
        rep['seconds_fit'] = time.time() - t1; rep['psi_after_fit'] = x[10:].tolist()
        t2 = time.time()
        if not last:
            if not same and not (pool_pre is not None):
                pool.close(); t_pool = time.time(); pool = make_pool(perf['tune_workers'], perf['tune_threads']); rep['seconds_pool_start_tune'] = time.time() - t_pool
            pool.noise_seeds = tuple(S['noise_seed_base'] + S['n_noise_seeds'] * l + j for j in range(S['n_noise_seeds']))
            if pool.split_seeds is False and perf['tune_workers'] >= len(L.TUNING) * nseed: pool.split_seeds = True
            x, rt = L.adam_inner(timed('tune', lambda xx: pool.rollouts('task', xx, None)), x, np.arange(10), S['lr'], K2=S['epochs'], label='tune theta, round %d' % l, stop=False, best=True,
                                 limit=float(np.log(1 + S['gain_limit'])) if S['gain_limit'] else None)
            rep['tune'] = dict(epochs=rt['epochs'], J0=rt['J0'], J_best=rt['J_best'], best_epoch=rt['best_epoch'], hits=rt['hits'], noise_seeds=list(pool.noise_seeds))
            say('tune theta: J0 %.4g -> %.4g (best epoch %d), gain-limit hits %s' % (rt['J0'], rt['J_best'], rt['best_epoch'], rt['hits']))
        rep['seconds_tune'] = time.time() - t2; rep['epoch_seconds'] = epoch_s; rep['perf'] = {k: v for k, v in perf.items() if k != 'warm_pool'}
    finally:
        pool.close()
    phi_next = x[:10]; psi_next = x[10:]
    rep.update(phi_next=None if last else phi_next.tolist(), psi_after=psi_next.tolist(), theta_factors_next=None if last else dict(zip(L.GAIN_NAMES, np.exp(phi_next - phi_l).round(4).tolist())),
               theta_factors_vs_default=dict(zip(L.GAIN_NAMES, np.exp(phi_l).round(4).tolist())), last_round=last, recommendation_if_last=dict(theta_best_round=keep['round']) if last else None)
    rep['seconds_total'] = time.time() - t_start
    st_next = dict(round=l + 1, phi=phi_next.tolist(), psi=psi_next.tolist(), keep=keep, settings=S, controller=L.CTRL,
                   history=st['history'] + [dict(round=l, rms3d_cm=rc['rms3d_cm'], phi=phi_l.tolist(), psi_after_fit=rep['psi_after_fit'])])
    rep['seed_info'] = dict(noise_seed_base=S['noise_seed_base'], note='noise seeds of the tuning step of round l: base + 2 l, base + 2 l + 1')
    rd.mkdir(exist_ok=True); json.dump(rep, open(rd / 'report.json', 'w'), indent=1)
    md = ['# Round %d report (%s, method version "A + B")' % (l, L.CTRL), '', 'Real cost (mean of the 4 tuning flights, 3-D RMS): **%.2f cm**; keep-best: round %d (%.2f cm).' % (rc['rms3d_cm'], keep['round'], keep['rms3d_cm']), '',
          '| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |', '|---|---|---|---|---|---|---|---|']
    for f in flights:
        md.append('| %s | %s | %d | %s | %s | %s | %.2f | %s |' % (f['scenario'], f['file'], f['attempt'], 'ok' if f['gate_ok'] else ';'.join(f['gate_reasons']), f['crashed'], f['floor'], f['rms3d_cm'] or float('nan'), 'yes' if f['used'] else 'no'))
    md += ['', 'Test flights (not used for the update): %s' % rep['test_cm'], '', 'beta (log-factors) after the fit: %s' % np.round(psi_next, 4).tolist(),
           'theta flown (factor to the default gains): %s' % rep['theta_factors_vs_default']]
    if not last:
        md += ['theta of the next round (factor to this round): %s' % rep['theta_factors_next'], 'gain-limit hits per gain (of 51 evaluations): %s' % dict(zip(L.GAIN_NAMES, rep['tune']['hits']))]
    md += ['', 'Desk time of this command: %.0f s (ingest %.0f s, fit %.0f s, tune %.0f s).' % (rep['seconds_total'], seconds_ingest, rep['seconds_fit'], rep['seconds_tune'])]
    (rd / 'report.md').write_text('\n'.join(md) + '\n')
    if not last:
        nd = sd / ('round_%d' % (l + 1)); nd.mkdir(exist_ok=True)
        json.dump(st_next, open(nd / 'state.json', 'w'), indent=1)
        gains_to_yaml(phi_next, nd / 'gains.yaml', 'round %d: theta_%d from round %d (method version A + B, keep-best round %d)' % (l + 1, l + 1, l, keep['round']))
    else:
        json.dump(st_next, open(rd / 'state_after.json', 'w'), indent=1)
        gains_to_yaml(np.array(keep['phi']), rd / 'gains_theta_best.yaml', 'theta_best = theta_%d (keep-best over the flown gain sets)' % keep['round'])
    say('round %d done in %.0f s' % (l, rep['seconds_total']))
    return rep


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--controller', default='S0'); ap.add_argument('--round', type=int); ap.add_argument('--logs'); ap.add_argument('--init', action='store_true')
    ap.add_argument('--state-dir', default=None); ap.add_argument('--n-rounds', type=int, default=None); ap.add_argument('--grad-mode', choices=['reverse', 'forward', 'mixed'], default=None)
    ap.add_argument('--top', default='auto', choices=['auto', 'logged', 'planned'], help='peer path of the S0 model: auto = logged for A1 / A8 (default), planned = frsim top path')
    ap.add_argument('--fix-rd', action='store_true'); ap.add_argument('--fit-rd', action='store_true')
    ap.add_argument('--fit-workers', type=int, default=PERF_DEFAULT['fit_workers']); ap.add_argument('--tune-workers', type=int, default=PERF_DEFAULT['tune_workers'], help='4 (one per scenario, 2 noise rows each) or 8 (one per scenario and noise seed)')
    ap.add_argument('--fit-threads', type=int, default=PERF_DEFAULT['fit_threads']); ap.add_argument('--tune-threads', type=int, default=PERF_DEFAULT['tune_threads'])
    ap.add_argument('--warmup', action='store_true', help='start the worker pool and compile all graphs NOW (while the round is flown), then wait for <logs>/flights.json and run the round in the same pool')
    a = ap.parse_args(); a.state_dir = a.state_dir or str(HERE / a.controller)
    assert a.controller == L.CTRL and a.controller in L.MODEL_MODULES, 'controller must be one of S0 S1 S2 S3'
    if a.n_rounds:
        SETTINGS['n_rounds'] = a.n_rounds
    if a.init:
        init_state(a.state_dir); say('initialised', a.state_dir); return
    perf = dict(fit_workers=a.fit_workers, fit_threads=a.fit_threads, tune_workers=a.tune_workers, tune_threads=a.tune_threads)
    if a.warmup:
        w = max(a.fit_workers, a.tune_workers); th = a.fit_threads; assert a.fit_threads == a.tune_threads, '--warmup keeps one pool: fit and tune threads must be equal'
        t0 = time.time(); pool = LP.PoolTop(w, states=tuple(SETTINGS['states']), threads=th, split_seeds=w >= len(L.TUNING) * SETTINGS['n_noise_seeds'], warm=True, n_seeds=SETTINGS['n_noise_seeds']); pool.wait_ready()
        say('worker pool of %d warm (all graphs compiled) after %.0f s; waiting for %s' % (w, time.time() - t0, Path(a.logs) / 'flights.json'))
        while not (Path(a.logs) / 'flights.json').exists(): time.sleep(5)
        time.sleep(2); perf.update(fit_workers=w, tune_workers=w, warm=True, warm_seconds=time.time() - t0)
        run_round(a.round, a.logs, a.state_dir, a.top, perf=perf, pool_pre=dict(key=(w, th), pool=pool)); return
    run_round(a.round, a.logs, a.state_dir, a.top, perf=perf)


if __name__ == '__main__':
    main()
