#!/usr/bin/env python3
"""Stage 5 B1: forward, noise-gate and gradient checks of the differentiable model of one controller against frsim (generalises checks/forward_check.py, dryrun/stage3d/noise_gate.py and
checks/gradient_check.py of S0). Controller from the environment: DIFFCOTUNE_CONTROLLER=S2|S1|S3 (S0 reproduces the S0 gates). Run with nice -n 19, one job at a time.
  forward  : 6 scenarios (first study cell of A1, A4, A8, CIRCLE, FIG8, OVAL) x 5 parameter rows, noise off in both; pass: relative 3-D RMS difference <= 0.5 % per cell and the same crash flag.
             Rows: default; gains1, gains2 (uniform +-30 %, seeds 1, 2); shift1 = thrust x 0.8, lag 45 ms (S1 / S3: + rpm delay 7.3 ms, fractional); shift2 = downwash x 1.3, plume x 0.7.
             For S1 / S3 frsim runs with the new fractional rpm-delay option (plant value rpm_delay_fractional = 1), the model's delay is fractional by construction.
  noise    : the Stage 3d revised gate, 24 cells = gains {default, round 1, round 3 of the Stage 3b loop (d)} x the 4 tuning scenarios x seeds {1000, 1001} on the true plant of the dry run
             (P1; S1 / S3: P1RD): a cell is stable when frsim moves < 0.5 % under a 1e-6 relative change of the thrust-law gain; stable cells: model within 0.5 %; other cells: the model must show the failure
             (floor rule or 3-D RMS > 3 x the round-1 value of the same scenario and seed). (The gain sets are those of S0's loop: all four controllers share the same 10 gains.)
  gradient : combined J_task and J_sysId (mean over A1, A8, CIRCLE, FIG8, tuning cells) autograd vs central finite differences (float64, h = 1e-6 log-parameters) for all theta and all beta at
             theta = defaults, beta = calibration; J_sysId against a frsim trace (noise off) of the controller with default gains on the dry-run plant. Gate of Stage 3: same sign and (relative <= 1 % or
             absolute <= 0.1 % of the largest component).
  python3 checks5.py forward|noise|gradient [--procs 4] [--threads 2] [--out FILE.json]"""
import argparse, json, os, sys, time
from multiprocessing import get_context
from pathlib import Path
import numpy as np
os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parents[1] / 'checks'))
import loop_lib as L  # noqa: E402
import reference as R  # noqa: E402
from frsim import config  # noqa: E402
from frsim.episode import EpisodeConfig, run_episode  # noqa: E402
from frsim.scenarios import study_cells, tuning_cells  # noqa: E402
CTRL = L.CTRL; NB = L.N_BETA; RD = CTRL in ('S1', 'S3')
SCEN6 = ['A1', 'A4', 'A8', 'CIRCLE', 'FIG8', 'OVAL']; TOL = 0.5
TRUE_PLANT = L.PLANTS['P1RD' if RD else 'P1']


def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)


def plant_of(over):
    o = dict(over)
    if RD: o.setdefault('rpm_delay_fractional', 1)
    return config.plant_with(o)


def psi_of(plant):
    M = L.model()
    v = [plant.thrust_law_gain_n, plant.motor_lag_ms, plant.downwash_scale, plant.plume_amplitude_n / config.PLUME_FIT_AMPLITUDE_N] + ([plant.rpm_delay_ms] if RD else [])
    return np.log(np.array(v) / M.BETA_START)


def fr(args):
    scen, cell, gains, over, seed, noise = args
    r = run_episode(EpisodeConfig(CTRL, scen, cell, gains, 'nominal', seed, plant=plant_of(over), noise=noise, record_state=True))
    return r.state_trace


def window_rms(t, pos, ref, scen_s):
    m = (t > config.SCORE_AFTER_S) & (t <= scen_s); e = pos[m] - ref[m]
    return 100 * float(np.sqrt((e * e).sum(-1).mean())), float(pos[m][:, 2].mean()), float(ref[m][:, 2].mean())


def rows_spec():
    out = {'default': (np.ones(10), {})}
    for k in (1, 2): out['gains%d' % k] = (np.random.default_rng(k).uniform(0.7, 1.3, 10), {})
    out['shift1'] = (np.ones(10), dict({'thrust_gain': 'x0.8', 'motor_lag_ms': 45.0}, **({'rpm_delay_ms': 7.3} if RD else {})))
    out['shift2'] = (np.ones(10), {'downwash_scale': 'x1.3', 'plume_scale': 'x0.7'})
    return out


def forward(a):
    import torch; torch.set_num_threads(a.threads); M = L.model()
    spec = rows_spec(); names = list(spec); cells = {s: study_cells(s)[0] for s in SCEN6}
    gains = {n: {k: float(v * f) for k, v, f in zip(M.GAIN_NAMES, M.GAIN_DEFAULT, spec[n][0])} for n in names}
    jobs = [(s, cells[s], gains[n], spec[n][1], 0, False) for s in SCEN6 for n in names]
    with get_context('spawn').Pool(a.procs) as p: res = p.map(fr, jobs, chunksize=1)
    say('frsim: %d episodes done' % len(jobs))
    phi = torch.as_tensor(np.log(np.stack([spec[n][0] for n in names])), dtype=torch.float64); psi = torch.as_tensor(np.stack([psi_of(plant_of(spec[n][1])) for n in names]), dtype=torch.float64)
    table = []
    for si, s in enumerate(SCEN6):
        ref = R.build(s, cells[s]); t1 = time.time()
        with torch.no_grad(): out = M.rollout(ref, phi, psi)
        for ni, n in enumerate(names):
            tr = res[si * len(names) + ni]; k = min(len(tr), len(out['t'])); tt = tr[:k, 0]
            rf, _, _ = window_rms(tt, tr[:k, 1:4], tr[:k, 14:17], ref.scenario_s); rm, _, _ = window_rms(tt, out['x'][:k, ni].numpy(), tr[:k, 14:17], ref.scenario_s)
            win = (tt > 0) & (tt <= ref.scenario_s); tm = float(M.tilt_deg(out['q'][:k, ni])[win].max()); tf = float(M.tilt_deg(torch.as_tensor(tr[:k, 7:11]))[win].max())
            d = np.linalg.norm(tr[:k, 1:4] - out['x'][:k, ni].numpy(), axis=-1).max() * 1000
            row = dict(scenario=s, row=n, rms3d_frsim_cm=rf, rms3d_model_cm=rm, rel_diff_pct=100 * abs(rm - rf) / rf, max_pos_diff_mm=float(d), crash_frsim=tf >= M.TILT_CRASH_DEG, crash_model=tm >= M.TILT_CRASH_DEG, seconds=time.time() - t1)
            row['pass'] = bool(row['rel_diff_pct'] <= TOL and row['crash_frsim'] == row['crash_model']); table.append(row)
        say(s, 'model batch %.0f s' % (time.time() - t1))
    for r in table: print('%-7s %-8s frsim %9.4f model %9.4f rel %8.4f %% max dx %9.3f mm crash %s/%s %s' % (r['scenario'], r['row'], r['rms3d_frsim_cm'], r['rms3d_model_cm'], r['rel_diff_pct'], r['max_pos_diff_mm'], r['crash_frsim'], r['crash_model'], r['pass']))
    ok = all(r['pass'] for r in table); say('FORWARD CHECK %s: %s (%d cells, tolerance %.1f %%)' % (CTRL, 'PASS' if ok else 'FAIL', len(table), TOL)); json.dump(dict(controller=CTRL, passed=ok, table=table), open(a.out, 'w'), indent=1); return 0 if ok else 1


def noise_gate(a):
    import torch; torch.set_num_threads(a.threads); M = L.model()
    d = json.load(open(HERE.parent / 'stage3b' / 'results' / 'full_P1_cotune.json')); phis = {'default': np.zeros(10), 'round1': np.array(d['rounds'][1]['phi']), 'round3': np.array(d['rounds'][3]['phi'])}
    seeds = (1000, 1001); over = TRUE_PLANT['overrides']; cells = L.TUNING
    jobs = []
    for s, c in cells:
        for sd in seeds:
            for n in phis:
                for eps in (0.0, 1e-6): jobs.append((s, c, L.gains_of(phis[n]), dict(over, thrust_gain='x%.12f' % (0.8 * (1 + eps))), sd, True))
    with get_context('spawn').Pool(a.procs) as p: res = p.map(fr, jobs, chunksize=1)
    say('frsim: %d episodes done' % len(jobs)); it = iter(res); table = []
    for s, c in cells:
        ref = R.build(s, c)
        for sd in seeds:
            fr_ = {n: (next(it), next(it)) for n in phis}
            phi = torch.as_tensor(np.stack([phis[n] for n in phis])); psi = torch.as_tensor(np.tile(L.PSI_TRUE, (len(phis), 1)))
            with torch.no_grad(): out = M.rollout(ref, phi, psi, noise=M.frsim_noise(sd, ref.n_ticks, config.plant_with(over)))
            r1 = window_rms(fr_['round1'][0][:, 0], fr_['round1'][0][:, 1:4], fr_['round1'][0][:, 14:17], ref.scenario_s)[0]
            for i, n in enumerate(phis):
                tr, tr6 = fr_[n]; k = min(len(tr), len(out['t'])); t = tr[:k, 0]
                rf, zf, zr = window_rms(t, tr[:k, 1:4], tr[:k, 14:17], ref.scenario_s); rm, zm, _ = window_rms(t, out['x'][:k, i].numpy(), tr[:k, 14:17], ref.scenario_s)
                r6 = window_rms(tr6[:, 0], tr6[:, 1:4], tr6[:, 14:17], ref.scenario_s)[0]; sens = 100 * abs(r6 - rf) / rf; stable = sens < 0.5; rel = 100 * abs(rm - rf) / rf
                fm = zm < 0.5 * zr
                passed = rel <= 0.5 if stable else bool(fm or rm > 3 * r1)
                table.append(dict(scenario=s, seed=sd, gains=n, rms_frsim_cm=rf, rms_model_cm=rm, rel_pct=rel, sens_change_pct=sens, stable=bool(stable), floor_frsim=bool(zf < 0.5 * zr), floor_model=bool(fm), round1_frsim_cm=r1, passed=bool(passed)))
        say(s, 'done')
    for r in table: print('%-7s %5d %-7s frsim %9.3f model %9.3f rel %8.3f sens %8.3f %% stable %-5s floor f/m %s/%s %s' % (r['scenario'], r['seed'], r['gains'], r['rms_frsim_cm'], r['rms_model_cm'], r['rel_pct'], r['sens_change_pct'], r['stable'], r['floor_frsim'], r['floor_model'], 'yes' if r['passed'] else 'NO'))
    ok = all(r['passed'] for r in table); say('NOISE GATE %s: %s; stable cells %d of %d' % (CTRL, 'PASS' if ok else 'FAIL', sum(r['stable'] for r in table), len(table))); json.dump(dict(controller=CTRL, passed=ok, table=table), open(a.out, 'w'), indent=1); return 0 if ok else 1


def gradient(a):
    import torch; torch.set_num_threads(a.threads); M = L.model(); import costs as C, ingest
    NAMES = list(M.GAIN_NAMES) + list(M.BETA_NAMES) + (['rpm_delay_ms'] if RD else []); n = len(NAMES); h = a.h
    per = {}; TUN = [(s, c) for s, c in L.TUNING if a.scen is None or s in a.scen]
    for s, c in TUN:
        ref = R.build(s, c); tr = fr((s, c, L.gains_of(np.zeros(10)), TRUE_PLANT['overrides'], 0, False)); tgt = ingest.from_trace(tr); t0 = time.time()
        z = torch.zeros(1, n + 0, dtype=torch.float64, requires_grad=True)
        with torch.enable_grad():
            out = M.rollout(ref, z[:, :10], z[:, 10:], compile=True); jt = C.j_task(out, ref.scenario_s); js = C.j_sysid(out, ingest.on_grid(tgt, out['t']), ref.scenario_s, states=('p', 'v', 'q'))
            gt = torch.autograd.grad(jt.sum(), z, retain_graph=True)[0][0].numpy(); gs = torch.autograd.grad(js.sum(), z)[0][0].numpy()
        eye = torch.eye(n, dtype=torch.float64) * h; zz = torch.cat([eye, -eye])
        with torch.no_grad(): o2 = M.rollout(ref, zz[:, :10], zz[:, 10:]); jtp = C.j_task(o2, ref.scenario_s); jsp = C.j_sysid(o2, ingest.on_grid(tgt, o2['t']), ref.scenario_s, states=('p', 'v', 'q'))
        per[s] = dict(J_task=float(jt), J_sysId=float(js), g_task=gt, f_task=((jtp[:n] - jtp[n:]) / (2 * h)).numpy(), g_sysid=gs, f_sysid=((jsp[:n] - jsp[n:]) / (2 * h)).numpy()); say(s, 'done %.0f s' % (time.time() - t0))

    def judge(g, f):
        big = max(np.abs(f).max(), np.abs(g).max()); rows = []
        for i, nm in enumerate(NAMES):
            err = abs(g[i] - f[i]); fin = np.isfinite(g[i]) and np.isfinite(f[i]); rel = float(err / abs(f[i])) if fin and f[i] != 0 else None
            ok = bool(fin and (np.sign(g[i]) == np.sign(f[i]) or max(abs(g[i]), abs(f[i])) < 1e-9 * big) and ((rel is not None and rel <= 0.01) or err <= 0.001 * big))
            rows.append(dict(param=nm, autograd=float(g[i]), fd=float(f[i]), rel_err=rel, abs_over_max=float(err / big), passed=ok))
        return rows
    result = dict(controller=CTRL, h=h, combined={}, single={}); allok = True
    for cn, gk, fk, jk in (('J_task', 'g_task', 'f_task', 'J_task'), ('J_sysId', 'g_sysid', 'f_sysid', 'J_sysId')):
        rows = judge(np.mean([per[s][gk] for s, _ in TUN], 0), np.mean([per[s][fk] for s, _ in TUN], 0)); ok = all(r['passed'] for r in rows); allok &= ok
        result['combined'][cn] = dict(passed=ok, rows=rows); print('== COMBINED %s: %s' % (cn, 'PASS' if ok else 'FAIL'))
        for r in rows: print('   %-15s autograd %+.6e fd %+.6e rel %-9s abs/max %.1e %s' % (r['param'], r['autograd'], r['fd'], ('%.2e' % r['rel_err']) if r['rel_err'] is not None else '-', r['abs_over_max'], '' if r['passed'] else '<-- FAIL'))
    for s, _ in TUN:
        result['single'][s] = {cn: judge(per[s][gk], per[s][fk]) for cn, gk, fk in (('J_task', 'g_task', 'f_task'), ('J_sysId', 'g_sysid', 'f_sysid'))}
        print('single %s: failing rows J_task %d, J_sysId %d (not gated)' % (s, *[sum(not r['passed'] for r in result['single'][s][cn]) for cn in ('J_task', 'J_sysId')]))
    say('COMBINED GRADIENT GATE %s: %s' % (CTRL, 'PASS' if allok else 'FAIL')); json.dump(result, open(a.out, 'w'), indent=1); return 0 if allok else 1


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('what', choices=['forward', 'noise', 'gradient']); ap.add_argument('--procs', type=int, default=4); ap.add_argument('--threads', type=int, default=2); ap.add_argument('--h', type=float, default=1e-6); ap.add_argument('--scen', nargs='+', default=None); ap.add_argument('--out')
    a = ap.parse_args(); a.out = a.out or str(HERE / 'results' / ('%s_%s_check.json' % (CTRL, a.what)))
    sys.exit({'forward': forward, 'noise': noise_gate, 'gradient': gradient}[a.what](a))
