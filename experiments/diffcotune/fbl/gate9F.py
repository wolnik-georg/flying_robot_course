#!/usr/bin/env python3
"""Stage F2.6: gradient gate of one FBL variant, the FBL counterpart of dryrun/stage8/gate9.py (that file is hard-wired to the pool of the tree controllers and is not changed). Single process, exact forward-mode gradient
(fsens_fbl.gradient_loop = the gradient the loop uses, 'cfwd') against a central finite difference ALONG THE GRADIENT (h 1e-8, relative <= 1e-3), SG_R 0, QUANT 0 (the model defaults), no hidden clamp:
  --preset   items 1-3 of gate9.py at theta_0 (preset 1): (1) J_sysId at the calibration beta (real traces = frsim FBL flights of plant P1, preset-1 gains, no noise), (2) J_task without noise, (3) J_task WITH noise
             (the tuning step: seeds 6000 / 6001) at the true beta of P1; 5 tuning scenarios each; -> results/gateF_<tag>.json
  --loop J   item 4 (guard at the fitted beta of every round of the dry-run json J): per round l and scenario, J_task with the noise seeds of round l and J_sysId at x = [phi_l, psi_after_fit_l]; -> results/gate9F_<tag>.json
  also for information: reverse mode is not evaluated here (F1: equal to forward mode to 1e-14 on CIRCLE).
Usage: DIFFCOTUNE_CONTROLLER=FBL DIFFCOTUNE_FBL_VARIANT=<v> DIFFCOTUNE_GRAD_MODE=cfwd python3 fbl/gate9F.py --tag FBLbase (--preset | --loop <json>)"""
import argparse, json, os, sys, time
from pathlib import Path
os.environ.setdefault('OMP_NUM_THREADS', '2'); os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun'))
import numpy as np, torch
import loop_lib as L
import fbl_model as FM, fsens_fbl as FS, costs as C, ingest
from frsim import config
from frsim.episode import EpisodeConfig, run_episode
import reference as R
NG = L.NG; NZ = NG + L.N_BETA; H = 1e-8; TOL = 1e-3; M = L.METHOD_SETTINGS[L.METHOD]
def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)


def value(ref, kind, z, noise_seeds, trace):
    z = np.asarray(z, float)
    with torch.no_grad():
        if kind == 'task' and noise_seeds:
            nz = np.stack([FM.frsim_noise(sd, ref.n_ticks) for sd in noise_seeds], 1); B = len(noise_seeds)
            out = FM.rollout(ref, np.tile(z[None, :NG], (B, 1)), np.tile(z[None, NG:], (B, 1)), noise=nz, compile=True)
            return float(C.j_task(out, ref.scenario_s).mean())
        out = FM.rollout(ref, z[None, :NG], z[None, NG:], compile=True)
        if kind == 'task': return float(C.j_task(out, ref.scenario_s).sum())
        return float(C.j_sysid(out, ingest.on_grid(ingest.from_trace(trace), out['t']), ref.scenario_s, states=M['states']).sum())


def check(scen, cell, kind, z, noise_seeds, trace):
    ref = R.build(scen, cell); idx = list(range(NG)) if kind == 'task' else list(range(NG, NZ))
    J, g = FS.gradient_loop(ref, ref.scenario_s, kind, z, idx, traces=[trace] if trace is not None else None, states=M['states'], noise_seeds=noise_seeds)
    ga = g[idx]; d = np.zeros(NZ); d[idx] = ga / (np.linalg.norm(ga) + 1e-300)
    fd = (value(ref, kind, np.asarray(z) + H * d, noise_seeds, trace) - value(ref, kind, np.asarray(z) - H * d, noise_seeds, trace)) / (2 * H)
    an = float(g @ d); rel = abs(an - fd) / (abs(fd) + 1e-300)
    return dict(scenario=scen, kind=kind, J=J, grad_norm=float(np.linalg.norm(ga)), forward=an, fd=fd, rel=rel, passed=bool(rel <= TOL and np.isfinite(rel)))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--tag', required=True); ap.add_argument('--preset', action='store_true'); ap.add_argument('--loop', nargs='*', default=[]); ap.add_argument('--outdir', default=str(HERE.parent / 'dryrun' / 'stage8' / 'results'))
    a = ap.parse_args(); out = dict(controller=L.CTRL, variant=FM.VARIANT, tag=a.tag, h=H, tol=TOL, sg_r=FM.SG_R, quant=FM.QUANT)
    plant = L.PLANTS['P1']; p = config.plant_with(plant['overrides']); g0 = L.gains_of(np.zeros(NG))
    traces = {}
    for s, c in L.TUNING: traces[s] = run_episode(EpisodeConfig('FBL', s, c, g0, plant['variant'], 0, plant=p, noise=False, record_state=True)).state_trace
    if a.preset:
        rows = []; x0 = np.zeros(NZ); xn = np.r_[np.zeros(NG), L.PSI_TRUE]
        for s, c in L.TUNING:
            for name, kind, z, ns, tr in (('J_sysId', 'sysid', x0, None, traces[s]), ('J_task_noisefree', 'task', x0, None, None), ('J_task_noise', 'task', xn, (6000, 6001), None)):
                r = check(s, c, kind, z, ns, tr); r['item'] = name; rows.append(r); say('%-16s %-7s J %.4e |g| %.3e fwd %.6e fd %.6e rel %.1e %s' % (name, s, r['J'], r['grad_norm'], r['forward'], r['fd'], r['rel'], 'PASS' if r['passed'] else 'FAIL'))
        out['rows'] = rows; out['passed'] = bool(all(r['passed'] for r in rows)); json.dump(out, open(Path(a.outdir) / ('gateF_%s.json' % a.tag), 'w'), indent=1); say('GRADIENT GATE (preset) %s: %s' % (a.tag, 'PASS' if out['passed'] else 'FAIL'))
    if a.loop:
        guard = []
        for lj in a.loop:
            rec = json.load(open(lj))
            for l, r in enumerate(rec['rounds']):
                try: x0r = np.r_[np.array(r['phi']), np.array(r['psi_after_fit'])]
                except KeyError: continue
                if len(x0r) != NZ: continue
                ent = dict(source=lj, round=l, rows=[])
                for s, c in L.TUNING:
                    for kind, ns, tr in (('task', (6000 + 2 * l, 6001 + 2 * l) if M['noise'] else None, None), ('sysid', None, traces[s])):
                        e = check(s, c, kind, x0r, ns, tr); ent['rows'].append(e); say('guard round %d %-6s %-7s fwd %.5e fd %.5e rel %.1e %s' % (l, kind, s, e['forward'], e['fd'], e['rel'], 'PASS' if e['passed'] else 'FAIL'))
                ent['passed'] = bool(all(e['passed'] for e in ent['rows'])); guard.append(ent)
        out['guard_fitted_beta'] = guard; out['passed'] = bool(all(e['passed'] for e in guard)); json.dump(out, open(Path(a.outdir) / ('gate9F_%s.json' % a.tag), 'w'), indent=1); say('GRADIENT GATE (guard) %s: %s' % (a.tag, 'PASS' if out['passed'] else 'FAIL'))
    return 0 if out.get('passed') else 1


if __name__ == '__main__':
    sys.exit(main())
