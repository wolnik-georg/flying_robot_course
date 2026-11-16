#!/usr/bin/env python3
"""Stage 3c C3 gates: the scenario-batched model on the GPU (s0_batch.rollout_rows) against the CPU model (s0_model.rollout, one scenario per call).
Forward: per tuning scenario the task 3-D RMS [cm] at three points (default gains and calibration; gains of round 1 of (d); true P1 beta), gate 0.5 %.
Gradient: the combined cost (mean over the 4 scenarios) J_task and J_sysId (position + velocity + attitude, one noisy P1 flight of theta_0 per scenario),
gradient with respect to all 14 parameters at the point x = (0, 0.5 * log true P1 beta) and at the default; gate of Stage 3: same sign and (relative <= 1 %
or absolute <= 0.1 % of the largest component). Usage: python3 dryrun/stage3c/c3_gates.py <f64|f32>   (GPU, one job at a time)"""
import json, os, sys, time
from pathlib import Path
os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
import numpy as np, torch
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent.parent))
import loop_lib as L  # noqa: E402
import reference as R  # noqa: E402
import s0_batch as SB  # noqa: E402
import ingest  # noqa: E402
dtn = sys.argv[1]; dt = {'f32': torch.float32, 'f64': torch.float64}[dtn]; dev = 'cuda'
D = json.load(open(HERE.parent / 'stage3b' / 'results' / 'full_P1_cotune.json'))
names = [s for s, c in L.TUNING]; refs = [R.build(s, c) for s, c in L.TUNING]; ends = [r.scenario_s for r in refs]; w = np.full(4, 0.25)
STATES = ('p', 'v', 'q')
pts = {'default': np.zeros(14), 'gains round 1 of (d)': np.concatenate([D['rounds'][1]['phi'], np.zeros(4)]), 'true P1 beta': np.concatenate([np.zeros(10), L.PSI_TRUE])}
gpt = {'default': np.zeros(14), 'x = (0, 0.5 log true beta)': np.concatenate([np.zeros(10), 0.5 * L.PSI_TRUE])}
res = dict(dtype=dtn)
# ---- CPU reference
t0 = time.time()
cpu_fwd = {k: [L.worker_rollout(('task_fwd', s, c, x, None))[0] for s, c in L.TUNING] for k, x in pts.items()}
print('CPU forward done %.0f s' % (time.time() - t0), flush=True)
flights = []
for i, (s, c) in enumerate(L.TUNING):
    tr, _, _ = L.worker_fly((s, c, L.gains_of(np.zeros(10)), L.P1, i)); flights.append(tr)
cpu_g = {}
for k, x in gpt.items():
    rt = [L.worker_rollout(('task', s, c, x, None)) for s, c in L.TUNING]
    rs = [L.worker_rollout(('sysid', s, c, x, [flights[i]], STATES)) for i, (s, c) in enumerate(L.TUNING)]
    cpu_g[k] = dict(task=(np.mean([r[0] for r in rt]), np.mean([r[1] for r in rt], axis=0)), sysid=(np.mean([r[0] for r in rs]), np.mean([r[1] for r in rs], axis=0)))
print('CPU gradients done %.0f s' % (time.time() - t0), flush=True)
# ---- GPU batched
def gpu(x, kind, grad):
    z = torch.tensor(x, dtype=dt, device=dev, requires_grad=grad)
    with torch.set_grad_enabled(grad):
        out = SB.rollout_rows(refs, z[:10], z[10:], dtype=dt, device=dev, compile=True)
        if kind == 'task':
            per = SB.j_task_rows(out, ends, w, per_row=True); J = (per * torch.as_tensor(w, dtype=dt, device=dev)).sum()
        else:
            tg = [ingest.on_grid(ingest.from_trace(f), out['t']) for f in flights]
            per = SB.j_sysid_rows(out, tg, ends, w, states=STATES, per_row=True); J = (per * torch.as_tensor(w, dtype=dt, device=dev)).sum()
    g = None
    if grad:
        J.backward(); g = z.grad.cpu().numpy()
    return per.detach().cpu().numpy(), float(J), g
t0 = time.time(); fwd = {}
for k, x in pts.items():
    per, _, _ = gpu(x, 'task', False); fwd[k] = per
rel = {k: (np.abs(np.sqrt(fwd[k]) / np.sqrt(np.array(cpu_fwd[k])) - 1) * 100).tolist() for k in pts}
print('forward  max relative difference of the task 3-D RMS (%) per point and scenario', {k: np.round(v, 4).tolist() for k, v in rel.items()}, '(%.0f s)' % (time.time() - t0), flush=True)
res['forward_rel_pct'] = rel; res['forward_gate_pass'] = bool(max(max(v) for v in rel.values()) <= 0.5); res['scenarios'] = names
def gate(ga, gc):
    big = np.max(np.abs(gc)); out = []
    for a, c in zip(ga, gc):
        rel_ = abs(a - c) / abs(c) if c != 0 else (0.0 if a == 0 else np.inf)
        out.append(dict(gpu=float(a), cpu=float(c), rel=float(rel_), abs_over_max=float(abs(a - c) / big), ok=bool(a * c >= 0 and (rel_ <= 0.01 or abs(a - c) / big <= 0.001))))
    return out
res['gradient'] = {}
for k, x in gpt.items():
    for kind in ('task', 'sysid'):
        t0 = time.time(); per, J, g = gpu(x, kind, True); Jc, gc = cpu_g[k][kind]; gt = gate(g, gc)
        res['gradient']['%s | %s' % (kind, k)] = dict(J_gpu=J, J_cpu=float(Jc), table=gt, passed=all(r['ok'] for r in gt))
        print('gradient %s at %s: J gpu %.8e cpu %.8e, all 14 parameters pass: %s, worst relative %.3g, worst abs/max %.3g (%.0f s)' % (kind, k, J, Jc, all(r['ok'] for r in gt), max(r['rel'] for r in gt), max(r['abs_over_max'] for r in gt), time.time() - t0), flush=True)
L.jdump(res, HERE / 'results' / ('c3_gates_%s.json' % dtn))
