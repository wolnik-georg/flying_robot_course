#!/usr/bin/env python3
"""Stage 7 step 1 c + d (S3 model, single process, eager):
c1  S3 model with the NS2 output forced to zero equals the S1 model (forward, max abs difference of x, v, q over the first 3000 blocks of A1, A8, CIRCLE, theta_0 and a random gain set);
c3  derivative of the NS2 output (NS2.faz, the controller-side call) with respect to the measured position and velocity: autograd against central finite differences (h 1e-6) at 3 states of A1 and A8 (25 / 50 / 75 % of the flight);
c4  how often the clamp (8 m/s^2) and the first-two-feeds zero hold are active on A1 and A8 (theta_0, beta after the round-0 fit of the rdfix run), plus the fraction of evaluations with a peer above;
d   INDI + NS2 interaction on A1 and A8 at theta_0: the NS2 hold (z, m/s^2) and the INDI disturbance term a_indi_z = a_rpm_f - a_imu_f of the S3 model and of the S1 model (same beta, no NS2): means over the scored window, correlation of hold and a_indi_z,
    z RMS error of both. Usage: DIFFCOTUNE_CONTROLLER=S3 python3 diag7_cd.py -> results/diag7_cd.json"""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent))
import torch
import loop_lib as L  # noqa
import reference as R  # noqa
import s1_model as S1, s2_model as S2, ns2_torch as NS2, s0_model as S0  # noqa
def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)
torch.set_num_threads(2)
rec = json.load(open(HERE.parent / 'stage5c_rdfix' / 'results' / 'loop_S3.json')); psi = np.array(rec['rounds'][0]['psi_after_fit'])
out = {}; dt = torch.float64
cells = dict(L.TUNING)
def refof(s): return R.build(s, cells[s])
phi0 = torch.zeros(1, 10, dtype=dt); psi_t = torch.tensor(psi[None], dtype=dt)
# c1 ------------------------------------------------------------------------------------------------
orig_faz = NS2.faz; c1 = {}
rng = np.random.default_rng(1); phi_r = torch.tensor(rng.uniform(-0.2, 0.2, (1, 10)), dtype=dt)
for s in ('A1', 'A8', 'CIRCLE'):
    ref = refof(s)
    for nm, ph in (('theta0', phi0), ('gains_random', phi_r)):
        a = S1.rollout(ref, ph, psi_t, n_blocks=3000, ns2=False)
        NS2.faz = lambda nets, rp, rv, z, v, m, act: torch.zeros(rp.shape[:-1], dtype=rp.dtype)
        try: b = S1.rollout(ref, ph, psi_t, n_blocks=3000, ns2=True)
        finally: NS2.faz = orig_faz
        c = S1.rollout(ref, ph, psi_t, n_blocks=3000, ns2=True)
        d_off = max(float((a[k] - b[k]).abs().max()) for k in ('x', 'v', 'q')); d_on = max(float((a[k] - c[k]).abs().max()) for k in ('x', 'v'))
        c1['%s/%s' % (s, nm)] = dict(max_abs_diff_ns2_zero_vs_S1=d_off, max_abs_diff_ns2_on_vs_S1=d_on); say('c1', s, nm, 'NS2 forced zero vs S1: %.2e; NS2 on vs S1: %.2e' % (d_off, d_on))
out['c1'] = c1
# rollouts with the NS2 hold recorded (eager, B = 1), c3 c4 d --------------------------------------------
rows = {}
def run(ref, ns2, rec_indi=True):
    ind = []; orig_bw = S1.bw_step
    def bw(fs, xin, bwc):
        fs2, o = orig_bw(fs, xin, bwc); ind.append((o[..., 0, 2] - o[..., 1, 2]).detach().clone()); return fs2, o       # a_indi_z of this block
    S1.bw_step = bw
    try:
        o = S1.rollout(ref, phi0, psi_t, ns2=ns2, keep_blocks=5)
    finally:
        S1.bw_step = orig_bw
    return o, torch.stack(ind)[:, 0].numpy()
nets = S0.net(dt, 'cpu')
c3 = {}; c4 = {}; dd = {}
for s in ('A1', 'A8'):
    ref = refof(s); t0 = time.time()
    o3, ind3 = run(ref, True); o1, ind1 = run(ref, False); say(s, 'rollouts done in %.0f s' % (time.time() - t0))
    x, v = o3['x'][:, 0], o3['v'][:, 0]; idx = o3['idx']; tt = ref.time_s[idx]; ntot = len(idx)
    tp = torch.as_tensor(ref.top_pos, dtype=dt); tv = torch.as_tensor(ref.top_vel, dtype=dt); act = torch.tensor([bool(ref.has_top)])
    sp = dict(top_pos=tp)
    # hold at every evaluation (rows are kept every 5 blocks = NS2 evaluation blocks)
    hold = []
    for j in range(ntot):
        k = 5 * j; i = 2 * k
        vpk = torch.zeros(3, dtype=dt) if j < 2 else (tp[i] - tp[i - 10]) / 0.01
        hold.append(float(NS2.faz(nets, (tp[i] - x[j])[None], (vpk - v[j])[None], x[j, 2:3], v[j][None], S0.M, act)[0]))
    hold = np.array(hold); win = (tt > 1.0) & (tt <= ref.scenario_s)
    c4[s] = dict(n_eval=int(ntot), peer_above_fraction=float(np.mean(tp[idx, 2].numpy() > x[:, 2].detach().numpy())), clamp_active_fraction=float(np.mean(np.abs(hold) >= 8 - 1e-9)),
                 zero_first_feeds=2, hold_mean=float(hold[win].mean()), hold_max=float(hold.max()), hold_min=float(hold.min()), hold_rms=float(np.sqrt((hold[win] ** 2).mean())))
    say('c4', s, c4[s])
    # c3 autograd vs FD of faz at 3 states
    res3 = []
    for frac in (0.25, 0.5, 0.75):
        j = int(frac * (ntot - 1)); i = 10 * j
        vpk = (tp[i] - tp[i - 10]) / 0.01
        def f(xm, vm): return NS2.faz(nets, (tp[i] - xm)[None], (vpk - vm)[None], xm[2:3], vm[None], S0.M, act)[0]
        xm = x[j].detach().clone().requires_grad_(True); vm = v[j].detach().clone().requires_grad_(True); y = f(xm, vm); gx, gv = torch.autograd.grad(y, (xm, vm), allow_unused=True)
        ad = np.r_[gx.numpy(), gv.numpy()]; fd = np.zeros(6); h = 1e-6
        for m_ in range(6):
            e = torch.zeros(6, dtype=dt); e[m_] = h; xp, vp, xn, vn = x[j].detach() + e[:3], v[j].detach() + e[3:], x[j].detach() - e[:3], v[j].detach() - e[3:]
            fd[m_] = (float(f(xp, vp)) - float(f(xn, vn))) / (2 * h)
        res3.append(dict(frac=frac, t=float(tt[j]), out=float(y), autograd=ad.tolist(), fd=fd.tolist(), max_abs_diff=float(np.abs(ad - fd).max()), max_abs=float(np.abs(fd).max()), clamped=bool(abs(float(y)) >= 8 - 1e-9)))
        say('c3', s, 'frac', frac, 'out %.3f' % float(y), 'max |autograd - fd| %.2e of max |fd| %.2e' % (np.abs(ad - fd).max(), np.abs(fd).max()))
    c3[s] = res3
    # d: a_indi_z against hold
    ind_ev3 = ind3[::5][:ntot]; ind_ev1 = ind1[::5][:ntot]; m = win[:len(ind_ev3)]
    corr = float(np.corrcoef(hold[m], ind_ev3[m])[0, 1]) if m.sum() > 2 else None
    z3 = (x[:, 2].detach().numpy() - ref.pos[idx, 2]); z1 = (o1['x'][:, 0, 2].detach().numpy() - ref.pos[idx, 2])
    dd[s] = dict(window=[1.0, float(ref.scenario_s)], hold_mean=float(hold[m].mean()), a_indi_z_mean_S3=float(ind_ev3[m].mean()), a_indi_z_mean_S1=float(ind_ev1[m].mean()), a_indi_z_rms_S3=float(np.sqrt((ind_ev3[m] ** 2).mean())), a_indi_z_rms_S1=float(np.sqrt((ind_ev1[m] ** 2).mean())),
                 corr_hold_aindi_S3=corr, corr_hold_aindi_S1=float(np.corrcoef(hold[m], ind_ev1[m])[0, 1]), z_rms_cm_S3=float(100 * np.sqrt((z3[m] ** 2).mean())), z_rms_cm_S1=float(100 * np.sqrt((z1[m] ** 2).mean())),
                 z_bias_cm_S3=float(100 * z3[m].mean()), z_bias_cm_S1=float(100 * z1[m].mean()))
    say('d', s, dd[s])
    np.savez(HERE / 'results' / ('diag7_cd_series_%s.npz' % s), t=tt, hold=hold, aindi_S3=ind_ev3, aindi_S1=ind_ev1, z3=z3, z1=z1, top_z=tp[idx, 2].numpy(), x3=x[:, 2].detach().numpy())
out.update(c3=c3, c4=c4, d=dd); json.dump(out, open(HERE / 'results' / 'diag7_cd.json', 'w'), indent=1); say('done')
