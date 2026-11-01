#!/usr/bin/env python3
"""Longer-horizon gradient tuning of S0: whole flights (takeoff, hold, pattern, return, landing = 18 s) are run as chained 3 s chunks with the plant state carried over
(truncated backpropagation through time: gradients flow inside a chunk, the state is detached between chunks). One CUDA graph per chunk (forward + backward), all chunks of
an iteration replay it; gradients are summed over the chunks. Loss per chunk: 0.5 mean + 0.5 mean of the worst 25 % rows (+ tilt hinge), plant randomised as in train_s0 --wide --robust.
Usage: python3 train_long.py --seed 0 --iters 300 --out ../out/diff_s0L_seed0.json"""
import argparse, json, math, sys, time
from pathlib import Path
import torch
sys.path.insert(0, str(Path(__file__).parent))
import s0_torch as T, s0_plant_torch as PT, traj
from train_s0 import NAMES, DEF, LO, HI, gains_from, rot2quat, LAM

T_SC = 10.0; T_TOT = traj.T_TO + traj.T_HOLD + T_SC + traj.T_LAND + traj.T_GND; CH = 3000          # chunk length [ms]
N_TOT = int(round(T_TOT * 1000)); NCH = N_TOT // CH


def make_plant(B, gen, dtype, dev):
    U = lambda lo, hi: lo + (hi - lo) * torch.rand(B, generator=gen, dtype=dtype, device=dev)
    stress = torch.rand(B, generator=gen, dtype=dtype, device=dev) < 0.3
    md = torch.where(torch.rand(B, generator=gen, dtype=dtype, device=dev) < 0.4, torch.zeros(B, device=dev), torch.randint(10, 61, (B,), generator=gen, device=dev).to(dtype)).long()
    md = torch.where(stress, torch.randint(40, 61, (B,), generator=gen, device=dev), md)
    r_ = torch.rand(B, generator=gen, dtype=dtype, device=dev); ffs = torch.where(r_ < 0.4, torch.full((B,), 0.4, dtype=dtype, device=dev), torch.ones(B, dtype=dtype, device=dev)); ffs = torch.where(r_ < 0.15, torch.zeros_like(ffs), ffs)
    return dict(lag=torch.where(stress, U(50, 64), U(26, 62)), fmax=0.2758, quad=True, kt=U(0.5, 1.0), gn=U(8, 20), md=md), ffs


def chunk_inputs(p, ffs, k, gen, plant, dtype, dev):
    B = ffs.numel(); t = (k * CH + torch.arange(CH, dtype=dtype, device=dev))[:, None] * 1e-3 + torch.zeros(1, B, dtype=dtype, device=dev)
    ref = traj.episode(p, t, T_SC); r = lambda *s: torch.randn(*s, generator=gen, dtype=dtype, device=dev)
    sp = dict(pos=ref['pos'], vel=ref['vel'], acc=ref['acc'] * ffs[None, :, None], jerk=ref['jerk'] * ffs[None, :, None])
    nz = dict(pos=5e-4 * r(CH, B, 3), vel=5e-3 * r(CH, B, 3), gyro=plant['gn'][None, :, None] * r(CH // 2 + 1, B, 3))
    return sp, nz


def init_carry(B, dtype, dev):
    z3 = torch.zeros(B, 3, dtype=dtype, device=dev); F = torch.full((B, 4), PT.M * T.G / 4, dtype=dtype, device=dev); z1 = torch.zeros(B, dtype=dtype, device=dev)
    q = torch.tensor([0., 0., 0., 1.], dtype=dtype, device=dev).expand(B, 4).clone()
    return [z3.clone(), z3.clone(), q, z3.clone(), F.clone(), z3.clone(), z3.clone(), F.clone(), F.clone(), z1.clone(), z1.clone(), z3[None].expand(61, B, 3).clone(), z3[None].expand(61, B, 3).clone()]


def chunk_loss(theta, sp, nz, plant, carry, B, dtype, dev, robust=True):
    out, fin = PT.rollout_blocks(sp, gains_from(theta, B, dtype, dev), plant, dtype=dtype, device=dev, noise=nz, carry=tuple(carry))
    err = torch.linalg.norm(out - sp['pos'][::10], dim=-1).clamp(max=1.0); row = (err ** 2).mean(0); rl = row + LAM * fin['tpen'] * (CH / 1000.0 / 3.0)
    L = (0.5 * rl.mean() + 0.5 * torch.topk(rl, max(1, B // 4)).values.mean()) if robust else rl.mean()
    return L * (CH / N_TOT), row, [c.detach() for c in fin['carry']], fin['tilt_max'].detach()


class ChunkGraph:
    """Static buffers + one CUDA graph for a chunk. grad=True: forward + backward (theta.grad written by the graph); grad=False: forward only."""
    def __init__(self, theta, B, dtype, dev, grad, sample):
        self.B, self.grad, self.theta = B, grad, theta
        self.sp, self.nz, self.plant, self.carry = [_clone(x) for x in sample]
        s = torch.cuda.Stream(); s.wait_stream(torch.cuda.current_stream())
        def body():
            if grad:
                L, row, carry_out, tm = chunk_loss(theta, self.sp, self.nz, self.plant, self.carry, B, dtype, dev); L.backward()
            else:
                with torch.no_grad(): L, row, carry_out, tm = chunk_loss(theta, self.sp, self.nz, self.plant, self.carry, B, dtype, dev)
            return L, row, carry_out, tm
        with torch.cuda.stream(s):
            for _ in range(3): theta.grad = None; body()
        torch.cuda.current_stream().wait_stream(s); torch.cuda.synchronize(); theta.grad = None
        self.g = torch.cuda.CUDAGraph()
        with torch.cuda.graph(self.g): self.L, self.row, self.carry_out, self.tm = body()
        self.gbuf = theta.grad; theta.grad = None            # the graph-owned gradient buffer (rewritten by every replay)

    def run(self, sp, nz, plant, carry):
        _copy(self.sp, sp); _copy(self.nz, nz); _copy(self.plant, plant); _copy(self.carry, carry); self.g.replay()


def _copy(dst, src):
    if isinstance(dst, dict):
        for k in dst: _copy(dst[k], src[k])
    elif isinstance(dst, (list, tuple)):
        for a, b in zip(dst, src): _copy(a, b)
    elif torch.is_tensor(dst): dst.copy_(src)


def _clone(x):
    if isinstance(x, dict): return {k: _clone(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)): return [_clone(v) for v in x]
    return x.clone() if torch.is_tensor(x) else x


def episode_pass(graph, p, ffs, plant, gen, dtype, dev, B, accumulate=None):
    """Run all chunks of one batch of flights through a ChunkGraph; returns row loss (B,), crash fraction, summed gradient."""
    carry = init_carry(B, dtype, dev); rows = torch.zeros(B, dtype=dtype, device=dev); tilt = torch.zeros(B, dtype=dtype, device=dev); Ltot = 0.0
    for k in range(NCH):
        sp, nz = chunk_inputs(p, ffs, k, gen, plant, dtype, dev)
        graph.run(sp, nz, plant, carry)
        rows += graph.row / NCH; tilt = torch.maximum(tilt, graph.tm); Ltot += float(graph.L) if False else 0.0
        if accumulate is not None and graph.grad: accumulate += graph.gbuf
        carry = [c.clone() for c in graph.carry_out]; carry[9].zero_(); carry[10].zero_()
    return rows, (tilt > 70).float().mean()


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--seed', type=int, default=0); ap.add_argument('--iters', type=int, default=300); ap.add_argument('--B', type=int, default=256); ap.add_argument('--lr', type=float, default=0.03)
    ap.add_argument('--val_every', type=int, default=25); ap.add_argument('--out', default='out_diff_long.json'); ap.add_argument('--dev', default='cuda')
    a = ap.parse_args(); dtype = torch.float32; dev = a.dev; B = a.B; BV = 128
    gen = torch.Generator(device=dev).manual_seed(a.seed); vgen = torch.Generator(device=dev).manual_seed(12345); vn = torch.Generator(device=dev).manual_seed(777)
    theta = torch.zeros(len(NAMES), dtype=dtype, device=dev, requires_grad=True); opt = torch.optim.Adam([theta], lr=a.lr); lo, hi = LO.to(dtype=dtype, device=dev), HI.to(dtype=dtype, device=dev)
    # validation flights fixed once (references, plant, noise realisation per chunk are regenerated from the same generator seed each time)
    vp = traj.sample_episode(BV, vgen, dtype, dev); vplant, vffs = make_plant(BV, vgen, dtype, dev)
    # graphs
    pt = traj.sample_episode(B, gen, dtype, dev); plant_t, ffs_t = make_plant(B, gen, dtype, dev); sp0, nz0 = chunk_inputs(pt, ffs_t, 0, gen, plant_t, dtype, dev)
    gtrain = ChunkGraph(theta, B, dtype, dev, True, (sp0, nz0, plant_t, init_carry(B, dtype, dev)))
    sv, nv = chunk_inputs(vp, vffs, 0, vn, vplant, dtype, dev); gval = ChunkGraph(theta, BV, dtype, dev, False, (sv, nv, vplant, init_carry(BV, dtype, dev)))
    def validate():
        g_ = torch.Generator(device=dev).manual_seed(777)
        with torch.no_grad(): rows, crash = episode_pass(gval, vp, vffs, vplant, g_, dtype, dev, BV)
        w25 = torch.topk(rows, BV // 4).values.mean()
        return dict(rms_cm=100 * float(torch.sqrt(rows.mean())), w25_cm=100 * float(torch.sqrt(w25)), crash=float(crash), sel=float(w25))
    v0 = validate(); print('baseline validation (full 18 s flights, 128 rows): rms %.2f cm, worst-25%% %.2f cm, crash %.3f' % (v0['rms_cm'], v0['w25_cm'], v0['crash']), flush=True)
    best_val = v0['sel']; best_theta = theta.detach().clone(); log = []; t_start = time.time()
    for it in range(a.iters):
        t0 = time.time(); p = traj.sample_episode(B, gen, dtype, dev); plant, ffs = make_plant(B, gen, dtype, dev); acc = torch.zeros_like(theta)
        rows, crash = episode_pass(gtrain, p, ffs, plant, gen, dtype, dev, B, accumulate=acc)
        theta.grad = acc; ok = bool(torch.isfinite(acc).all())
        if ok:
            torch.nn.utils.clip_grad_norm_([theta], 5.0); th_old = theta.detach().clone(); opt.step()
            with torch.no_grad(): d = (theta - th_old).clamp(-0.1, 0.1); theta.copy_((th_old + d).clamp(lo, hi))
        rec = dict(it=it, loss_rms_cm=100 * float(torch.sqrt(rows.mean())), crash=float(crash), ok=ok, sec=time.time() - t0, theta=theta.detach().cpu().tolist()); log.append(rec)
        if it % a.val_every == 0 or it == a.iters - 1:
            v = validate(); rec['val'] = v
            if v['sel'] < best_val and v['crash'] <= v0['crash']: best_val = v['sel']; best_theta = theta.detach().clone()
            print('it %3d loss %.2f cm crash %.3f | val %.2f cm w25 %.2f cm crash %.3f (best sel %.2e) | %.1fs/it' % (it, rec['loss_rms_cm'], rec['crash'], v['rms_cm'], v['w25_cm'], v['crash'], best_val, rec['sec']), flush=True)
    gains = {k: float(DEF[i] * math.exp(float(best_theta[i]))) for i, k in enumerate(NAMES)}
    json.dump(dict(seed=a.seed, gains=gains, theta=best_theta.cpu().tolist(), baseline_val=v0, best_val_sel=best_val, log=log, minutes=(time.time() - t_start) / 60), open(a.out, 'w'), indent=1)
    print('saved', a.out, gains)


if __name__ == '__main__': main()
