#!/usr/bin/env python3
"""Gradient tuning of S0 WITH downwash, per scenario type (A1 / A4 / A8): whole flights from the scenario bank (NS2-grid settings, none a study cell), bottom drone controlled, top drone scripted,
downwash = NS2 force truth + fitted plume torque (as plant_core). Chained 3 s chunks with detached carry (truncated BPTT), one CUDA graph per chunk, robust per-chunk loss
(0.5 mean + 0.5 worst 25 % rows + tilt hinge), plant randomised like train_long. Usage: python3 train_scen.py --scen A8 --seed 0 --iters 300 --out ../out/diff_s0D_A8_seed0.json"""
import argparse, json, math, sys, time
from pathlib import Path
import numpy as np, torch
sys.path.insert(0, str(Path(__file__).parent))
import s0_torch as T, s0_plant_torch as PT, ns2_torch as NS2
from train_s0 import NAMES, DEF, LO, HI, gains_from, LAM
from train_long import make_plant, _copy, _clone
CH = 1500; OUT = Path(__file__).parent.parent / 'out'; W_NS2 = '/home/georg/Desktop/flying_robot_course/flying_drone_stack/tools/residual/weights/ns2_v2.npz'


def dw_params(dtype, dev):
    d = json.load(open(OUT / 'dw_params.json'))
    return dict(net=NS2.load(W_NS2, dtype, dev), dws=T.K(d['dws'], dtype, dev), plume=tuple(T.K(v, dtype, dev) for v in d['plume']))


def init_carry(B, dtype, dev):
    z3 = torch.zeros(B, 3, dtype=dtype, device=dev); F = torch.full((B, 4), PT.M * T.G / 4, dtype=dtype, device=dev); z1 = torch.zeros(B, dtype=dtype, device=dev)
    q = torch.tensor([0., 0., 0., 1.], dtype=dtype, device=dev).expand(B, 4).clone()
    x0 = z3.clone()
    return [x0, z3.clone(), q, z3.clone(), F.clone(), z3.clone(), z3.clone(), F.clone(), F.clone(), z1.clone(), z1.clone(), z3.clone(), x0[None].expand(61, B, 3).clone(), z3[None].expand(61, B, 3).clone()]


def chunk_inputs(bank, idx, nvalid, ffs, k, gen, plant, dtype, dev):
    B = idx.numel(); a = bank[idx, k * CH:(k + 1) * CH].to(dev, non_blocking=True).permute(1, 0, 2).contiguous()         # (CH, B, 18)
    sp = dict(pos=a[..., 0:3], vel=a[..., 3:6], acc=a[..., 6:9] * ffs[None, :, None], jerk=a[..., 9:12] * ffs[None, :, None], tp=a[..., 12:15], tv=a[..., 15:18])
    r = lambda *s: torch.randn(*s, generator=gen, dtype=dtype, device=dev)
    nz = dict(pos=5e-4 * r(CH, B, 3), vel=5e-3 * r(CH, B, 3), gyro=plant['gn'][None, :, None] * r(CH // 2 + 1, B, 3))
    valid = ((k * CH + torch.arange(0, CH, 10, device=dev))[:, None] < nvalid[None, :].to(dev)).to(dtype)               # (CH/10, B) samples inside the flight
    return sp, nz, valid


def chunk_loss(theta, sp, nz, plant, carry, valid, nsamp, B, dtype, dev, nch):
    out, fin = PT.rollout_blocks(sp, gains_from(theta, B, dtype, dev), plant, dtype=dtype, device=dev, noise=nz, carry=tuple(carry))
    err = torch.linalg.norm(out - sp['pos'][::10], dim=-1).clamp(max=1.0); row = (err ** 2 * valid).sum(0) / nsamp           # contribution of this chunk to the row's episode mean
    rl = row + LAM * fin['tpen'] / nch
    L = 0.5 * rl.mean() + 0.5 * torch.topk(rl, max(1, B // 4)).values.mean()
    return L, row, [c.detach() for c in fin['carry']], fin['tilt_max'].detach()


class ChunkGraph:
    def __init__(self, theta, B, dtype, dev, grad, sample, nch):
        self.B, self.grad, self.theta = B, grad, theta
        self.sp, self.nz, self.plant, self.carry, self.valid, self.nsamp = [_clone(x) for x in sample]
        s = torch.cuda.Stream(); s.wait_stream(torch.cuda.current_stream())
        def body():
            if grad:
                L, row, co, tm = chunk_loss(theta, self.sp, self.nz, self.plant, self.carry, self.valid, self.nsamp, B, dtype, dev, nch); L.backward()
            else:
                with torch.no_grad(): L, row, co, tm = chunk_loss(theta, self.sp, self.nz, self.plant, self.carry, self.valid, self.nsamp, B, dtype, dev, nch)
            return L, row, co, tm
        with torch.cuda.stream(s):
            for _ in range(3): theta.grad = None; body()
        torch.cuda.current_stream().wait_stream(s); torch.cuda.synchronize(); theta.grad = None
        self.g = torch.cuda.CUDAGraph()
        with torch.cuda.graph(self.g): self.L, self.row, self.carry_out, self.tm = body()
        self.gbuf = theta.grad; theta.grad = None

    def run(self, sp, nz, plant, carry, valid, nsamp):
        _copy(self.sp, sp); _copy(self.nz, nz); _copy(self.plant, plant); _copy(self.carry, carry); self.valid.copy_(valid); self.nsamp.copy_(nsamp); self.g.replay()


def episode_pass(graph, bank, idx, nvalid, ffs, plant, gen, dtype, dev, nch, accumulate=None):
    B = idx.numel(); carry = init_carry(B, dtype, dev); rows = torch.zeros(B, dtype=dtype, device=dev); tilt = torch.zeros(B, dtype=dtype, device=dev); nsamp = (nvalid.to(dev).to(dtype) / 10.0)
    for k in range(nch):
        sp, nz, valid = chunk_inputs(bank, idx, nvalid, ffs, k, gen, plant, dtype, dev)
        graph.run(sp, nz, plant, carry, valid, nsamp)
        rows += graph.row; tilt = torch.maximum(tilt, graph.tm)
        if accumulate is not None and graph.grad: accumulate += graph.gbuf
        carry = [c.clone() for c in graph.carry_out]; carry[9].zero_(); carry[10].zero_()
    return rows, (tilt > 70).float().mean()


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--scen', default='A8'); ap.add_argument('--seed', type=int, default=0); ap.add_argument('--iters', type=int, default=300); ap.add_argument('--B', type=int, default=192)
    ap.add_argument('--lr', type=float, default=0.03); ap.add_argument('--val_every', type=int, default=25); ap.add_argument('--out', default='out_diff_scen.json'); ap.add_argument('--dev', default='cuda')
    a = ap.parse_args(); dtype = torch.float32; dev = a.dev; B = a.B; BV = 96
    d = np.load(OUT / f'bank_{a.scen}.npz'); bank = torch.from_numpy(d['bank']).pin_memory(); nvalid_all = torch.from_numpy(d['n_valid']).long(); NI = bank.shape[0]; nch = int(math.ceil(bank.shape[1] / CH))
    if bank.shape[1] < nch * CH: bank = torch.cat([bank, bank[:, -1:].expand(-1, nch * CH - bank.shape[1], -1)], 1).contiguous().pin_memory()
    gen = torch.Generator(device=dev).manual_seed(a.seed); vgen = torch.Generator(device=dev).manual_seed(12345); vn = torch.Generator(device=dev).manual_seed(777)
    theta = torch.zeros(len(NAMES), dtype=dtype, device=dev, requires_grad=True); opt = torch.optim.Adam([theta], lr=a.lr); lo, hi = LO.to(dtype=dtype, device=dev), HI.to(dtype=dtype, device=dev)
    dw = dw_params(dtype, dev)
    def rows_batch(g, n):
        idx = torch.randint(0, NI, (n,), generator=g, device=dev).cpu(); pl, ffs = make_plant(n, g, dtype, dev); pl['dw'] = dw; return idx, nvalid_all[idx], ffs, pl
    vidx, vnv, vffs, vpl = rows_batch(vgen, BV)
    tidx, tnv, tffs, tpl = rows_batch(gen, B)
    ts_ = chunk_inputs(bank, tidx, tnv, tffs, 0, gen, tpl, dtype, dev); gtrain = ChunkGraph(theta, B, dtype, dev, True, (ts_[0], ts_[1], tpl, init_carry(B, dtype, dev), ts_[2], (tnv.to(dev).to(dtype) / 10.0)), nch)
    vs_ = chunk_inputs(bank, vidx, vnv, vffs, 0, vn, vpl, dtype, dev); gval = ChunkGraph(theta, BV, dtype, dev, False, (vs_[0], vs_[1], vpl, init_carry(BV, dtype, dev), vs_[2], (vnv.to(dev).to(dtype) / 10.0)), nch)
    def validate():
        g_ = torch.Generator(device=dev).manual_seed(777)
        with torch.no_grad(): rows, crash = episode_pass(gval, bank, vidx, vnv, vffs, vpl, g_, dtype, dev, nch)
        w25 = torch.topk(rows, BV // 4).values.mean()
        return dict(rms_cm=100 * float(torch.sqrt(rows.mean())), w25_cm=100 * float(torch.sqrt(w25)), crash=float(crash), sel=float(w25))
    v0 = validate(); print('%s baseline validation (%d chunks, %d bank instances, %d rows): rms %.2f cm, worst-25%% %.2f cm, crash %.3f' % (a.scen, nch, NI, BV, v0['rms_cm'], v0['w25_cm'], v0['crash']), flush=True)
    best_val = v0['sel']; best_theta = theta.detach().clone(); log = []; t_start = time.time()
    for it in range(a.iters):
        t0 = time.time(); idx, nv, ffs, pl = rows_batch(gen, B); acc = torch.zeros_like(theta)
        rows, crash = episode_pass(gtrain, bank, idx, nv, ffs, pl, gen, dtype, dev, nch, accumulate=acc)
        theta.grad = acc; ok = bool(torch.isfinite(acc).all())
        if ok:
            torch.nn.utils.clip_grad_norm_([theta], 5.0); th_old = theta.detach().clone(); opt.step()
            with torch.no_grad(): dd = (theta - th_old).clamp(-0.1, 0.1); theta.copy_((th_old + dd).clamp(lo, hi))
        rec = dict(it=it, loss_rms_cm=100 * float(torch.sqrt(rows.mean())), crash=float(crash), ok=ok, sec=time.time() - t0, theta=theta.detach().cpu().tolist()); log.append(rec)
        if it % a.val_every == 0 or it == a.iters - 1:
            v = validate(); rec['val'] = v
            if v['sel'] < best_val and v['crash'] <= v0['crash']: best_val = v['sel']; best_theta = theta.detach().clone()
            print('it %3d loss %.2f cm crash %.3f | val %.2f cm w25 %.2f cm crash %.3f (best sel %.2e) | %.1fs/it' % (it, rec['loss_rms_cm'], rec['crash'], v['rms_cm'], v['w25_cm'], v['crash'], best_val, rec['sec']), flush=True)
    gains = {k: float(DEF[i] * math.exp(float(best_theta[i]))) for i, k in enumerate(NAMES)}
    json.dump(dict(scen=a.scen, seed=a.seed, gains=gains, theta=best_theta.cpu().tolist(), baseline_val=v0, best_val_sel=best_val, log=log, minutes=(time.time() - t_start) / 60), open(a.out, 'w'), indent=1)
    print('saved', a.out, gains)


if __name__ == '__main__': main()
