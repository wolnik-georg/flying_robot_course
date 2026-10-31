#!/usr/bin/env python3
"""Gradient tuning of the S0 gains through the differentiable S0 + plant (von Prittwitz-style window schedule, DiffTune-style projected update inside a box).
Option A: solo drone, no downwash, trajectories from traj.py (circle / figure-8 / move+landing / takeoff), domain randomisation over the plant (lag, state delay, gyro noise, kt, feedforward scale).
Usage: python3 train_s0.py --seed 0 --iters 300 --out ../out/diff_s0_s0.json"""
import argparse, json, math, sys, time
from pathlib import Path
import torch
sys.path.insert(0, str(Path(__file__).parent))
import s0_torch as T, s0_plant_torch as PT, traj

# name: (default, lo factor, hi factor) as in run_job.S0_PARAMS
PARAMS = dict(Pxy=(7., .25, 4), Pz=(7., .25, 4), Dxy=(4., .25, 4), Dz=(4., .25, 4), Iz=(1.5, .1, 4), KRxy=(.007, .25, 3), KRz=(.008, .25, 3), Kwxy=(.00115, .25, 3), Kwz=(.002, .25, 3), KIxy=(.03, .1, 4))
NAMES = list(PARAMS); DEF = torch.tensor([PARAMS[k][0] for k in NAMES]); LO = torch.tensor([math.log(PARAMS[k][1]) for k in NAMES]); HI = torch.tensor([math.log(PARAMS[k][2]) for k in NAMES])


def gains_from(theta, B, dtype, dev):
    p = T.K(DEF, dtype, dev) * torch.exp(theta)
    P, Pz, D, Dz, Iz, KRxy, KRz, Kwxy, Kwz, KI = p.unbind()
    e = lambda a, b, c: torch.stack([a, b, c]).expand(B, 3)
    return dict(Kpos_P=e(P, P, Pz), Kpos_D=e(D, D, Dz), Kpos_Iz=Iz, KR=e(KRxy, KRxy, KRz), Komega=e(Kwxy, Kwxy, Kwz), KI=e(KI, KI, T.K(0.03, dtype, dev)), Kpos_I_limit=T.K(2.0, dtype, dev))


def rot2quat(R):
    w = torch.sqrt(torch.clamp(1 + R[..., 0, 0] + R[..., 1, 1] + R[..., 2, 2], min=1e-6)) / 2
    return torch.stack([(R[..., 2, 1] - R[..., 1, 2]) / (4 * w), (R[..., 0, 2] - R[..., 2, 0]) / (4 * w), (R[..., 1, 0] - R[..., 0, 1]) / (4 * w), w], -1)


ROBUST = False     # --robust: loss = 0.5 * mean + 0.5 * mean of the worst 25 % of rows, plus a tilt hinge above 20 deg, and 30 % stress rows (lag 50-64 ms, state delay 40-60 ms)
LAM = 0.05
WIDE = False       # --wide: plant randomisation covers the full robustness variants (lag to 62 ms, state delay to 60 ms, kt 0.5-1, feedforward scale 0 / 0.4 / 1)


def make_batch(B, n_ms, gen, dtype, dev, kiz):
    """Window batch: reference arrays (n, B, 3), initial states, plant variant."""
    p = traj.sample_family(B, gen, dtype, dev); U = lambda lo, hi: lo + (hi - lo) * torch.rand(B, generator=gen, dtype=dtype, device=dev)
    fam = p['fam']; t0 = torch.where(fam == 3, torch.zeros(B, dtype=dtype, device=dev), torch.where(fam == 2, U(0.0, 0.6), U(0.0, 6.0)))
    t = t0[None] + torch.arange(n_ms, dtype=dtype, device=dev)[:, None] * 1e-3
    ref = traj.evaluate(p, t)
    r_ = torch.rand(B, generator=gen, dtype=dtype, device=dev)
    ffs = torch.where(r_ < 0.4, torch.full((B,), 0.4, dtype=dtype, device=dev), torch.ones(B, dtype=dtype, device=dev))
    if WIDE: ffs = torch.where(r_ < 0.15, torch.zeros(B, dtype=dtype, device=dev), ffs)
    sp = dict(pos=ref['pos'], vel=ref['vel'], acc=ref['acc'] * ffs[None, :, None], jerk=ref['jerk'] * ffs[None, :, None])
    a0 = ref['acc'][0] * ffs[:, None] + torch.tensor([0, 0, T.G], dtype=dtype, device=dev)       # feedforward the controller sees at t0
    th0 = PT.M * torch.linalg.norm(a0, dim=-1); yc = torch.tensor([0., 1., 0.], dtype=dtype, device=dev).expand(B, 3)
    xb = T.vnorm(T.vcross(yc, a0)); yb = T.vnorm(T.vcross(a0, xb)); zb = T.vcross(xb, yb); q0 = rot2quat(torch.stack([xb, yb, zb], -1))
    ground = (fam == 3)
    F0 = torch.where(ground[:, None], torch.full((B, 4), PT.M * T.G / 4, dtype=dtype, device=dev), (th0 / 4)[:, None].expand(B, 4))
    Fc0 = torch.where(ground[:, None], F0, 0.2 * 0.618 * torch.sqrt(F0 / 0.1098))
    q0 = torch.where(ground[:, None], torch.tensor([0., 0., 0., 1.], dtype=dtype, device=dev).expand(B, 4), q0)
    x0 = torch.where(ground[:, None], ref['pos'][0] * torch.tensor([1., 1., 0.], dtype=dtype, device=dev), ref['pos'][0]); v0 = torch.where(ground[:, None], torch.zeros_like(ref['vel'][0]), ref['vel'][0])
    ipz0 = torch.where(ground, torch.zeros(B, dtype=dtype, device=dev), ((4 * Fc0.mean(-1) / PT.M - th0 / PT.M) / kiz).clamp(-2, 2))
    md = torch.where(torch.rand(B, generator=gen, dtype=dtype, device=dev) < 0.4, torch.zeros(B, device=dev), torch.randint(10, 61 if WIDE else 41, (B,), generator=gen, device=dev).to(dtype)).long()
    stress = (torch.rand(B, generator=gen, dtype=dtype, device=dev) < 0.3) if ROBUST else torch.zeros(B, dtype=torch.bool, device=dev)
    md = torch.where(stress, torch.randint(40, 61, (B,), generator=gen, device=dev), md)
    plant = dict(lag=torch.where(stress, U(50, 64), U(26, 62 if WIDE else 50)), fmax=0.2758, quad=True, kt=U(0.5 if WIDE else 0.7, 1.0), gn=U(8, 20), md=md)
    return sp, plant, dict(x0=x0, v0=v0, q0=q0, F0=F0, Fc0=Fc0, ipz0=ipz0)


def make_noise(plant, n_ms, B, gen, dtype, dev):
    r = lambda *s: torch.randn(*s, generator=gen, dtype=dtype, device=dev)
    return dict(pos=5e-4 * r(n_ms, B, 3), vel=5e-3 * r(n_ms, B, 3), gyro=plant['gn'][None, :, None] * r(n_ms // 2 + 1, B, 3))


def loss_fn(theta, batch, dtype, dev, noise):
    sp, plant, init = batch; B = sp['pos'].shape[1]
    out, fin = PT.rollout_blocks(sp, gains_from(theta, B, dtype, dev), plant, dtype=dtype, device=dev, noise=noise, **init)
    err = torch.linalg.norm(out - sp['pos'][::10], dim=-1).clamp(max=1.0)            # (n/10, B) m, a diverged row counts as 1 m
    row = (err ** 2).mean(0)
    if ROBUST:
        rl = row + LAM * fin['tpen']; k = max(1, B // 4)
        L = 0.5 * rl.mean() + 0.5 * torch.topk(rl, k).values.mean()
    else: L = row.mean()
    return L, (fin['tilt_max'] > 70).float().mean(), fin['tilt_max'].max(), row


def _copy(dst, src):
    if isinstance(dst, dict):
        for k in dst: _copy(dst[k], src[k])
    elif torch.is_tensor(dst): dst.copy_(src)


class Graphed:
    """One CUDA graph of forward + backward for a fixed window length: the per-step kernels (thousands of tiny ones) are replayed with no launch overhead."""
    def __init__(self, theta, batch, noise, dtype, dev):
        self.theta, self.batch, self.noise = theta, batch, noise
        s = torch.cuda.Stream(); s.wait_stream(torch.cuda.current_stream())
        with torch.cuda.stream(s):
            for _ in range(3):
                theta.grad = None; L, c, tm, _ = loss_fn(theta, batch, dtype, dev, noise); L.backward()
        torch.cuda.current_stream().wait_stream(s); torch.cuda.synchronize()
        self.g = torch.cuda.CUDAGraph(); theta.grad = None
        with torch.cuda.graph(self.g):
            self.L, self.crash, self.tilt, _ = loss_fn(theta, batch, dtype, dev, noise); self.L.backward()

    def run(self, batch, noise):
        _copy(self.batch, batch); _copy(self.noise, noise); self.g.replay()
        return self.L.item(), self.crash.item(), self.theta.grad


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--seed', type=int, default=0); ap.add_argument('--iters', type=int, default=300); ap.add_argument('--Bs', type=int, nargs=3, default=[512, 512, 256], help='batch rows per window stage (the 4 s graph needs fewer rows: the laptop GPU has 3.7 GB)')
    ap.add_argument('--win', type=float, nargs=3, default=[0.5, 1.0, 2.0], help='window lengths [s] (schedule: 3 stages, advance on plateau)'); ap.add_argument('--lr', type=float, default=0.03)
    ap.add_argument('--patience', type=int, default=12); ap.add_argument('--out', default='out_diff_s0.json'); ap.add_argument('--dev', default='cuda'); ap.add_argument('--f64', action='store_true'); ap.add_argument('--val_every', type=int, default=20); ap.add_argument('--wide', action='store_true'); ap.add_argument('--robust', action='store_true')
    a = ap.parse_args(); globals()['WIDE'] = a.wide; globals()['ROBUST'] = a.robust; dtype = torch.float64 if a.f64 else torch.float32; dev = a.dev
    gen = torch.Generator(device=dev).manual_seed(a.seed); vgen = torch.Generator(device=dev).manual_seed(12345)
    theta = torch.zeros(len(NAMES), dtype=dtype, device=dev, requires_grad=True); opt = torch.optim.Adam([theta], lr=a.lr)
    lo, hi = LO.to(dtype=dtype, device=dev), HI.to(dtype=dtype, device=dev); stage = 0; best = float('inf'); since = 0; ema = None; log = []; best_theta = theta.detach().clone(); t_start = time.time()
    val = make_batch(128, 2500, vgen, dtype, dev, 1.5); val_noise = make_noise(val[1], 2500, 128, torch.Generator(device=dev).manual_seed(777), dtype, dev)
    def validate(th):
        with torch.no_grad(): L, c, tm, row = loss_fn(th, val, dtype, dev, val_noise)
        w25 = torch.topk(row, max(1, row.numel() // 4)).values.mean()
        return dict(rms_cm=100 * float(torch.sqrt(row.mean())), w25_cm=100 * float(torch.sqrt(w25)), crash=float(c), sel=float(w25 if ROBUST else row.mean()))
    v0 = validate(theta.detach()); print('baseline validation (2.5 s windows, 128 rows): rms %.2f cm, worst-25%% %.2f cm, crash %.3f' % (v0['rms_cm'], v0['w25_cm'], v0['crash']), flush=True); best_val = v0['sel']
    graphed = None; kiz = 1.5
    for it in range(a.iters):
        n_ms = int(a.win[stage] * 1000); Bs = a.Bs[stage]; t0 = time.time()
        if graphed is not None and graphed.n != n_ms:
            graphed = None; import gc; gc.collect(); torch.cuda.empty_cache()
        batch = make_batch(Bs, n_ms, gen, dtype, dev, kiz); noise = make_noise(batch[1], n_ms, Bs, gen, dtype, dev)
        if graphed is None or graphed.n != n_ms:
            graphed = None; torch.cuda.empty_cache()
            graphed = Graphed(theta, *[__import__('copy').deepcopy(x) if False else _clone(x) for x in (batch, noise)], dtype, dev); graphed.n = n_ms
        Lv, crash, g = graphed.run(batch, noise); ok = bool(torch.isfinite(g).all())
        if ok:
            torch.nn.utils.clip_grad_norm_([theta], 5.0); th_old = theta.detach().clone(); opt.step()
            with torch.no_grad():
                d = (theta - th_old).clamp(-0.1, 0.1); theta.copy_((th_old + d).clamp(lo, hi))           # per-step change of a gain <= 10 %, projection onto the box
            kiz = float(DEF[4] * math.exp(float(theta[4])))
        rms = 100 * math.sqrt(Lv); ema = rms if ema is None else 0.9 * ema + 0.1 * rms
        if ema < best * 0.99: best = ema; since = 0
        else: since += 1
        if since >= a.patience and stage < len(a.win) - 1: stage += 1; best = float('inf'); since = 0; print('-> window %.1f s' % a.win[stage], flush=True)
        rec = dict(it=it, win=a.win[stage], loss_rms_cm=rms, crash=crash, ok=ok, sec=time.time() - t0, theta=theta.detach().cpu().tolist()); log.append(rec)
        if it % a.val_every == 0 or it == a.iters - 1:
            v = validate(theta.detach()); rec['val_rms_cm'] = v['rms_cm']; rec['val_crash'] = v['crash']
            if v['sel'] < best_val and v['crash'] <= v0['crash']: best_val = v['sel']; best_theta = theta.detach().clone()
            print('it %3d win %.1f loss %.2f cm crash %.3f | val %.2f cm w25 %.2f cm crash %.3f (best sel %.2e) | %.2fs/it' % (it, a.win[stage], rms, crash, v['rms_cm'], v['w25_cm'], v['crash'], best_val, rec['sec']), flush=True)
    gains = {k: float(DEF[i] * math.exp(float(best_theta[i]))) for i, k in enumerate(NAMES)}
    json.dump(dict(seed=a.seed, gains=gains, theta=best_theta.cpu().tolist(), baseline_val=v0, best_val_sel=best_val, log=log, minutes=(time.time() - t_start) / 60), open(a.out, 'w'), indent=1)
    print('saved', a.out, gains)


def _clone(x):
    if isinstance(x, dict): return {k: _clone(v) for k, v in x.items()}
    if isinstance(x, tuple): return tuple(_clone(v) for v in x)
    return x.clone() if torch.is_tensor(x) else x


if __name__ == '__main__': main()
