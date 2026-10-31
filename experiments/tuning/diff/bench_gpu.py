import sys, time, torch, numpy as np
sys.path.insert(0, '.')
import s0_plant_torch as PT
def mk(B, n, dtype, dev):
    t = torch.arange(n, dtype=dtype, device=dev)[:, None, None] * 1e-3; w = 1.0
    p = torch.cat([0.5 * torch.sin(w * t), 0.25 * torch.sin(2 * w * t), 0.5 + 0 * t], -1).expand(n, B, 3).contiguous()
    v = torch.cat([0.5 * w * torch.cos(w * t), 0.5 * w * torch.cos(2 * w * t), 0 * t], -1).expand(n, B, 3).contiguous()
    a = torch.cat([-0.5 * w * w * torch.sin(w * t), -w * w * torch.sin(2 * w * t), 0 * t], -1).expand(n, B, 3).contiguous()
    j = torch.cat([-0.5 * w ** 3 * torch.cos(w * t), -2 * w ** 3 * torch.cos(2 * w * t), 0 * t], -1).expand(n, B, 3).contiguous()
    return dict(pos=p, vel=v, acc=a, jerk=j)
for dev, dtype, B, n in [('cpu', torch.float32, 64, 1000), ('cuda', torch.float32, 64, 1000), ('cuda', torch.float32, 1024, 1000)]:
    sp = mk(B, n, dtype, dev)
    f = lambda v: torch.as_tensor(v, dtype=dtype, device=dev)
    g = dict(Kpos_P=f([7., 7., 7.]).expand(B, 3).clone().requires_grad_(), Kpos_D=f([4., 4., 4.]).expand(B, 3).clone().requires_grad_(), Kpos_Iz=f(1.5), KR=f([.007, .007, .008]).expand(B, 3).clone().requires_grad_(),
             Komega=f([.00115, .00115, .002]).expand(B, 3).clone().requires_grad_(), KI=f([.03, .03, .03]).expand(B, 3).clone().requires_grad_(), Kpos_I_limit=f(2.0))
    x0 = sp['pos'][0].clone(); v0 = sp['vel'][0].clone()
    if dev == 'cuda': torch.cuda.synchronize()
    t0 = time.time(); out, fin = PT.rollout(sp, g, dict(lag=30., fmax=0.2758, quad=True), dtype=dtype, device=dev, x0=x0, v0=v0, F0=torch.full((B, 4), PT.M * 9.81 / 4, dtype=dtype, device=dev))
    loss = ((out - sp['pos'][::10]) ** 2).mean(); 
    if dev == 'cuda': torch.cuda.synchronize()
    t1 = time.time(); loss.backward()
    if dev == 'cuda': torch.cuda.synchronize()
    t2 = time.time(); gk = g['Kpos_P'].grad
    print(dev, dtype, 'B', B, 'n', n, 'fwd %.1fs bwd %.1fs' % (t1 - t0, t2 - t1), 'loss %.3e' % loss.item(), 'grad finite', bool(torch.isfinite(gk).all()), 'grad Kpos_P[0]', gk[0].tolist(), 'mem MB', (torch.cuda.max_memory_allocated() // 2**20) if dev == 'cuda' else '-')
