#!/usr/bin/env python3
"""Ceiling of a residual that depends only on the flatness-preserving features (r, dz) (what the FBL structure allows) versus a richer input set, on our data:
a small MLP is fitted to the label (a_res z) on the training flights and scored on the test flights (same split as real_experiments.py). Not a controller; an upper bound.
  ~/.pyenv/versions/flying_robots/bin/python experiments/fbl/stage4_knode/ceiling.py"""
import json, sys
from pathlib import Path
import numpy as np, torch
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent))
import real_experiments as RE   # noqa: E402
torch.set_num_threads(4)
kw = dict(stride=5, thrust="cmd", thrust_scale=1.0)
tr = RE.train_flights(); te = RE.test_flights()
TR = RE.pairs(tr, **kw); TE = RE.pairs(te, **kw)
def feats(P, rich):
    y = P["y0"][:, 0, :].numpy(); r = P["r"]; dz = P["dz"]
    f = [r, dz]
    if rich: f += [y[:, 30 - 13 + 3] - y[:, 3], y[:, 30 - 13 + 4] - y[:, 4], y[:, 30 - 13 + 5] - y[:, 5], y[:, 5], y[:, 2]]   # neighbour minus ego velocity (3), ego vz, ego z
    return np.stack(f, 1).astype(np.float32)
out = {}
for name, rich in (("(r, dz) only", False), ("(r, dz) + rel. velocity + own vz, z (NS2-like)", True)):
    Xtr, Xte = feats(TR, rich), feats(TE, rich); mu, sd = Xtr.mean(0), Xtr.std(0) + 1e-6
    Xtr = torch.tensor((Xtr - mu) / sd); Xte = torch.tensor((Xte - mu) / sd); ytr = torch.tensor(TR["label"][:, 2], dtype=torch.float32)
    torch.manual_seed(0); net = torch.nn.Sequential(torch.nn.Linear(Xtr.shape[1], 64), torch.nn.Tanh(), torch.nn.Linear(64, 64), torch.nn.Tanh(), torch.nn.Linear(64, 1))
    opt = torch.optim.Adam(net.parameters(), 3e-3)
    for ep in range(40):
        perm = torch.randperm(len(Xtr))
        for i in range(0, len(Xtr), 1024):
            idx = perm[i:i + 1024]; loss = torch.nn.functional.smooth_l1_loss(net(Xtr[idx])[:, 0], ytr[idx], beta=0.2); opt.zero_grad(); loss.backward(); opt.step()
    with torch.no_grad(): p = net(Xte)[:, 0].numpy()
    out[name] = RE.score(np.c_[p, p, p], TE["label"]); print(name, out[name], flush=True)
json.dump(out, open(HERE.parents[1] / "fbl/out/stage4/ceiling.json", "w"), indent=1)
