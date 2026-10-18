#!/usr/bin/env python3
"""FBL Stage 1: our own training of the learned-FBL downwash residual (supervised), leave-one-flight-out.

Model = the bundle's structure: a_dist(world) = ROM(z0, r0; C_D)·z_hat  +  R_flow · net_f([r0, z0]),   net_f = Linear(2,h) - Tanh - Linear(h,3).
The ROM acceleration is linear in C_D (bundle `downwash_force_fo`: C_D multiplies the integrated jet force), so it is computed once per sample (C_D = 1, S = 0.0997)
and scaled; C_D is a learnable scalar. Training target: our label a_res (world) minus the far-field baseline per axis, on samples with the neighbour above,
0.1 < z0 < 1.3 m, r0 < 0.6 m (50 Hz). This replaces the paper's KNODE trajectory loss for now (no FBL flights available); the authors' script can later replace
`train_fit` and nothing else (same checkpoint layout: net_f.* state_dict of the bundle's ODEFunc + C_D in a json).

  ~/.pyenv/versions/flying_robots/bin/python experiments/fbl/stage1_training/s1_fit_residual.py [--epochs 400] [--no-loo] [--tag v0]
Outputs experiments/fbl/out/stage1/<tag>/ : loo_results.json, report.md, fbl_residual_<tag>.pth (all-data fit), MODEL_CARD.json, fit_overview.png. Never overwrites an existing tag folder.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from common import bundle as B  # noqa: E402
from common import data as D  # noqa: E402
from common.vehicle import OUR  # noqa: E402

OUT = HERE.parent / "out" / "stage1"
VARIANTS = {   # name: (hidden units, use ROM, use NN)
    "zero": (0, False, False),
    "rom_only": (0, True, False),
    "nn_only_h9": (9, False, True),
    "rom_nn_h9": (9, True, True),
    "rom_nn_h4": (4, True, True),      # hardware-like small force network of the paper (4 neurons)
}


def build_dataset(step: int):
    b = B.load(); B.set_vehicle(b, OUR["mass"], OUR["arm_length"])
    df = D.load_bank(step=step)
    far = df[(df.r0 > 0.6) & (df.dz > 0.1)]
    base = np.array([far.a_res_x.mean(), far.a_res_y.mean(), far.a_res_z.mean()])
    reg = df[(df.z0 > 0.1) & (df.z0 < 1.3) & (df.r0 < 0.6)].reset_index(drop=True)
    z0 = reg.z0.values; r0 = reg.r0.values
    rom1 = []
    for i in range(0, len(reg), 4000):                                     # ROM acceleration along z for C_D = 1 (linear in C_D)
        z = torch.as_tensor(z0[i:i + 4000].copy(), dtype=torch.float64); r = torch.as_tensor(r0[i:i + 4000].copy(), dtype=torch.float64)
        rom1.append((-b.force.downwash_force_fo(z, r, torch.zeros_like(z), C_d=1.0, S=0.0997) / OUR["mass"]).numpy())
    rom1 = np.concatenate(rom1)
    rel = torch.tensor(np.stack([reg.dx.values, reg.dy.values, reg.dz.values], 1))
    zb = torch.zeros_like(rel); zb[:, 2] = 1.0
    Rf = b.dw.flow_frame_to_world(rel, zb).float()                          # (N,3,3) local flow frame -> world
    y = np.stack([reg.a_res_x.values, reg.a_res_y.values, reg.a_res_z.values], 1) - base
    return reg, dict(feat=torch.tensor(np.stack([r0, z0], 1), dtype=torch.float32), rom1=torch.tensor(rom1, dtype=torch.float32),
                     R=Rf, y=torch.tensor(y, dtype=torch.float32), base=base)


class Fit(nn.Module):
    def __init__(self, hidden, use_rom, use_nn, cd0=0.45):
        super().__init__()
        self.use_rom, self.use_nn = use_rom, use_nn
        self.net_f = nn.Sequential(nn.Linear(2, hidden), nn.Tanh(), nn.Linear(hidden, 3)) if use_nn else None
        self.c_d = nn.Parameter(torch.tensor(float(cd0)), requires_grad=use_rom)

    def forward(self, feat, rom1, R):
        out = torch.zeros(feat.shape[0], 3)
        if self.use_rom:
            out[:, 2] = out[:, 2] + self.c_d * rom1
        if self.use_nn:
            out = out + torch.bmm(R, self.net_f(feat).unsqueeze(2)).squeeze(2)
        return out


def train_fit(ds, idx, hidden, use_rom, use_nn, epochs, seed=0):
    """Fit one variant on the samples `idx` (the replaceable training step)."""
    torch.manual_seed(seed)
    m = Fit(hidden, use_rom, use_nn)
    if not (use_rom or use_nn):
        return m
    if use_rom and not use_nn:                                              # closed form: C_D = sum(y*rom)/sum(rom^2)
        r = ds["rom1"][idx]; yz = ds["y"][idx, 2]
        m.c_d.data = torch.tensor(float((r * yz).sum() / (r * r).sum()))
        return m
    opt = torch.optim.Adam(m.parameters(), lr=2e-2)
    w = torch.tensor([0.3, 0.3, 1.0])                                       # x/y labels are small and noisy: lower weight
    f, r1, R, y = ds["feat"][idx], ds["rom1"][idx], ds["R"][idx], ds["y"][idx]
    for _ in range(epochs):
        opt.zero_grad()
        loss = (((m(f, r1, R) - y) ** 2) * w).mean()
        loss.backward(); opt.step()
    return m


def metrics(y, p, mask=None):
    if mask is not None:
        y, p = y[mask], p[mask]
    if len(y) < 30:
        return None
    e = ((y - p) ** 2).mean() ** 0.5; e0 = (y ** 2).mean() ** 0.5
    sst = ((y - y.mean()) ** 2).sum()
    return dict(n=int(len(y)), rmse=float(e), rmse_zero=float(e0), r2=float(1 - ((y - p) ** 2).sum() / sst) if sst > 0 else float("nan"), pct_vs_zero=float(100 * (1 - e / e0)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--epochs", type=int, default=400); ap.add_argument("--step", type=int, default=10)
    ap.add_argument("--tag", default="v0"); ap.add_argument("--no-loo", action="store_true")
    a = ap.parse_args()
    out = OUT / a.tag
    if out.exists():
        sys.exit(f"refusing to overwrite {out}: use a new --tag")
    out.mkdir(parents=True)
    reg, ds = build_dataset(a.step)
    fl = reg.flight.values; flights = list(dict.fromkeys(fl))
    gate = ((reg.r0 < 0.2) & (reg.dz > 0)).values
    yz = ds["y"][:, 2].numpy()
    print(f"{len(reg)} samples, {len(flights)} flights, baseline {ds['base'].round(3)}")
    res = {"epochs": a.epochs, "step": a.step, "baseline": ds["base"].tolist(), "variants": {}}
    if not a.no_loo:
        for name, (h, rom, nnn) in VARIANTS.items():
            folds = {}
            for k, f in enumerate(flights):
                te = np.flatnonzero(fl == f); tr = np.flatnonzero(fl != f)
                m = train_fit(ds, torch.tensor(tr), h, rom, nnn, a.epochs)
                with torch.no_grad():
                    p = m(ds["feat"][te], ds["rom1"][te], ds["R"][te]).numpy()
                folds[f] = {"region_z": metrics(yz[te], p[:, 2]), "gate_z": metrics(yz[te], p[:, 2], gate[te]), "scenario": str(reg.scenario.values[te][0]), "c_d": float(m.c_d)}
            res["variants"][name] = folds
            gz = [v["gate_z"]["r2"] for v in folds.values() if v["gate_z"]]
            print(f"{name:11s} LOO gate-z R2 median {np.median(gz):.2f}  ({len(gz)} flights)", flush=True)
        (out / "loo_results.json").write_text(json.dumps(res, indent=1))
    # all-data fit of the main variant (rom_nn_h9) and the hardware-like one, saved bundle-compatible
    allidx = torch.arange(len(reg))
    for name in ("rom_nn_h9", "rom_nn_h4"):
        h, rom, nnn = VARIANTS[name]
        m = train_fit(ds, allidx, h, rom, nnn, a.epochs)
        with torch.no_grad():
            p = m(ds["feat"], ds["rom1"], ds["R"]).numpy()
        res.setdefault("all_data_fit", {})[name] = {"region_z": metrics(yz, p[:, 2]), "gate_z": metrics(yz, p[:, 2], gate), "c_d": float(m.c_d)}
        sd = {f"net_f.{k}": v for k, v in m.net_f.state_dict().items()}
        torch.save({"state_dict_net_f": sd, "c_d": float(m.c_d), "S": 0.0997, "hidden": h, "baseline_removed": ds["base"].tolist(), "vehicle": OUR}, out / f"fbl_residual_{a.tag}_{name}.pth")
    man_sha = hashlib.sha256(D.MANIFEST.read_bytes()).hexdigest()
    git = subprocess.run(["git", "rev-parse", "HEAD"], cwd=D.ROOT, capture_output=True, text=True).stdout.strip()
    (out / "MODEL_CARD.json").write_text(json.dumps({"tag": a.tag, "git_commit": git, "command": " ".join(sys.argv), "manifest": str(D.MANIFEST.relative_to(D.ROOT)), "manifest_sha256": man_sha,
        "bundle_sha256": B.bundle_hash(), "n_samples": int(len(reg)), "n_flights": len(flights), "epochs": a.epochs, "loss": "supervised weighted MSE on a_res minus far baseline (x,y weight 0.3, z 1.0)",
        "note": "own training, NOT the paper's KNODE loss; replace train_fit when the authors' script arrives"}, indent=1))
    (out / "all_data_fit.json").write_text(json.dumps(res["all_data_fit"], indent=1))
    print(json.dumps(res["all_data_fit"], indent=1))


if __name__ == "__main__":
    main()
