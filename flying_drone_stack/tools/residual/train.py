#!/usr/bin/env python3
"""Train the residual model and export weights in the firmware's upload format.

    # real data (merged uSD logs from tools/merge_usd_logs.py)
    python3 train.py merged_*.csv -o weights/a1_geometric.npz

    # pipeline rehearsal with no flight data -- NOT a result
    python3 train.py --synthetic -o weights/synthetic.npz

The exported `.npz` carries the flat 19297-float vector the drone expects (`phi_S | phi_L | phi_G
| rho_S | rho_L`, `phi_L`/`rho_L` zeroed -- see `model.py`), plus the two normalisation stats
folded into it and enough provenance to say later which flights a set of weights came from.
Weights with no provenance are weights nobody can defend in a thesis.

2026-09-17 rewrite for the Neural-Swarm2 architecture (`model.NeuralSwarm2`, `dataset.build`'s
5-tuple). The two decisions that blocked this were already made in `dataset.py`'s own 2026-09-15
rewrite, not here: the target is `a_res_z` alone because the network only ever predicts a Z force
(a scoping fact of Strategy 2, not a loader choice), and `z_floor` now defaults to 0.0 (off)
because `phi_G` models ground effect explicitly -- dropping low-altitude rows would starve exactly
the input meant to learn it. What was actually missing here: the `DeepSets` import (removed from
`model.py`), the single `normalisation()` call (now two: `normalisation_rel` for the 6-wide
neighbour term, `normalisation_ground` for the 4-wide ground term -- `fold_normalisation` needs
both, separately, because a relative position and a vehicle's own height are not the same
distribution), the `ground` tensor the model's `forward()` now takes as a third argument, and the
grams<->acceleration unit conversion (the network's native output unit is the reference's grams;
`a_res_*` is acceleration -- see `--mass` below).

Needs torch (system python3 has it; the pyenv `flying_robots` env does not).
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))

import dataset  # noqa: E402
import model as M  # noqa: E402
from model import NeuralSwarm2, N_WEIGHTS, fold_normalisation, firmware_forward  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="*", help="merged CSVs from merge_usd_logs.py")
    ap.add_argument("--synthetic", action="store_true",
                    help="train on a made-up downwash function to rehearse the pipeline")
    ap.add_argument("-o", "--out", default="weights.npz")
    ap.add_argument("--epochs", type=int, default=300)
    ap.add_argument("--batch", type=int, default=1024)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--weight-decay", type=float, default=1e-5)
    ap.add_argument("--z-floor", type=float, default=0.0,
                    help="drop samples below this altitude. Default 0.0 (off): phi_G models "
                         "ground effect explicitly, so dropping low rows starves that input. "
                         "Only set this for a different analysis that deliberately wants "
                         "ground-effect samples excluded.")
    ap.add_argument("--mass", type=float, default=M.DEFAULT_MASS,
                    help=f"kg, for the grams<->m/s^2 conversion (default {M.DEFAULT_MASS} = "
                         f"g_indi_mass's default). Pass the REAL per-platform mass the logs "
                         f"were flown with, or the exported weights' unit conversion is wrong "
                         f"even though the network itself trained fine.")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = ap.parse_args()

    if not args.logs and not args.synthetic:
        ap.error("give merged CSVs, or --synthetic to rehearse the pipeline")

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    if args.synthetic:
        print("=== SYNTHETIC DATA -- pipeline rehearsal only, not a result ===")
        rel, mask, ground, y = dataset.synthetic(seed=args.seed)
        provenance = ["<synthetic>"]
    else:
        rel, mask, ground, y, _ = dataset.build(args.logs, z_floor=args.z_floor)
        provenance = list(args.logs)

    mu_rel, sigma_rel = dataset.normalisation_rel(rel, mask)
    mu_g, sigma_g = dataset.normalisation_ground(ground)
    print(f"normalisation (rel)    mu={np.array2string(mu_rel, precision=4)} "
          f"sigma={np.array2string(sigma_rel, precision=4)}")
    print(f"normalisation (ground) mu={np.array2string(mu_g, precision=4)} "
          f"sigma={np.array2string(sigma_g, precision=4)}")
    if np.any(sigma_rel == 1.0) or np.any(sigma_g == 1.0):
        print("  note: a sigma of exactly 1.0 means that input never varied in this data. "
              "Check the flights actually excited it before trusting the model there.")

    tr, va = dataset.split(len(y), seed=args.seed)
    print(f"train {tr.sum()} / val {va.sum()} (contiguous blocks, not shuffled samples)")

    # Normalisation is applied here during training and folded into layer 1 at export, so the
    # trained network and the shipped network are arithmetically the same function. Masked
    # neighbour slots are zeroed after normalising too -- redundant with forward()'s own
    # post-phi masking (0 * phi_s(x) == 0 regardless of x), kept only so a masked slot's *input*
    # reads as the empty padding it represents, not a stray normalised value.
    rel_n = ((rel - mu_rel) / sigma_rel).astype(np.float32) * mask[..., None]
    ground_n = ((ground - mu_g) / sigma_g).astype(np.float32)

    # The network's native output unit is the reference's grams; a_res_* is acceleration. This is
    # a fixed, known linear scale (not learned), so it's applied to the prediction at loss time
    # rather than to the target -- keeps every printed RMSE in the same m/s^2 the rest of the
    # project's logs and plots use.
    accel_per_gram = M.GRAMS_TO_NEWTONS / args.mass

    device = torch.device(args.device)
    net = NeuralSwarm2().to(device)
    opt = torch.optim.Adam(net.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs)
    loss_fn = torch.nn.SmoothL1Loss(beta=0.2)   # Huber: log spikes should not steer the fit

    X = torch.from_numpy(rel_n[tr]).to(device)
    M_ = torch.from_numpy(mask[tr]).to(device)
    G = torch.from_numpy(ground_n[tr]).to(device)
    Y = torch.from_numpy(y[tr]).to(device)
    Xv = torch.from_numpy(rel_n[va]).to(device)
    Mv = torch.from_numpy(mask[va]).to(device)
    Gv = torch.from_numpy(ground_n[va]).to(device)
    Yv = torch.from_numpy(y[va]).to(device)

    n = len(X)
    best, best_state = float("inf"), None
    t0 = time.time()
    for ep in range(args.epochs):
        net.train()
        perm = torch.randperm(n, device=device)
        tot = 0.0
        for i in range(0, n, args.batch):
            idx = perm[i:i + args.batch]
            opt.zero_grad()
            pred_accel = net(X[idx], M_[idx], G[idx]) * accel_per_gram
            loss = loss_fn(pred_accel, Y[idx])
            loss.backward()
            opt.step()
            tot += loss.detach().item() * len(idx)
        sched.step()

        net.eval()
        with torch.no_grad():
            vp = net(Xv, Mv, Gv) * accel_per_gram
            v_rmse = float(torch.sqrt(torch.mean((vp - Yv) ** 2)))
        if v_rmse < best:
            best = v_rmse
            best_state = {k: v.detach().clone() for k, v in net.state_dict().items()}
        if ep % 20 == 0 or ep == args.epochs - 1:
            print(f"  epoch {ep:4d}  train {tot / n:.5f}  val RMSE {v_rmse:.4f} m/s^2"
                  f"{'  *' if v_rmse == best else ''}")

    net.load_state_dict(best_state)
    print(f"best val RMSE {best:.4f} m/s^2  ({time.time() - t0:.1f}s)")

    # The number that decides whether the model is worth deploying at all: a model that beats
    # predicting zero by nothing has learned nothing, however small its RMSE looks.
    base = float(np.sqrt(np.mean(y[va] ** 2)))
    print(f"baseline (predict zero) {base:.4f} m/s^2  ->  "
          f"{100 * (1 - best / base):.1f}% reduction")
    if best >= base:
        print("  WARNING: the model is no better than predicting zero. Do not deploy it.",
              file=sys.stderr)

    # ── Export ──────────────────────────────────────────────────────────────
    exported = fold_normalisation(net.cpu(), mu_rel, sigma_rel, mu_g, sigma_g)
    w = exported.flatten()

    # The folded model must agree with the trained one on real inputs, or the fold is wrong.
    # Checked against the *firmware's* NumPy evaluation, not just torch, so the gate, the ground
    # term and the flat layout are all exercised, not just the two nn.Sequential stacks.
    idx = np.random.default_rng(0).choice(len(y), size=min(200, len(y)), replace=False)
    with torch.no_grad():
        ref = (net(torch.from_numpy(rel_n[idx]), torch.from_numpy(mask[idx]),
                    torch.from_numpy(ground_n[idx])).numpy() * accel_per_gram)
    # firmware_forward derives the ground term from own_z/own_vel itself; ground[:,0] = 0 - own_z
    # and ground[:,1:] = -own_vel, so both are recoverable from the tensor dataset.build already
    # produced rather than needing their own separate return.
    own_z = -ground[idx, 0]
    own_vel = -ground[idx, 1:]
    got = np.array([firmware_forward(
        w, [(rel[i, k, :3], rel[i, k, 3:]) for k in range(rel.shape[1]) if mask[i, k] > 0],
        own_z[j], own_vel[j], mass=args.mass, apply_clamp=False)[0]
        for j, i in enumerate(idx)])
    err = float(np.abs(ref - got).max())
    print(f"fold check: max |trained - exported| = {err:.2e} m/s^2")
    if err > 1e-3:
        raise SystemExit("Normalisation fold does not reproduce the trained model. Do not "
                         "upload these weights.")

    n_clamp = sum(firmware_forward(
        w, [(rel[i, k, :3], rel[i, k, 3:]) for k in range(rel.shape[1]) if mask[i, k] > 0],
        own_z[j], own_vel[j], mass=args.mass)[1]
        for j, i in enumerate(idx))
    if n_clamp:
        print(f"  WARNING: {n_clamp}/{len(idx)} sampled predictions hit the 8 m/s^2 output "
              f"clamp. That is a fault signature, not a strong prediction.", file=sys.stderr)

    try:
        rev = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"],
                                      cwd=Path(__file__).resolve().parent,
                                      stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        rev = "unknown"

    meta = {
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "git": rev,
        "sources": provenance,
        "synthetic": bool(args.synthetic),
        "n_samples": int(len(y)),
        "mass_kg": args.mass,
        "val_rmse": best,
        "baseline_rmse": base,
        "epochs": args.epochs,
        "seed": args.seed,
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, weights=w.astype(np.float32),
             mu_rel=mu_rel, sigma_rel=sigma_rel, mu_ground=mu_g, sigma_ground=sigma_g,
             meta=json.dumps(meta))
    print(f"wrote {out}  ({N_WEIGHTS} weights)")
    if args.synthetic:
        print("REMINDER: synthetic weights. Do not fly them as if they meant anything.")


if __name__ == "__main__":
    main()
