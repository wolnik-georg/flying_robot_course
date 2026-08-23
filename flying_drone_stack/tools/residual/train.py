#!/usr/bin/env python3
"""Train the residual model and export weights in the firmware's upload format.

    # real data (merged uSD logs from tools/merge_usd_logs.py)
    python3 train.py merged_*.csv -o weights/a1_geometric.npz

    # pipeline rehearsal with no flight data -- NOT a result
    python3 train.py --synthetic -o weights/synthetic.npz

The exported `.npz` carries the flat 987-float vector the drone expects, plus the normalisation
that was folded into it and enough provenance to say later which flights a set of weights came
from. Weights with no provenance are weights nobody can defend in a thesis.

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
from model import DeepSets, N_WEIGHTS, fold_normalisation, firmware_forward  # noqa: E402


def rmse(a, b):
    return float(np.sqrt(np.mean((a - b) ** 2)))


def evaluate(model, rel, mask, y, device):
    model.eval()
    with torch.no_grad():
        pred = model(torch.from_numpy(rel).to(device),
                     torch.from_numpy(mask).to(device)).cpu().numpy()
    return pred


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
                    help="drop samples below this altitude (ground effect is a different force)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = ap.parse_args()

    if not args.logs and not args.synthetic:
        ap.error("give merged CSVs, or --synthetic to rehearse the pipeline")

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    if args.synthetic:
        print("=== SYNTHETIC DATA -- pipeline rehearsal only, not a result ===")
        rel, mask, y = dataset.synthetic(seed=args.seed)
        provenance = ["<synthetic>"]
    else:
        rel, mask, y, _ = dataset.build(args.logs, z_floor=args.z_floor)
        provenance = list(args.logs)

    mu, sigma = dataset.normalisation(rel, mask)
    print(f"normalisation mu    = {np.array2string(mu, precision=4)}")
    print(f"normalisation sigma = {np.array2string(sigma, precision=4)}")
    if np.any(sigma == 1.0):
        print("  note: a sigma of exactly 1.0 means that input never varied in this data. "
              "Check the flights actually excited it before trusting the model there.")

    tr, va = dataset.split(len(y), seed=args.seed)
    print(f"train {tr.sum()} / val {va.sum()} (contiguous blocks, not shuffled samples)")

    # Normalisation is applied here during training and folded into layer 1 at export, so the
    # trained network and the shipped network are arithmetically the same function.
    rel_n = ((rel - mu) / sigma).astype(np.float32) * mask[..., None]

    device = torch.device(args.device)
    model = DeepSets().to(device)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs)
    loss_fn = torch.nn.SmoothL1Loss(beta=0.2)   # Huber: log spikes should not steer the fit

    X = torch.from_numpy(rel_n[tr]).to(device)
    M = torch.from_numpy(mask[tr]).to(device)
    Y = torch.from_numpy(y[tr]).to(device)
    Xv = torch.from_numpy(rel_n[va]).to(device)
    Mv = torch.from_numpy(mask[va]).to(device)
    Yv = torch.from_numpy(y[va]).to(device)

    n = len(X)
    best, best_state = float("inf"), None
    t0 = time.time()
    for ep in range(args.epochs):
        model.train()
        perm = torch.randperm(n, device=device)
        tot = 0.0
        for i in range(0, n, args.batch):
            idx = perm[i:i + args.batch]
            opt.zero_grad()
            loss = loss_fn(model(X[idx], M[idx]), Y[idx])
            loss.backward()
            opt.step()
            tot += float(loss) * len(idx)
        sched.step()

        model.eval()
        with torch.no_grad():
            vp = model(Xv, Mv)
            v_rmse = float(torch.sqrt(torch.mean((vp - Yv) ** 2)))
        if v_rmse < best:
            best = v_rmse
            best_state = {k: v.detach().clone() for k, v in model.state_dict().items()}
        if ep % 20 == 0 or ep == args.epochs - 1:
            print(f"  epoch {ep:4d}  train {tot / n:.5f}  val RMSE {v_rmse:.4f} m/s^2"
                  f"{'  *' if v_rmse == best else ''}")

    model.load_state_dict(best_state)
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
    exported = fold_normalisation(model.cpu(), mu, sigma)
    w = exported.flatten()

    # The folded model must agree with the trained one on real inputs, or the fold is wrong.
    # Checked against the *firmware's* NumPy evaluation, not just torch, so the distance guards
    # and the flat layout are exercised too.
    idx = np.random.default_rng(0).choice(len(y), size=min(200, len(y)), replace=False)
    with torch.no_grad():
        ref = model(torch.from_numpy(rel_n[idx]), torch.from_numpy(mask[idx])).numpy()
    got = np.array([firmware_forward(
        w, [(rel[i, k, :3], rel[i, k, 3:]) for k in range(rel.shape[1]) if mask[i, k] > 0],
        apply_clamp=False)[0] for i in idx])
    err = float(np.abs(ref - got).max())
    print(f"fold check: max |trained - exported| = {err:.2e} m/s^2")
    if err > 1e-3:
        raise SystemExit("Normalisation fold does not reproduce the trained model. Do not "
                         "upload these weights.")

    n_clamp = sum(firmware_forward(w, [(rel[i, k, :3], rel[i, k, 3:])
                                       for k in range(rel.shape[1]) if mask[i, k] > 0])[1]
                  for i in idx)
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
        "val_rmse": best,
        "baseline_rmse": base,
        "epochs": args.epochs,
        "seed": args.seed,
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, weights=w.astype(np.float32), mu=mu, sigma=sigma,
             meta=json.dumps(meta))
    print(f"wrote {out}  ({N_WEIGHTS} weights)")
    if args.synthetic:
        print("REMINDER: synthetic weights. Do not fly them as if they meant anything.")


if __name__ == "__main__":
    main()
