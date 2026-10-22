#!/usr/bin/env python3
"""Step 3 of the FBL training task: KNODE training on our own flight logs (force only), C_D identification, thrust-input sensitivity, comparison with NS2 and the supervised fit.
Train: Omar-tree S0 flights of the NS2 collection (bank_ns2train_2026-10-16/17, bottom = cf5, old bent vehicle). Test: pilot flights of 2026-10-19/20 with the NN NOT applied (S0, S1; other bottom vehicle).
  ~/.pyenv/versions/flying_robots/bin/python experiments/fbl/stage4_knode/real_experiments.py --out experiments/fbl/out/stage4/real_v0 [--stage cd|train|sens|all]"""
import argparse, glob, json, sys, time
from pathlib import Path
import numpy as np, torch
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent))
from common.vehicle import OUR                    # noqa: E402
from knode_train import setup_ode, one_step_loss, train_sgd, predict   # noqa: E402
from real_data import load_flight, make_pairs, REPO  # noqa: E402

OUTLIERS = {"A2_2026-10-16_18-02-54", "A2_2026-10-16_18-26-13", "R1_2026-10-17_13-23-04"}
WX_FORCE = torch.tensor([1.0] * 10 + [0.0] * 3)          # paper weights, body rates 0 (force-only training, no torque input)


def train_flights():
    out = []
    for b in ("bank_ns2train_2026-10-16", "bank_ns2train_2026-10-17"):
        for d in sorted(glob.glob(str(REPO / "experiments/logs" / b / "*/"))):
            nm = Path(d).name
            if nm in OUTLIERS: continue
            f = glob.glob(d + "/*_merged_usd.csv")
            if f: out.append(f[0])
    return out


def test_flights():
    cache = REPO / "experiments/results/nn_vs_indi_2026-10-22/cache"; man = {}
    for b in sorted(glob.glob(str(REPO / "experiments/logs/bank_*2026-10-19"))) + sorted(glob.glob(str(REPO / "experiments/logs/bank_*2026-10-20"))):
        for fl in json.load(open(Path(b) / "training_manifest.json"))["flights"]: man[fl["folder"]] = REPO / fl["path"]
    res = []
    for z in sorted(cache.glob("*.npz")):
        d = np.load(z, allow_pickle=True)
        if float(d["coef_pred"]) > -0.3 and str(d["folder"]) in man: res.append(str(man[str(d["folder"])]))
    return res


def pairs(files, **kw):
    P = [make_pairs(load_flight(f), **kw) for f in files]; P = [p for p in P if len(p["y0"])]
    cat = lambda k: np.concatenate([p[k] for p in P])
    return dict(y0=torch.tensor(cat("y0")).unsqueeze(1), target=torch.tensor(cat("target")), label=cat("label"), nn=cat("nn"), r=cat("r"), dz=cat("dz"), dt=P[0]["dt"], fid=np.concatenate([np.full(len(p["y0"]), i) for i, p in enumerate(P)]), n_flights=len(P))


def residual_pred(b, ode, y0, c_d, chunk=20000):
    """predicted residual acceleration [3:6] of the model (ROM with c_d + nets) minus the model without downwash (C_d 0)."""
    out = []; old_cd = b.nominal.C_d
    for i in range(0, len(y0), chunk):
        y = y0[i:i + chunk]
        with torch.no_grad():
            b.nominal.C_d = float(c_d); f_m = ode.forward(torch.tensor(0.0), y)[:, 0, 3:6]
            b.nominal.C_d = 0.0; f_n = ode.nominal_model.forward(torch.tensor(0.0), y)[:, 0, 3:6]
        out.append((f_m - f_n).numpy())
    b.nominal.C_d = old_cd                                                     # the bundle's C_d is a module global: restore it
    return np.concatenate(out)


def score(pred, label, mask=None):
    p = pred[:, 2]; y = label[:, 2]
    if mask is not None: p, y = p[mask], y[mask]
    e = p - y; return dict(n=int(len(y)), rmse=float(np.sqrt((e ** 2).mean())), r2=float(1 - (e ** 2).mean() / y.var()), corr=float(np.corrcoef(p, y)[0, 1]), bias=float(e.mean()))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--stage", default="all"); ap.add_argument("--iters", type=int, default=1500)
    ap.add_argument("--thrust", default="cmd"); ap.add_argument("--thrust-scale", type=float, default=1.0); ap.add_argument("--stride", type=int, default=5); ap.add_argument("--tag", default="base"); ap.add_argument("--loss", default="force", choices=["force", "vel"]); ap.add_argument("--lr", type=float, default=1e-2); a = ap.parse_args()
    global WX_FORCE
    if a.loss == "vel": WX_FORCE = torch.tensor([0.0] * 3 + [1.0] * 3 + [0.0] * 7)          # velocity only (the residual force acts on v; p and q are noise here)
    out = a.out if a.out.is_absolute() else REPO / a.out; out.mkdir(parents=True, exist_ok=True)
    kw = dict(stride=a.stride, thrust=a.thrust, thrust_scale=a.thrust_scale)
    tr_files, te_files = train_flights(), test_flights()
    rng = np.random.default_rng(0); val_idx = set(rng.choice(len(tr_files), size=max(4, len(tr_files) // 8), replace=False).tolist())
    TR = pairs([f for i, f in enumerate(tr_files) if i not in val_idx], **kw); VA = pairs([f for i, f in enumerate(tr_files) if i in val_idx], **kw); TE = pairs(te_files, **kw)
    print(f"train {len(TR['y0'])} rows / {TR['n_flights']} flights, val {len(VA['y0'])} / {VA['n_flights']}, test {len(TE['y0'])} / {TE['n_flights']} (dt {TR['dt']:.3f} s, thrust {a.thrust} x{a.thrust_scale})", flush=True)
    res = json.load(open(out / "results.json")) if (out / "results.json").exists() else {}
    # E1: C_D identification, nets at zero, one-step loss (force weights) on train + val rows
    b, ode = setup_ode(0.3, vehicle=OUR)
    for net in (ode.net_f, ode.net_t):
        for p in net.parameters(): torch.nn.init.zeros_(p)
    grid = [0.0, 0.236, 0.3, 0.4, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.85, 1.0]; cd = {}
    for c in grid:
        b.nominal.C_d = c; cd[c] = dict(train=one_step_loss(ode, TR["y0"], TR["target"], TR["dt"], wx=WX_FORCE), val=one_step_loss(ode, VA["y0"], VA["target"], VA["dt"], wx=WX_FORCE))
        rp = residual_pred(b, ode, TE["y0"], c); cd[c]["test_corr_label"] = score(rp, TE["label"])["corr"]; cd[c]["test_rmse_label"] = score(rp, TE["label"])["rmse"]
        print(f"C_D {c:5.3f}  one-step loss train {cd[c]['train']:.4e} val {cd[c]['val']:.4e}  | test label RMSE {cd[c]['test_rmse_label']:.3f} corr {cd[c]['test_corr_label']:.3f}", flush=True)
    c_star = min((c for c in grid if c > 0), key=lambda c: cd[c]["val"]); res[f"cd_grid_{a.tag}"] = {str(k): v for k, v in cd.items()}; res[f"c_d_star_{a.tag}"] = c_star
    print("C_D* (min validation loss) =", c_star, flush=True)
    if a.stage == "cd": json.dump(res, open(out / "results.json", "w"), indent=1); return
    # E2: force-only KNODE training at C_D*
    b, ode = setup_ode(c_star, vehicle=OUR, train_torque_net=False)
    t0 = time.time(); st, hist = train_sgd(ode, TR["y0"], TR["target"], TR["dt"], val=(VA["y0"], VA["target"]), iters=a.iters, wx=WX_FORCE, lr=a.lr)
    torch.save({"state_dict": ode.state_dict(), "c_d": c_star, "thrust": a.thrust, "thrust_scale": a.thrust_scale, "stride": a.stride}, out / f"knode_{a.tag}.pth")
    rp = residual_pred(b, ode, TE["y0"], c_star); gate = (TE["r"] < 0.2)
    zero = np.zeros_like(TE["label"]); rom = None
    b2, ode0 = setup_ode(c_star, vehicle=OUR)
    for net in (ode0.net_f, ode0.net_t):
        for p in net.parameters(): torch.nn.init.zeros_(p)
    rp0 = residual_pred(b2, ode0, TE["y0"], c_star)
    res[a.tag] = dict(train_seconds=time.time() - t0, final_val=one_step_loss(ode, VA["y0"], VA["target"], VA["dt"], wx=WX_FORCE), val_rom_only=cd[c_star]["val"], c_d=c_star, thrust=a.thrust, thrust_scale=a.thrust_scale,
                      knode=score(rp, TE["label"]), knode_gate=score(rp, TE["label"], gate), rom_only=score(rp0, TE["label"]), rom_only_gate=score(rp0, TE["label"], gate),
                      ns2_v2=score(TE["nn"], TE["label"]), ns2_v2_gate=score(TE["nn"], TE["label"], gate), n_test=int(len(rp)))
    print(json.dumps({k: v for k, v in res[a.tag].items() if isinstance(v, dict)}, indent=0)[:1500])
    json.dump(res, open(out / "results.json", "w"), indent=1)


if __name__ == "__main__":
    main()
