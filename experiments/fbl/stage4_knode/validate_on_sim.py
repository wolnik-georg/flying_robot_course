#!/usr/bin/env python3
"""Check of our KNODE training script on synthetic flights with a known downwash (Task 1, step 2).
Truth: bundle dynamics with ROM C_d 0.55; model under training: ROM C_d 0.3 + two 9-neuron nets (the bundle's ODEFunc). Training data like the paper's simulation recipe:
18 s = 4 segments (stacked and static top, dz 0.3 and 0.4, speed 0.4 m/s, 200 Hz) flown by a nominal controller; test: stacked and static top at dz 0.2 (OOD), 0.3, 0.4, 0.5 (OOD).
Compared: Nominal (no downwash), ROM only, the authors' sample checkpoint, ours. Metric as the paper's Table 1: RMSE of the acceleration (m/s^2) and angular acceleration (rad/s^2) residual.
  ~/.pyenv/versions/flying_robots/bin/python experiments/fbl/stage4_knode/validate_on_sim.py --out experiments/fbl/out/stage4/sim_check_v0"""
import argparse, json, sys, time
from pathlib import Path
import numpy as np, torch
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent))
from knode_sim import simulate           # noqa: E402
from knode_train import make_pairs, one_step_loss, setup_ode, train   # noqa: E402
from common import bundle as B           # noqa: E402

C_ROM, C_GT = 0.3, 0.55


def deriv(b, f, y, c_d):
    old = b.nominal.C_d; b.nominal.C_d = float(c_d)
    try:
        with torch.no_grad(): return f(torch.tensor(0.0), y)[:, 0, :]
    finally: b.nominal.C_d = old


def residual_errors(b, ode, y):
    """RMSE of acceleration [3:6] and angular acceleration [10:13] residual vs the truth, for Nominal (0), ROM only, and the model (ROM + nets)."""
    f_gt = deriv(b, ode.nominal_model.forward, y, C_GT); f_no = deriv(b, ode.nominal_model.forward, y, 0.0)
    f_rom = deriv(b, ode.nominal_model.forward, y, C_ROM); f_mod = deriv(b, ode.forward, y, C_ROM)
    r = lambda a, s: float(torch.sqrt(((a[:, s]) ** 2).mean()))
    out = {}
    for name, f in (("nominal", f_no), ("rom", f_rom), ("model", f_mod)):
        out[name] = (r(f_gt - f, slice(3, 6)), r(f_gt - f, slice(10, 13)))
    return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--iters", type=int, default=2000); ap.add_argument("--lr", type=float, default=2e-2); a = ap.parse_args()
    out = a.out if a.out.is_absolute() else Path(__file__).resolve().parents[3] / a.out; out.mkdir(parents=True, exist_ok=True)
    cache = out / "data.npz"
    segs = [("stacked", 0.3), ("stacked", 0.4), ("static", 0.3), ("static", 0.4)]
    train_sets = [simulate(dz=dz, kind=k, duration=4.5, seed=i) for i, (k, dz) in enumerate(segs)]
    ys, ts = zip(*[make_pairs(d["X"], d["U"], d["XT"], d["dt"]) for d in train_sets]); y0 = torch.cat(ys); tg = torch.cat(ts)
    print("training rows", len(y0), "(= %.1f s at 200 Hz)" % (len(y0) * 0.005), flush=True)
    b, ode = setup_ode(C_ROM)
    ode0_loss = None
    t0 = time.time(); best, hist = train(ode, y0, tg, 0.005, iters=a.iters, lr=a.lr)
    print("train time %.0f s, final loss %.3e" % (time.time() - t0, min(hist)), flush=True)
    torch.save({"state_dict": ode.state_dict(), "iters": a.iters, "lr": a.lr}, out / "knode_sim_check.pth")
    # models to compare
    sample = b.nnm.ODEFunc(); ck = torch.load(B.BUNDLE / "KNODE/Models/sample_model.pth", map_location="cpu", weights_only=False); sample.load_state_dict(ck["state_dict"] if "state_dict" in ck else ck)
    rom_only = b.nnm.ODEFunc()
    for net in (rom_only.net_f, rom_only.net_t):
        for p in net.parameters(): torch.nn.init.zeros_(p)
    models = {"ROM only": rom_only, "authors' sample": sample, "ours": ode}
    res = {"train_rows": int(len(y0)), "iters": a.iters, "final_loss": float(min(hist)), "cases": {}}
    print("%-22s %-10s %-22s %-22s %-22s" % ("case", "model", "accel RMSE [m/s2]", "ang. accel RMSE [rad/s2]", "one-step loss"))
    for kind in ("stacked", "static"):
        for dz in (0.2, 0.3, 0.4, 0.5):
            d = simulate(dz=dz, kind=kind, duration=9.0, seed=10); yt, tt = make_pairs(d["X"], d["U"], d["XT"], d["dt"])
            key = f"{kind} dz{dz}"; res["cases"][key] = {}
            nom = None
            for mn, m in models.items():
                b.nominal.C_d = C_ROM; e = residual_errors(b, m, yt); ls = one_step_loss(m, yt, tt, 0.005)     # the model under test uses the ROM with C_d 0.3
                if nom is None: res["cases"][key]["Nominal"] = {"acc": e["nominal"][0], "ang": e["nominal"][1]}; nom = e["nominal"]
                res["cases"][key][mn] = {"acc": e["model"][0], "ang": e["model"][1], "one_step_loss": ls}
                print("%-22s %-10s %-22.3f %-22.2f %-22.3e" % (key, mn, e["model"][0], e["model"][1], ls))
            print("%-22s %-10s %-22.3f %-22.2f" % (key, "Nominal", nom[0], nom[1]))
    json.dump(res, open(out / "results.json", "w"), indent=1)
    # figure: residual z-acceleration vs vertical separation at r = 0 (neighbour upright above), truth vs ROM vs models
    zs = np.linspace(0.12, 0.8, 60); yy = torch.zeros(len(zs), 1, 30); yy[:, 0, 2] = 0.5; yy[:, 0, 9] = 1.0; yy[:, 0, 19] = torch.tensor(0.5 + zs, dtype=torch.float32); yy[:, 0, 26] = 1.0; yy[:, 0, 13] = 0.3
    fig, ax = plt.subplots(figsize=(6.5, 4))
    ax.plot(zs, deriv(b, ode.nominal_model.forward, yy, C_GT)[:, 5] - deriv(b, ode.nominal_model.forward, yy, 0.0)[:, 5], "k", lw=2, label="truth (ROM C_d 0.55)")
    ax.plot(zs, deriv(b, ode.nominal_model.forward, yy, C_ROM)[:, 5] - deriv(b, ode.nominal_model.forward, yy, 0.0)[:, 5], "0.6", ls="--", label="ROM only (C_d 0.3)")
    for mn, c in (("authors' sample", "tab:orange"), ("ours", "tab:blue")):
        ax.plot(zs, deriv(b, models[mn].forward, yy, C_ROM)[:, 5] - deriv(b, ode.nominal_model.forward, yy, 0.0)[:, 5], c=c, label=f"ROM + net: {mn}")
    ax.axvspan(0.3, 0.4, color="0.9", zorder=0); ax.set_xlabel("vertical separation [m] (r = 0), grey = training range"); ax.set_ylabel("downwash acceleration z [m/s²]"); ax.legend(fontsize=8); ax.set_title("KNODE training check on synthetic truth", fontsize=9)
    fig.tight_layout(); fig.savefig(out / "residual_vs_dz.png", dpi=130)


if __name__ == "__main__":
    main()
