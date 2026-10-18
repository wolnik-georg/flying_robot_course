#!/usr/bin/env python3
"""FBL Stage 0: do the authors' residual and our NS2 label mean the same thing, and how far is their physics model from our data?

  ~/.pyenv/versions/flying_robots/bin/python experiments/fbl/stage0_definitions/s0_check_definitions.py [--step 10]

Checks (all read-only on the bundle, our data from the released NS2 v2 manifest):
  1. bundle integrity hash, our vehicle constants against the bundle's simulated 30 g vehicle
  2. geometry: the bundle's point_line_distance (r0, z0) against our (horizontal distance, vertical gap)
  3. coverage of (r0, z0) in our 73-flight bank against the paper's hardware training regime (0.4 m separation, stacked + static top)
  4. ROM only (physics, no network) against our label a_res_z in the downwash region: scale, correlation, best-fit drag coefficient C_D
  5. the authors' sample network (30 g simulation plant) against our label: how far off
Writes experiments/fbl/out/stage0/{stage0_results.json, stage0_report.md, stage0_overview.png}.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from common import bundle as B  # noqa: E402
from common import data as D  # noqa: E402
from common.vehicle import BUNDLE_SIM, OUR  # noqa: E402

OUT = HERE.parent / "out" / "stage0"


def rom_acc_z(b, z0, r0, c_d, s, mass, chunk=4000):
    """Bundle ROM disturbance acceleration along world z (negative = down) [m/s^2], batched."""
    out = []
    for i in range(0, len(z0), chunk):
        z = torch.as_tensor(z0[i:i + chunk], dtype=torch.float64)
        r = torch.as_tensor(r0[i:i + chunk], dtype=torch.float64)
        f = b.force.downwash_force_fo(z, r, torch.zeros_like(z), C_d=c_d, S=s)
        out.append((-f / mass).numpy())
    return np.concatenate(out)


def stats(y, p):
    ok = np.isfinite(y) & np.isfinite(p)
    y, p = y[ok], p[ok]
    k = float(np.dot(p, y) / max(np.dot(p, p), 1e-12))
    return dict(n=int(ok.sum()), pearson=float(np.corrcoef(y, p)[0, 1]) if p.std() > 0 else float("nan"), scale_fit=k,
                rmse=float(np.sqrt(np.mean((y - p) ** 2))), rmse_zero=float(np.sqrt(np.mean(y ** 2))), mean_label=float(y.mean()), mean_pred=float(p.mean()))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--step", type=int, default=10, help="subsample the 500 Hz merged logs (10 -> 50 Hz)")
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    b = B.load()
    B.set_vehicle(b, OUR["mass"], OUR["arm_length"])
    res = {"bundle_sha256": B.bundle_hash(), "vehicle_ours": OUR, "vehicle_bundle_sim": BUNDLE_SIM}

    df = D.load_bank(step=a.step)
    res["n_samples"] = int(len(df)); res["n_flights"] = int(df.flight.nunique())

    # 2. geometry check against the bundle's own function
    nm = b.nominal.NominalModel()
    above = np.flatnonzero(df.dz.values > 0)
    idx = np.random.default_rng(0).choice(above, 2000, replace=False)       # the bundle assumes the neighbour is the UPPER vehicle
    ego = torch.tensor(df[["e_x", "e_y", "e_z"]].values[idx]); top = torch.tensor(df[["n_x", "n_y", "n_z"]].values[idx])
    zb = torch.zeros_like(ego); zb[:, 2] = 1.0
    r0b, z0b = nm.point_line_distance(ego, top, zb)
    res["geometry"] = dict(max_abs_r0_diff=float(np.max(np.abs(r0b.numpy() - df.r0.values[idx]))), max_abs_z0_diff=float(np.max(np.abs(z0b.numpy() - df.z0.values[idx]))),
                           z0_note="compared on samples with the neighbour above (bundle z0 is unsigned and assumes the upper vehicle)")

    # 3. coverage
    reg = df[(df.z0 > 0.1) & (df.z0 < 1.3) & (df.r0 < 0.6)]
    res["coverage"] = dict(
        n_region=int(len(reg)),
        z0_hist_edges=[0.1, 0.3, 0.45, 0.6, 0.8, 1.0, 1.3],
        z0_hist_seconds=[float(x) / (500 / a.step) for x in np.histogram(reg.z0, [0.1, 0.3, 0.45, 0.6, 0.8, 1.0, 1.3])[0]],
        r0_hist_edges=[0, 0.05, 0.1, 0.2, 0.3, 0.45, 0.6],
        r0_hist_seconds=[float(x) / (500 / a.step) for x in np.histogram(reg.r0, [0, 0.05, 0.1, 0.2, 0.3, 0.45, 0.6])[0]],
        paper_hw_regime="vertical separation 0.4 m (0.3 m out-of-distribution), stacked (r0 = 0) and static top with the bottom vehicle passing underneath at 0.4 m/s; 28 s of data")

    # 4. ROM against label (downwash region, baseline removed)
    far = df[(df.r0 > 0.6) & (df.z0 > 0.1)]
    base = float(far.a_res_z.mean()); res["baseline_far_a_res_z"] = base
    reg = reg.assign(y=reg.a_res_z - base)
    ycol = reg.y.values; r0v = reg.r0.values; z0v = reg.z0.values
    table = {}
    for name, (cd, s) in {"hw_CD_0.236": (0.236, 0.0997), "default_CD_0.3": (0.3, 0.0997), "sim_CD_0.7": (0.7, 0.0997)}.items():
        table[name] = stats(ycol, rom_acc_z(b, z0v, r0v, cd, s, OUR["mass"]))
    grid = [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.7, 1.0, 1.5]
    err = [(cd, stats(ycol, rom_acc_z(b, z0v, r0v, cd, 0.0997, OUR["mass"]))["rmse"]) for cd in grid]
    best = min(err, key=lambda t: t[1])
    res["rom_vs_label"] = table
    res["rom_C_D_grid_rmse"] = {str(c): e for c, e in err}
    res["rom_best_C_D"] = best[0]
    res["rom_best"] = stats(ycol, rom_acc_z(b, z0v, r0v, best[0], 0.0997, OUR["mass"]))

    # 5. the authors' sample network (simulation plant) on our data: ROM + NN total acceleration along z
    try:
        ckpt = torch.load(B.BUNDLE / "KNODE/Models/sample_model.pth", map_location="cpu", weights_only=True)
        ode = b.nnm.ODEFunc()
        ode.load_state_dict(ckpt["state_dict"] if "state_dict" in ckpt else ckpt)
        net = b.dw.DownwashResidual(ode, use_nn=True, use_nominal=True).eval()
        n = len(reg)
        y = torch.zeros(n, 30, dtype=torch.float32)
        y[:, 0:3] = torch.tensor(reg[["e_x", "e_y", "e_z"]].values); y[:, 3:6] = torch.tensor(reg[["e_vx", "e_vy", "e_vz"]].values)
        y[:, 6:10] = torch.tensor(reg[["e_qx", "e_qy", "e_qz", "e_qw"]].values)          # bundle order [x, y, z, w]
        y[:, 17:20] = torch.tensor(reg[["n_x", "n_y", "n_z"]].values); y[:, 20:23] = torch.tensor(reg[["n_vx", "n_vy", "n_vz"]].values)
        y[:, 23:27] = torch.tensor(reg[["n_qx", "n_qy", "n_qz", "n_qw"]].values)
        B.set_vehicle(b, BUNDLE_SIM["mass"], BUNDLE_SIM["arm_length"])
        net.nominal_model.mass = BUNDLE_SIM["mass"]
        with torch.no_grad():
            pred = torch.cat([net.acceleration(y[i:i + 2000])[0] for i in range(0, n, 2000)]).numpy()      # acceleration() returns (acceleration, jerk)
        res["sample_net_vs_label_z"] = stats(ycol, pred[:, 2])
        res["sample_net_vs_label_xy_rmse"] = [float(np.sqrt(np.mean((reg[f"a_res_{k}"].values - pred[:, i]) ** 2))) for i, k in enumerate("xy")]
        B.set_vehicle(b, OUR["mass"], OUR["arm_length"])
    except Exception as e:  # keep the stage reproducible even if the checkpoint interface differs
        res["sample_net_vs_label_z"] = {"error": repr(e)[:300]}

    # figure
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(15, 4))
    bins = np.array([0.15, 0.3, 0.45, 0.6, 0.8, 1.0, 1.3]); mid = 0.5 * (bins[1:] + bins[:-1])
    sel = reg[reg.r0 < 0.1]
    lab = [float(sel.y[(sel.z0 >= lo) & (sel.z0 < hi)].mean()) if ((sel.z0 >= lo) & (sel.z0 < hi)).sum() > 20 else np.nan for lo, hi in zip(bins[:-1], bins[1:])]
    ax[0].plot(mid, lab, "ko-", label="our label, r0 < 0.1 m"); zz = np.linspace(0.15, 1.3, 40)
    for nme, cd in (("ROM C_D 0.236", 0.236), (f"ROM best C_D {best[0]}", best[0])):
        ax[0].plot(zz, rom_acc_z(b, zz, np.full_like(zz, 0.03), cd, 0.0997, OUR["mass"]), "--", label=nme)
    ax[0].set_xlabel("vertical gap z0 [m]"); ax[0].set_ylabel("a_z [m/s²]"); ax[0].legend(); ax[0].set_title("push below the other vehicle")
    h = ax[1].hist2d(reg.r0, reg.z0, bins=[np.linspace(0, 0.6, 25), np.linspace(0.1, 1.3, 25)], cmap="viridis")
    ax[1].axhline(0.4, color="r", ls=":"); ax[1].set_xlabel("r0 [m]"); ax[1].set_ylabel("z0 [m]"); ax[1].set_title("our bank (red: paper's 0.4 m)"); plt.colorbar(h[3], ax=ax[1])
    ax[2].plot([e[0] for e in err], [e[1] for e in err], "o-"); ax[2].set_xscale("log"); ax[2].set_xlabel("ROM drag coefficient C_D"); ax[2].set_ylabel("RMSE vs label [m/s²]"); ax[2].set_title("ROM fit")
    fig.tight_layout(); fig.savefig(OUT / "stage0_overview.png", dpi=110)

    (OUT / "stage0_results.json").write_text(json.dumps(res, indent=1))
    md = ["# FBL Stage 0 results (generated by s0_check_definitions.py)", "",
          f"Bundle sha256 {res['bundle_sha256'][:16]}…; {res['n_flights']} flights, {res['n_samples']} samples at {500 // a.step} Hz; baseline (far from the other vehicle) a_res_z {base:+.3f} m/s².", "",
          "## Geometry", f"max |r0 diff| {res['geometry']['max_abs_r0_diff']:.2e}, max |z0 diff| {res['geometry']['max_abs_z0_diff']:.2e} ({res['geometry']['z0_note']}).", "",
          "## ROM (physics only) against our label inside the downwash region (0.1 < z0 < 1.3 m, r0 < 0.6 m, baseline removed)",
          "| model | n | Pearson r | scale fit (label ≈ k·pred) | RMSE | RMSE of zero | mean label | mean pred |", "|---|---|---|---|---|---|---|---|"]
    for k, v in {**table, f"best C_D {best[0]}": res["rom_best"], "authors' sample net (30 g sim)": res["sample_net_vs_label_z"]}.items():
        if "error" in v:
            md.append(f"| {k} | error: {v['error']} |"); continue
        md.append(f"| {k} | {v['n']} | {v['pearson']:+.2f} | {v['scale_fit']:+.2f} | {v['rmse']:.3f} | {v['rmse_zero']:.3f} | {v['mean_label']:+.2f} | {v['mean_pred']:+.2f} |")
    md += ["", "## Coverage (seconds in the region)", f"z0 bins {res['coverage']['z0_hist_edges']}: {[round(x) for x in res['coverage']['z0_hist_seconds']]}",
           f"r0 bins {res['coverage']['r0_hist_edges']}: {[round(x) for x in res['coverage']['r0_hist_seconds']]}", ""]
    (OUT / "stage0_report.md").write_text("\n".join(md))
    print("\n".join(md))


if __name__ == "__main__":
    main()
