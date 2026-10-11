#!/usr/bin/env python3
"""Meeting 2026-10-12 figures, part 2: sections 3 (replay, A8), 4 (RPM filter), 5 (simulation NS2 vs geometric)."""
from __future__ import annotations
import csv, json, os
from pathlib import Path
os.environ["PYTHONWARNINGS"] = "ignore"
import matplotlib.pyplot as plt
import numpy as np
from a8_crossings import zero_crossings

HERE = Path(__file__).resolve().parent
OUT = HERE / "out" / "meeting_2026-10-12"
OUT.mkdir(parents=True, exist_ok=True)
Z_BAND = 2.0


def band(ax):
    ax.axhspan(-Z_BAND, Z_BAND, color="#4FB39A", alpha=0.15, zorder=0); ax.axhline(0, color="k", lw=0.6, alpha=0.4)


# ---------------- section 5: simulation, NS2 vs geometric ----------------
def fig_sim():
    EP = HERE / "out" / "ns2_closed_loop_sil" / "episodes"
    coh = [("Geometric (NS2 off), simulation", "rnn0_rs1", "#4D4D4D"), ("NS2 100 Hz, res_sign −1, simulation", "rnn1_rs-1", "#009E73")]
    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    num = {}
    for lab, pat, col in coh:
        dips, steady = [], []
        for sd in range(5):
            f = EP / f"torque_c0.0032_s1.0_seed{sd}_{pat}_sym.json"
            if not f.is_file():
                continue
            lg = json.loads(f.read_text()).get("logs")
            if lg is None:
                continue
            t, pos, sp = (np.asarray(lg[k], float) for k in ("t", "pos_bot", "sp_bot"))
            tc = zero_crossings(t, pos[:, 1]); s = t - (tc[0] - 5.0)
            ez = (pos[:, 2] - sp[:, 2]) * 100
            m = (s >= 0) & (s <= 26)
            axes[0].plot(s[m], pos[m, 2], color=col, lw=0.9, alpha=0.8, label=lab if sd == 0 else None)
            axes[1].plot(s[m], ez[m], color=col, lw=0.9, alpha=0.8)
            tcs = [x - (tc[0] - 5.0) for x in tc]
            w = (s >= 6) & (s <= 22)
            for x in tcs: w &= ~((s >= x - 1.5) & (s <= x + 1.5))
            steady.append(float(ez[w].mean())); dips += [float(ez[(s >= x - 1) & (s <= x + 1)].min()) for x in tcs]
        num[lab] = dict(n_seeds=len(steady), steady_cm=float(np.mean(steady)), crossing_mean_cm=float(np.mean(dips)), crossing_max_abs_cm=float(np.max(np.abs(dips))))
    axes[0].axhline(0.5, ls="--", color="k", lw=0.8, label="commanded z"); axes[0].set_ylabel("z (m)"); axes[0].legend(fontsize=8)
    axes[0].set_title("A8, bottom drone — SIMULATION, NS2 (res_sign −1) vs geometric (one line per seed)")
    band(axes[1]); axes[1].set_ylim(-10, 6); axes[1].set_ylabel("z error (cm)"); axes[1].set_xlabel("time since scenario start (s)")
    fig.tight_layout(); fig.savefig(OUT / "fig5_sim_ns2_vs_geometric.png"); plt.close(fig)
    return num


# ---------------- section 3: replay of ONE A8 flight through all three controllers ----------------
def fig_replay():
    D = HERE / "out" / "indi_replay" / "A8"
    inp = np.load(D / "inputs.npz")
    pos = inp["pos"]; tt = np.arange(len(pos)) / 1000.0
    up = pos[:, 2] > 0.3
    tc = zero_crossings(tt[up], pos[up, 1]); t0 = tc[0] - 5.0
    names = [("ours", "replay_ours_L0.csv", "#D55E00", 1.6), ("Omar C", "replay_omar_c_L0.csv", "#005AB5", 1.6), ("Omar Rust", "replay_omar_rust_L0.csv", "#A50F15", 0.8)]
    data = {}
    for lab, f, col, lw in names:
        rows = np.genfromtxt(D / f, delimiter=",", names=True)
        data[lab] = rows
    fig, axes = plt.subplots(4, 1, figsize=(10, 9.5), sharex=True)
    for (lab, f, col, lw) in names:
        r = data[lab]; s = r["tick"] / 1000.0 - t0
        m = (s >= 0) & (s <= 26)
        axes[0].plot(s[m], r["thrust_si"][m], color=col, lw=lw, label=lab if lab != "Omar Rust" else "Omar Rust (on top of Omar C)")
        axes[1].plot(s[m], r["tau_x"][m] * 1e3, color=col, lw=lw)
        axes[2].plot(s[m], r["tau_y"][m] * 1e3, color=col, lw=lw)
        axes[3].plot(s[m], r["tau_z"][m] * 1e3, color=col, lw=lw)
    axes[0].set_ylabel("thrust (N)"); axes[1].set_ylabel("roll torque (mN·m)"); axes[2].set_ylabel("pitch torque (mN·m)"); axes[3].set_ylabel("yaw torque (mN·m)")
    axes[3].set_xlabel("time since scenario start (s)"); axes[0].legend(fontsize=8, loc="lower right")
    axes[0].set_title("A8, one flown flight (10-02, uSD inputs) replayed open loop through the three controllers")
    for ax in axes: ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(OUT / "fig4_replay_a8.png"); plt.close(fig)
    out = {}
    c, r_, o = data["Omar C"], data["Omar Rust"], data["ours"]
    for k in ("thrust_si", "tau_x", "tau_y", "tau_z"):
        out[k] = dict(mean_ours=float(o[k].mean()), mean_omar_c=float(c[k].mean()), mean_omar_rust=float(r_[k].mean()),
                      rms_c_vs_rust=float(np.sqrt(np.mean((c[k] - r_[k]) ** 2))), rms_ours_vs_c=float(np.sqrt(np.mean((o[k] - c[k]) ** 2))),
                      corr_ours_vs_c=float(np.corrcoef(o[k], c[k])[0, 1]), rms_ours=float(np.sqrt(np.mean(o[k] ** 2))), rms_omar_c=float(np.sqrt(np.mean(c[k] ** 2))))
    return out


# ---------------- section 4: RPM filter, per flight ----------------
def fig_rpm():
    rows = list(csv.DictReader(open(HERE / "out" / "a8_compare_2026-10-09" / "flights_used.csv")))
    groups = [("10-02, older firmware", "Ours INDI", "#999999"), ("10-09 filter ON", "Ours INDI (10-09 filter ON)", "#D55E00"), ("10-09 filter OFF", "Ours INDI (10-09 filter OFF)", "#F0B000")]
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.6))
    num = {}
    xi = 0
    for lab, v, col in groups:
        fl = [r for r in rows if r["variant"] == v and r["source"] == "usd"]
        for r in fl:
            vb = float(r["vbat_min"])
            axes[0].scatter(xi, float(r["steady_mean_cm"]), color=col, s=80, zorder=3)
            axes[1].scatter(xi, float(r["gyro_x_sd"]), color=col, s=80, zorder=3)
            axes[0].annotate(f"{vb:.2f} V", (xi, float(r["steady_mean_cm"])), textcoords="offset points", xytext=(8, -3), fontsize=8)
            axes[1].annotate(f"{vb:.2f} V", (xi, float(r["gyro_x_sd"])), textcoords="offset points", xytext=(8, -3), fontsize=8)
            xi += 1
        num[lab] = dict(steady_cm=[float(r["steady_mean_cm"]) for r in fl], gyro_x_sd=[float(r["gyro_x_sd"]) for r in fl], vbat_min=[float(r["vbat_min"]) for r in fl])
        xi += 1
    ticks, labs = [], []
    k = 0
    for lab, v, col in groups:
        n = len([r for r in rows if r["variant"] == v and r["source"] == "usd"]); ticks.append(k + (n - 1) / 2); labs.append(lab); k += n + 1
    for ax, yl, t in ((axes[0], "steady z error (cm)", "steady z error per flight"), (axes[1], "gyro_x sd (deg/s)", "roll-rate spread per flight (label = lowest battery voltage)")):
        ax.set_xticks(ticks); ax.set_xticklabels(labs, fontsize=8); ax.set_ylabel(yl); ax.set_title(t, fontsize=10); ax.grid(alpha=0.3); ax.set_ylim(bottom=0)
    ratios = [("filter ON", 0.95, "#D55E00"), ("older version", 1.07, "#999999"), ("filter OFF", 1.19, "#F0B000"), ("Omar Rust,\nfix ON", 1.01, "#A50F15")]
    for i, (l, v, c) in enumerate(ratios):
        axes[2].bar(i, v, color=c); axes[2].text(i, v + 0.02, f"{v:.2f}", ha="center", fontsize=9)
    axes[2].axhline(1.0, color="k", lw=0.8, ls="--"); axes[2].set_xticks(range(4)); axes[2].set_xticklabels([r[0] for r in ratios], fontsize=8)
    axes[2].set_ylim(0, 1.4); axes[2].set_title("motor response at invalid-reading instants\n÷ at random instants (1 = no effect)", fontsize=10)
    fig.suptitle("RPM filter, our INDI on A8 (uSD): no filter-related change in level or spread", fontsize=11)
    fig.tight_layout(); fig.savefig(OUT / "fig6_rpm_filter.png"); plt.close(fig)
    return num


if __name__ == "__main__":
    res = dict(sim=fig_sim(), replay=fig_replay(), rpm=fig_rpm())
    (OUT / "numbers2.json").write_text(json.dumps(res, indent=1))
    print(json.dumps(res, indent=1))
