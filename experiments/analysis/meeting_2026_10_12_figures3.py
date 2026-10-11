#!/usr/bin/env python3
"""Meeting 2026-10-12, part 3: RPM filter evidence (section 4), replay extras (section 3), sim vs hardware overlay (section 5)."""
from __future__ import annotations
import json, os, sys
from pathlib import Path
os.environ["PYTHONWARNINGS"] = "ignore"
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parents[1] / "flying_drone_stack" / "tools"))
import decode_usd_log as dec
from a8_crossings import zero_crossings
OUT = HERE / "out" / "meeting_2026-10-12"; USD = HERE.parents[1] / "experiments" / "logs" / "usd_raw"

FLIGHTS = [("filter ON", "#D55E00", "cf5_A8_thesis00_2026-10-09_17-40-36", "o"), ("filter ON", "#D55E00", "cf5_A8_thesis01_2026-10-09_17-42-12", "o"),
           ("filter OFF", "#E6A100", "cf5_A8_thesis00_2026-10-09_17-56-07", "o"), ("filter OFF", "#E6A100", "cf5_A8_thesis01_2026-10-09_17-57-38", "s")]  # last: battery sag 3.15 V


def steady_mask(r):
    t, z, y = r["t"], r["z"], r["y"]; up = z > 0.3
    t0, t1 = t[up][0], t[up][-1]
    W = (t >= t0 + 6) & (t <= t1 - 4) & up
    for tc in zero_crossings(t[up], y[up]): W &= ~((t >= tc - 1.5) & (t <= tc + 1.5))
    return W, up


def rpm_fig():
    R = [dec.load(str(USD / f"{n}.bin")) for _, _, n, _ in FLIGHTS]
    # ---- offline reconstruction of the firmware sentinel rule on the logged raw DShot stream, reference = optical deck rpm
    err_old, err_new = [], []
    ex = None
    for lab, _, n, _ in FLIGHTS:
        r = dec.load(str(USD / f"{n}.bin")); up = r["z"] > 0.3
        for i in range(1, 5):
            raw = r[f"motor_m{i}_rpm"].astype(float); deck = r[f"rpm_m{i}"].astype(float)
            new = raw.copy(); old = raw.copy(); prev = 0.0
            for k in range(len(raw)):
                if raw[k] >= 65000: old[k] = 0.0; new[k] = prev
                else: prev = raw[k]
            ks = np.where(up & (raw >= 65000))[0]
            ks = ks[(ks > 30) & (ks < len(raw) - 30)]
            err_old += [abs(old[k] - deck[k]) for k in ks]; err_new += [abs(new[k] - deck[k]) for k in ks]
            if ex is None and lab == "filter ON" and len(ks):
                k = ks[len(ks) // 2]; ex = (r["t"][k - 25:k + 26] - r["t"][k], raw[k - 25:k + 26], deck[k - 25:k + 26], old[k - 25:k + 26], new[k - 25:k + 26], i)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6), gridspec_kw=dict(width_ratios=[1.6, 0.8]))
    ax = axes[0]; t, raw, deck, old, new, mi = ex
    ax.plot(t * 1e3, deck, color="#888888", lw=1.4, label="optical deck RPM (reference)")
    ax.plot(t * 1e3, np.where(raw >= 65000, np.nan, raw), color="#0072B2", lw=1, marker=".", label="raw DShot reading")
    kk = np.where(raw >= 65000)[0]; ax.scatter(t[kk] * 1e3, [24000] * len(kk), marker="^", color="#0072B2", s=60, zorder=5, label="invalid reading (0xFFFF = 65535, off scale)")
    ax.plot(t * 1e3, old, color="#CC0000", lw=1.6, ls="--", label="used without the fix: 0")
    ax.plot(t * 1e3, new, color="#009E73", lw=1.6, label="used with the fix: last good value")
    ax.set_ylim(-1000, 25000); ax.set_xlabel("time around the invalid reading (ms)"); ax.set_ylabel(f"motor {mi} RPM")
    ax.set_title("One invalid DShot reading in flight (10-09, A8)", fontsize=10); ax.legend(fontsize=8, loc="lower right"); ax.grid(alpha=0.3)
    ax = axes[1]; ea, en = np.array(err_old), np.array(err_new)
    ax.bar([0, 1], [np.median(ea), np.median(en)], color=["#CC0000", "#009E73"])
    for i, v in enumerate([np.median(ea), np.median(en)]): ax.text(i, v + 300, f"{v:,.0f} RPM", ha="center", fontsize=9)
    ax.set_xticks([0, 1]); ax.set_xticklabels(["without the fix", "with the fix"]); ax.set_ylabel("|used RPM − deck RPM|, median (RPM)")
    ax.set_title(f"Error at all {len(ea)} invalid readings\n(4 flights, 4 motors)".replace("\\n", "\n"), fontsize=10); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(OUT / "fig6a_rpm_invalid_readings.png", dpi=150); plt.close(fig)

    rows = []
    for (lab, col, n, mk), r in zip(FLIGHTS, R):
        W, up = steady_mask(r)
        ez = (r["z"] - r["ctrltarget_z"]) * 100; lat = np.hypot(r["x"] - r["ctrltarget_x"], r["y"] - r["ctrltarget_y"]) * 100
        er = r["e_r_norm"] * 1e3
        rows.append(dict(flight=n[-19:], label=lab, z_mean=float(ez[W].mean()), z_sd=float(ez[W].std()), lat_mean=float(lat[W].mean()), lat_sd=float(lat[W].std()),
                         er_mean=float(er[W].mean()), er_sd=float(er[W].std())))
    keys = [("z_mean", "z_sd", "z error (cm)"), ("lat_mean", "lat_sd", "lateral position error (cm)"), ("er_mean", "er_sd", "attitude error e_r (×10⁻³)")]
    fig, axes = plt.subplots(1, 3, figsize=(12, 4.2))
    for ax, (km, ks_, ttl) in zip(axes, keys):
        for i, ((lab, col, n, mk), r_) in enumerate(zip(FLIGHTS, rows)):
            ax.errorbar(i, r_[km], yerr=r_[ks_], color=col, marker=mk, ms=9, capsize=5, lw=1.6)
        ax.set_xticks([0.5, 2.5]); ax.set_xticklabels(["filter ON\n(2 flights)", "filter OFF\n(2 flights)"]); ax.set_xlim(-0.6, 3.6); ax.grid(alpha=0.3); ax.set_title(ttl, fontsize=10)
        if km != "z_mean": ax.set_ylim(bottom=0)
    fig.suptitle("Our INDI on A8, same day, uSD: steady-flight mean ± sd per flight (square = battery sag to 3.15 V)", fontsize=10)
    fig.tight_layout(); fig.savefig(OUT / "fig6b_rpm_on_vs_off.png", dpi=150); plt.close(fig)
    return dict(n_invalid=len(ea), median_err_old=float(np.median(ea)), median_err_new=float(np.median(en)), flights=rows)


def replay_extras():
    D = HERE / "out" / "indi_replay" / "A8"; inp = np.load(D / "inputs.npz"); pos = inp["pos"]; tt = np.arange(len(pos)) / 1000.0
    up = pos[:, 2] > 0.3; tc = zero_crossings(tt[up], pos[up, 1]); t0 = tc[0] - 5
    out = {}
    for lab, f in (("ours", "replay_ours_L0.csv"), ("omar", "replay_omar_c_L0.csv")):
        r = np.genfromtxt(D / f, delimiter=",", names=True); s = r["tick"] / 1000 - t0
        pk = [float(r["thrust_si"][(s >= x - t0 - t0 + t0 - 1) & (s <= x - t0 + 1)].max()) for x in tc]
        out[lab] = dict(thrust_peak_at_crossings=pk, thrust_min=float(r["thrust_si"].min()), thrust_max=float(r["thrust_si"].max()),
                        rms=dict(thrust=float(np.sqrt((r["thrust_si"] ** 2).mean())), tau_x=float(np.sqrt((r["tau_x"] ** 2).mean())), tau_y=float(np.sqrt((r["tau_y"] ** 2).mean())), tau_z=float(np.sqrt((r["tau_z"] ** 2).mean()))),
                        sd=dict(thrust=float(r["thrust_si"].std()), tau_x=float(r["tau_x"].std()), tau_y=float(r["tau_y"].std()), tau_z=float(r["tau_z"].std())))
    return out


def sim_hw_fig():
    import a8_variant_comparison as a8
    a8._build_usd_index(); rows, _, _ = a8.process_all(); rows = [r for r in rows if r.source == "usd"]
    by = {}
    for r in rows: by.setdefault(r.variant, []).append(r)
    EP = HERE / "out" / "ns2_closed_loop_sil" / "episodes"
    cols = [("Geometric (NS2 off)", "rnn0_rs1", "Geometric baseline"), ("NS2 100 Hz, res_sign −1", "rnn1_rs-1", "NS2 res_sign −1")]
    fig, axes = plt.subplots(2, 2, figsize=(13, 7), sharex=True)
    for j, (ttl, pat, hv) in enumerate(cols):
        for sd in range(5):
            f = EP / f"torque_c0.0032_s1.0_seed{sd}_{pat}_sym.json"
            lg = json.loads(f.read_text()).get("logs") if f.is_file() else None
            if lg is None: continue
            t, pos, sp = (np.asarray(lg[k], float) for k in ("t", "pos_bot", "sp_bot"))
            tc = zero_crossings(t, pos[:, 1]); s_ = t - (tc[0] - 5); m = (s_ >= 0) & (s_ <= 26)
            axes[0, j].plot(s_[m], pos[m, 2], color="#0072B2", lw=0.9, alpha=0.7, label="simulation (5 seeds)" if sd == 0 else None)
            axes[1, j].plot(s_[m], (pos[m, 2] - sp[m, 2]) * 100, color="#0072B2", lw=0.9, alpha=0.7)
        for i, r in enumerate(by[hv]):
            tt, z, ez = a8.plot_trace(r)
            axes[0, j].plot(tt, z, color="#E69F00", lw=1.0, alpha=0.9, label=f"hardware, uSD ({len(by[hv])} flights)" if i == 0 else None)
            axes[1, j].plot(tt, ez, color="#E69F00", lw=1.0, alpha=0.9)
        axes[0, j].axhline(0.5, ls="--", color="k", lw=0.8); axes[0, j].set_title(ttl); axes[0, j].legend(fontsize=8, loc="lower right")
        axes[1, j].axhspan(-2, 2, color="#4FB39A", alpha=0.15, zorder=0); axes[1, j].axhline(0, color="k", lw=0.6, alpha=0.4); axes[1, j].set_ylim(-10, 6)
        axes[1, j].set_xlabel("time since scenario start (s)")
    axes[0, 0].set_ylabel("z (m)"); axes[1, 0].set_ylabel("z error (cm)")
    fig.suptitle("A8, bottom drone: closed-loop simulation against hardware", fontsize=11)
    fig.tight_layout(); fig.savefig(OUT / "fig5_sim_vs_hw_ns2_geometric.png", dpi=150); plt.close(fig)


if __name__ == "__main__":
    sim_hw_fig()
    res = dict(rpm=rpm_fig(), replay=replay_extras())
    (OUT / "numbers3.json").write_text(json.dumps(res, indent=1)); print(json.dumps(res, indent=1))
