#!/usr/bin/env python3
"""Final figures / animation / numbers of the simulation showcase (validated rebuild 2026-10-09).

Supersedes the plotting and doc writer of `sim_showcase.py` (its simulate stage and the cached SIL episodes are kept).
Run with the pyenv python:  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/sim_showcase_finalize.py
Every SIL number here is evaluated with `a8_crossings.zero_crossings` and the same window/dip definitions as docs/71.
"""
import csv, glob, json, subprocess, sys
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from a8_crossings import zero_crossings  # noqa: E402
import a8_variant_comparison as A  # noqa: E402

OUT = HERE / "out" / "sim_showcase_2026-10-09"
FIG, ANIM, RES = OUT / "figs", OUT / "anim", OUT / "results"
EP = HERE / "out" / "ns2_closed_loop_sil" / "episodes"
CMP = HERE / "out" / "a8_compare_2026-10-09"
for d in (FIG, ANIM):
    d.mkdir(parents=True, exist_ok=True)
A._build_usd_index()

C_SIL, C_HW = "#0072B2", "#E69F00"
COH = {"network off": ("rnn0_rs1", "Geometric baseline"), "res_sign +1": ("rnn1_rs1", "NS2 res_sign +1"),
       "res_sign −1": ("rnn1_rs-1", "NS2 res_sign −1")}
SEEDS = range(5)


# ---------------------------------------------------------------- SIL helpers
def aligned(t, y, z, sp):
    """time since scenario start (first crossing = 5.0 s), e_z in cm, crossing times."""
    tc = zero_crossings(t, y)
    t0 = tc[0] - 5.0
    return t - t0, (z - sp) * 100.0, [x - t0 for x in tc]


def metrics(s, ez, tc):
    """steady (window 6..22 s, crossings +-1.5 s excluded), dips (min in +-1 s), same as docs/71."""
    w = (s >= 6.0) & (s <= 22.0)
    for x in tc:
        w &= ~((s >= x - 1.5) & (s <= x + 1.5))
    steady = float(np.mean(ez[w]))
    dips = [float(np.min(ez[(s >= x - 1) & (s <= x + 1)])) for x in tc]
    return steady, dips


def sil_ns2(pat):
    rows = []
    for sd in SEEDS:
        f = EP / f"torque_c0.0032_s1.0_seed{sd}_{pat}_sym.json"
        if not f.is_file():
            continue
        d = json.loads(f.read_text())
        if "logs" not in d:
            rows.append({"seed": sd, "failed": str(d.get("error", ""))[-80:]})
            continue
        lg = d["logs"]
        t, pos, sp, top = (np.asarray(lg[k], float) for k in ("t", "pos_bot", "sp_bot", "pos_top"))
        s, ez, tc = aligned(t, pos[:, 1], pos[:, 2], sp[:, 2])
        st, dips = metrics(s, ez, tc)
        rows.append({"seed": sd, "s": s, "ez": ez, "tc": tc, "steady": st, "dips": dips, "pos": pos, "top": top, "t": t})
    return rows


def sil_omar(name):
    r = json.loads((RES / f"{name}.json").read_text())
    tr = r["trace"]
    t, pos, sp = (np.asarray(tr[k], float) for k in ("t", "pos", "sp"))
    s, ez, tc = aligned(t, pos[:, 1], pos[:, 2], sp[:, 2])
    st, dips = metrics(s, ez, tc)
    return {"s": s, "ez": ez, "tc": tc, "steady": st, "dips": dips, "rel": float(np.mean(dips)) - st}


def hw_flights(variant):
    rows = list(csv.DictReader(open(CMP / "flights_used.csv")))
    return [r for r in rows if r["variant"] == variant]


def hw_trace(r):
    fm = NS(date=r["date"], stamp=r["stamp"], source=r["source"])
    tt, _, ez = A.plot_trace(fm)
    return tt, ez


def hw_dips(variant):
    return [d for r in hw_flights(variant) for d in json.loads(r["dips_cm"])]


# ---------------------------------------------------------------- numbers
def collect():
    out = {"cohorts": {}, "omar": {}}
    for name, (pat, hwv) in COH.items():
        rows = [r for r in sil_ns2(pat) if "dips" in r]
        failed = [r for r in sil_ns2(pat) if "failed" in r]
        alld = [d for r in rows for d in r["dips"]]
        hd = hw_dips(hwv)
        out["cohorts"][name] = {
            "sil_seeds": [r["seed"] for r in rows], "sil_failed": [(r["seed"], r["failed"]) for r in failed],
            "sil_n_cross": len(alld), "sil_dip_mean": float(np.mean(alld)), "sil_dip_sd": float(np.std(alld, ddof=1)),
            "sil_steady": float(np.mean([r["steady"] for r in rows])),
            "hw_n_cross": len(hd), "hw_dip_mean": float(np.mean(hd)), "hw_dip_sd": float(np.std(hd, ddof=1)),
            "hw_steady": float(np.mean([float(r["steady_mean_cm"]) for r in hw_flights(hwv)])),
            "hw_n_flights": len(hw_flights(hwv)),
        }
    for nm in ("omar_rust_0", "omar_rust_15", "omar_c_0", "omar_c_15"):
        o = sil_omar(nm)
        out["omar"][nm] = {k: o[k] for k in ("steady", "dips", "rel")}
    (RES / "final_numbers.json").write_text(json.dumps(out, indent=1))
    return out


# ---------------------------------------------------------------- figures
def fig_dips(num):
    fig, ax = plt.subplots(figsize=(8.5, 4.6))
    x = np.arange(3); w = 0.36
    for i, (k, c) in enumerate(num["cohorts"].items()):
        ax.bar(i - w / 2, c["sil_dip_mean"], w, yerr=c["sil_dip_sd"], color=C_SIL, capsize=4, label="SIL (all crossings of seeds 0–4)" if i == 0 else None)
        ax.bar(i + w / 2, c["hw_dip_mean"], w, yerr=c["hw_dip_sd"], color=C_HW, capsize=4, label="hardware (all crossings)" if i == 0 else None)
        ax.text(i, 0.6, f"Δ = {c['sil_dip_mean'] - c['hw_dip_mean']:+.2f} cm", ha="center", fontsize=9)
        ax.text(i - w / 2, c["sil_dip_mean"] - c["sil_dip_sd"] - 0.7, f"{c['sil_dip_mean']:.1f}", ha="center", fontsize=8)
        ax.text(i + w / 2, c["hw_dip_mean"] - c["hw_dip_sd"] - 0.7, f"{c['hw_dip_mean']:.1f}", ha="center", fontsize=8)
    ax.set_xticks(x); ax.set_xticklabels(list(num["cohorts"])); ax.set_ylabel("crossing dip, mean ± sd over crossings (cm)")
    ax.set_ylim(-14.5, 2); ax.axhline(0, color="k", lw=0.6); ax.legend(loc="lower left", fontsize=8)
    ax.set_title("NS2 closed-loop SIL vs hardware — A8, bottom drone (shallower is better)")
    fig.tight_layout(); fig.savefig(FIG / "sim_vs_hw_dips.png", dpi=150); plt.close(fig)


def overlay(ax, sil_list, hw_rows, label_sil, label_hw, sil_mean=True):
    for s, ez in sil_list:
        m = (s >= 0) & (s <= 26)
        ax.plot(s[m], ez[m], color=C_SIL, lw=0.6, alpha=0.35)
    if sil_mean and len(sil_list) > 1:
        grid = np.arange(0, 26, 0.05)
        ax.plot(grid, np.mean([np.interp(grid, s, ez) for s, ez in sil_list], axis=0), color=C_SIL, lw=2.0, label=label_sil)
    elif sil_list:
        s, ez = sil_list[0]; m = (s >= 0) & (s <= 26)
        ax.plot(s[m], ez[m], color=C_SIL, lw=2.0, label=label_sil)
    for i, r in enumerate(hw_rows):
        tt, ez = hw_trace(r)
        ax.plot(tt, ez, color=C_HW, lw=0.9, alpha=0.8, label=label_hw if i == 0 else None)
    ax.axhspan(-2, 2, color="#4FB39A", alpha=0.15, zorder=0); ax.axhline(0, color="k", lw=0.5)
    ax.set_xlim(0, 26)


def fig_traces(num):
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for ax, (name, (pat, hwv)) in zip(axes, COH.items()):
        rows = [r for r in sil_ns2(pat) if "ez" in r]
        c = num["cohorts"][name]
        overlay(ax, [(r["s"], r["ez"]) for r in rows], hw_flights(hwv),
                f"SIL mean of {len(rows)} seeds (thin: single seeds)", f"hardware, {c['hw_n_flights']} flights")
        ax.set_ylim(-15, 8); ax.set_ylabel("z error (cm)")
        ax.set_title(f"{name}: SIL dip {c['sil_dip_mean']:+.1f} cm vs hardware {c['hw_dip_mean']:+.1f} cm", fontsize=10)
        ax.legend(loc="lower right", fontsize=8)
    axes[-1].set_xlabel("time since scenario start (s), all traces aligned on the first crossing (5.0 s)")
    fig.tight_layout(); fig.savefig(FIG / "sim_vs_hw_traces.png", dpi=150); plt.close(fig)


def fig_variants(num):
    sil_geo = [r for r in sil_ns2("rnn0_rs1") if "ez" in r]
    spec = [("Geometric baseline", [(r["s"], r["ez"]) for r in sil_geo], "Geometric baseline"),
            ("Omar Rust exact (kpos_iz 0)", None, "omar_rust_0"), ("Omar Rust + Iz 1.5", None, "omar_rust_15"),
            ("Omar C exact (Kpos_Iz 0)", None, "omar_c_0"), ("Omar C + Iz 1.5", None, "omar_c_15"),
            ("Ours INDI", [], "Ours INDI")]
    fig, axes = plt.subplots(2, 3, figsize=(16, 8.5), sharex=True)
    for ax, (title, sil, key) in zip(axes.ravel(), spec):
        hwv = {"omar_rust_0": "Omar Rust exact (kpos_iz 0)", "omar_rust_15": "Omar Rust + Iz 1.5", "omar_c_0": "Omar C exact (Kpos_Iz 0)",
               "omar_c_15": "Omar C + Iz 1.5"}.get(key, key)
        if sil is None:
            o = sil_omar(key); sil = [(o["s"], o["ez"])]
        overlay(ax, sil, hw_flights(hwv), "SIL" + (f" ({len(sil)} seeds)" if len(sil) > 1 else " (1 deterministic run)"), f"hardware ({len(hw_flights(hwv))} flights)")
        ax.set_ylim(-26, 32); ax.set_title(title); ax.set_ylabel("z error (cm)")
        if title == "Ours INDI":
            ax.text(0.5, 0.72, "SIL: not feasible in this harness\n(all 4 attempts crash, see docs/72)", transform=ax.transAxes, ha="center", fontsize=10,
                    bbox=dict(boxstyle="round", fc="#fff3cd", ec="#c9a227"))
        ax.legend(loc="lower right", fontsize=8)
    for ax in axes[1]:
        ax.set_xlabel("time since scenario start (s)")
    fig.suptitle("Controller variants: SIL (NS2 plant) vs hardware, A8 bottom drone", fontsize=13)
    fig.tight_layout(); fig.savefig(FIG / "sim_variants_vs_hw.png", dpi=150); plt.close(fig)


def table_rows(num):
    hwt = {r["variant"]: r for r in csv.DictReader(open(CMP / "table_a8_variants.csv"))}
    sg = num["cohorts"]["network off"]
    rows = [("Geometric (= NS2 off)", f"{sg['sil_steady']:+.1f}", f"{sg['sil_dip_mean']:+.1f}", "n/a",
             hwt["Geometric baseline"])]
    om = num["omar"]
    for lab, key, hv in [("Omar Rust exact", "omar_rust_0", "Omar Rust exact (kpos_iz 0)"), ("Omar Rust + Iz 1.5", "omar_rust_15", "Omar Rust + Iz 1.5"),
                         ("Omar C exact", "omar_c_0", "Omar C exact (Kpos_Iz 0)"), ("Omar C + Iz 1.5", "omar_c_15", "Omar C + Iz 1.5")]:
        rows.append((lab, f"{om[key]['steady']:+.1f}", f"{np.mean(om[key]['dips']):+.1f}", f"{om[key]['rel']:+.1f}", hwt[hv]))
    rows.append(("Ours INDI", "—", "—", "—", hwt["Ours INDI"]))
    return rows


def fig_table(num):
    rows = table_rows(num)
    cells = [[r[0], r[1], r[3] if r[0] != "Geometric (= NS2 off)" else f"{float(r[2]) - float(r[1]):+.1f}", f"{float(r[4]['steady_mean_cm']):+.1f}",
              f"{float(r[4]['dip_rel_mean_cm']):+.1f}", r[4]["n"]] for r in rows]
    fig, ax = plt.subplots(figsize=(10, 3.6)); ax.axis("off")
    t = ax.table(cellText=cells, colLabels=["variant", "SIL steady (cm)", "SIL dip rel. to steady (cm)", "HW steady (cm)", "HW dip rel. to steady (cm)", "HW n flights"],
                 loc="center", cellLoc="center")
    t.auto_set_font_size(False); t.set_fontsize(9); t.scale(1, 1.5)
    ax.set_title("Controller variants — steady z error and crossing dip relative to the steady level (A8)", fontsize=11)
    fig.tight_layout(); fig.savefig(FIG / "sim_variants_table.png", dpi=150); plt.close(fig)


# ---------------------------------------------------------------- animation
def animate():
    cases = [("network off", sil_ns2("rnn0_rs1")[0]), ("res_sign −1", sil_ns2("rnn1_rs-1")[0])]
    fps = 25
    fig = plt.figure(figsize=(12, 7.2), dpi=100)
    gs = fig.add_gridspec(3, 2, height_ratios=[1.0, 1.1, 0.8], hspace=0.55, wspace=0.18)
    panels = []
    for j, (name, r) in enumerate(cases):
        axt = fig.add_subplot(gs[0, j]); axs = fig.add_subplot(gs[1, j]); axe = fig.add_subplot(gs[2, j])
        s = r["s"]; pos, top, ez, tc = r["pos"], r["top"], r["ez"], r["tc"]
        idx = np.where((s >= 0) & (s <= 26))[0]
        axt.set_xlim(-1.2, 1.2); axt.set_ylim(-0.9, 0.9); axt.set_aspect("equal"); axt.set_title(f"{name} — top view (x–y)", fontsize=10)
        axs.set_xlim(-0.9, 0.9); axs.set_ylim(0.2, 1.25); axs.axhline(0.5, color="gray", ls="--", lw=1); axs.axhline(1.0, color="gray", ls=":", lw=1)
        axs.set_title("side view (y–z), dashed: commanded heights 0.5 / 1.0 m", fontsize=9, pad=8); axs.set_xlabel("y (m)"); axs.set_ylabel("z (m)")
        axe.plot(s[idx], ez[idx], color="#bbbbbb", lw=1); axe.axhspan(-2, 2, color="#4FB39A", alpha=0.15)
        for x in tc:
            axe.axvline(x, color="k", lw=0.6, alpha=0.4)
        axe.set_xlim(0, 26); axe.set_ylim(-9, 5); axe.set_xlabel("t (s)"); axe.set_ylabel("bottom z error (cm)")
        b1, = axt.plot([], [], "o", color="#d62728", ms=9, label="bottom (cf5)"); t1, = axt.plot([], [], "o", color="#1f77b4", ms=9, label="top")
        b2, = axs.plot([], [], "o", color="#d62728", ms=11); t2, = axs.plot([], [], "o", color="#1f77b4", ms=11)
        tr, = axs.plot([], [], "-", color="#d62728", lw=0.8, alpha=0.5)
        cur, = axe.plot([], [], "-", color=C_SIL, lw=1.8); dot, = axe.plot([], [], "o", color=C_SIL)
        txt = axs.text(0.02, 0.06, "", transform=axs.transAxes, fontsize=10, family="monospace")
        axt.legend(loc="upper right", fontsize=7)
        panels.append(dict(idx=idx, s=s, pos=pos, top=top, ez=ez, b1=b1, t1=t1, b2=b2, t2=t2, tr=tr, cur=cur, dot=dot, txt=txt, mn=[0.0]))
    nfr = int(26 * fps)
    writer = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgba", "-s", "1200x720", "-r", str(fps), "-i", "-",
                               "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", str(ANIM / "a8_ns2_off_vs_minus1.mp4")], stdin=subprocess.PIPE)
    for f in range(nfr):
        tnow = f / fps
        for p in panels:
            k = int(np.searchsorted(p["s"], tnow))
            k = min(k, len(p["s"]) - 1)
            p["b1"].set_data([p["pos"][k, 0]], [p["pos"][k, 1]]); p["t1"].set_data([p["top"][k, 0]], [p["top"][k, 1]])
            p["b2"].set_data([p["pos"][k, 1]], [p["pos"][k, 2]]); p["t2"].set_data([p["top"][k, 1]], [p["top"][k, 2]])
            m = (p["s"] <= tnow) & (p["s"] >= max(0, tnow - 3))
            p["tr"].set_data(p["pos"][m, 1], p["pos"][m, 2])
            mm = (p["s"] >= 0) & (p["s"] <= tnow)
            p["cur"].set_data(p["s"][mm], p["ez"][mm]); p["dot"].set_data([p["s"][k]], [p["ez"][k]])
            p["mn"][0] = min(p["mn"][0], float(p["ez"][k]))
            p["txt"].set_text(f"t={tnow:4.1f}s  z err {p['ez'][k]:+5.1f} cm\n min so far {p['mn'][0]:+5.1f} cm")
        fig.canvas.draw()
        writer.stdin.write(np.asarray(fig.canvas.buffer_rgba()).tobytes())
    writer.stdin.close(); writer.wait()
    plt.close(fig)
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", str(ANIM / "a8_ns2_off_vs_minus1.mp4"), "-vf", "fps=10,scale=700:-1:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse",
                    str(ANIM / "a8_ns2_off_vs_minus1.gif")], check=True)


if __name__ == "__main__":
    num = collect()
    fig_dips(num); fig_traces(num); fig_variants(num); fig_table(num)
    if "--no-anim" not in sys.argv:
        animate()
    print(json.dumps(num, indent=1)[:2500])
