#!/usr/bin/env python3
"""Does the tracking error depend on the battery, per strategy? Pilot flights 2026-10-19/20 (A8, A1, A4), bottom vehicle, uSD logs only.
Battery measure: mean loaded cell voltage of the bottom drone while airborne (z > 0.2 m), from the merged uSD csv.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/battery_vs_error.py
Writes experiments/results/battery_analysis_2026-10-20/{per_flight.csv,results.md,results.json,battery_vs_error.png}"""
import glob, json
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]; RES = REPO / "experiments/results"; OUT = RES / "battery_analysis_2026-10-20"; OUT.mkdir(exist_ok=True)
ORDER = ["S0", "S1", "S2", "S3"]; COL = {"S0": "#7f7f7f", "S1": "#1f77b4", "S2": "#2ca02c", "S3": "#d62728"}; INDI = {"S0": 0, "S1": 1, "S2": 0, "S3": 1}
DRAIN = {"17-55-03", "17-56-35"}
csvs = {Path(p).name.replace("_merged_usd.csv", ""): p for p in glob.glob(str(REPO / "experiments/logs/bank_*/A*/*_merged_usd.csv"))}
def vbat(name):
    df = pd.read_csv(csvs[name], comment="#"); pb = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf5_")})[0]
    z = df[pb + ".z"].values; v = df[pb + ".vbat"].values; w = z > 0.2
    return float(v[w].mean()), float(v[w].min())
rows = []
a8 = json.load(open(RES / "v2val_2026-10-19/v2val_summary.json"))
for b in ("pilot2", "pilot3", "pilot5"): a8 += json.load(open(RES / f"{b}_2026-10-20/a8_summary.json"))
for r in a8:
    if r["file"].split("_")[-1] in DRAIN: continue
    rows.append(dict(scenario="A8", strategy=r["name"], file=r["file"], err=r["dip_mean_cm"], err_name="dip_cm", rms=r["z_rms_cm"], setting=""))
a1 = json.load(open(RES / "a1_2026-10-19/a1_summary.json"))
for b in ("pilot2", "pilot3", "pilot5", "video"): a1 += json.load(open(RES / f"{b}_2026-10-20/a1_summary.json"))
for r in a1: rows.append(dict(scenario="A1", strategy=r["strategy"], file=r["file"], err=r["z_mean_cm"], err_name="z_mean_cm", rms=r["z_rms_cm"], setting=""))
a4 = json.load(open(RES / "pilot_all_2026-10-20/cell_summary_all.json"))
for b in ("pilot5", "pilot6"):
    a4 += [dict(r, strategy="S1") for r in json.load(open(RES / f"{b}_2026-10-20/cell_summary.json"))]
for r in a4:
    if r["scenario"] != "A4" or r["offset"] < 0.01: continue
    rows.append(dict(scenario="A4", strategy=r["strategy"], file=r["folder"], err=r["z_rms_cm"], err_name="z_rms_cm", rms=r["z_rms_cm"], setting=f"offset {r['offset']:.2f}"))
R = pd.DataFrame(rows); R[["vbat_mean", "vbat_min"]] = [vbat(f) for f in R.file]; R["indi"] = R.strategy.map(INDI)
R.to_csv(OUT / "per_flight.csv", index=False)
rng = np.random.default_rng(0)
def slope(v, y): return np.polyfit(v, y, 1)[0] / 10          # cm per 0.1 V
def boot(v, y, n=4000):
    s = []
    for _ in range(n):
        i = rng.integers(0, len(v), len(v))
        if np.ptp(v[i]) < 1e-6: continue
        s.append(slope(v[i], y[i]))
    return np.percentile(s, [2.5, 97.5])
def rank(x):
    return pd.Series(x).rank().values
L = ["# Battery vs tracking error (pilot flights 2026-10-19/20)", "", "Battery = mean loaded voltage of the bottom drone while airborne. Error: A8 mean crossing dip per flight [cm, negative = low], A1 mean z error [cm], A4 z RMS [cm] (A4: the offset effect removed within each strategy by subtracting its offset-cell mean, strategy mean kept). Slope = cm per 0.1 V (positive slope for dip/z error = higher battery gives a smaller negative error; for RMS positive = worse). Bootstrap 95 % interval over flights.", ""]
res = {}
for sc in ("A8", "A1", "A4"):
    L += [f"## {sc}", "| strategy | n | battery range [V] | r | Spearman | slope [cm / 0.1 V] | 95 % interval |", "|---|---|---|---|---|---|---|"]
    d = R[R.scenario == sc].copy()
    if sc == "A4": d["err"] = d.err - d.groupby(["strategy", "setting"]).err.transform("mean") + d.groupby("strategy").err.transform("mean")
    for s in ORDER:
        g = d[d.strategy == s]; v = g.vbat_mean.values; y = g.err.values
        if len(g) < 4: continue
        r_ = np.corrcoef(v, y)[0, 1]; sp = np.corrcoef(rank(v), rank(y))[0, 1]; sl = slope(v, y); ci = boot(v, y)
        res[f"{sc}_{s}"] = dict(n=len(g), r=r_, spearman=sp, slope=sl, ci=[float(ci[0]), float(ci[1])], vmin=float(v.min()), vmax=float(v.max()))
        L.append(f"| {s} | {len(g)} | {v.min():.2f}–{v.max():.2f} | {r_:+.2f} | {sp:+.2f} | {sl:+.2f} | [{ci[0]:+.2f}, {ci[1]:+.2f}] |")
    for grp, name in ((1, "INDI on (S1+S3)"), (0, "INDI off (S0+S2)")):
        g = d[d.indi == grp]; y = g.err.values - g.groupby("strategy").err.transform("mean").values; v = g.vbat_mean.values - g.groupby("strategy").vbat_mean.transform("mean").values
        sl = np.polyfit(v, y, 1)[0] / 10; r_ = np.corrcoef(v, y)[0, 1]; ci = boot(v, y)
        res[f"{sc}_pooled_{'indi' if grp else 'geo'}"] = dict(n=len(g), r=r_, slope=sl, ci=[float(ci[0]), float(ci[1])])
        L.append(f"| **{name}, strategy means removed** | {len(g)} | | {r_:+.2f} | | {sl:+.2f} | [{ci[0]:+.2f}, {ci[1]:+.2f}] |")
    L += ["", "Mean battery per strategy (is the comparison confounded?): " + ", ".join(f"{s} {d[d.strategy == s].vbat_mean.mean():.2f} V" for s in ORDER), ""]
    # adjusted comparison: common slope per group (INDI on / off), strategies compared at the common battery
    v0 = d.vbat_mean.mean(); adj = {}
    for grp in (0, 1):
        g = d[d.indi == grp]; y = g.err.values - g.groupby("strategy").err.transform("mean").values; v = g.vbat_mean.values - g.groupby("strategy").vbat_mean.transform("mean").values
        b_ = np.polyfit(v, y, 1)[0]
        for s in ORDER:
            if INDI[s] == grp: gs = g[g.strategy == s]; adj[s] = (gs.err.mean(), gs.err.mean() + b_ * (v0 - gs.vbat_mean.mean()))
    L += [f"Strategy means raw -> adjusted to the common battery {v0:.2f} V (common slope per INDI group): " + ", ".join(f"{s} {adj[s][0]:+.1f} -> {adj[s][1]:+.1f}" for s in ORDER), ""]
    res[f"{sc}_adjusted"] = {s: dict(raw=float(adj[s][0]), adj=float(adj[s][1])) for s in ORDER}; res[f"{sc}_v0"] = float(v0)
(OUT / "results.md").write_text("\n".join(L) + "\n"); json.dump(res, open(OUT / "results.json", "w"), indent=1); print("\n".join(L))
fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
for a_, sc, yl in zip(ax, ("A8", "A1", "A4"), ("A8 mean crossing dip [cm]", "A1 mean z error [cm]", "A4 z RMS [cm]")):
    for s in ORDER:
        g = R[(R.scenario == sc) & (R.strategy == s)]; a_.scatter(g.vbat_mean, g.err, c=COL[s], label=s, s=26)
        if len(g) >= 4 and np.ptp(g.vbat_mean) > 0: k = np.polyfit(g.vbat_mean, g.err, 1); xs = np.array([g.vbat_mean.min(), g.vbat_mean.max()]); a_.plot(xs, np.polyval(k, xs), c=COL[s], lw=1.2)
    a_.set_xlabel("mean loaded battery voltage while airborne [V]"); a_.set_ylabel(yl); a_.set_title(sc + (" (all offsets)" if sc == "A4" else ""))
ax[0].legend(); fig.suptitle("Tracking error against battery voltage, one dot per flight, line = linear fit per strategy"); fig.tight_layout(); fig.savefig(OUT / "battery_vs_error.png", dpi=130)

# ---- summary figure: slope per strategy with interval, and raw vs adjusted means
fig, ax = plt.subplots(1, 2, figsize=(14, 4.6), gridspec_kw=dict(width_ratios=[1.1, 1]))
y = 0; yt = []; yl = []
for sc, unit in (("A8", "dip"), ("A1", "mean z error"), ("A4", "z RMS")):
    for s in ORDER:
        k = f"{sc}_{s}"
        if k not in res: continue
        r_ = res[k]; lo, hi = max(r_["ci"][0], -4), min(r_["ci"][1], 6)
        ax[0].plot([lo, hi], [y, y], c=COL[s], lw=3, alpha=.5); ax[0].plot(r_["slope"], y, "o", c=COL[s], ms=8)
        sig = "  *" if (r_["ci"][0] > 0 or r_["ci"][1] < 0) else ""
        yt.append(y); yl.append(f"{sc} {s}  (n={r_['n']}){sig}"); y -= 1
    y -= .6
ax[0].axvline(0, c="k", lw=1); ax[0].set_yticks(yt); ax[0].set_yticklabels(yl, fontsize=8); ax[0].set_xlim(-4, 6)
ax[0].set_xlabel("change of the error per +0.1 V battery [cm]   (dot = fit, bar = 95 % interval, * = excludes 0)")
ax[0].set_title("Battery effect per strategy: in A1 only INDI (S1, S3) leans on it\n(A8, A1: positive = higher battery, higher drone)")
w = 0.38
for i, sc in enumerate(("A8", "A1", "A4")):
    for j, s in enumerate(ORDER):
        x = i * 5 + j; a_ = res[f"{sc}_adjusted"][s]
        ax[1].bar(x - w / 2, a_["raw"], w, color=COL[s], alpha=.45, label="as measured" if (i, j) == (0, 0) else None)
        ax[1].bar(x + w / 2, a_["adj"], w, color=COL[s], label="adjusted to the same battery" if (i, j) == (0, 0) else None)
ax[1].set_xticks([j + i * 5 for i in range(3) for j in range(4)]); ax[1].set_xticklabels(ORDER * 3, fontsize=8)
for i, (sc, t) in enumerate((("A8", "A8 dip [cm]"), ("A1", "A1 mean z error [cm]"), ("A4", "A4 z RMS [cm]"))): ax[1].text(i * 5 + 1.5, ax[1].get_ylim()[1] * .92 if False else 17, t, ha="center", fontsize=9)
ax[1].axhline(0, c="k", lw=.8); ax[1].set_ylim(-21, 19); ax[1].legend(loc="lower right", fontsize=8)
ax[1].set_title("Same ranking with all strategies on the same battery")
fig.tight_layout(); fig.savefig(OUT / "battery_summary.png", dpi=130)
