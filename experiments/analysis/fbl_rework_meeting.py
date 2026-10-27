#!/usr/bin/env python3
"""FBL (S4) after the rework (stock rate-PID inner loop, lab 2026-10-26) against S0-S3 of the unseen block (2026-10-23, same cells) and against the 2026-10-24 flights.
Bottom vehicle cf5, merged uSD banks only. Metrics = the meeting-doc metrics (unseen_block_analysis.flight_row): A8 crossing dip, A1 mean z error (2..15 s), A4 z RMS;
plus the horizontal RMS (same window). Variants (settings from the branch recorded in meta.json / lab confirmation, see docs/lab_sessions/2026-10-26.md):
  final  = study_FBL_stock (preset 1, inner 1, cd 0.55, network on): bank_fbl_b1_ours (A1 dz 0.45, A4 0.075 / 0.125, A8) and bank_fbl_bV_learned (A1 dz 0.5)
  rom    = study_FBL_V_rom  (cd 0.55, network off): bank_fbl_b3_cd055
  cd236  = study_FBL_cd236  (cd 0.236, network off): bank_fbl_b3_cd236
  before = 2026-10-24 flights (P-only inner loop): bank_fbl2 / bank_fbl3
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/fbl_rework_meeting.py"""
import glob, json, sys
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]; LOGS = REPO / "experiments/logs"
sys.path.insert(0, str(REPO / "experiments/analysis")); sys.path.insert(0, str(REPO / "flying_drone_stack/tools"))
import unseen_block_analysis as U  # noqa: E402
OUT = REPO / "experiments/results/fbl_rework_2026-10-26"; OUT.mkdir(parents=True, exist_ok=True)
SC = {"S0": "#7f7f7f", "S1": "#1f77b4", "S2": "#17becf", "S3": "#d62728", "S4": "#9467bd", "S4 before": "#c5b0d5"}
BANKS = {"final": ["bank_fbl_b1_ours_2026-10-26", "bank_fbl_bV_learned_2026-10-26"], "rom": ["bank_fbl_b3_cd055_2026-10-26"], "cd236": ["bank_fbl_b3_cd236_2026-10-26"],
         "before": ["bank_fbl2_2026-10-24", "bank_fbl3_2026-10-24"]}


def banks_flights(bank):
    m = json.load(open(LOGS / bank / "training_manifest.json"))
    for fl in m["flights"]:
        if fl["scenario"] not in ("A1", "A4", "A8"): continue
        meta = json.load(open(LOGS / f"{fl['scenario']}_{fl['date']}_{fl['stamp']}.meta.json")); yield fl, meta, REPO / fl["path"]


def load(csv):
    df = pd.read_csv(csv, comment="#"); pb = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf5")})[0]
    b = df[[c for c in df.columns if c.startswith(pb + ".")]].rename(columns=lambda c: c.split(".", 1)[1]); b["t"] = df.t.values
    pt = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf_second")})
    if pt: b["z_top"] = df[pt[0] + ".z"].values
    return b


def window(b, sc):
    t = b.t.values; air = b.z.values > 0.2
    return (air & (t >= 2.0) & (t <= 15.0)) if sc == "A1" else (air & (t >= 1.0))


def z_rms(b, sc):
    w = window(b, sc); return 100 * float(np.sqrt((((b.z - b.ctrltarget_z).values[w]) ** 2).mean()))


def xy_rms(b, sc):
    w = window(b, sc); return 100 * float(np.sqrt((((b.x - b.ctrltarget_x) ** 2 + (b.y - b.ctrltarget_y) ** 2).values[w]).mean()))


def ax_rms(b, sc, a):
    w = window(b, sc); return 100 * float(np.sqrt((((b[a] - b["ctrltarget_" + a]) ** 2).values[w]).mean()))


def cellname(sc, p):
    return f"A1 dz{p.get('dz')}" if sc == "A1" else (f"A4 dz{p.get('dz')} off{p.get('offset')} v{p.get('speed')}" if sc == "A4" else f"A8 dz{p.get('dz')} dur{p.get('duration', 4.375):g}")


rows = []
# ---- FBL flights
for var, banks in BANKS.items():
    for bank in banks:
        for fl, meta, csv in banks_flights(bank):
            b = load(csv); r = U.flight_row(csv, meta, "S4 " + var, bank); r.update(variant=var, xy_rms=xy_rms(b, fl["scenario"]), z_rms=z_rms(b, fl["scenario"]), x_rms=ax_rms(b, fl["scenario"], "x"), y_rms=ax_rms(b, fl["scenario"], "y"), stamp=fl["stamp"], folder=fl["folder"])
            f = b.fbl_active.values > 0 if "fbl_active" in b else np.zeros(len(b), bool)
            r.update(steps=int(b.fbl_steps.max()) if "fbl_steps" in b else 0, faults=int(b.fbl_faults.max()) if "fbl_faults" in b else 0, trip=int(b.fbl_tripped.max()) if "fbl_tripped" in b else 0)
            w = window(b, fl["scenario"]); yaw = b.yaw_deg.values[w]
            r.update(yaw_max=float(np.abs(yaw).max()), yaw_mean=float(np.mean(yaw)), roll_max=float(np.abs(b.roll_deg.values[w]).max()), pitch_max=float(np.abs(b.pitch_deg.values[w]).max()),
                     x_bias=100 * float((b.x - b.ctrltarget_x).values[w].mean()), y_bias=100 * float((b.y - b.ctrltarget_y).values[w].mean()),
                     res_az=float(np.nanmean(b.fbl_res_az.values[w])) if "fbl_res_az" in b else np.nan, thrust=float(np.nanmean(b.fbl_thrust.values[w])) if "fbl_thrust" in b else np.nan)
            rows.append(r)
F = pd.DataFrame(rows)
# ---- S0-S3 reference (unseen block, same cells): z metric from the published per_flight.csv, xy recomputed
REF = pd.read_csv(REPO / "experiments/results/unseen_block_2026-10-23/per_flight.csv"); refrows = []
for bank in REF.bank.unique():
    strat = dict(zip(REF.folder, REF.strategy))
    for fl, meta, csv in banks_flights(bank):
        if fl["folder"] not in strat: continue
        b = load(csv); r = U.flight_row(csv, meta, strat[fl["folder"]], bank); r.update(variant="ref", xy_rms=xy_rms(b, fl["scenario"]), z_rms=z_rms(b, fl["scenario"]), x_rms=ax_rms(b, fl["scenario"], "x"), y_rms=ax_rms(b, fl["scenario"], "y"), stamp=fl["stamp"], folder=fl["folder"]); refrows.append(r)
R = pd.DataFrame(refrows)
F.to_csv(OUT / "fbl_flights.csv", index=False); R.to_csv(OUT / "ref_flights.csv", index=False)

CELLS = [("A1 dz0.45", "A1 hold dz 0.45: mean z error [cm]"), ("A8 dz0.4 dur6.25", "A8 swap: crossing dip [cm]"), ("A4 dz0.4 off0.075 v0.35", "A4 offset 0.075: z RMS [cm]"), ("A4 dz0.4 off0.125 v0.35", "A4 offset 0.125: z RMS [cm]")]
print(sorted(R.cell.unique())); print(sorted(F.cell.unique()))


def cell_vals(df, cell, strat, col="metric"):
    v = df[(df.cell == cell) & (df.strategy == strat)][col].values; return v[~np.isnan(v)]


# ---- summary table (markdown for the meeting doc)
def fmt(v, d=1, pm=True):
    return "–" if len(v) == 0 else (f"{np.mean(v):.{d}f}" + (f" ± {np.std(v, ddof=0):.{d}f}" if len(v) > 1 and pm else "") + f" (n={len(v)})")


lines = []
for cell, _ in CELLS:
    lines.append(f"{cell}: " + " | ".join(f"{s} {fmt(cell_vals(R, cell, s))}" for s in ("S0", "S1", "S2", "S3")) + " | S4 " + fmt(cell_vals(F, cell, "S4 final")) + " | S4before " + fmt(cell_vals(F, cell, "S4 before")))
open(OUT / "summary_z.txt", "w").write("\n".join(lines)); print("\n".join(lines))

# ---- Figure 1: z metric per cell, S0-S3 + S4 now + S4 before (+ A1 dz 0.5 against the pilot)
PILOT = U.PILOT
fig, axs = plt.subplots(1, 5, figsize=(19, 4.2))
names = ["S0", "S1", "S2", "S3", "S4 before", "S4 now"]
for ax, (cell, title) in zip(axs, CELLS):
    vals = [cell_vals(R, cell, s) for s in ("S0", "S1", "S2", "S3")] + [cell_vals(F, cell, "S4 before"), cell_vals(F, cell, "S4 final")]
    cols = [SC[s] for s in ("S0", "S1", "S2", "S3", "S4 before", "S4")]
    for i, (v, c) in enumerate(zip(vals, cols)):
        if len(v): ax.bar(i, np.mean(v), color=c, alpha=0.85, width=0.7); ax.errorbar(i, np.mean(v), np.std(v), color="k", capsize=3, lw=1) if len(v) > 1 else None; ax.scatter([i] * len(v), v, color="k", s=14, zorder=3)
    ax.set_xticks(range(6)); ax.set_xticklabels(names, rotation=30, fontsize=8); ax.set_title(title, fontsize=9); ax.axhline(0, color="k", lw=0.6)
v5 = F[(F.cell == "A1 dz0.5") & (F.variant == "final")].metric.values; ax = axs[4]
for i, s in enumerate(("S0", "S1", "S2", "S3")):
    pj = PILOT["A1"][s]; ax.bar(i, pj["mean"], color=SC[s], alpha=0.85, width=0.7); ax.errorbar(i, pj["mean"], pj["std"], color="k", capsize=3, lw=1); ax.scatter([i] * len(pj["per_flight"]), pj["per_flight"], color="k", s=14, zorder=3)
ax.bar(4, np.mean(v5), color=SC["S4"], alpha=0.85, width=0.7); ax.errorbar(4, np.mean(v5), np.std(v5), color="k", capsize=3, lw=1); ax.scatter([4] * len(v5), v5, color="k", s=14, zorder=3)
ax.set_xticks(range(5)); ax.set_xticklabels(["S0", "S1", "S2", "S3", "S4 now"], rotation=30, fontsize=8); ax.set_title("A1 hold dz 0.5 (pilot S0-S3): mean z error [cm]", fontsize=9); ax.axhline(0, color="k", lw=0.6)
fig.suptitle("Vertical tracking: S4 FBL after the rework (final settings) against S0-S3 and against the 2026-10-24 FBL flights; bars = mean, whisker = std between flights, dots = flights", fontsize=10)
fig.tight_layout(rect=[0, 0, 1, 0.93]); fig.savefig(OUT / "fbl2_vs_s0s3_z.png", dpi=140); plt.close(fig)

# ---- Figure 2: horizontal RMS
fig, axs = plt.subplots(1, 4, figsize=(15, 4.0))
for ax, (cell, title) in zip(axs, CELLS):
    vals = [cell_vals(R, cell, s, "xy_rms") for s in ("S0", "S1", "S2", "S3")] + [cell_vals(F, cell, "S4 before", "xy_rms"), cell_vals(F, cell, "S4 final", "xy_rms")]
    for i, (v, s) in enumerate(zip(vals, ("S0", "S1", "S2", "S3", "S4 before", "S4"))):
        if len(v): ax.bar(i, np.mean(v), color=SC[s], alpha=0.85, width=0.7); ax.errorbar(i, np.mean(v), np.std(v), color="k", capsize=3, lw=1) if len(v) > 1 else None; ax.scatter([i] * len(v), v, color="k", s=14, zorder=3)
    ax.set_xticks(range(6)); ax.set_xticklabels(names, rotation=30, fontsize=8); ax.set_title(title.split(":")[0] + ": horizontal RMS [cm]", fontsize=9)
fig.suptitle("Horizontal tracking error (sqrt(x^2 + y^2) RMS against the plan): S4 FBL against S0-S3", fontsize=10)
fig.tight_layout(rect=[0, 0, 1, 0.93]); fig.savefig(OUT / "fbl2_vs_s0s3_xy.png", dpi=140); plt.close(fig)

# ---- Figure 3: C_D A/B, z error over time + downwash term in A1
fig, axs = plt.subplots(2, 3, figsize=(16, 7), gridspec_kw={"height_ratios": [2, 1.2]})
VC = {"cd236": ("#e377c2", "C_D 0.236, network off"), "rom": ("#2ca02c", "C_D 0.55, network off"), "final": ("#9467bd", "C_D 0.55, network on")}
for j, (cell, title) in enumerate([CELLS[0], CELLS[1], CELLS[3]]):
    ax = axs[0, j]
    for var in ("cd236", "rom", "final"):
        s = F[(F.cell == cell) & (F.variant == var)]
        for _, r in s.iterrows():
            b = load(REPO / f"experiments/logs/{[x for x in BANKS[var] if (LOGS / x / r.folder).exists()][0]}/{r.folder}/{r.folder}_merged_usd.csv"); w = b.z.values > 0.2
            ax.plot(b.t[w], 100 * (b.z - b.ctrltarget_z)[w], color=VC[var][0], lw=0.9, label=f"{VC[var][1]}" if _ == s.index[0] else None)
    ax.set_title(title.replace("[cm]", "(z error over time)"), fontsize=9); ax.set_ylabel("z error [cm]"); ax.set_xlabel("time [s]"); ax.axhline(0, color="k", lw=0.5)
    if j == 0: ax.legend(fontsize=8)
ax = axs[1, 0]
for var in ("cd236", "rom", "final"):
    s = F[(F.cell == "A1 dz0.45") & (F.variant == var)]
    for _, r in s.iterrows():
        b = load(REPO / f"experiments/logs/{[x for x in BANKS[var] if (LOGS / x / r.folder).exists()][0]}/{r.folder}/{r.folder}_merged_usd.csv"); w = (b.z.values > 0.2) & (b.t.values > 2)
        ax.plot(b.t[w], b.fbl_res_az[w].rolling(50, min_periods=1).mean(), color=VC[var][0], lw=1.1)
ax.set_title("A1 dz 0.45: downwash term added to the z acceleration [m/s²]", fontsize=9); ax.set_xlabel("time [s]")
for j in (1, 2): axs[1, j].axis("off")
tab = F[F.variant.isin(["cd236", "rom", "final"]) & F.cell.isin([c for c, _ in CELLS])].groupby(["variant", "cell"]).metric.mean().unstack(0)
axs[1, 1].text(0, 1, "z metric (cm), mean (n=1 per cell)\n" + tab.round(2).rename(columns={"cd236": "C_D 0.236", "rom": "0.55 net off", "final": "0.55 net on"}).to_string(), family="monospace", fontsize=8, va="top"); axs[1, 1].axis("off")
fig.suptitle("C_D 0.236 (authors) vs 0.55 (ours), network off, preset 1; third curve = final settings with the network on", fontsize=10)
fig.tight_layout(rect=[0, 0, 1, 0.95]); fig.savefig(OUT / "fbl2_cd_ab.png", dpi=140); plt.close(fig)

# ---- Figure 4: before / after traces (A4 0.125 and A8)
def pick_before(cell):
    s = F[(F.variant == "before") & (F.cell == cell)].sort_values("stamp"); return s.iloc[0] if len(s) else None


fig, axs = plt.subplots(4, 2, figsize=(14, 11), sharex="col")
for j, cell in enumerate(["A4 dz0.4 off0.125 v0.35", "A8 dz0.4 dur6.25"]):
    now = F[(F.variant == "final") & (F.cell == cell)].iloc[0]; bef = F[(F.variant == "before") & (F.cell == cell)]
    # 2026-10-24 flights of the same cell (offset 0.125 at dz 0.5 in the 10-24 session: use the cell nearest in settings)
    if len(bef) == 0:
        bef = F[(F.variant == "before") & (F.cell.str.startswith(cell.split()[0]))]
        if cell.startswith("A4"): bef = bef[bef.cell.str.contains("off0.125")]
    br = bef.sort_values("stamp").iloc[0]
    for lab, r, c in (("2026-10-24 (P-only inner loop)", br, "#ff7f0e"), ("now (stock rate PID)", now, SC["S4"])):
        bank = [x for k in ("before", "final") for x in BANKS[k] if (LOGS / x / r.folder).exists()][0]; b = load(REPO / f"experiments/logs/{bank}/{r.folder}/{r.folder}_merged_usd.csv")
        w = (b.z.values > 0.2) & (b.t.values >= 0); t = b.t[w]
        for i, (v, yl) in enumerate(((100 * (b.y - b.ctrltarget_y)[w], "y error [cm]"), (100 * (b.x - b.ctrltarget_x)[w], "x error [cm]"), (b.yaw_deg[w], "yaw [deg] (cmd 0)"), (np.hypot(b.roll_deg, b.pitch_deg)[w], "tilt [deg]"))):
            axs[i, j].plot(t, v, color=c, lw=1.0, label=lab if i == 0 else None); axs[i, j].set_ylabel(yl)
    axs[0, j].set_title(f"{cell.split()[0]} {'offset 0.125' if cell.startswith('A4') else 'swap'} (before: {br.cell}, {br.stamp})", fontsize=9); axs[3, j].set_xlabel("time [s]"); axs[0, j].legend(fontsize=8)
fig.suptitle("Before / after the rework: sideways error, yaw and tilt over time (one flight each, FBL-controlled)", fontsize=10); fig.tight_layout(rect=[0, 0, 1, 0.96]); fig.savefig(OUT / "fbl2_before_after.png", dpi=140); plt.close(fig)

# ---- health table
H = F[F.variant.isin(["final"])].sort_values(["cell", "stamp"])[["cell", "stamp", "steps", "faults", "trip", "yaw_max", "yaw_mean", "roll_max", "pitch_max", "x_bias", "y_bias", "res_az", "thrust", "xy_rms", "metric"]]
H.round(2).to_csv(OUT / "health_final.csv", index=False); print(H.round(2).to_string(index=False))
B = F[F.variant == "before"].sort_values(["cell", "stamp"])[["cell", "stamp", "steps", "trip", "yaw_max", "yaw_mean", "x_bias", "y_bias", "xy_rms", "metric"]]; B.round(2).to_csv(OUT / "health_before.csv", index=False); print(B.round(2).to_string(index=False))


# ================= meeting-doc figures (style of the 2026-10-22 overview) =================
PAST = {"S0": "#cccccc", "S1": "#a6c8e0", "S2": "#a8dba8", "S3": "#f0a8a8", "S4r": "#e8dcf5", "S4": "#cdb8e6"}; DOT = {"S0": "#7f7f7f", "S1": "#1f77b4", "S2": "#2ca02c", "S3": "#d62728", "S4r": "#b39ad6", "S4": "#7b3fb5"}
STR = ["S0", "S1", "S2", "S3", "S4r", "S4"]; LAB = {"S0": "S0 geometric", "S1": "S1 INDI", "S2": "S2 geom. + NS2", "S3": "S3 INDI + NS2", "S4r": "S4 FBL ROM (network off)", "S4": "S4 FBL ROM + learned"}
A4C = ["A4 dz0.4 off0.075 v0.35", "A4 dz0.4 off0.125 v0.35"]


def vals(cell, s, col):
    if s == "S4": return cell_vals(F, cell, "S4 final", col)
    if s == "S4r": return cell_vals(F, cell, "S4 rom", col)
    return cell_vals(R, cell, s, col)


def bars(ax, groups, col, ylabel, title, glabels):
    n = len(STR); wd = 0.8 / n
    for gi, cell in enumerate(groups):
        for si, s in enumerate(STR):
            v = vals(cell, s, col) if not (isinstance(cell, tuple)) else np.asarray(cell[1].get(s, []))
            if len(v) == 0: continue
            x = gi - 0.4 + wd * (si + 0.5)
            ax.bar(x, np.mean(v), wd * 0.92, color=PAST[s], label=LAB[s] if gi == 0 else None)
            if len(v) > 1: ax.errorbar(x, np.mean(v), np.std(v, ddof=1), color="k", capsize=2.5, lw=1)
            ax.scatter(np.full(len(v), x), v, color=DOT[s], s=14, zorder=3)
    ax.set_xticks(range(len(groups))); ax.set_xticklabels(glabels, fontsize=9); ax.set_ylabel(ylabel, fontsize=9); ax.set_title(title, fontsize=10); ax.axhline(0, color="k", lw=0.6)


pilotA1 = {s: np.asarray(PILOT["A1"][s]["per_flight"]) for s in ("S0", "S1", "S2", "S3")}; pilotA1["S4"] = F[(F.cell == "A1 dz0.5") & (F.variant == "final")].metric.values
fig, axs = plt.subplots(3, 3, figsize=(17, 11.5))
bars(axs[0, 0], ["A1 dz0.45", ("trained", pilotA1)], "metric", "mean z error in the hold [cm]", "A1 stacked hold: mean height error", ["dz 0.45 (not in NS2 training)", "dz 0.5 (in NS2 training; S0-S3 of 10-22,\nS4 learned = 3 flights, ROM not flown)"])
bars(axs[0, 1], ["A8 dz0.4 dur6.25"], "metric", "mean crossing dip per flight [cm]", "A8 swap: height dip at the crossing", ["dz 0.4, 0.35 m/s"])
bars(axs[0, 2], A4C, "z_rms", "z RMS [cm]", "A4 offset line: height error (RMS)", ["offset 0.075 m", "offset 0.125 m"])
for i, (col, an) in enumerate((("x_rms", "x"), ("y_rms", "y")), start=1):
    bars(axs[i, 0], ["A1 dz0.45"], col, f"{an} RMS [cm]", f"A1 hold: {an} error (RMS)", ["dz 0.45"])
    bars(axs[i, 1], ["A8 dz0.4 dur6.25"], col, f"{an} RMS [cm]", f"A8 swap: {an} error (RMS)", ["dz 0.4, 0.35 m/s"])
    bars(axs[i, 2], A4C, col, f"{an} RMS [cm]", f"A4 offset line: {an} error (RMS)", ["offset 0.075 m", "offset 0.125 m"])
axs[0, 2].legend(fontsize=8, loc="upper right")
fig.suptitle("S4 FBL against the unified controller tree (S0-S3), bottom drone, real flights, uSD logs only\nbars = mean over flights, whisker = ± std between flights, dots = single flights; S4: 1 flight per cell; FBL Nominal (no downwash model) not flown yet", fontsize=10)
fig.tight_layout(rect=[0, 0, 1, 0.94]); fig.savefig(OUT / "fbl2_overview.png", dpi=140); plt.close(fig)

# ---- within FBL: every flight by setting
VAR = [("before", "2026-10-24 (P-only inner loop)", "#ff7f0e", "o"), ("cd236", "C_D 0.236, network off", "#e377c2", "s"), ("rom", "C_D 0.55, network off", "#2ca02c", "^"), ("final", "C_D 0.55, network on (final)", "#7b3fb5", "D")]
CL = [("A1 dz0.45", "A1 hold\ndz 0.45"), ("A1 dz0.5", "A1 hold\ndz 0.5"), ("A8 dz0.4 dur6.25", "A8 swap"), (A4C[0], "A4 offset\n0.075 m"), (A4C[1], "A4 offset\n0.125 m")]
fig, axs = plt.subplots(1, 2, figsize=(15, 4.6))
for ax, col, yl in ((axs[0], "z_rms", "z RMS [cm]"), (axs[1], "xy_rms", "horizontal RMS [cm]")):
    for ci, (cell, _) in enumerate(CL):
        for vi, (var, lab, c, mk) in enumerate(VAR):
            v = F[(F.cell == cell) & (F.variant == var)][col].values
            if len(v): ax.scatter(np.full(len(v), ci - 0.3 + 0.2 * vi) + np.linspace(-0.04, 0.04, len(v)) * (len(v) > 1), v, color=c, marker=mk, s=36, edgecolor="k", linewidth=0.4, zorder=3, label=lab if (ci == 0 and var != "final") or (ci == 1 and var == "final") else None)
    ax.set_xticks(range(len(CL))); ax.set_xticklabels([l for _, l in CL], fontsize=9); ax.set_ylabel(yl); ax.set_yscale("log"); ax.grid(axis="y", alpha=0.3, which="both")
h, l = axs[0].get_legend_handles_labels(); fig.legend(h, l, loc="lower center", ncol=4, fontsize=9); axs[0].set_title("vertical tracking: z RMS (log scale)", fontsize=10); axs[1].set_title("horizontal tracking: sqrt(x² + y²) RMS (log scale)", fontsize=10)
fig.suptitle("All FBL flights with data (one dot = one flight, uSD logs). Not shown: the 2 flights with the authors' code gains (preset 0) flipped on takeoff", fontsize=10)
fig.tight_layout(rect=[0, 0.07, 1, 0.93]); fig.savefig(OUT / "fbl2_variants.png", dpi=140); plt.close(fig)

# ---- time series per cell: S4 (ROM + learned, and ROM) against S0 / S1 / S3; rows z, x, y error
fig, axs = plt.subplots(3, 4, figsize=(18, 9.5), sharex="col")
TC = [("A1 dz0.45", "A1 hold, dz 0.45"), ("A8 dz0.4 dur6.25", "A8 swap"), (A4C[0], "A4 offset 0.075 m"), (A4C[1], "A4 offset 0.125 m")]
ERR = (("z", "z error [cm]"), ("x", "x error [cm]"), ("y", "y error [cm]"))
for j, (cell, title) in enumerate(TC):
    for s_, c, lw in (("S0", "#9a9a9a", 0.8), ("S1", "#1f77b4", 0.8), ("S3", "#d62728", 0.8)):
        for k, (_, r) in enumerate(R[(R.cell == cell) & (R.strategy == s_)].iterrows()):
            b = load(REPO / f"experiments/logs/{r.bank}/{r.folder}/{r.folder}_merged_usd.csv"); w = (b.z.values > 0.2) & (b.t.values >= 0)
            for i, (a, _) in enumerate(ERR): axs[i, j].plot(b.t[w], 100 * (b[a] - b["ctrltarget_" + a])[w], color=c, lw=lw, alpha=0.8, label=LAB[s_] if (k == 0 and i == 0) else None)
    for var, c, lw, ls, lab in (("rom", "#b39ad6", 1.4, "--", LAB["S4r"]), ("final", "#7b3fb5", 1.6, "-", LAB["S4"])):
        for _, r in F[(F.cell == cell) & (F.variant == var)].iterrows():
            bank = [x for x in BANKS[var] if (LOGS / x / r.folder).exists()][0]; b = load(REPO / f"experiments/logs/{bank}/{r.folder}/{r.folder}_merged_usd.csv"); w = (b.z.values > 0.2) & (b.t.values >= 0)
            for i, (a, _) in enumerate(ERR): axs[i, j].plot(b.t[w], 100 * (b[a] - b["ctrltarget_" + a])[w], color=c, lw=lw, ls=ls, label=lab if i == 0 else None)
    axs[0, j].set_title(title, fontsize=10); axs[2, j].set_xlabel("time [s]")
    for i, (_, yl) in enumerate(ERR): axs[i, j].axhline(0, color="k", lw=0.5); axs[i, j].set_ylabel(yl if j == 0 else "")
axs[0, 0].legend(fontsize=8)
fig.suptitle("Tracking error over time (measured minus planned position): S4 FBL against S0 / S1 / S3, every flight, bottom drone, uSD logs", fontsize=10); fig.tight_layout(rect=[0, 0, 1, 0.96]); fig.savefig(OUT / "fbl2_timeseries.png", dpi=140); plt.close(fig)
F.to_csv(OUT / "fbl_flights.csv", index=False); R.to_csv(OUT / "ref_flights.csv", index=False)
