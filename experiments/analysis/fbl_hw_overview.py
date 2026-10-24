#!/usr/bin/env python3
"""FBL hardware flights 2026-10-24: position error (x, y, z) and orientation (roll, pitch, yaw) of the bottom drone (cf5) against the plan, every FBL flight,
per cell, against the S0-S3 flights of the unseen block (2026-10-23) and the 4 same-day flights that ran as S0 (init-wipe, bank_fbl_2026-10-24).
Plan = ctrltarget_x/y/z; commanded yaw = 0 (Omar's desired yaw rpyd_z = 0 in every S0-S3 flight); the plan's accelerations need < 1.1 deg of tilt, so roll / pitch are shown as flown.
FBL's own logged desired attitude (ctrlOmarIndi.rpyd under FBL) is not used: it reads -30..-58 deg pitch on average while the drone flies level (logging-side value, see doc).
Window: t >= 0 (scenario start), plan z > 0.3 m, z > 0.2 m; FBL flights only while FBL is active (cut at the latch trip).
Gain blocks identified from the logs (see block()): preset 1 = 5 flights, _kr (rate loop x0.5) = 3, _kp (xy position x0.7, = frozen proposal) = 4.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/fbl_hw_overview.py --out experiments/results/fbl_hw_overview_2026-10-24"""
import argparse, json
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]
ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); a = ap.parse_args(); out = a.out if a.out.is_absolute() else REPO / a.out; out.mkdir(parents=True, exist_ok=True)
FBL = ["bank_fbl2_2026-10-24", "bank_fbl3_2026-10-24"]; S0DAY = "bank_fbl_2026-10-24"
REF = pd.read_csv(REPO / "experiments/results/unseen_block_2026-10-23/per_flight.csv")
BCOL = {"preset 1": "#9467bd", "rate x0.5 (_kr)": "#ff7f0e", "xy x0.7 (_kp)": "#2ca02c"}
SCOL = {"S0": "#7f7f7f", "S1": "#1f77b4", "S2": "#17becf", "S3": "#d62728", "S0 same day": "#bcbd22"}
CELLS = ["A1", "A4 0.075", "A4 0.125", "A8"]

def block(stamp):
    # identified from the logs (rate gains from torque vs rate error, position gains from the logged body rates; fbl_replay_check.py), NOT from the time:
    # the time-based split put 14-44 into _kr (it flew preset 1) and 14-54 into _kp (it flew _kr)
    return BLOCK[stamp]
BLOCK = {**{s: "preset 1" for s in ("14-25-32", "14-27-01", "14-29-07", "14-30-39", "14-44-24")}, **{s: "rate x0.5 (_kr)" for s in ("14-46-44", "14-50-08", "14-54-00")},
         **{s: "xy x0.7 (_kp)" for s in ("14-55-45", "14-57-11", "14-59-53", "15-01-21")}}

def cell_of(scen, meta):
    return f"A4 {meta['params'].get('offset'):g}" if scen == "A4" else scen

def load(path, fbl):
    df = pd.read_csv(REPO / path, comment="#"); p = [c.split(".")[0] for c in df.columns if c.startswith("cf5")][0]
    g = lambda n: df[f"{p}.{n}"].values if f"{p}.{n}" in df else np.full(len(df), np.nan)
    d = dict(t=df.t.values, ex=g("x") - g("ctrltarget_x"), ey=g("y") - g("ctrltarget_y"), ez=g("z") - g("ctrltarget_z"), roll=g("roll_deg"), pitch=g("pitch_deg"), yaw=g("yaw_deg"))
    m = (d["t"] >= 0) & (g("ctrltarget_z") > 0.3) & (g("z") > 0.2); d["trip_t"] = None
    if fbl:
        tr = g("fbl_tripped") > 0
        if tr.any(): d["trip_t"] = float(d["t"][np.argmax(tr)]); m &= d["t"] < d["trip_t"]
        m &= g("fbl_active") > 0
    d["m"] = m; return d

def manifest(bank):
    for fl in json.load(open(REPO / "experiments/logs" / bank / "training_manifest.json"))["flights"]:
        if fl["scenario"] not in ("A1", "A4", "A8"): continue
        meta = json.load(open(REPO / "experiments/logs" / f"{fl['scenario']}_{fl['date']}_{fl['stamp']}.meta.json")); yield fl, meta

flights = []                                                    # (label, group, cell, stamp, data)
for b in FBL:
    for fl, meta in manifest(b): flights.append(dict(kind="FBL", group=block(fl["stamp"]), cell=cell_of(fl["scenario"], meta), stamp=fl["stamp"], d=load(fl["path"], True)))
for fl, meta in manifest(S0DAY): flights.append(dict(kind="ref", group="S0 same day", cell=cell_of(fl["scenario"], meta), stamp=fl["stamp"], d=load(fl["path"], False)))
for b in REF.bank.unique():
    strat = dict(zip(REF.folder, REF.strategy))
    for fl, meta in manifest(b):
        if fl["folder"] in strat: flights.append(dict(kind="ref", group=strat[fl["folder"]], cell=cell_of(fl["scenario"], meta), stamp=fl["stamp"], d=load(fl["path"], False)))

rms = lambda v: float(np.sqrt(np.nanmean(v ** 2)))
rows = []
for f in flights:
    d, m = f["d"], f["d"]["m"]
    rows.append(dict(kind=f["kind"], group=f["group"], cell=f["cell"], stamp=f["stamp"], tripped=d["trip_t"] is not None, trip_t=d["trip_t"], dur_s=float(m.sum() * np.median(np.diff(d["t"]))),
                     ex_rms_cm=100 * rms(d["ex"][m]), ey_rms_cm=100 * rms(d["ey"][m]), ez_rms_cm=100 * rms(d["ez"][m]), exy_max_cm=100 * float(np.nanmax(np.hypot(d["ex"], d["ey"])[m])),
                     roll_rms=rms(d["roll"][m]), pitch_rms=rms(d["pitch"][m]), yaw_mean=float(np.nanmean(d["yaw"][m])), yaw_absmax=float(np.nanmax(np.abs(d["yaw"][m])))))
T = pd.DataFrame(rows).sort_values(["cell", "kind", "group", "stamp"]); T.to_csv(out / "per_flight_errors.csv", index=False)

# ---------- per cell: time series of every FBL flight against one flight each of S0 and S3 ----------
QTY = [("ex", 100, "x error [cm]"), ("ey", 100, "y error [cm]"), ("ez", 100, "z error [cm]"), ("roll", 1, "roll [deg]"), ("pitch", 1, "pitch [deg]"), ("yaw", 1, "yaw [deg] (cmd 0)")]
for cell in CELLS:
    fig, axs = plt.subplots(len(QTY), 1, figsize=(11, 12), sharex=True)
    for s in ("S0", "S3"):
        f = next((f for f in flights if f["cell"] == cell and f["group"] == s), None)
        if f is None: continue
        d = f["d"]; m = d["m"]
        for ax, (k, sc, _) in zip(axs, QTY): ax.plot(d["t"][m], sc * d[k][m], c=SCOL[s], lw=0.8, alpha=0.6, label=f"{s} {f['stamp'][:5]} (10-23)")
    for f in [f for f in flights if f["cell"] == cell and f["kind"] == "FBL"]:
        d = f["d"]; m = d["m"]; c = BCOL[f["group"]]; lab = f"FBL {f['group']} {f['stamp'][:5]}" + (f" TRIP {d['trip_t']:.1f}s" if d["trip_t"] else "")
        for ax, (k, sc, _) in zip(axs, QTY):
            ax.plot(d["t"][m], sc * d[k][m], c=c, lw=1.4, ls="--" if d["trip_t"] else "-", label=lab)
            if d["trip_t"]: ax.axvline(d["trip_t"], c=c, lw=0.8, ls=":")
    for ax, (_, _, yl) in zip(axs, QTY): ax.set_ylabel(yl, fontsize=8); ax.grid(alpha=0.3); ax.axhline(0, c="k", lw=0.5)
    axs[0].legend(fontsize=6, ncol=3, loc="upper left"); axs[-1].set_xlabel("time since scenario start [s] (FBL lines end at the latch trip)")
    fig.suptitle(f"{cell}: bottom drone against the plan, every FBL flight (colour = gain block, dashed = tripped) vs S0 / S3", fontsize=9)
    fig.tight_layout(); fig.savefig(out / f"cell_{cell.replace(' ', '_')}.png", dpi=120); plt.close(fig)

# ---------- summary: per-flight values per cell and group ----------
MET = [("ex_rms_cm", "x error rms [cm]"), ("ey_rms_cm", "y error rms [cm]"), ("ez_rms_cm", "z error rms [cm]"), ("roll_rms", "roll rms [deg]"), ("pitch_rms", "pitch rms [deg]"), ("yaw_mean", "yaw mean [deg] (cmd 0)")]
GROUPS = ["S0", "S1", "S2", "S3", "S0 same day"] + list(BCOL); GCOL = {**SCOL, **BCOL}
fig, axs = plt.subplots(len(MET), len(CELLS), figsize=(14, 13), sharex=True)
for j, cell in enumerate(CELLS):
    for i, (k, yl) in enumerate(MET):
        ax = axs[i, j]
        for x, g in enumerate(GROUPS):
            v = T[(T.cell == cell) & (T.group == g)]
            if not len(v): continue
            ax.bar(x, v[k].mean(), color=GCOL[g], alpha=0.35); ax.scatter(np.full(len(v), x), v[k], c=GCOL[g], s=22, zorder=3, edgecolors="k", linewidths=0.4)
            for _, r in v[v.tripped].iterrows(): ax.scatter(x, r[k], marker="x", c="r", s=40, zorder=4)
        ax.grid(alpha=0.3, axis="y"); ax.axhline(0, c="k", lw=0.5)
        if j == 0: ax.set_ylabel(yl, fontsize=8)
        if i == 0: ax.set_title(cell, fontsize=9)
    axs[-1, j].set_xticks(range(len(GROUPS))); axs[-1, j].set_xticklabels(["S0", "S1", "S2", "S3", "S0\nsame day", "FBL\npreset1", "FBL\n_kr", "FBL\n_kp"], fontsize=7)
fig.suptitle("Per flight (dots; red x = tripped, value up to the trip), bar = mean. S0-S3 = unseen block 10-23, S0 same day + FBL = 10-24", fontsize=9)
fig.tight_layout(); fig.savefig(out / "summary_per_flight.png", dpi=120); plt.close(fig)

S = T.groupby(["cell", "group"]).agg(n=("stamp", "size"), trips=("tripped", "sum"), **{k: (k, "mean") for k, _ in MET}).reset_index(); S.to_csv(out / "summary_by_group.csv", index=False)
pd.set_option("display.width", 220); print(S.round(1).to_string(index=False))
