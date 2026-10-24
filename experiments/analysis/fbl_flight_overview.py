#!/usr/bin/env python3
"""Plots for the FBL flights of 2026-10-24 (bottom drone cf5, merged uSD banks): lateral tracking, z error, tilt, distance to the top drone for A4 offset 0.125 (the cell that
destabilised FBL) against S0/S1/S3 references; per-flight tilt overview by gain block; A1 lateral excursion per strategy.
Blocks by time (pilot's account): 14:20-14:40 study_FBL (preset 1), 14:40-14:52 study_FBL_kr (rate loop x0.5), after 14:52 study_FBL_kp (xy position x0.7, = the frozen proposal).
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/fbl_flight_overview.py --out experiments/results/fbl_all_2026-10-24"""
import argparse, json
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]
ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); a = ap.parse_args(); out = a.out if a.out.is_absolute() else REPO / a.out; out.mkdir(parents=True, exist_ok=True)
FBL = ["bank_fbl2_2026-10-24", "bank_fbl3_2026-10-24"]; REFB = ["bank_unseen_S0_2026-10-23", "bank_unseen_S1_2026-10-23", "bank_unseen_S3_2026-10-23", "bank_unseen_S3S2_2026-10-23"]
BCOL = {"preset 1 (study_FBL)": "#9467bd", "rate loop x0.5 (_kr)": "#ff7f0e", "xy x0.7 (_kp)": "#2ca02c"}
def block(stamp):
    h, m = int(stamp[:2]), int(stamp[3:5]); mins = 60 * h + m
    return "preset 1 (study_FBL)" if mins < 14 * 60 + 40 else "rate loop x0.5 (_kr)" if mins < 14 * 60 + 52 else "xy x0.7 (_kp)"

def load(path):
    df = pd.read_csv(REPO / path, comment="#"); cols = sorted({c.split(".")[0] for c in df.columns if "." in c and not c.startswith("rel")})
    pb = [c for c in cols if c.startswith("cf5")][0]; pt = [c for c in cols if c.startswith("cf_second")][0]
    g = lambda p, n: df[f"{p}.{n}"].values if f"{p}.{n}" in df else np.full(len(df), np.nan)
    d = dict(t=df.t.values, x=g(pb, "x"), y=g(pb, "y"), z=g(pb, "z"), tx=g(pb, "ctrltarget_x"), ty=g(pb, "ctrltarget_y"), tz=g(pb, "ctrltarget_z"),
             roll=g(pb, "roll_deg"), pitch=g(pb, "pitch_deg"), top_x=g(pt, "x"), top_y=g(pt, "y"), top_z=g(pt, "z"), trip=g(pb, "fbl_tripped"), active=g(pb, "fbl_active"))
    d["tilt"] = np.maximum(np.abs(d["roll"]), np.abs(d["pitch"])); d["hdist"] = np.hypot(d["top_x"] - d["x"], d["top_y"] - d["y"]); return d

def flights(banks, scen=None):
    for b in banks:
        for fl in json.load(open(REPO / "experiments/logs" / b / "training_manifest.json"))["flights"]:
            if scen and fl["scenario"] != scen: continue
            meta = json.load(open(REPO / "experiments/logs" / f"{fl['scenario']}_{fl['date']}_{fl['stamp']}.meta.json")); yield fl, meta, b

def strat_of(bank, fl):                                     # S2/S3 share a bank: read the strategy from the logs
    import sys; sys.path.insert(0, str(REPO / "experiments/analysis")); import ns2_applied_check as N
    f = N.fit(str(REPO / "experiments/logs/usd_raw" / fl["cf5"]))
    return None if f is None else ("S3" if f["coef_pred"] < -0.6 else "S1") if f["indi"] else ("S2" if f["coef_pred"] < -0.6 else "S0")

# ---------- figure 1: A4 offset 0.125 ----------
fig, axs = plt.subplots(4, 1, figsize=(11, 10), sharex=True); ref_done = set()
for b in REFB:
    for fl, meta, bank in flights([b], "A4"):
        if abs(meta["params"].get("offset", 0) - 0.125) > 1e-6: continue
        s = strat_of(bank, fl)
        if s in ("S0", "S1", "S3") and s not in ref_done:
            ref_done.add(s); d = load(fl["path"]); c = {"S0": "#7f7f7f", "S1": "#1f77b4", "S3": "#d62728"}[s]; m = d["z"] > 0.3
            axs[0].plot(d["t"], d["y"] - d["ty"], c=c, lw=0.9, alpha=0.7, label=f"{s} (reference)"); axs[1].plot(d["t"][m], 100 * (d["z"] - d["tz"])[m], c=c, lw=0.9, alpha=0.7)
            axs[2].plot(d["t"], d["tilt"], c=c, lw=0.9, alpha=0.7); axs[3].plot(d["t"], d["hdist"], c=c, lw=0.9, alpha=0.7)
for fl, meta, bank in flights(FBL, "A4"):
    if abs(meta["params"].get("offset", 0) - 0.125) > 1e-6: continue
    d = load(fl["path"]); bl = block(fl["stamp"]); m = d["z"] > 0.3; tr = d["trip"] > 0; lab = f"FBL {fl['stamp']} {bl}" + (" TRIP" if tr.any() else "")
    axs[0].plot(d["t"], d["y"] - d["ty"], c=BCOL[bl], lw=1.4, ls="-" if not tr.any() else "--", label=lab); axs[1].plot(d["t"][m], 100 * (d["z"] - d["tz"])[m], c=BCOL[bl], lw=1.4, ls="-" if not tr.any() else "--")
    axs[2].plot(d["t"], d["tilt"], c=BCOL[bl], lw=1.4, ls="-" if not tr.any() else "--"); axs[3].plot(d["t"], d["hdist"], c=BCOL[bl], lw=1.4, ls="-" if not tr.any() else "--")
    if tr.any():
        k = int(np.argmax(tr)); [ax.axvline(d["t"][k], c=BCOL[bl], lw=0.8, ls=":") for ax in axs]
for ax, yl in zip(axs, ("bottom y - commanded y [m]", "z error [cm]", "tilt max(|roll|,|pitch|) [deg]", "horizontal distance to top drone [m]")): ax.set_ylabel(yl, fontsize=8); ax.grid(alpha=0.3)
axs[1].set_ylim(-40, 60); axs[2].set_ylim(0, 90); axs[3].set_xlabel("flight time [s]"); axs[0].legend(fontsize=6, ncol=2); fig.suptitle("A4 line, dz 0.4, offset 0.125: FBL flights (colour = gain block, dashed = latch tripped, dotted line = trip) against S0 / S1 / S3", fontsize=9)
fig.tight_layout(); fig.savefig(out / "fbl_a4_offset0125.png", dpi=130)

# ---------- figure 2: per-flight overview ----------
rows = []
for fl, meta, bank in flights(FBL):
    d = load(fl["path"]); m = d["z"] > 0.3; rows.append(dict(stamp=fl["stamp"], scen=fl["scenario"], off=meta["params"].get("offset"), block=block(fl["stamp"]), tilt_rms=float(np.sqrt(np.mean(d["tilt"][m] ** 2))),
                                                          tilt_max=float(np.nanmax(d["tilt"][m])), trip=bool((d["trip"] > 0).any())))
P = pd.DataFrame(rows).sort_values("stamp"); P.to_csv(out / "fbl_flight_overview.csv", index=False)
fig, axs = plt.subplots(2, 1, figsize=(11, 6), sharex=True); xs = np.arange(len(P))
for ax, k, yl in zip(axs, ("tilt_rms", "tilt_max"), ("tilt rms, airborne [deg]", "tilt max [deg]")):
    ax.bar(xs, P[k], color=[BCOL[b] for b in P.block]); ax.set_ylabel(yl, fontsize=8); ax.grid(alpha=0.3, axis="y")
    for x, (_, r) in zip(xs, P.iterrows()):
        if r.trip: ax.text(x, r[k], "TRIP", ha="center", va="bottom", fontsize=7, color="r")
axs[1].set_xticks(xs); axs[1].set_xticklabels([f"{r.scen}{'' if r.off is None or r.off != r.off else ' ' + str(r.off)}\n{r.stamp[:5]}" for _, r in P.iterrows()], fontsize=6)
axs[0].legend(handles=[plt.Rectangle((0, 0), 1, 1, color=c) for c in BCOL.values()], labels=list(BCOL), fontsize=7)
fig.suptitle("FBL flights 2026-10-24, tilt by flight and gain block (trips marked)", fontsize=9); fig.tight_layout(); fig.savefig(out / "fbl_tilt_overview.png", dpi=130)

# ---------- figure 3: A1 lateral excursion per strategy ----------
st = {}
for key, banks in (("S0", REFB), ("S1", REFB), ("S2", REFB), ("S3", REFB), ("FBL", FBL)):
    for fl, meta, bank in flights(banks, "A1"):
        s = "FBL" if key == "FBL" else strat_of(bank, fl)
        if s != key: continue
        d = load(fl["path"]); m = (d["t"] > 2) & (d["t"] < 15) & (d["z"] > 0.3)
        st.setdefault(s, []).append(dict(d=d, rms=float(np.sqrt(np.nanmean(((d["x"] - d["tx"]) ** 2 + (d["y"] - d["ty"]) ** 2)[m]))), hd=float(np.nanmean(d["hdist"][m])), tilt=float(np.sqrt(np.nanmean(d["tilt"][m] ** 2)))))
fig, axs = plt.subplots(1, 3, figsize=(13, 3.8)); cols = {"S0": "#7f7f7f", "S1": "#1f77b4", "S2": "#2ca02c", "S3": "#d62728", "FBL": "#9467bd"}; tab = []
for i, k in enumerate(("S0", "S1", "S2", "S3", "FBL")):
    if k not in st: continue
    for ax, key in zip(axs, ("rms", "hd", "tilt")):
        v = [e[key] for e in st[k]]; ax.bar(i, np.mean(v), color=cols[k], alpha=0.85); ax.scatter(np.full(len(v), i), v, c="k", s=10, zorder=3)
    tab.append(dict(strategy=k, n=len(st[k]), xy_rms_cm=100 * np.mean([e["rms"] for e in st[k]]), mean_hdist_cm=100 * np.mean([e["hd"] for e in st[k]]), tilt_rms_deg=np.mean([e["tilt"] for e in st[k]])))
for ax, t in zip(axs, ("bottom x-y deviation from its commanded position, rms [m]", "mean horizontal distance to the top drone [m]", "tilt rms [deg]")): ax.set_xticks(range(5)); ax.set_xticklabels(["S0", "S1", "S2", "S3", "FBL"]); ax.set_title(t, fontsize=8); ax.grid(alpha=0.3, axis="y")
fig.suptitle("A1 stacked hold (dz 0.45, 2-15 s): does FBL move sideways out from under the top drone?", fontsize=9); fig.tight_layout(); fig.savefig(out / "fbl_a1_lateral.png", dpi=130)
pd.DataFrame(tab).to_csv(out / "a1_lateral.csv", index=False); print(pd.DataFrame(tab).round(2).to_string(index=False)); print(P.round(1).to_string(index=False))
