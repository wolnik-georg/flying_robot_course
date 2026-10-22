#!/usr/bin/env python3
"""Analysis of the unseen-settings block (plan `docs/lab_sessions/ns2_unseen_block_plan_2026-10-22.md`, pre-registered pass rule).
Per flight (bottom vehicle cf5, merged uSD banks only; strategy read from the log, not from the yaml):
  A8: crossing dip (mean over the 4 crossings of the lowest z error within +-0.75 s * duration/4.375 of the crossing, relative to the steady offset)
  A1: mean z error in 2..15 s          A4: z RMS (airborne, from 1 s)
Per cell and strategy: mean, std between flights, n. Reference = the pilot numbers of the meeting doc (unified_overview summary.json; A4 interpolated in the offset).
Relative improvement RI = (|S0| - |S2|) / |S0| (geometric, clean NS2 effect, primary) and (|S1| - |S3|) / |S1| (INDI, secondary); retention R = RI_unseen / RI_pilot.
Verdict (pre-registered): R >= 0.7 no material overfit, 0.4 <= R < 0.7 partial, R < 0.4 confirmed (per cell and pooled over the cells).
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/unseen_block_analysis.py --banks bank_unseen1_2026-11-XX [...] --out experiments/results/unseen_block_<date>
  dry run on the pilot flights: --banks bank_pilot2_2026-10-20 bank_pilot3_2026-10-20 ... (retention must come out about 1)"""
import argparse, glob, json, sys
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "experiments/analysis")); sys.path.insert(0, str(REPO / "flying_drone_stack/tools"))
from a8_crossings import zero_crossings  # noqa: E402
import ns2_applied_check as N  # noqa: E402

PILOT = json.load(open(REPO / "experiments/results/unified_overview_2026-10-20/summary.json")); ORDER = ["S0", "S1", "S2", "S3"]


def strategy(binpath):
    f = N.fit(str(binpath))
    if f is None: return None
    return ("S3" if f["coef_pred"] < -0.6 else "S1") if f["indi"] else ("S2" if f["coef_pred"] < -0.6 else "S0")


def flight_row(csv, meta, strat, bank):
    df = pd.read_csv(csv, comment="#"); pb = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf5_")})[0]
    b = df[[c for c in df.columns if c.startswith(pb + ".")]].rename(columns=lambda c: c.split(".", 1)[1]); tt = df.t.values
    ez = (b.z - b.ctrltarget_z).values; air = b.z.values > 0.2; p = meta["params"]; sc = meta["scenario"] if "scenario" in meta else Path(csv).name.split("_")[0]
    row = dict(folder=Path(csv).parent.name, bank=bank, scenario=sc, strategy=strat, dz=p.get("dz"), vbat_start=float(b.vbat[air].iloc[0]) if "vbat" in b else np.nan)
    if sc == "A8":
        dur = p.get("duration", 4.375); win_s = 0.75 * dur / 4.375
        cr = zero_crossings(tt, b.y.values, n=4, hyst=0.15, spacing=dur, tol=1.0)
        win = np.zeros(len(tt), bool)
        for c in cr: win |= np.abs(tt - c) < win_s
        steady = (b.z.values > 0.3) & ~win & (tt > 3); base = float(np.median(ez[steady])) if steady.any() else 0.0
        dips = [float(ez[np.abs(tt - c) < win_s].min() - base) for c in cr]
        row.update(cell=f"A8 dz{p.get('dz')} dur{dur:g}", metric=100 * float(np.mean(dips)) if dips else np.nan, n_cross=len(cr), setting=dur)
    elif sc == "A1":
        w = (tt >= 2.0) & (tt <= 15.0) & air
        row.update(cell=f"A1 dz{p.get('dz')}", metric=100 * float(ez[w].mean()), setting=p.get("dz"))
    elif sc == "A4":
        w = air & (tt >= 1.0)
        row.update(cell=f"A4 dz{p.get('dz')} off{p.get('offset')} v{p.get('speed')}", metric=100 * float(np.sqrt((ez[w] ** 2).mean())), setting=p.get("offset"))
    return row


def pilot_ref(scenario, setting, strat):
    if scenario == "A8": return abs(PILOT["A8"][strat]["mean"])
    if scenario == "A1": return abs(PILOT["A1"][strat]["mean"])
    offs = sorted(float(k) for k in PILOT["A4"]); v = [PILOT["A4"][str(o).rstrip("0") if str(o).rstrip("0") in PILOT["A4"] else str(o)][strat]["mean"] for o in offs]
    return float(np.interp(setting, offs, v))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--banks", nargs="+", required=True); ap.add_argument("--out", type=Path, required=True); a = ap.parse_args()
    out = a.out if a.out.is_absolute() else REPO / a.out; out.mkdir(parents=True, exist_ok=True); rows = []
    for bank in a.banks:
        man = json.load(open(REPO / "experiments/logs" / bank / "training_manifest.json"))
        for fl in man["flights"]:
            if fl["scenario"] not in ("A8", "A1", "A4"): continue
            binp = REPO / "experiments/logs/usd_raw" / fl["cf5"]
            try:
                meta = json.load(open(REPO / "experiments/logs" / f"{fl['scenario']}_{fl['date']}_{fl['stamp']}.meta.json")); meta["scenario"] = fl["scenario"]
                s = strategy(binp)
                if s is None: print("skip (short)", fl["folder"]); continue
                rows.append(flight_row(REPO / fl["path"], meta, s, bank))
            except Exception as e:                                                   # noqa: BLE001
                print("FAILED", fl["folder"], repr(e))
    R = pd.DataFrame(rows); R.to_csv(out / "per_flight.csv", index=False)
    C = R.groupby(["cell", "scenario", "strategy"]).agg(n=("metric", "size"), mean=("metric", "mean"), std=("metric", "std"), setting=("setting", "first")).reset_index(); C.to_csv(out / "cells.csv", index=False)
    pd.set_option("display.width", 200); print(C.round(2).to_string(index=False))
    ret = []
    for cell, g in C.groupby("cell"):
        d = {r.strategy: r for r in g.itertuples()}; sc = g.scenario.iloc[0]; st = g.setting.iloc[0]
        rec = dict(cell=cell, scenario=sc)
        for tag, base, ns in (("geometric", "S0", "S2"), ("INDI", "S1", "S3")):
            if base in d and ns in d:
                ri_u = (abs(d[base].mean) - abs(d[ns].mean)) / abs(d[base].mean); pb, pn = pilot_ref(sc, st, base), pilot_ref(sc, st, ns); ri_p = (pb - pn) / pb
                rec.update({f"RI_unseen_{tag}": ri_u, f"RI_pilot_{tag}": ri_p, f"R_{tag}": ri_u / ri_p if ri_p > 0 else np.nan})
        ret.append(rec)
    T = pd.DataFrame(ret); T.to_csv(out / "retention.csv", index=False); print(T.round(2).to_string(index=False))
    if "R_geometric" in T and T.R_geometric.notna().any():
        Rg = float(T.R_geometric.mean()); verdict = "no material overfit" if Rg >= 0.7 else "partial" if Rg >= 0.4 else "overfit confirmed"
        print(f"\npooled retention (geometric, mean over cells) {Rg:.2f} -> {verdict}")
        json.dump({"R_geometric_pooled": Rg, "verdict": verdict, "cells": T.to_dict("records")}, open(out / "verdict.json", "w"), indent=1)
    cells = sorted(C.cell.unique()); fig, axs = plt.subplots(1, len(cells), figsize=(3.6 * len(cells), 3.6), squeeze=False)
    for ax, cell in zip(axs[0], cells):
        g = R[R.cell == cell]; sc = g.scenario.iloc[0]; st = C[C.cell == cell].setting.iloc[0]
        for i, s in enumerate(ORDER):
            y = np.abs(g[g.strategy == s].metric.values)
            if len(y): ax.bar(i, y.mean(), color=["#7f7f7f", "#1f77b4", "#2ca02c", "#d62728"][i], alpha=0.8); ax.scatter(np.full(len(y), i), y, c="k", s=10, zorder=3)
            try: ax.plot([i - 0.4, i + 0.4], [pilot_ref(sc, st, s)] * 2, c="k", lw=2)
            except Exception: pass
        ax.set_xticks(range(4)); ax.set_xticklabels(ORDER); ax.set_title(cell, fontsize=8); ax.set_ylabel({"A8": "|crossing dip| [cm]", "A1": "|mean z error| [cm]", "A4": "z RMS [cm]"}[sc])
    fig.suptitle("unseen settings: bars/dots = new flights, black line = pilot value at the pilot setting (A4: interpolated in offset)", fontsize=8); fig.tight_layout(); fig.savefig(out / "unseen_block.png", dpi=130)


if __name__ == "__main__":
    main()
