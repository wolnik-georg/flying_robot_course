#!/usr/bin/env python3
"""Stage B of the NN-vs-INDI check (supervisor feedback 2026-10-22). Reads the cache of nn_vs_indi_raw.py and the merged banks (gate only).
Flights where the NN was NOT applied (S0, S1), bottom drone, airborne. Compares per axis:
  pred    = logged NN output (ns2 v2),
  L_zd    = zero-delay label: raw (a_imu - a_rpm) smoothed offline with filtfilt (Butterworth 2nd order, fc in FCS),
  L_caus  = the controller's own causal INDI estimate (30 Hz filtered a_imu_f - a_rpm_f) as it would be logged onboard.
Metrics: correlation, best lag (ms, positive = label lags the NN i.e. the NN is faster), slope of label on NN, RMS ratio; all samples and inside the NS2 gate.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/nn_vs_indi_check.py"""
import glob, json
from pathlib import Path
import numpy as np, pandas as pd
from scipy.signal import butter, filtfilt
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
import argparse
ap = argparse.ArgumentParser(); ap.add_argument("--banks", nargs="*", default=None, help="bank folder names under experiments/logs (default: the pilot banks of 2026-10-19/20)"); ap.add_argument("--out", type=Path, default=None, help="output dir (default: the 2026-10-22 results folder)"); ARGS = ap.parse_args()
REPO = Path(__file__).resolve().parents[2]; CACHE = REPO / "experiments/results/nn_vs_indi_2026-10-22/cache"; RES = ARGS.out or REPO / "experiments/results/nn_vs_indi_2026-10-22"; RES.mkdir(parents=True, exist_ok=True)
FC = 5.0; FCS = (3.0, 5.0, 10.0); FS = 500.0; MAXLAG = 0.4
man = {}
for b in (sorted(str(REPO / "experiments/logs" / x) for x in ARGS.banks) if ARGS.banks else sorted(glob.glob(str(REPO / "experiments/logs/bank_*2026-10-19")) + glob.glob(str(REPO / "experiments/logs/bank_*2026-10-20")))):
    for fl in json.load(open(Path(b) / "training_manifest.json"))["flights"]:
        man[fl["folder"]] = REPO / fl["path"]


def lp(x, fc):
    b, a = butter(2, fc / (FS / 2)); return filtfilt(b, a, x, axis=0)


def best_lag(p, y, fs=FS, maxlag=MAXLAG):
    """lag (s) maximising corr(p(t), y(t+lag)); positive: y follows p."""
    n = int(maxlag * fs); best = (-2, 0)
    p = p - p.mean(); y = y - y.mean(); den = np.sqrt((p ** 2).sum() * (y ** 2).sum()) + 1e-12
    for k in range(-n, n + 1, 2):
        c = (p[max(0, -k):len(p) - max(0, k)] * y[max(0, k):len(y) - max(0, -k)]).sum() / den
        if c > best[0]: best = (c, k / fs)
    return best[1], best[0]


def gate_mask(name, tb, zb):
    """boolean gate on the bin time base via the merged CSV (offset by cross-correlation of the bottom z)."""
    df = pd.read_csv(man[name], comment="#"); pb = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf5_")})[0]; pt = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf_second_")})[0]
    g = lambda p, k: df[f"{p}.{k}"].values
    gate = (np.abs(g(pt, "x") - g(pb, "x")) < 0.2) & (np.abs(g(pt, "y") - g(pb, "y")) < 0.2) & (g(pt, "z") > g(pb, "z")) & (np.abs(g(pt, "vx") - g(pb, "vx")) < 1.5)
    tm = df["t"].values; zm = g(pb, "z")
    grid = np.arange(0, tm[-1], 0.01); zmi = np.interp(grid, tm, zm)
    tg = np.arange(tb[0], tb[-1], 0.01); zbi = np.interp(tg, tb, zb)
    a = zmi - zmi.mean(); b = zbi - zbi.mean(); n = len(a)
    best = (-2, 0)
    for s in range(0, len(b) - n // 2, 1):                      # offset: merged time 0 = bin time tg[s]
        m = min(n, len(b) - s)
        if m < n // 2: break
        c = (a[:m] * b[s:s + m]).sum() / (np.sqrt((a[:m] ** 2).sum() * (b[s:s + m] ** 2).sum()) + 1e-12)
        if c > best[0]: best = (c, s)
    t0 = tg[best[1]]
    return np.interp(tb - t0, tm, gate.astype(float)) > 0.5, best[0]


rows, ex = [], {}
for f in sorted(CACHE.glob("*.npz")):
    z = np.load(f, allow_pickle=True); name = str(z["folder"])
    if name not in man: continue                                  # cache may hold flights of other banks
    if float(z["coef_pred"]) < -0.3:                             # NS2 applied or unclear -> the label is the residual left AFTER the NN, skip
        continue
    indi = bool(z["indi"]); strat = "S1" if indi else "S0"; t = z["t"]
    air = z["z"] > 0.2
    if air.sum() < 1500: continue
    i0 = np.argmax(air); w = air & (t > t[i0] + 1.0) & (t < t[np.where(air)[0][-1]] - 0.5)
    raw = z["a_imu_raw"] - z["a_rpm_raw"]; caus = z["a_imu_f"] - z["a_rpm_f"]; pred = z["pred"]
    gate, gc = gate_mask(name, t, z["z"])
    for ax, an in enumerate("xyz"):
        for fc in FCS:
            L = lp(raw[:, ax], fc); P = lp(pred[:, ax], fc)       # NN output smoothed with the same filter (it is a stepped 100 Hz signal)
            for sel_name, sel in (("all", w), ("gate", w & gate)):
                if sel.sum() < 300: continue
                lag, c = best_lag(P[sel], L[sel]); s = np.polyfit(P[sel], L[sel], 1)[0] if P[sel].std() > 1e-6 else np.nan
                rows.append(dict(folder=name, scenario=str(z["scenario"]), strat=strat, axis=an, fc=fc, sel=sel_name, corr0=np.corrcoef(P[sel], L[sel])[0, 1],
                                 lag_ms=1000 * lag, corr_lag=c, slope=s, rms_nn=np.sqrt((P[sel] ** 2).mean()), rms_label=np.sqrt((L[sel] ** 2).mean()), mean_nn=P[sel].mean(), mean_label=L[sel].mean(), rmse=np.sqrt(((P[sel] - L[sel]) ** 2).mean()), n=int(sel.sum()), gate_align=gc))
        if an == "z":
            L = lp(raw[:, ax], FC); sel = w & gate if (w & gate).sum() > 300 else w
            lag_c, _ = best_lag(caus[sel, ax], L[sel])            # causal INDI estimate vs zero-delay label
            rows.append(dict(folder=name, scenario=str(z["scenario"]), strat=strat, axis="z", fc=FC, sel="causal_vs_zd", lag_ms=1000 * lag_c, corr_lag=np.corrcoef(caus[sel, ax], L[sel])[0, 1], slope=np.nan, n=int(sel.sum())))
    if name not in ex and strat in ("S0", "S1"): ex[name] = dict(t=t[w] - t[w][0], P=lp(pred[:, 2], FC)[w], L=lp(raw[:, 2], FC)[w], C=caus[w, 2], G=gate[w], scen=str(z["scenario"]), strat=strat)
    print(name, strat, f"gate_align_corr {gc:.3f}", flush=True)
R = pd.DataFrame(rows); R.to_csv(RES / "per_flight.csv", index=False)
pd.set_option("display.width", 200)
S = R[(R.axis == "z") & (R.fc == FC) & (R.sel != "causal_vs_zd")].groupby(["scenario", "strat", "sel"]).agg(n_fl=("folder", "nunique"), corr=("corr_lag", "median"), lag_ms=("lag_ms", "median"), slope=("slope", "median"),
                                                                                                   rms_nn=("rms_nn", "median"), rms_label=("rms_label", "median"), mean_nn=("mean_nn", "median"), mean_label=("mean_label", "median"), rmse=("rmse", "median")).round(2)
print(S.to_string()); S.to_csv(RES / "summary_z_5Hz.csv")
C = R[R.sel == "causal_vs_zd"].groupby(["scenario", "strat"]).lag_ms.median().round(1); print("causal INDI estimate vs zero-delay label, lag ms:\n", C.to_string())
FCT = R[(R.axis == "z") & (R.sel == "all")].groupby(["fc"]).agg(corr=("corr_lag", "median"), lag_ms=("lag_ms", "median"), slope=("slope", "median")).round(2); print(FCT.to_string())
AX = R[(R.fc == FC) & (R.sel == "gate")].groupby(["axis"]).agg(corr=("corr_lag", "median"), lag_ms=("lag_ms", "median"), slope=("slope", "median"), rms_nn=("rms_nn", "median"), rms_label=("rms_label", "median")).round(2); print(AX.to_string())
json.dump({"fc_hz": FC, "n_flights": int(R.folder.nunique()), "by_cell": S.reset_index().to_dict("records"), "causal_lag_ms": C.reset_index().to_dict("records")}, open(RES / "summary.json", "w"), indent=1)

# figure 1: example time series, figure 2: lag + slope per flight
fig, axs = plt.subplots(2, 2, figsize=(13, 7))
pick = {}
for k, v in ex.items(): pick.setdefault((v["scen"], v["strat"]), k)
for a_, key in zip(axs.ravel(), [("A8", "S0"), ("A8", "S1"), ("A1", "S0"), ("A4", "S1")]):
    if key not in pick: a_.set_visible(False); continue
    v = ex[pick[key]]; a_.plot(v["t"], v["L"], c="k", lw=1.6, label=f"label, zero-delay ({FC:g} Hz)"); a_.plot(v["t"], v["C"], c="tab:orange", lw=1, label="onboard INDI estimate (causal 30 Hz)"); a_.plot(v["t"], v["P"], c="tab:blue", lw=1.6, label="NN output (v2)")
    a_.fill_between(v["t"], a_.get_ylim()[0], a_.get_ylim()[1], where=v["G"], color="0.9", zorder=0, label="in gate"); a_.set_title(f"{key[0]} {key[1]}  z axis  ({pick[key]})", fontsize=9); a_.set_xlabel("flight time [s]"); a_.set_ylabel("residual accel z [m/s²]")
axs[0, 0].legend(fontsize=7, loc="lower left"); fig.tight_layout(); fig.savefig(RES / "examples.png", dpi=130); plt.close(fig)
Zg = R[(R.axis == "z") & (R.fc == FC) & (R.sel == "gate")]
fig, axs = plt.subplots(1, 3, figsize=(13, 3.8))
for a_, col, ttl, lim in zip(axs, ("lag_ms", "slope", "corr_lag"), ("lag of label behind NN [ms] (>0: NN faster)", "slope label / NN (1 = same magnitude)", "correlation at best lag"), ((-400, 400), (0, 3), (0, 1))):
    for strat, c in (("S0", "tab:gray"), ("S1", "tab:blue")):
        for j, sc in enumerate(("A1", "A4", "A8")):
            y = Zg[(Zg.strat == strat) & (Zg.scenario == sc)][col].values
            a_.scatter(np.full(len(y), j + (0.12 if strat == "S1" else -0.12)) + np.random.uniform(-0.05, 0.05, len(y)), y, c=c, s=18, label=strat if j == 0 else None)
    a_.set_xticks(range(3)); a_.set_xticklabels(("A1", "A4", "A8")); a_.set_title(ttl, fontsize=9); a_.set_ylim(*lim); a_.axhline(0 if col == "lag_ms" else 1, c="k", lw=0.5)
axs[0].legend(fontsize=8); fig.tight_layout(); fig.savefig(RES / "lag_slope.png", dpi=130)
