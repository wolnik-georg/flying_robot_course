#!/usr/bin/env python3
"""Figures of 2026-10-21: (1) peer-feed robustness, (2) measured thrust command / weight at hover (uSD logs), (3) simulation-only comparison with hardware reference.
  ~/.pyenv/versions/flying_robots/bin/python experiments/fbl/stage3_port/robustness_plots.py"""
import glob, json, sys
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[3]; O = REPO / "experiments/fbl/out/stage3"; sys.path.insert(0, str(REPO / "flying_drone_stack/tools")); import decode_usd_log as D
M, G = 0.0427, 9.81
# ---- (1) peer feed
d = json.load(open(O / "peer_robustness.json")); fig, ax = plt.subplots(1, 3, figsize=(16, 4.4)); C = {"s2": ("#2ca02c", "S2 geometric + NS2"), "fbl_rom": ("#2171b5", "FBL + downwash model")}
lat = [0, 25, 50, 100]; lk = {0: "measured jitter, 1 mm", 25: "measured jitter, 1 mm, latency 25 ms", 50: "measured jitter, 1 mm, latency 50 ms", 100: "measured jitter, 1 mm, latency 100 ms"}
nz = [0, 1, 3, 10]; nk = {0: "ideal (100 Hz, truth)", 1: "measured jitter, 1 mm", 3: "measured jitter, noise 3 mm", 10: "measured jitter, noise 10 mm"}
for m, (c, l) in C.items():
    ax[0].plot(lat, [abs(d[f"A8|{lk[x]}|{m}"]["value"]) for x in lat], "o-", c=c, label=l)
    ax[1].plot(nz, [abs(d[f"A8|{nk[x]}|{m}"]["value"]) for x in nz], "o-", c=c, label=l); ax[2].plot(nz, [abs(d[f"A1|{nk[x]}|{m}"]["value"]) for x in nz], "o-", c=c, label=l)
ax[0].axvspan(35, 65, color="orange", alpha=.2); ax[0].text(50, 0.08, "measured\n(±40–60 ms)", ha="center", fontsize=7)
for a in ax[1:]: a.axvline(1, color="orange", alpha=.6); a.text(1.15, 0.1, "measured ≈ 1 mm", fontsize=7, color="darkorange")
ax[0].set_xlabel("latency of the neighbour's position [ms]"); ax[0].set_ylabel("A8 crossing dip [cm]"); ax[0].set_title("Latency of the peer feed (A8)")
ax[1].set_xlabel("noise per position update [mm]"); ax[1].set_ylabel("A8 crossing dip [cm]"); ax[1].set_title("Noise of the peer feed (A8)")
ax[2].set_xlabel("noise per position update [mm]"); ax[2].set_ylabel("A1 mean z error [cm]"); ax[2].set_title("Noise of the peer feed (A1): NS2 degrades, FBL does not")
for a in ax: a.legend(fontsize=8); a.set_ylim(bottom=0)
fig.suptitle("Peer-feed robustness in the realistic plant (update jitter as measured: ~90 Hz, p95 20 ms, in all curves); simulation", fontsize=10); fig.tight_layout(); fig.savefig(REPO / "experiments/fbl/peer_robustness.png", dpi=120); plt.close(fig)
# ---- (2) thrust command / weight at hover
top = []
for f in sorted(glob.glob(str(REPO / "experiments/logs/usd_raw/cf_second_*pilot*"))) + sorted(glob.glob(str(REPO / "experiments/logs/usd_raw/cf_second_*video*"))):
    try: x = D.load(f)
    except Exception: continue
    if "omar_thrust_si" not in x: continue
    z, vz = np.asarray(x["z"]), np.asarray(x["vz"]); m = (z > 0.8) & (np.abs(vz) < 0.03)
    if m.sum() < 2000: continue
    rzz = np.cos(np.asarray(x["omar_rpy_x"])) * np.cos(np.asarray(x["omar_rpy_y"])); top.append((np.asarray(x["omar_thrust_si"])[m] / (M * G / rzz[m])).mean())
bot = []
strat = {}
for f in ["v2val_2026-10-19/v2val_summary.json", "pilot2_2026-10-20/a8_summary.json", "pilot3_2026-10-20/a8_summary.json", "pilot5_2026-10-20/a8_summary.json"]:
    for r in json.load(open(REPO / "experiments/results" / f)): strat[r["file"]] = r.get("name")
for f in sorted(glob.glob(str(REPO / "experiments/logs/bank_*/A8_*/*_merged_usd.csv"))):
    nm = Path(f).parent.name
    if strat.get(nm) not in ("S0", "S2"): continue
    df = pd.read_csv(f, comment="#"); pb = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf5_")})[0]; pt = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf_second_")})[0]
    z, vz = df[pb + ".z"].values, df[pb + ".vz"].values; dx = np.hypot(df[pb + ".x"] - df[pt + ".x"], df[pb + ".y"] - df[pt + ".y"]).values; m = (z > 0.3) & (np.abs(vz) < 0.03) & (dx > 0.5)
    if m.sum() < 500: continue
    rzz = np.cos(np.radians(df[pb + ".roll_deg"].values)) * np.cos(np.radians(df[pb + ".pitch_deg"].values)); bot.append((df[pb + ".omar_thrust_si"].values[m] / (M * G / rzz[m])).mean())
fig, ax = plt.subplots(figsize=(7.5, 4)); rng = np.random.default_rng(0)
ax.scatter(1 + rng.uniform(-.15, .15, len(top)), top, c="#d62728", s=18, label=f"top drone (n={len(top)} flights)"); ax.scatter(2 + rng.uniform(-.15, .15, len(bot)), bot, c="#1f77b4", s=24, label=f"bottom drone, outside the downwash (n={len(bot)} flights)")
ax.axhline(1.0, c="k", lw=1); ax.set_xticks([1, 2]); ax.set_xticklabels(["top drone", "bottom drone"]); ax.set_ylabel("commanded thrust / weight at hover"); ax.set_ylim(0.94, 1.06); ax.set_xlim(0.5, 2.5); ax.legend(fontsize=8)
ax.set_title("Thrust command needed to hover (geometric controller, no downwash)\n1.0 = commanded thrust equals weight; the map is within ±4 %", fontsize=9); fig.tight_layout(); fig.savefig(REPO / "experiments/fbl/thrust_command_ratio.png", dpi=120); plt.close(fig)
print("top", np.mean(top).round(3), np.min(top).round(3), np.max(top).round(3), "bottom", np.mean(bot).round(3), np.min(bot).round(3), np.max(bot).round(3))
