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
# ---- (4) C_D and ki_z sensitivity
d = json.load(open(O / "cd_and_kiz.json")); fig, ax = plt.subplots(1, 2, figsize=(12, 4.2)); cds = [0.0, 0.2, 0.3, 0.43, 0.6, 0.8]; kis = [3, 8, 12, 16, 24]
for cs, c, lab in (("A8", "#d62728", "A8 dip [cm]"), ("A1", "#1f77b4", "A1 mean z error [cm]"), ("A4 0.05", "#2ca02c", "A4 0.05 m z RMS [cm]")):
    ax[0].plot(cds, [abs(d[f"cd {x}|{cs}"]["value"]) for x in cds], "o-", c=c, label=lab); ax[1].plot(kis, [abs(d[f"ki_z {x}|{cs}"]["value"]) for x in kis], "o-", c=c, label=lab)
ax[0].axvline(0.43, c="k", ls=":", lw=1); ax[0].text(0.44, ax[0].get_ylim()[1] * 0.9, "fitted 0.43\n(default)", fontsize=8); ax[0].set_xlabel("downwash coefficient C_D used by FBL"); ax[0].set_ylabel("error [cm], smaller is better"); ax[0].set_title("Downwash model strength: best around 0.6; too weak (≤ 0.3) costs 2–3x")
ax[1].axvline(12, c="k", ls=":", lw=1); ax[1].text(12.3, ax[1].get_ylim()[1] * 0.9, "default 12", fontsize=8); ax[1].set_xlabel("z-integral gain ki_z"); ax[1].set_ylabel("error [cm], smaller is better"); ax[1].set_title("z-integral gain: 12–16 best, 24 oscillates in A8")
for a in ax: a.legend(fontsize=8); a.set_ylim(bottom=0); a.title.set_fontsize(9)
fig.suptitle("Calibrated plant (97.7 % thrust, downwash 1.3x prediction), tuned FBL; simulation", fontsize=10); fig.tight_layout(); fig.savefig(REPO / "experiments/fbl/cd_kiz_sensitivity.png", dpi=120); plt.close(fig)
# ---- (5) scenario matrix
d = json.load(open(O / "scenario_matrix.json")); names = []
for k in d:
    n = k.split("|")[0]
    if n not in names: names.append(n)
import re as _re
def _nk(n): return [float(x) for x in _re.findall(r"[0-9.]+", n)]
order = sorted([n for n in names if n.startswith("A8")], key=_nk) + sorted([n for n in names if n.startswith(("A2", "A3", "A5"))], key=lambda n: (n[:2], _nk(n))) + sorted([n for n in names if n.startswith("R1")], key=_nk)
fig, ax = plt.subplots(2, 1, figsize=(13, 8), sharex=True); w = 0.27
for a, key, yl in ((ax[0], "z_rms", "z RMS [cm]"), (ax[1], "z_dip", "worst downward z error inside the gate [cm]")):
    for k, (m, c, l) in enumerate((("omar", "#7f7f7f", "S0 geometric"), ("s2", "#2ca02c", "S2 geometric + NS2"), ("fbl_rom", "#2171b5", "FBL + downwash model"))):
        v = [abs(d[f"{n}|{m}"][key]) for n in order]; a.bar(np.arange(len(order)) + (k - 1) * w, v, w, color=c, label=l)
    a.set_ylabel(yl); a.grid(axis="y", alpha=.3)
ax[0].legend(fontsize=8, ncol=3); ax[1].set_xticks(range(len(order))); ax[1].set_xticklabels(order, rotation=45, ha="right", fontsize=8)
ax[0].set_title("FBL on more scenarios (calibrated plant: thrust 97.7 %, downwash 1.3x, measured peer feed; simulation)\nFBL is lowest in every case, never tripped or diverged; the weakest cases are the moving stack (A2) and the random flights (R1, A5)", fontsize=9)
fig.tight_layout(); fig.savefig(REPO / "experiments/fbl/scenario_matrix.png", dpi=115); plt.close(fig)
# ---- (6) parameter sweep
d = json.load(open(O / "param_sweep.json")); fig, ax = plt.subplots(1, 3, figsize=(16, 4.6))
for j, (scen, key, yl) in enumerate((("A8", "z_dip", "A8 crossing dip [cm]"), ("A1", "z_mean", "A1 mean z error [cm]"))):
    data = [[abs(x[key]) for x in d if x["scen"] == scen and x["mode"] == m] for m in ("s2", "fbl_rom")]
    bp = ax[j].boxplot(data, labels=["S2 geometric + NS2", "FBL + downwash model"], patch_artist=True, widths=.5)
    for b_, c in zip(bp["boxes"], ("#2ca02c", "#2171b5")): b_.set_facecolor(c); b_.set_alpha(.5)
    for i_, v in enumerate(data): ax[j].scatter(np.random.default_rng(0).normal(i_ + 1, .04, len(v)), v, s=8, c="k", alpha=.5)
    ax[j].set_ylabel(yl + " (absolute, smaller is better)"); ax[j].set_title(f"{scen}: 60 random parameter draws", fontsize=10)
f = [x for x in d if x["mode"] == "fbl_rom" and x["scen"] == "A8"]; sc = ax[2].scatter([x["lag"] for x in f], [x["delay"] for x in f], c=[x["tilt"] for x in f], cmap="viridis_r", s=45, vmin=0, vmax=60); fig.colorbar(sc, ax=ax[2], label="max tilt of FBL [deg] (latch at 60)")
ax[2].set_xlabel("motor lag [ms] (identified 44)"); ax[2].set_ylabel("command delay [ms] (assumed 4)"); ax[2].set_title("Where the margin gets thin (A8): lag and delay both high", fontsize=10)
fig.suptitle("Parameter-uncertainty sweep, same draws for both: mass ±5 %, inertia ±20 %, lag 30–60 ms, delay 2–8 ms, thrust 0.95–1.04, downwash 1.0–1.5x, C_D 0.3–0.8; simulation", fontsize=10); fig.tight_layout(); fig.savefig(REPO / "experiments/fbl/param_sweep.png", dpi=115); plt.close(fig)
# ---- (7) lower-gain fallback
fb = json.load(open(O / "fallback_gains.json")); res = fb["results"]; draws = [t for t in fb["draws"] if t != "nominal"]
vars_ = ["preset 1 (default)", "kr x0.7", "kr x0.5", "xy position x0.7", "kr x0.7 + xy pos x0.7", "kr x0.5 + xy pos x0.5"]
fig, ax = plt.subplots(1, 2, figsize=(13, 4.2))
tilt = [max(res[f"{v}|{t}|{s}"]["tilt"] for t in draws for s in ("A8", "A1")) for v in vars_]
ax[0].bar(range(len(vars_)), tilt, color=["#d62728"] + ["#2171b5"] * 5); ax[0].axhline(60, c="k", ls=":", lw=1); ax[0].text(0, 56, "tilt latch 60°", fontsize=8)
for i, v in enumerate(tilt): ax[0].text(i, v + 1, f"{v:.0f}°", ha="center", fontsize=8)
ax[0].set_xticks(range(len(vars_))); ax[0].set_xticklabels([v.replace(" + ", "\n+ ") for v in vars_], fontsize=7.5); ax[0].set_ylabel("max tilt over the worst draws [deg]"); ax[0].set_title(f"The {len(draws)} draws where the default reached > 8° (high lag and delay)", fontsize=9)
cost = [[abs(res[f"{v}|nominal|A8"]["z_dip"]), res[f"{v}|nominal|A8"]["z_rms"], res[f"{v}|nominal|A1"]["z_rms"]] for v in vars_]
w = .27
for k, (lab, c) in enumerate((("A8 dip [cm]", "#d62728"), ("A8 z RMS [cm]", "#1f77b4"), ("A1 z RMS [cm]", "#2ca02c"))): ax[1].bar(np.arange(len(vars_)) + (k - 1) * w, [x[k] for x in cost], w, color=c, label=lab)
ax[1].set_xticks(range(len(vars_))); ax[1].set_xticklabels([v.replace(" + ", "\n+ ") for v in vars_], fontsize=7.5); ax[1].legend(fontsize=8); ax[1].set_ylabel("error in the nominal plant [cm]"); ax[1].set_title("Cost of the lower gains at nominal: none visible", fontsize=9)
fig.suptitle("Lower-gain fallback (live parameters, no reflash): kr = ctrlFbl.kr_xy / kr_z, xy position = kp_xy / kv_xy / ki_xy; simulation", fontsize=10); fig.tight_layout(); fig.savefig(REPO / "experiments/fbl/fallback_gains.png", dpi=115); plt.close(fig)
# ---- (8) battery sag
bs = json.load(open(O / "battery_sag.json")); fig, ax = plt.subplots(1, 3, figsize=(16, 4.4)); fades = [0.0, 0.05, 0.10, 0.15]
for j, (scen, ttl) in enumerate((("A1", "A1 hold (15 s)"), ("A8", "A8 swap"), ("R1", "R1 random flight (30 s)"))):
    for m, c, l in (("omar", "#7f7f7f", "S0 geometric"), ("s2", "#2ca02c", "S2 geometric + NS2"), ("fbl_rom", "#2171b5", "FBL + downwash model")):
        ax[j].plot([100 * f for f in fades], [bs[f"fade|{f}|{scen}|{m}"]["z_rms"] for f in fades], "o-", c=c, label=l)
    ax[j].set_xlabel("thrust per command lost by the end of the flight [%]"); ax[j].set_ylabel("z RMS [cm]"); ax[j].set_title(ttl, fontsize=10); ax[j].grid(alpha=.3)
ax[0].legend(fontsize=8); fig.suptitle("Battery sag in the calibrated plant: FBL's z error hardly grows (its z integral, gain 12, compensates), the geometric strategies sag further; simulation", fontsize=10); fig.tight_layout(); fig.savefig(REPO / "experiments/fbl/battery_sag.png", dpi=115); plt.close(fig)
