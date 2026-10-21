#!/usr/bin/env python3
"""A1 videos of the four strategies (static camera, 4K): synchronised clips and a 2x2 grid for the meeting doc.
Sync = takeoff onset (first strong motion of the two drones leaving the floor, found with frame differencing at 10 fps; the values below).
Crop = the square where the stack takes off, hovers and lands (x 1060..2920, y 100..1960 of 3840x2160). Layout: columns INDI off / on, rows NS2 off / on.
  python3 experiments/analysis/a1_videos.py [--src ~/Desktop]   -> docs/meetings/assets/2026-10-22/A1_{S0..S3}.mp4, A1_grid.mp4"""
import argparse, json, subprocess
from pathlib import Path
import numpy as np, pandas as pd
REPO = Path(__file__).resolve().parents[2]; OUT = REPO / "docs/meetings/assets/2026-10-22"; OUT.mkdir(parents=True, exist_ok=True)
ap = argparse.ArgumentParser(); ap.add_argument("--src", default=str(Path.home() / "Desktop")); a = ap.parse_args()
V = [("S0", "S0 geometric", "Flight_A1_Geometric_Controller.MOV", 4.6), ("S1", "S1 INDI", "Flight_A1_INDI_Controller.MOV", 4.7),
     ("S2", "S2 geometric + NS2", "Flight_A1_Geometric_and_NS2_Controller.MOV", 5.2), ("S3", "S3 INDI + NS2", "Flight_A1_INDI_and_NS2_Controller.MOV", 5.0)]
PRE, DUR = 1.0, 38.0                     # start 1 s before the takeoff onset, 38 s long (landing ends about 37 s after the onset in all four)
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
def vf(label, w=720, h=720):
    return (f"crop=1860:1860:1060:100,fps=30,scale={w}:{h},drawtext=fontfile={FONT}:text='{label}':x=14:y=14:fontsize=34:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=8,"
            f"drawtext=fontfile={FONT}:text='t = %{{eif\\:max(0\\,trunc(t-{PRE}))\\:d}} s':x=14:y={h - 50}:fontsize=30:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=6")
enc = ["-c:v", "libx264", "-preset", "slow", "-crf", "27", "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-an"]
for k, lab, f, t0 in V:
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", f"{t0 - PRE}", "-t", f"{DUR}", "-i", str(Path(a.src) / f), "-vf", vf(lab), *enc, str(OUT / f"A1_{k}.mp4")], check=True)
    print("clip", k, flush=True)
# --- z-error curve under each tile (uSD logs of the video flights, hold only; aligned by the landing onset seen in the video: hold start = landing onset - 20 s)
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
RES = REPO / "experiments/results/video_2026-10-20/a1_summary.json"; BANK = REPO / "experiments/logs/bank_video_2026-10-20"
LAND = {"S0": 39.7, "S1": 39.7, "S2": 40.7, "S3": 39.6}          # landing onset in the video file [s] (motion burst of the drones leaving the hold height)
FLIGHTS = {"S0": ["19-15-41", "19-19-50"], "S1": ["19-23-38"], "S2": ["19-27-06"], "S3": ["19-29-14"]}   # S0: two video flights, the filmed one is not identified
COL = {"S0": "#7f7f7f", "S1": "#1f77b4", "S2": "#2ca02c", "S3": "#d62728"}
def zerr(tag):
    d = next((BANK / x for x in [p.name for p in BANK.glob("A1_*")] if x.endswith(tag)))
    df = pd.read_csv(next(d.glob("*_merged_usd.csv")), comment="#"); pb = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf5_")})[0]
    return df["t"].values, 100 * (df[pb + ".z"].values - df[pb + ".ctrltarget_z"].values)
STRIP_W, STRIP_H = 720, 220; X0, XW = 0.11, 0.86            # axes position in the strip (fraction of the width), used for the moving cursor
for k, lab, f, t0 in V:
    fig = plt.figure(figsize=(STRIP_W / 100, STRIP_H / 100), dpi=100); ax = fig.add_axes([X0, 0.24, XW, 0.70])
    hs = LAND[k] - 20 - t0                                   # hold start, seconds after takeoff onset
    for j, tag in enumerate(FLIGHTS[k]):
        t, e = zerr(tag); ax.plot(hs + t, e, color=COL[k], lw=1.2, alpha=0.9 if len(FLIGHTS[k]) == 1 else 0.6)
    ax.axhline(0, color="k", lw=0.8); ax.set_xlim(-PRE, DUR - PRE); ax.set_ylim(-35, 15); ax.grid(alpha=0.3)
    ax.set_xlabel("time since takeoff [s]  (curve = hold only)" + ("; both S0 flights" if k == "S0" else ""), fontsize=8, labelpad=1); ax.set_ylabel("z error [cm]", fontsize=8, labelpad=1); ax.tick_params(labelsize=8)
    fig.savefig(OUT / f"_strip_{k}.png"); plt.close(fig)
ins = []
for k, lab, f, t0 in V: ins += ["-ss", f"{t0 - PRE}", "-t", f"{DUR}", "-i", str(Path(a.src) / f)]
for k, *_ in V: ins += ["-loop", "1", "-framerate", "30", "-t", f"{DUR}", "-i", str(OUT / f"_strip_{k}.png")]
cur = f"overlay=x='{STRIP_W}*({X0}+{XW}*t/{DUR})-1':y=0:eval=frame"          # moving cursor (drawbox has no time variable)
fc = f"color=c=black:s=3x{STRIP_H}:r=30:d={DUR},format=yuv420p,split=4[b0][b1][b2][b3];" + ";".join(f"[{i}:v]{vf(V[i][1])}[v{i}];[{4 + i}:v]format=yuv420p[p{i}];[p{i}][b{i}]{cur},format=yuv420p[s{i}];[v{i}][s{i}]vstack[t{i}]" for i in range(4)) + ";[t0][t1][t2][t3]xstack=inputs=4:layout=0_0|w0_0|0_h0|w0_h0[out]"
subprocess.run(["ffmpeg", "-v", "error", "-y", *ins, "-filter_complex", fc, "-map", "[out]", "-t", f"{DUR}", *enc, str(OUT / "A1_grid.mp4")], check=True); print("grid done")
for k, *_ in V: (OUT / f"_strip_{k}.png").unlink()
