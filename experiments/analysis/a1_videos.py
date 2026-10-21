#!/usr/bin/env python3
"""A1 videos of the four strategies (static camera, 4K): synchronised clips and a 2x2 grid for the meeting doc.
Sync = takeoff onset (first strong motion of the two drones leaving the floor, found with frame differencing at 10 fps; the values below).
Crop = the square where the stack takes off, hovers and lands (x 1060..2920, y 100..1960 of 3840x2160). Layout: columns INDI off / on, rows NS2 off / on.
  python3 experiments/analysis/a1_videos.py [--src ~/Desktop]   -> docs/meetings/assets/2026-10-22/A1_{S0..S3}.mp4, A1_grid.mp4"""
import argparse, subprocess
from pathlib import Path
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
ins = []
for k, lab, f, t0 in V: ins += ["-ss", f"{t0 - PRE}", "-t", f"{DUR}", "-i", str(Path(a.src) / f)]
fc = ";".join(f"[{i}:v]{vf(V[i][1])}[v{i}]" for i in range(4)) + ";[v0][v1][v2][v3]xstack=inputs=4:layout=0_0|w0_0|0_h0|w0_h0[out]"
subprocess.run(["ffmpeg", "-v", "error", "-y", *ins, "-filter_complex", fc, "-map", "[out]", *enc, str(OUT / "A1_grid.mp4")], check=True); print("grid done")
