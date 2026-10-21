#!/usr/bin/env python3
"""Overlay of the four A1 videos (static camera): the drones of every strategy in one frame, each strategy in its own colour, with fading trails.
Sync and crop as in a1_videos.py. Background plate = early frames (upper part, before takeoff) + hold frames (lower part); foreground = pixels that differ from the plate.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/a1_overlay.py   -> docs/meetings/assets/2026-10-22/A1_overlay.mp4
Colours as in the plots: S0 grey-white, S1 blue, S2 green, S3 red."""
import subprocess, sys
from pathlib import Path
import numpy as np
from scipy import ndimage as ndi
REPO = Path(__file__).resolve().parents[2]; OUT = REPO / "docs/meetings/assets/2026-10-22/A1_overlay.mp4"; SRC = Path.home() / "Desktop"
V = [("S0", "Flight_A1_Geometric_Controller.MOV", 4.6, (230, 230, 230)), ("S1", "Flight_A1_INDI_Controller.MOV", 4.7, (60, 140, 255)),
     ("S2", "Flight_A1_Geometric_and_NS2_Controller.MOV", 5.2, (60, 220, 90)), ("S3", "Flight_A1_INDI_and_NS2_Controller.MOV", 5.0, (255, 70, 70))]
PRE, DUR, FPS, W = 1.0, 38.0, 15, 960
def reader(f, t0, fps=FPS, ss=None, dur=None):
    p = subprocess.Popen(["ffmpeg", "-v", "error", "-ss", f"{t0 - PRE if ss is None else ss}", "-t", f"{DUR if dur is None else dur}", "-i", str(SRC / f), "-vf", f"crop=1860:1860:1060:100,fps={fps},scale={W}:{W}", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"], stdout=subprocess.PIPE)
    return p
def frames(p):
    n = W * W * 3
    while True:
        b = p.stdout.read(n)
        if len(b) < n: return
        yield np.frombuffer(b, np.uint8).reshape(W, W, 3)
# background plate: early frames for the upper part (before takeoff, no drone up there), hold frames of all four for the lower part
early, hold = [], []
for k, f, t0, c in V:
    p = reader(f, t0, fps=2, ss=0.0, dur=2.5); early += [x.copy() for x in frames(p)]
    p = reader(f, t0, fps=1, ss=t0 + 8, dur=20); hold += [x.copy() for x in frames(p)]
cut = int(0.59 * W)
plate = np.median(np.stack(hold), axis=0); plate[:cut] = np.median(np.stack(early), axis=0)[:cut]; plate = plate.astype(np.float32)
dim = (plate * 0.30).astype(np.float32)
ps = [reader(f, t0) for k, f, t0, c in V]; gens = [frames(p) for p in ps]
enc = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{W}", "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "slow", "-crf", "26", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(OUT)], stdin=subprocess.PIPE)
trail = [np.zeros((W, W), np.float32) for _ in V]; DEC = 0.93; n = 0
while True:
    try: fr = [next(g) for g in gens]
    except StopIteration: break
    img = dim.copy()
    for i, (k, f, t0, col) in enumerate(V):
        diff = fr[i].astype(np.float32) - plate; diff -= np.median(diff, axis=(0, 1), keepdims=True)      # remove the global exposure change of this frame
        d = ndi.gaussian_filter(np.abs(diff).max(axis=2), 2.0)
        m = ndi.binary_opening(d > 42, structure=np.ones((5, 5)), iterations=2)                           # thin static edges (net, ladder, window frames) vanish, drones stay
        m = ndi.binary_dilation(m, iterations=3).astype(np.float32)
        trail[i] = np.maximum(trail[i] * DEC, m)
        img = np.maximum(img, trail[i][..., None] * np.array(col, np.float32)[None, None, :])
    enc.stdin.write(np.clip(img, 0, 255).astype(np.uint8).tobytes()); n += 1
enc.stdin.close(); enc.wait(); print("frames", n)
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"; tmp = OUT.with_name("A1_overlay_raw.mp4"); OUT.rename(tmp)
dt = ",".join(f"drawtext=fontfile={FONT}:text='{lab}':x=16:y={16 + 36 * i}:fontsize=26:fontcolor=0x{c[0]:02x}{c[1]:02x}{c[2]:02x}:box=1:boxcolor=black@0.6:boxborderw=5" for i, (lab, c) in enumerate([("S0 geometric", V[0][3]), ("S1 INDI", V[1][3]), ("S2 geometric + NS2", V[2][3]), ("S3 INDI + NS2", V[3][3])]))
dt += f",drawtext=fontfile={FONT}:text='t = %{{eif\\:max(0\\,trunc(t-{PRE}))\\:d}} s':x=16:y={W - 46}:fontsize=28:fontcolor=white:box=1:boxcolor=black@0.6:boxborderw=5"
subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", str(tmp), "-vf", dt, "-c:v", "libx264", "-preset", "slow", "-crf", "26", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(OUT)], check=True); tmp.unlink()
