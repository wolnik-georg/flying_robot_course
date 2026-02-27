#!/usr/bin/env python3
"""Lightweight plotting using Pillow to avoid matplotlib/NumPy binary issues.
Generates: top-down x/y, z vs t, MEKF position error vs t for assignment5 modes.
"""
import csv
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
import math

modes = ["circle", "figure8", "hover"]
out_dir = Path("results/images")
out_dir.mkdir(parents=True, exist_ok=True)

W, H = 800, 600
MARGIN = 40


def read_csv(path):
    with open(path, newline="") as f:
        r = list(csv.DictReader(f))
    # convert numeric where possible
    for row in r:
        for k in list(row.keys()):
            try:
                row[k] = float(row[k])
            except Exception:
                pass
    return r


def draw_topdown(rows_planned, rows_cl, out_path):
    # extract x,y
    px = [r["x"] for r in rows_planned]
    py = [r["y"] for r in rows_planned]
    cx = [r["sim_x"] for r in rows_cl]
    cy = [r["sim_y"] for r in rows_cl]
    xs = px + cx
    ys = py + cy
    if not xs:
        return
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    # add tiny margins
    dx = maxx - minx if maxx > minx else 1.0
    dy = maxy - miny if maxy > miny else 1.0
    minx -= dx * 0.05
    maxx += dx * 0.05
    miny -= dy * 0.05
    maxy += dy * 0.05

    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)
    # draw axes box
    ax0 = (MARGIN, MARGIN, W - MARGIN, H - MARGIN)
    d.rectangle(ax0, outline="black")

    def proj(x, y):
        sx = MARGIN + (x - minx) / (maxx - minx) * (W - 2 * MARGIN)
        sy = H - MARGIN - (y - miny) / (maxy - miny) * (H - 2 * MARGIN)
        return (sx, sy)

    # planned path
    if len(px) > 1:
        pts = [proj(x, y) for x, y in zip(px, py)]
        d.line(pts, fill="blue", width=2)
    # closed-loop
    if len(cx) > 1:
        pts = [proj(x, y) for x, y in zip(cx, cy)]
        d.line(pts, fill="red", width=2)

    d.text((MARGIN, 5), "Top-down (x,y) - planned=blue closed-loop=red", fill="black")
    img.save(out_path)


def draw_z_time(rows_planned, rows_cl, out_path):
    pt = [(r["t"], r["z"]) for r in rows_planned]
    ct = [(r["t"], r["sim_z"]) for r in rows_cl]
    xs = [t for t, _ in pt] + [t for t, _ in ct]
    zs = [z for _, z in pt] + [z for _, z in ct]
    if not xs:
        return
    minx, maxx = min(xs), max(xs)
    minz, maxz = min(zs), max(zs)
    if minz == maxz:
        minz -= 0.1
        maxz += 0.1
    img = Image.new("RGB", (W, int(H / 2)), "white")
    d = ImageDraw.Draw(img)
    d.rectangle((0, 0, W, int(H / 2)), outline="black")

    def proj(t, z):
        sx = MARGIN + (t - minx) / (maxx - minx) * (W - 2 * MARGIN)
        sy = int(H / 2) - MARGIN - (z - minz) / (maxz - minz) * (H / 2 - 2 * MARGIN)
        return (sx, sy)

    if len(pt) > 1:
        pts = [proj(t, z) for t, z in pt]
        d.line(pts, fill="blue", width=2)
    if len(ct) > 1:
        pts = [proj(t, z) for t, z in ct]
        d.line(pts, fill="red", width=2)
    d.text((MARGIN, 5), "z(t) - planned=blue closed-loop=red", fill="black")
    img.save(out_path)


def draw_mekf_err(rows_cl, out_path):
    # compute pos error if mekf present
    t = []
    err = []
    for r in rows_cl:
        if all(
            k in r and isinstance(r[k], float)
            for k in ("sim_x", "sim_y", "sim_z", "mekf_px", "mekf_py", "mekf_pz")
        ):
            dx = r["sim_x"] - r["mekf_px"]
            dy = r["sim_y"] - r["mekf_py"]
            dz = r["sim_z"] - r["mekf_pz"]
            t.append(r["t"])
            err.append(math.sqrt(dx * dx + dy * dy + dz * dz))
    if not t:
        return
    minx, maxx = min(t), max(t)
    miny, maxy = 0.0, max(err)
    img = Image.new("RGB", (W, int(H / 2)), "white")
    d = ImageDraw.Draw(img)
    d.rectangle((0, 0, W, int(H / 2)), outline="black")

    def proj(tt, ee):
        sx = MARGIN + (tt - minx) / (maxx - minx) * (W - 2 * MARGIN)
        sy = int(H / 2) - MARGIN - (ee - miny) / (maxy - miny) * (H / 2 - 2 * MARGIN)
        return (sx, sy)

    pts = [proj(tt, ee) for tt, ee in zip(t, err)]
    if len(pts) > 1:
        d.line(pts, fill="purple", width=2)
    d.text((MARGIN, 5), "MEKF pos error [m]", fill="black")
    img.save(out_path)


for mode in modes:
    planned = Path(f"results/data/assignment5_planned_{mode}.csv")
    cl = Path(f"results/data/assignment5_closedloop_{mode}.csv")
    if not planned.exists() or not cl.exists():
        print(f"Skipping {mode}, files missing")
        continue
    print("Plotting", mode)
    rows_p = read_csv(planned)
    rows_c = read_csv(cl)
    draw_topdown(rows_p, rows_c, out_dir / f"assignment5_topdown_{mode}.png")
    draw_z_time(rows_p, rows_c, out_dir / f"assignment5_z_{mode}.png")
    draw_mekf_err(rows_c, out_dir / f"mekf_pos_err_{mode}.png")

print("Done")
