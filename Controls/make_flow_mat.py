#!/usr/bin/env python3
"""Generate a PMW3901-optimised ground pattern and save as flow_mat.pdf.
Print at 100% scale (no scaling), place under the drone.
Requires: pip install reportlab
"""
import math

try:
    from reportlab.lib.pagesizes import A4
    from reportlab.pdfgen import canvas
except ImportError:
    print("Install reportlab first:  pip install reportlab")
    raise

W, H = A4  # 595 x 842 pt  (1 pt = 1/72 inch)

c = canvas.Canvas("Controls/flow_mat.pdf", pagesize=A4)
c.setFillColorRGB(1, 1, 1)
c.rect(0, 0, W, H, fill=1, stroke=0)

# --- Checkerboard: 15mm squares (good spatial frequency for PMW3901) ---
sq = 15 * 72 / 25.4  # 15 mm in points ≈ 42.5 pt
c.setFillColorRGB(0, 0, 0)
cols = int(W / sq) + 1
rows = int(H / sq) + 1
for row in range(rows):
    for col in range(cols):
        if (row + col) % 2 == 0:
            c.rect(col * sq, row * sq, sq, sq, fill=1, stroke=0)

# --- Bold border so you can see the page edge ---
c.setStrokeColorRGB(0, 0, 0)
c.setLineWidth(4)
c.rect(4, 4, W - 8, H - 8, fill=0, stroke=1)

# --- Label ---
c.setFillColorRGB(0.5, 0.5, 0.5)
c.setFont("Helvetica", 10)
c.drawCentredString(
    W / 2, 20, "PMW3901 flow mat — print at 100%, place under Crazyflie"
)

c.save()
print("Saved: Controls/flow_mat.pdf")
print("Print at 100% scale (A4), tape to the floor, place Crazyflie on top.")
