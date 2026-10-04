#!/usr/bin/env python3
"""Minimal SVG bar chart (no matplotlib)."""

from __future__ import annotations

import json
from pathlib import Path

OUT = Path(__file__).resolve().parent / "out" / "indi_sil_package"


def main() -> None:
    data = json.loads((OUT / "results.json").read_text())["results"]
    rows = [r for r in data if "metrics" in r]
    labels = [r["label"] for r in rows]
    gyro = [r["metrics"]["gyro_rms_deg_s"] for r in rows]
    w = 900
    h = 320
    m = 40
    bar_w = max(2, (w - 2 * m) / max(len(rows), 1) - 1)
    gmax = max(max(gyro), 300.0)
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">',
        f'<rect width="100%" height="100%" fill="#fafafa"/>',
        f'<text x="{m}" y="20" font-size="12">Gyro RMS [deg/s] — red band 260–290 (flight ours)</text>',
    ]
    for i, (lab, g) in enumerate(zip(labels, gyro)):
        x = m + i * (bar_w + 1)
        bh = (g / gmax) * (h - 2 * m)
        y = h - m - bh
        lines.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w:.1f}" height="{bh:.1f}" fill="#4682b4"/>')
        if i % 4 == 0:
            lines.append(f'<text x="{x:.1f}" y="{h-8}" font-size="5" transform="rotate(65 {x:.1f},{h-8})">{lab[:22]}</text>')
    y260 = h - m - (260 / gmax) * (h - 2 * m)
    y290 = h - m - (290 / gmax) * (h - 2 * m)
    lines.append(f'<rect x="{m}" y="{y290:.1f}" width="{w-2*m}" height="{y260-y290:.1f}" fill="red" opacity="0.12"/>')
    lines.append("</svg>")
    (OUT / "fig_grid_gyro.svg").write_text("\n".join(lines))
    print("Wrote", OUT / "fig_grid_gyro.svg")


if __name__ == "__main__":
    main()
