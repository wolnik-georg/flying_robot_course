#!/usr/bin/env python3
"""SVG heatmap: ours gyro RMS vs (delay_ms, f_rpm)."""

from __future__ import annotations

import json
from pathlib import Path

OUT = Path(__file__).resolve().parent / "out" / "indi_sil_rpm_delay"


def main() -> None:
    p = OUT / "results.json"
    if not p.is_file():
        return
    data = json.loads(p.read_text())
    rows = data.get("sweep_results", [])
    grid = {}
    for r in rows:
        if "ours" not in r.get("label", ""):
            continue
        rm = r.get("rpm_meas") or r["config"]["rpm_meas"]
        grid[(rm["delay_ms"], rm["f_rpm"])] = r["metrics"]["gyro_rms_deg_s"]
    if not grid:
        return
    ds = sorted({k[0] for k in grid})
    fs = sorted({k[1] for k in grid}, reverse=True)
    w, h = 420, 280
    cw = (w - 80) / len(fs)
    ch = (h - 60) / len(ds)
    mx = max(grid.values()) if grid else 1
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">']
    lines.append('<text x="10" y="16" font-size="11">Ours gyro RMS [deg/s] vs delay_ms × f_rpm</text>')
    for i, d in enumerate(ds):
        for j, f in enumerate(fs):
            v = grid.get((d, f), 0)
            intensity = min(1.0, v / max(mx, 260))
            color = f"rgb({int(255*intensity)},{int(80*(1-intensity))},{int(120*(1-intensity))})"
            x = 70 + j * cw
            y = 30 + i * ch
            lines.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{cw-2:.1f}" height="{ch-2:.1f}" fill="{color}"/>')
            lines.append(f'<text x="{x+2:.1f}" y="{y+10:.1f}" font-size="7">{v:.0f}</text>')
        lines.append(f'<text x="5" y="{30+i*ch+ch/2:.1f}" font-size="8">{d}ms</text>')
    for j, f in enumerate(fs):
        lines.append(f'<text x="{70+j*cw:.1f}" y="{h-8}" font-size="7">{f}Hz</text>')
    lines.append("</svg>")
    (OUT / "fig_sweep_gyro_heatmap.svg").write_text("\n".join(lines))
    print("Wrote", OUT / "fig_sweep_gyro_heatmap.svg")


if __name__ == "__main__":
    main()
