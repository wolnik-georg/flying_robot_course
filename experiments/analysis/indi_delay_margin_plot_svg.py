#!/usr/bin/env python3
"""SVG plots for delay-margin study (no matplotlib)."""

from __future__ import annotations

import json
from pathlib import Path

OUT = Path(__file__).resolve().parent / "out" / "indi_delay_margin"


def svg_stability_map(margins: dict, out_path: Path) -> None:
    krs = sorted(m for m in margins if margins[m] is not None)
    if not krs:
        krs = sorted(margins.keys())
    w, h = 420, 260
    ml, mb = 50, 40
    pw, ph = w - ml - 20, h - mb - 20
    max_d = 4.0
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">',
        '<rect width="100%" height="100%" fill="#fafafa"/>',
        f'<text x="{ml}" y="16" font-size="12">Min cmd dead time (ms) for limit cycle, gyro LPF 80 Hz</text>',
    ]
    for i, kr in enumerate(krs):
        y = mb + i * (ph / max(len(krs) - 1, 1))
        d = margins.get(kr)
        val = d if d is not None else max_d
        bw = (val / max_d) * pw
        col = "#c44" if d is not None else "#aaa"
        lines.append(f'<text x="4" y="{y+4}" font-size="10">kr={kr}</text>')
        lines.append(f'<rect x="{ml}" y="{y-8}" width="{bw:.1f}" height="14" fill="{col}"/>')
        lab = f"{d:.1f}" if d is not None else ">4"
        lines.append(f'<text x="{ml+bw+4}" y="{y+4}" font-size="10">{lab}</text>')
    lines.append(f'<line x1="{ml}" y1="{h-mb}" x2="{ml+pw}" y2="{h-mb}" stroke="#333"/>')
    lines.append(f'<text x="{ml+pw/2}" y="{h-4}" text-anchor="middle" font-size="10">dead time (ms)</text>')
    lines.append("</svg>")
    out_path.write_text("\n".join(lines))


def svg_baseline_bars(cases: list[dict], out_path: Path) -> None:
    w, h = 480, 220
    ml, mb = 120, 50
    pw, ph = w - ml - 30, h - mb - 30
    max_g = max((c["metrics"]["gyro_rms_deg_s"] for c in cases if "metrics" in c), default=300)
    max_g = max(max_g, 300)
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">',
        '<rect width="100%" height="100%" fill="#fafafa"/>',
        '<text x="10" y="16" font-size="12">Baseline validation — gyro RMS (°/s)</text>',
        '<line x1="60" y1="30" x2="60" y2="70" stroke="#888"/><text x="65" y="45" font-size="9">flight band 260-290</text>',
    ]
    for i, c in enumerate(cases):
        if "metrics" not in c:
            continue
        g = c["metrics"]["gyro_rms_deg_s"]
        y = mb + i * 22
        bw = (g / max_g) * pw
        lines.append(f'<text x="4" y="{y+4}" font-size="8">{c.get("label","")[:18]}</text>')
        lines.append(f'<rect x="{ml}" y="{y-6}" width="{bw:.1f}" height="12" fill="#48a"/>')
        lines.append(f'<text x="{ml+bw+4}" y="{y+4}" font-size="9">{g:.1f}</text>')
    lines.append("</svg>")
    out_path.write_text("\n".join(lines))


def main() -> None:
    summary = json.loads((OUT / "summary.json").read_text())
    svg_stability_map(summary.get("margins_per_kr", {}), OUT / "fig_dead_kr_margin.svg")
    results = json.loads((OUT / "results.json").read_text())
    baseline = [r for r in results.get("results", []) if r.get("label", "").startswith("baseline_")]
    svg_baseline_bars(baseline, OUT / "fig_baseline_gyro.svg")
    print("wrote SVGs to", OUT)


if __name__ == "__main__":
    main()
