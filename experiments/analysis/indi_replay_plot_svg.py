#!/usr/bin/env python3
"""SVG plots for INDI replay (no matplotlib)."""

from __future__ import annotations

import json
from pathlib import Path

ANALYSIS = Path(__file__).resolve().parent
OUT = ANALYSIS / "out" / "indi_replay"


def svg_rms_bars(summary: dict, out_path: Path) -> None:
    rows = []
    for flight, block in summary.items():
        for level, lv in block.get("levels", {}).items():
            for key, m in lv.items():
                if not key.endswith("_thrust_si") or "omar_c_vs_rust" in key:
                    continue
                if "ours_vs_omar" in key and m.get("n"):
                    rows.append((f"{flight} {level} thrust", m["rms"], m["max_abs"]))
    w, h = 520, 40 + 22 * max(len(rows), 1)
    ml = 180
    pw = w - ml - 60
    max_r = max((r[1] for r in rows), default=0.01) * 1.1
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">',
        '<rect width="100%" height="100%" fill="#fafafa"/>',
        '<text x="10" y="18" font-size="12">INDI replay — thrust RMS diff (ours vs Omar C)</text>',
    ]
    for i, (lab, rms, mx) in enumerate(rows):
        y = 36 + i * 22
        bw = (rms / max_r) * pw
        lines.append(f'<text x="4" y="{y+4}" font-size="9">{lab}</text>')
        lines.append(f'<rect x="{ml}" y="{y-6}" width="{bw:.1f}" height="12" fill="#48a"/>')
        lines.append(f'<text x="{ml+bw+4}" y="{y+4}" font-size="9">{rms:.4f} (max {mx:.4f})</text>')
    lines.append("</svg>")
    out_path.write_text("\n".join(lines))


def main() -> None:
    p = OUT / "compare_summary.json"
    if not p.is_file():
        return
    summary = json.loads(p.read_text())
    svg_rms_bars(summary, OUT / "fig_replay_thrust_rms.svg")
    print("wrote", OUT / "fig_replay_thrust_rms.svg")


if __name__ == "__main__":
    main()
