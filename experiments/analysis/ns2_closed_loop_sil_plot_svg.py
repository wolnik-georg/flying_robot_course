#!/usr/bin/env python3
"""Simple SVG bar charts for NS2 closed-loop SIL."""

from __future__ import annotations

import json
from pathlib import Path

OUT = Path(__file__).resolve().parent / "out" / "ns2_closed_loop_sil"


def bar_chart(cases: list[tuple[str, float]], title: str, path: Path, ymax: float | None = None) -> None:
    w, h = 520, 40 + 22 * max(len(cases), 1)
    ml = 180
    pw = w - ml - 40
    mx = ymax or max((v for _, v in cases), default=1.0) * 1.1
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">',
        f'<rect width="100%" height="100%" fill="#fafafa"/>',
        f'<text x="8" y="16" font-size="12">{title}</text>',
    ]
    for i, (lab, val) in enumerate(cases):
        y = 28 + i * 22
        bw = 0 if not (val == val) else (val / mx) * pw
        lines.append(f'<text x="4" y="{y+4}" font-size="8">{lab[:24]}</text>')
        lines.append(f'<rect x="{ml}" y="{y-6}" width="{bw:.1f}" height="12" fill="#369"/>')
        lines.append(f'<text x="{ml+bw+4}" y="{y+4}" font-size="9">{val:.2f}</text>')
    lines.append("</svg>")
    path.write_text("\n".join(lines))


def main() -> None:
    data = json.loads((OUT / "results.json").read_text())
    rs = [r for r in data["results"] if "error" not in r and r.get("phase") == "matrix" and r.get("scenario") == "A8"]
    en0 = [r for r in rs if r.get("config", {}).get("rnn_en") == 0 and r.get("config", {}).get("div") == 10 and r.get("config", {}).get("peer_hz") == 100]
    en1 = [r for r in rs if r.get("config", {}).get("rnn_en") == 1 and r.get("config", {}).get("div") == 10 and r.get("config", {}).get("peer_hz") == 100]
    if en0 and en1:
        bar_chart(
            [("en0 rms_z", en0[0]["tracking_bottom_cm"]["rms_z_cm"]), ("en1 rms_z", en1[0]["tracking_bottom_cm"]["rms_z_cm"])],
            "A8 bottom z RMS (cm) div10 peer100",
            OUT / "fig_a8_en0_vs_en1_z.svg",
        )
    base = [r for r in data["results"] if r.get("label", "").startswith("A1_en0")]
    if base:
        bar_chart(
            [(r["label"][-6:], r["bottom_gyro_rms_deg_s"]) for r in base[:4]],
            "A1 bottom gyro RMS (deg/s)",
            OUT / "fig_a1_gyro.svg",
            ymax=50,
        )
    iso = [r for r in data["results"] if "isolation" in r.get("label", "")]
    if iso:
        bar_chart(
            [(r["label"], r.get("top_gyro_rms_deg_s", float("nan"))) for r in iso],
            "Isolation test — top gyro RMS",
            OUT / "fig_isolation_top_gyro.svg",
        )
    print("wrote SVGs", OUT)


if __name__ == "__main__":
    main()
