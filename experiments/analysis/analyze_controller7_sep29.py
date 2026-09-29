#!/usr/bin/env python3
"""Time-boxed desk pass: controller=7 failure logs from 2026-09-29 (if any)."""
from __future__ import annotations

import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
LOGS = ROOT / "experiments/logs"


def scan_radio_csv(path: Path) -> dict:
    meta = {}
    rows = 0
    gyro_max = 0.0
    pitch_max = 0.0
    t_span = 0.0
    t0 = None
    t_last = None
    with path.open() as f:
        hdr = None
        for ln in f:
            if ln.startswith("# meta:"):
                m = re.match(r"# meta:(\w+)=(.+)", ln.strip())
                if m:
                    meta[m.group(1)] = m.group(2)
            elif ln.startswith("time_s,"):
                hdr = ln.strip().split(",")
            elif hdr and ln.strip() and not ln.startswith("#"):
                rows += 1
                d = dict(zip(hdr, ln.strip().split(",")))
                t = float(d["time_s"])
                t0 = t if t0 is None else t0
                t_last = t
                for g in ("gyro_x", "gyro_y", "gyro_z"):
                    gyro_max = max(gyro_max, abs(float(d[g])))
                pitch_max = max(pitch_max, abs(float(d["pitch"])))
    if t0 is not None and t_last is not None:
        t_span = t_last - t0
    return {
        "path": str(path),
        "meta_controller": meta.get("controller"),
        "rows": rows,
        "dur_s": t_span,
        "gyro_max_dps": gyro_max,
        "pitch_max_deg": pitch_max,
    }


def main() -> int:
    hits = []
    for p in sorted(LOGS.glob("A1_cf5_2026-09-29*.csv")):
        hits.append(scan_radio_csv(p))
    print("=== cf5 radio logs 2026-09-29 ===")
    for h in hits:
        print(h)
    c7 = [h for h in hits if h.get("meta_controller") == "7"]
    print("\nmeta controller=7 count:", len(c7))
    yaml_path = ROOT.parent / "crazyswarm2/crazyflie/config/crazyflies.yaml"
    if yaml_path.is_file():
        data = yaml.safe_load(yaml_path.read_text())
        cf5 = data["robots"].get("cf5", {}).get("firmware_params", {})
        print("\ncf5 yaml stabilizer.controller:", cf5.get("stabilizer", {}).get("controller"))
    print(
        "\nNote: naindi.rs (c=7) hardcodes Briesewitz KPOS 12/10.5/2, uses g_indi_mass (~0.041), "
        "per-motor kt — not Omar's algorithm or gains."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
