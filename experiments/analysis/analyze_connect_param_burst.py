#!/usr/bin/env python3
"""Desk helper: connect-time firmware_params burst counts (cf5 vs cf_second)."""
from __future__ import annotations

import re
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
YAML = ROOT.parent / "crazyswarm2/crazyflie/config/crazyflies.yaml"
DEBUG_LOG = ROOT / "debug/lab_logs/debug.log"


def merged_firmware_params(data: dict, robot: str) -> dict[str, object]:
    out: dict[str, object] = {}
    robot_cfg = data["robots"][robot]
    rtype = data["robot_types"].get(robot_cfg.get("type"), {})
    for src in (
        data.get("all", {}).get("firmware_params") or {},
        rtype.get("firmware_params") or {},
        robot_cfg.get("firmware_params") or {},
    ):
        for group, params in src.items():
            if not isinstance(params, dict):
                continue
            for param, val in params.items():
                out[f"{group}.{param}"] = val
    return out


def parse_cf5_log(path: Path) -> tuple[int, float] | None:
    if not path.is_file():
        return None
    lines = [ln for ln in path.read_text().splitlines() if "[cf5] Update parameter" in ln]
    if not lines:
        return None
    ts = []
    for ln in lines:
        m = re.search(r"\[INFO\] \[([0-9.]+)\]", ln)
        if m:
            ts.append(float(m.group(1)))
    return len(lines), (ts[-1] - ts[0]) if len(ts) > 1 else 0.0


def main() -> int:
    data = yaml.safe_load(YAML.read_text())
    cf5 = merged_firmware_params(data, "cf5")
    cf2 = merged_firmware_params(data, "cf_second")
    only_cf5 = sorted(set(cf5) - set(cf2))
    only_cf2 = sorted(set(cf2) - set(cf5))
    print("yaml firmware_params count cf5:", len(cf5))
    print("yaml firmware_params count cf_second:", len(cf2))
    print("cf5-only keys:", only_cf5)
    print("cf_second-only keys:", only_cf2)
    meas = parse_cf5_log(DEBUG_LOG)
    if meas:
        n, span = meas
        print(f"debug.log cf5 measured updates: {n} span_s={span:.6f}")
    else:
        print("debug.log: no cf5 Update parameter lines")
    print(
        "note: ctrlOmarIndi (~22 params) lives on cf5 brushless firmware TOC but is NOT "
        "in yaml connect push; overflow is unpaced host setParam burst + boot-time syslink load."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
