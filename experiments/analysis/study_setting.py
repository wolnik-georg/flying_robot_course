"""Map flight meta sidecars to comparative-study grid cells (speed / dz)."""
from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any

SPEED_GRID = (0.2, 0.3, 0.4, 0.5)
DZ_GRID = (0.25, 0.5, 0.75)
DEV_FRAC = 0.10


def _fmeta(meta: dict, key: str, default: float | None = None) -> float | None:
    if key in meta and meta[key] is not None:
        return float(meta[key])
    p = meta.get("params") or {}
    pk = key.replace("param_", "")
    if pk in p and p[pk] is not None:
        return float(p[pk])
    r = meta.get("realised") or {}
    rk = key.replace("realised_", "")
    if rk in r and r[rk] is not None:
        return float(r[rk])
    return default


def nearest_grid(value: float, grid: tuple[float, ...]) -> tuple[float, float]:
    best = min(grid, key=lambda g: abs(g - value))
    return best, abs(value - best) / best if best else float("nan")


def extract_setting(meta: dict) -> dict[str, Any]:
    """Return speed_mps, dz_m, nominal grid values, realised values, deviation flags."""
    scenario = (meta.get("scenario") or meta.get("param_scenario") or "").upper()
    out: dict[str, Any] = {"scenario": scenario}
    dz = _fmeta(meta, "param_dz")
    if dz is None:
        dz = _fmeta(meta, "dz")
    speed = _fmeta(meta, "realised_peak_speed")
    if speed is None:
        speed = _fmeta(meta, "param_speed")
    if speed is None:
        speed = _fmeta(meta, "realised_speed_requested")
    out["dz_m_realised"] = dz
    out["speed_mps_realised"] = speed
    if dz is not None:
        g, dev = nearest_grid(dz, DZ_GRID)
        out["dz_m_nominal"] = g
        out["dz_deviation_frac"] = dev
        out["dz_off_grid"] = dev > DEV_FRAC
    else:
        out["dz_m_nominal"] = None
        out["dz_deviation_frac"] = float("nan")
        out["dz_off_grid"] = False
    if speed is not None:
        g, dev = nearest_grid(speed, SPEED_GRID)
        out["speed_mps_nominal"] = g
        out["speed_deviation_frac"] = dev
        out["speed_off_grid"] = dev > DEV_FRAC
    else:
        out["speed_mps_nominal"] = None
        out["speed_deviation_frac"] = float("nan")
        out["speed_off_grid"] = False
    if scenario == "A1":
        out["setting_key"] = f"dz={out.get('dz_m_nominal')}"
    elif scenario == "A8":
        out["setting_key"] = f"speed={out.get('speed_mps_nominal')}"
    elif scenario in ("A2", "A3", "A4", "A5", "A7"):
        out["setting_key"] = f"speed={out.get('speed_mps_nominal')}"
    else:
        out["setting_key"] = ""
    return out


def parse_sidecar(path: Path) -> dict:
    return json.loads(path.read_text())


def stamp_from_meta_path(path: Path) -> tuple[str, str]:
    m = re.match(r"([A-Za-z0-9]+)_(\d{4}-\d{2}-\d{2})_(\d{2}-\d{2}-\d{2})\.meta\.json$", path.name)
    if not m:
        raise ValueError(path.name)
    return m.group(2), m.group(3)
