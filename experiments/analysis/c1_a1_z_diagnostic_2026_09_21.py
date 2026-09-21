#!/usr/bin/env python3
"""Task 4 (docs/37): cf5 z-error shape for healthy vs unmergeable A1 flights.

All errors use the same reference: scenario `commanded_trajectory()` for role bottom,
with lag search via `find_offset()` (3D position, same as `merge_usd_logs.py --meta`).
Merged CSVs are already at `# meta:t_zero=scenario_start` — no lag search on those.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "flying_drone_stack/tools"))
sys.path.insert(0, str(ROOT / "experiments/analysis"))
sys.path.insert(0, "/home/georg/Desktop/crazyswarm2/crazyflie_examples")

from decode_usd_log import load as usd_load  # noqa: E402
from find_flight_window import commanded_trajectory, find_offset  # noqa: E402

RAW = ROOT / "experiments/logs/usd_raw/2026-09-21_THESIS1"
HEALTHY = [
    ("12-51-16", ROOT / "experiments/logs/c1_2026-09-21_merged/A1_2026-09-21_12-51-16/A1_2026-09-21_12-51-16_merged_usd.csv"),
    ("13-25-10", ROOT / "experiments/logs/c1_2026-09-21_merged/A1_2026-09-21_13-25-10/A1_2026-09-21_13-25-10_merged_usd.csv"),
]
# Best-fit cf5 bottom `.bin` per stamp (`2026-09-21_PAIRING.md` grid search).
UNMERGE: dict[str, Path] = {
    "12-40-41": RAW / "cf5__thesis04_thesis04_2026-09-21_12-56-12.bin",
    "12-43-34": RAW / "cf5__thesis06_thesis06_2026-09-21_12-56-14.bin",
    "12-49-36": RAW / "cf5__thesis07_thesis07_2026-09-21_12-56-15.bin",
    "13-27-27": RAW / "cf5_A1_pm_thesis16_thesis16_2026-09-21_13-33-01.bin",
    "13-28-49": RAW / "cf5_A1_pm_thesis17_thesis17_2026-09-21_13-33-01.bin",
    "13-30-30": RAW / "cf5_A1_pm_thesis18_thesis18_2026-09-21_13-33-01.bin",
}


def _prefix_for_cf5(header: list[str]) -> str:
    for c in header:
        if c.startswith("cf5") and c.endswith(".z"):
            return c[: -len(".z")]
    raise KeyError("no cf5*.z in merged header")


def _load_merged_xyz(csv: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    lines = csv.read_text().splitlines()
    hdr_i = next(i for i, l in enumerate(lines) if l.strip() and not l.startswith("#"))
    hdr = lines[hdr_i].split(",")
    cols = {c: i for i, c in enumerate(hdr)}
    pref = _prefix_for_cf5(hdr)
    data = np.loadtxt(csv, delimiter=",", skiprows=hdr_i + 1, ndmin=2)
    t = data[:, cols["t"]]
    x = data[:, cols[f"{pref}.x"]]
    y = data[:, cols[f"{pref}.y"]]
    z = data[:, cols[f"{pref}.z"]]
    return t, x, y, z


def _classify_shape(err: np.ndarray) -> str:
    if len(err) < 50:
        return "too few samples"
    slope = np.polyfit(np.arange(len(err)), err, 1)[0]
    if np.std(err) < 0.025 and abs(np.mean(err)) > 0.08:
        return "roughly constant bias"
    if slope < -0.002:
        return "growing divergence"
    if np.std(err) > 0.08:
        return "oscillatory"
    return "mixed"


def z_stats_aligned(meta: dict, t: np.ndarray, x: np.ndarray, y: np.ndarray, z: np.ndarray, *,
                    scenario_relative_time: bool) -> dict:
    ts, cmd = commanded_trajectory(meta, "bottom")
    total = float(meta["duration"])
    pos = np.stack([x, y, z], axis=1)
    t = np.asarray(t, dtype=float)

    if scenario_relative_time:
        lag = 0.0
        tt = t
        mask = (tt >= 0.0) & (tt <= total)
        if mask.sum() < 200:
            return {"error": "too few samples in scenario window"}
        cmd_pts = np.stack([np.interp(tt[mask], ts, cmd[:, ax]) for ax in range(3)], axis=1)
        mse = float(np.mean(np.sum((pos[mask] - cmd_pts) ** 2, axis=1)))
    else:
        if t[-1] - t[0] < total:
            return {"error": f"recording {t[-1]-t[0]:.1f}s shorter than scenario {total:.1f}s"}
        lag, mse, _ = find_offset(t, pos, ts, cmd, t[0], t[-1] - total)
        if lag is None:
            return {"error": "find_offset: no lag covers scenario"}
        tt = t - lag
        mask = (tt >= 0.0) & (tt <= total)
        if mask.sum() < 200:
            return {"error": "too few samples after lag align"}

    cmd_z = np.interp(tt[mask], ts, cmd[:, 2])
    err = z[mask] - cmd_z
    pos_rms_cm = float(np.sqrt(mse) * 100.0) if mse is not None else float(
        np.sqrt(np.mean(np.sum((pos[mask] - np.stack(
            [np.interp(tt[mask], ts, cmd[:, ax]) for ax in range(3)], axis=1)) ** 2, axis=1))) * 100.0
    )
    return {
        "n": int(mask.sum()),
        "align_lag_s": float(lag),
        "pos_rms_3d_cm": pos_rms_cm,
        "z_rmse_mm": float(np.sqrt(np.mean(err ** 2)) * 1000),
        "z_bias_mm": float(np.mean(err) * 1000),
        "z_std_mm": float(np.std(err) * 1000),
        "shape": _classify_shape(err),
        "reference": "commanded_trajectory+bottom",
    }


def z_stats_merged(csv: Path, meta_path: Path) -> dict:
    meta = json.loads(meta_path.read_text())
    t, x, y, z = _load_merged_xyz(csv)
    out = z_stats_aligned(meta, t, x, y, z, scenario_relative_time=True)
    out["source"] = str(csv.name)
    return out


def z_stats_bin(meta_path: Path, bin_path: Path) -> dict:
    meta = json.loads(meta_path.read_text())
    raw = usd_load(bin_path)
    for k in ("x", "y", "z"):
        if k not in raw:
            return {"error": f"no {k} in decoded log", "bin": bin_path.name}
    out = z_stats_aligned(
        meta,
        raw["t"],
        np.asarray(raw["x"], dtype=float),
        np.asarray(raw["y"], dtype=float),
        np.asarray(raw["z"], dtype=float),
        scenario_relative_time=False,
    )
    out["bin"] = bin_path.name
    return out


def main():
    out = {"healthy_merged": {}, "unmergeable": {}}
    for stamp, csv in HEALTHY:
        meta = csv.parent / f"A1_2026-09-21_{stamp}.meta.json"
        out["healthy_merged"][stamp] = z_stats_merged(csv, meta)
    for stamp, bpath in UNMERGE.items():
        meta = ROOT / f"experiments/logs/c1_2026-09-21_merged/A1_2026-09-21_{stamp}/A1_2026-09-21_{stamp}.meta.json"
        if not meta.is_file():
            meta = ROOT / f"experiments/logs/A1_2026-09-21_{stamp}.meta.json"
        if bpath.is_file() and meta.is_file():
            out["unmergeable"][stamp] = z_stats_bin(meta, bpath)
        elif meta.is_file():
            out["unmergeable"][stamp] = {"error": "bin missing", "bin": str(bpath.name)}
    print(json.dumps(out, indent=2))
    out_path = ROOT / "experiments/analysis/out/c1_2026-09-21/a1_z_diagnostic.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, indent=2) + "\n")


if __name__ == "__main__":
    main()
