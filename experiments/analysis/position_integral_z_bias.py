#!/usr/bin/env python3
"""Desk: steady-state Z tracking bias on C.1 merged logs (integral-fixable bias check).

Uses ctrltarget_z vs logged position (stateEstimate / EKF, columns x,y,z) on merged uSD CSVs.
Phases: scenario + approach* only (excludes ramp/land). Also reports a tighter "near-hover"
subset where |vz| < 0.05 m/s and |d ctrltarget_z/dt| < 0.02 m/s.

Outputs JSON under experiments/analysis/out/position_integral_z_bias_2026-09-26.json
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import metrics as M  # noqa: E402

sys.path.insert(0, str(Path("/home/georg/Desktop/crazyswarm2/crazyflie_examples")))


def resolve_vehicle_names(raw: dict, meta: dict) -> dict:
    out = {}
    for logical in meta["names"]:
        if logical in raw:
            out[logical] = raw[logical]
            continue
        for k, v in raw.items():
            if k.startswith(f"{logical}_"):
                out[logical] = v
                break
    return out


def z_stats(v: M.VehicleLog, lo: float, hi: float) -> dict | None:
    if v.pos is None or v.pos_des is None:
        return None
    m = (v.t >= lo) & (v.t <= hi)
    if not np.any(m):
        return None
    pos = v.pos[m]
    des = v.pos_des[m]
    t = v.t[m]
    ez = pos[:, 2] - des[:, 2]
    vz = np.gradient(pos[:, 2], t) if len(t) > 2 else np.zeros(len(t))
    dtz = np.gradient(des[:, 2], t) if len(t) > 2 else np.zeros(len(t))
    calm = (np.abs(vz) < 0.05) & (np.abs(dtz) < 0.02)
    out = {
        "n": int(len(ez)),
        "z_err_mean_m": float(np.mean(ez)),
        "z_err_std_m": float(np.std(ez)),
        "z_err_median_m": float(np.median(ez)),
        "z_rmse_m": float(np.sqrt(np.mean(ez ** 2))),
        "ctrltarget_z_mean_m": float(np.mean(des[:, 2])),
        "z_mean_m": float(np.mean(pos[:, 2])),
    }
    if np.any(calm):
        ec = ez[calm]
        out["calm_n"] = int(len(ec))
        out["calm_z_err_mean_m"] = float(np.mean(ec))
        out["calm_z_err_std_m"] = float(np.std(ec))
    else:
        out["calm_n"] = 0
        out["calm_z_err_mean_m"] = float("nan")
        out["calm_z_err_std_m"] = float("nan")
    return out


def meta_path_for(entry: dict, merged_root: Path, date: str) -> Path:
    sc, stamp = entry["scenario"], entry["stamp"]
    p = merged_root / f"{sc}_{date}_{stamp}" / f"{sc}_{date}_{stamp}.meta.json"
    if p.is_file():
        return p
    return ROOT / "experiments/logs" / f"{sc}_{date}_{stamp}.meta.json"


def process_manifest(manifest_path: Path, date: str, merged_root: Path) -> list[dict]:
    from crazyflie_examples.formations import scenarios as S

    manifest = json.loads(manifest_path.read_text())
    rows = []
    for ent in manifest:
        if ent.get("merge_status") not in (None, "merged"):
            continue
        if ent.get("training_eligible") is False:
            continue
        if ent.get("merge_status") != "merged" and "merged_usd" not in ent.get("path", ""):
            continue
        merged = ROOT / ent["path"]
        meta_path = meta_path_for(ent, merged_root, date)
        if not meta_path.exists() or not merged.is_file():
            continue
        meta = json.loads(meta_path.read_text())
        sc = S.build(meta["scenario"], **meta["params"])
        vehicles = resolve_vehicle_names(M.load_merged_csv(merged), meta)
        # Merged uSD CSVs use t=0 at scenario start (# meta:t_zero=scenario_start).
        # meta["t_start_sim"] is wall-clock ROS time — only for non-aligned logs.
        sample = next(iter(vehicles.values()))
        t0: float | None = 0.0 if sample.t_zero == "scenario_start" else float(meta["t_start_sim"])
        for name, v in vehicles.items():
            for phase, (lo, hi) in M.phase_windows(v, sc, t0).items():
                if phase in ("ramp", "land"):
                    continue
                st = z_stats(v, lo, hi)
                if st is None:
                    continue
                rows.append({
                    "date": date,
                    "scenario": ent["scenario"],
                    "stamp": ent.get("stamp", ""),
                    "vehicle": name,
                    "phase": phase,
                    "study_vehicle": name == meta["names"][0],
                    "alignment_rms_cm": ent.get("alignment_rms_cm"),
                    "merge_flags": ent.get("merge_flags", []),
                    **st,
                })
    return rows


def summarize(rows: list[dict]) -> dict:
    # Primary: geometric study vehicle (cf5), scenario phase only
    primary = [r for r in rows if r["vehicle"] == "cf5" and r["phase"] == "scenario"]
    means = [r["z_err_mean_m"] for r in primary]
    calm_means = [r["calm_z_err_mean_m"] for r in primary if r.get("calm_n", 0) > 50]

    def sign_consistency(vals: list[float]) -> dict:
        if not vals:
            return {"n": 0}
        pos = sum(1 for v in vals if v > 0.01)
        neg = sum(1 for v in vals if v < -0.01)
        tiny = len(vals) - pos - neg
        return {
            "n": len(vals),
            "mean_of_means_m": float(np.mean(vals)),
            "std_across_flights_m": float(np.std(vals)),
            "median_m": float(np.median(vals)),
            "positive_gt_1cm": pos,
            "negative_lt_minus_1cm": neg,
            "within_plus_minus_1cm": tiny,
            "same_sign_fraction": float(max(pos, neg) / len(vals)) if vals else float("nan"),
        }

    cf_second_scenario = [
        r for r in rows if r["vehicle"] == "cf_second" and r["phase"] == "scenario"
    ]
    a2 = [r for r in primary if r["scenario"] == "A2"]

    return {
        "cf5_scenario_phase": sign_consistency(means),
        "cf5_scenario_calm_subset": sign_consistency(calm_means),
        "cf_second_scenario_phase": sign_consistency(
            [r["z_err_mean_m"] for r in cf_second_scenario]
        ),
        "a2_cf5_scenario": [
            {k: r[k] for k in ("stamp", "z_err_mean_m", "z_rmse_m", "calm_z_err_mean_m",
                               "alignment_rms_cm", "merge_flags")}
            for r in a2
        ],
        "n_rows_total": len(rows),
        "n_cf5_scenario_flights": len(primary),
    }


def main() -> None:
    bundles = [
        (
            ROOT / "experiments/logs/c1_2026-09-21_merged/manifest_2026-09-21_c1.json",
            "2026-09-21",
            ROOT / "experiments/logs/c1_2026-09-21_merged",
        ),
        (
            ROOT / "experiments/logs/c1_2026-09-23_merged/manifest_2026-09-23_c1.json",
            "2026-09-23",
            ROOT / "experiments/logs/c1_2026-09-23_merged",
        ),
    ]
    all_rows: list[dict] = []
    for mp, date, mroot in bundles:
        if mp.exists():
            all_rows.extend(process_manifest(mp, date, mroot))

    out_dir = ROOT / "experiments/analysis/out"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "position_integral_z_bias_2026-09-26.json"
    payload = {"rows": all_rows, "summary": summarize(all_rows)}
    out_path.write_text(json.dumps(payload, indent=2))
    print(f"wrote {out_path}")
    s = payload["summary"]["cf5_scenario_phase"]
    if s.get("n", 0):
        print(
            f"cf5 scenario: n={s['n']} mean_of_means={s['mean_of_means_m']:.4f} m "
            f"std_across_flights={s['std_across_flights_m']:.4f} m"
        )
    else:
        print("cf5 scenario: no rows — check manifest/meta paths")


if __name__ == "__main__":
    main()
