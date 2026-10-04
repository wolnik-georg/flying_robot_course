#!/usr/bin/env python3
"""Sweep RPM measurement (d, f_rpm); baseline validation; optional doc-58 grid."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
ANALYSIS = Path(__file__).resolve().parent
OUT = ANALYSIS / "out" / "indi_sil_rpm_delay"
SIM = ANALYSIS / "indi_sil_rpm_delay_sim.py"
CS2_SIM = Path("/home/georg/Desktop/crazyswarm2/crazyflie_sim")

DELAYS_MS = [0, 10, 20, 30, 50]
F_RPM = [1000, 100, 50, 20]

OURS_FLOWN = dict(
    label="baseline_ours_flown",
    bottom_controller="oot",
    ctrl_mode=3,
    res_sign=1,
    indi_overrides=dict(kr=2400, kw=170, fc_bw=206, filt_dt_us=1000, filt_prewarp=1),
    kp_xy=64,
    kp_z=48,
    kv_xy=5,
    kv_z=7,
    ki_z=0,
)

OMAR_C = dict(
    label="baseline_omar_c",
    bottom_controller="oot4",
    kp_xy=64,
    kp_z=48,
    kv_xy=5,
    kv_z=7,
)

FLIGHT = {
    "ours": {"gyro": (260, 290), "f": (4.7, 5.7), "z_cm": (-19, -17)},
    "omar": {"gyro": (55, 97), "f": (3.2, 3.9), "z_cm": (2, 4)},
}


def run_one(cfg: dict) -> dict:
    proc = subprocess.run(
        [sys.executable, str(SIM), json.dumps(cfg)],
        cwd=str(ANALYSIS),
        capture_output=True,
        text=True,
        env={
            **os.environ,
            "PYTHONPATH": f"{REPO / 'flying_drone_stack/firmware_app/host'}:"
            f"/home/georg/Desktop/crazyflie-firmware/build:{CS2_SIM}",
        },
    )
    if proc.returncode != 0:
        return {"label": cfg.get("label"), "error": (proc.stderr or proc.stdout)[-1500:]}
    for ln in proc.stdout.splitlines():
        if ln.strip().startswith("{"):
            return json.loads(ln)
    return {"label": cfg.get("label"), "error": "no json"}


def in_band(v: float, band: tuple[float, float]) -> bool:
    return band[0] <= v <= band[1]


def score_baseline(ours_m: dict, omar_m: dict) -> dict:
    ok = {"ours_gyro": False, "ours_f": False, "omar_gyro": False, "omar_f": False}
    if ours_m:
        ok["ours_gyro"] = in_band(ours_m["gyro_rms_deg_s"], FLIGHT["ours"]["gyro"])
        ok["ours_f"] = in_band(ours_m.get("dominant_hz_gyro_x") or 0, FLIGHT["ours"]["f"])
    if omar_m:
        ok["omar_gyro"] = in_band(omar_m["gyro_rms_deg_s"], FLIGHT["omar"]["gyro"])
        ok["omar_f"] = in_band(omar_m.get("dominant_hz_gyro_x") or 0, FLIGHT["omar"]["f"])
    hits = sum(ok.values())
    return {"checks": ok, "hits": hits, "qualitative_match": hits >= 3}


def build_sweep() -> list[dict]:
    cfgs = []
    for d in DELAYS_MS:
        for f in F_RPM:
            for base, name in ((OURS_FLOWN, "ours"), (OMAR_C, "omar")):
                c = {**base, "label": f"sweep_{name}_d{d}_f{f}", "rpm_meas": {"delay_ms": d, "f_rpm": f}}
                cfgs.append(c)
    return cfgs


def build_grid(rpm_meas: dict) -> list[dict]:
    runs = []
    kr_kw = [(2400, 170), (987, 109), (483, 76)]
    pos_sets = [
        ("flown", 64, 48, 5, 7),
        ("geo", 40, 30, 8, 10),
        ("matched", 26, 26, 3.2, 3.2),
    ]
    for rs in (+1, -1):
        for kr, kw in kr_kw:
            for pname, kpx, kpz, kvx, kvz in pos_sets:
                runs.append(
                    dict(
                        label=f"grid_rs{rs:+d}_kr{kr}_{pname}",
                        bottom_controller="oot",
                        ctrl_mode=3,
                        res_sign=rs,
                        indi_overrides=dict(kr=kr, kw=kw, fc_bw=206, filt_dt_us=1000, filt_prewarp=1),
                        kp_xy=kpx,
                        kp_z=kpz,
                        kv_xy=kvx,
                        kv_z=kvz,
                        ki_z=0,
                        rpm_meas=rpm_meas,
                    )
                )
    return runs


def write_sweep_table(results: list[dict]) -> None:
    lines = [
        "| delay_ms | f_rpm | ours gyro | ours f | ours regime | omar gyro | omar f | omar regime | score |",
        "|---:|---:|---:|---:|---|---:|---:|---:|---:|",
    ]
    by_key = {}
    for r in results:
        if "metrics" not in r:
            continue
        rm = r.get("rpm_meas") or r["config"].get("rpm_meas", {})
        key = (rm.get("delay_ms"), rm.get("f_rpm"))
        by_key.setdefault(key, {})[ "ours" if "ours" in r["label"] else "omar"] = r
    for key in sorted(by_key.keys()):
        d, f = key
        row = by_key[key]
        om, oo = row.get("omar", {}).get("metrics", {}), row.get("ours", {}).get("metrics", {})
        sc = score_baseline(oo, om)
        lines.append(
            f"| {d} | {f} | {oo.get('gyro_rms_deg_s', float('nan')):.1f} | "
            f"{oo.get('dominant_hz_gyro_x', 0):.2f} | {oo.get('regime','?')} | "
            f"{om.get('gyro_rms_deg_s', float('nan')):.1f} | {om.get('dominant_hz_gyro_x', 0):.2f} | "
            f"{om.get('regime','?')} | {sc['hits']}/4 |"
        )
    (OUT / "sweep_summary.md").write_text("\n".join(lines) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sweep-only", action="store_true")
    parser.add_argument("--grid", action="store_true")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    configs = build_sweep()
    results = []
    for i, cfg in enumerate(configs):
        print(f"({i+1}/{len(configs)}) {cfg['label']}", flush=True)
        results.append(run_one(cfg))

    best = None
    best_hits = -1
    paired = {}
    for r in results:
        if "metrics" not in r:
            continue
        rm = r.get("rpm_meas") or r["config"]["rpm_meas"]
        k = (rm["delay_ms"], rm["f_rpm"])
        paired.setdefault(k, {})["ours" if "ours" in r["label"] else "omar"] = r["metrics"]
    for k, pair in paired.items():
        sc = score_baseline(pair.get("ours"), pair.get("omar"))
        if sc["hits"] > best_hits:
            best_hits = sc["hits"]
            best = {"delay_ms": k[0], "f_rpm": k[1], **sc}

    payload = {
        "baseline_validity": {
            "flight_reproduced": best_hits >= 3 if best else False,
            "best_sweep": best,
            "interpretation": "exploratory" if not best or best_hits < 3 else "conditional",
        },
        "sweep_results": results,
    }

    if args.grid and best and best.get("hits", 0) >= 3:
        rpm_meas = {"delay_ms": best["delay_ms"], "f_rpm": best["f_rpm"], "noise_std_rpm": 100.0}
        grid_cfgs = build_grid(rpm_meas)
        grid_res = []
        for cfg in grid_cfgs:
            print("grid", cfg["label"], flush=True)
            grid_res.append(run_one(cfg))
        payload["grid_results"] = grid_res
        payload["grid_rpm_meas"] = rpm_meas
        payload["grid_mode"] = "validated_plant"
    elif args.grid:
        # Illustrative: single best-amplitude sweep point only
        rpm_meas = {"delay_ms": 50, "f_rpm": 20, "noise_std_rpm": 100.0}
        grid_cfgs = build_grid(rpm_meas)[:6]
        grid_res = [run_one(c) for c in grid_cfgs]
        payload["grid_results"] = grid_res
        payload["grid_rpm_meas"] = rpm_meas
        payload["grid_mode"] = "illustrative_subset"
        payload["grid_skipped_full"] = "baseline not validated; full doc-58 grid not run"

    (OUT / "results.json").write_text(json.dumps(payload, indent=2))
    write_sweep_table(results)
    print("Best sweep:", best)
    print("Wrote", OUT / "results.json")


if __name__ == "__main__":
    main()
