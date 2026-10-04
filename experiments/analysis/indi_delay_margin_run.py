#!/usr/bin/env python3
"""Delay-margin sweeps, baseline validation, gain/sign grid (new files only)."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
ANALYSIS = Path(__file__).resolve().parent
OUT = ANALYSIS / "out" / "indi_delay_margin"
SIM = ANALYSIS / "indi_delay_margin_sim.py"
BUDGET = OUT / "hardware_delay_budget.json"
CS2_SIM = Path("/home/georg/Desktop/crazyswarm2/crazyflie_sim")

DEAD_MS = [0, 0.5, 1, 1.5, 2, 3, 4]
KR_KW = [
    (2400, 170),
    (1200, 120),
    (987, 109),
    (600, 90),
    (483, 76),
]

FLIGHT = {"ours_gyro": (260, 290), "ours_f": (4.7, 5.7), "omar_gyro": (55, 97)}

OURS_BASE = dict(
    bottom_controller="oot",
    ctrl_mode=3,
    res_sign=1,
    indi_overrides=dict(kr=2400, kw=170, fc_bw=206, filt_dt_us=1000, filt_prewarp=1),
    kp_xy=64,
    kp_z=48,
    kv_xy=5,
    kv_z=7,
    ki_z=0,
    duration_s=12.0,
    seed=0,
)

OMAR = dict(label="omar_c", bottom_controller="oot4", kp_xy=64, kp_z=48, kv_xy=5, kv_z=7, duration_s=12.0, seed=0)

ENV = {
    **os.environ,
    "PYTHONPATH": f"{REPO / 'flying_drone_stack/firmware_app/host'}:/home/georg/Desktop/crazyflie-firmware/build:{CS2_SIM}",
}


def run_one(cfg: dict) -> dict:
    proc = subprocess.run(
        [sys.executable, str(SIM), json.dumps(cfg)],
        cwd=str(ANALYSIS),
        capture_output=True,
        text=True,
        env=ENV,
    )
    if proc.returncode != 0:
        return {"label": cfg.get("label"), "error": (proc.stderr or proc.stdout)[-1200:]}
    for ln in proc.stdout.splitlines():
        if ln.strip().startswith("{"):
            return json.loads(ln)
    return {"label": cfg.get("label"), "error": "no json"}


def load_budget() -> dict:
    if not BUDGET.is_file():
        subprocess.run([sys.executable, str(ANALYSIS / "indi_delay_margin_budget.py")], check=True)
    return json.loads(BUDGET.read_text())


def budget_latency(budget: dict, gyro_lpf: bool = True, noise: bool = True) -> dict:
    """Desk-derived latency bundle (not tuned to match flight)."""
    return {
        "cmd_dead_ms": float(budget["command_path_extra_ms_nominal"]),
        "gyro_lpf_hz": 80.0 if gyro_lpf else 0,
        "acc_lpf_hz": 30.0 if gyro_lpf else 0,
        "sensor_delay_ms": 0.0,
        "gyro_noise_deg_s": 1.0 if noise else 0.0,
        "spool_asym": 1.0,
    }


def is_limit_cycle(m: dict) -> bool:
    if not m:
        return False
    r = m.get("regime", "")
    g = m.get("gyro_rms_deg_s", 0)
    return r in ("limit_cycle",) or g >= 80


def margin_ms(results: list[dict], kr: int, with_lpf: bool) -> float | None:
    """Smallest dead time in DEAD_MS (with gyro LPF) that yields limit cycle for kr."""
    subset = [
        r
        for r in results
        if r.get("kr") == kr
        and r.get("gyro_lpf") == with_lpf
        and "metrics" in r
        and r.get("role") == "ours"
    ]
    subset.sort(key=lambda r: r["dead_ms"])
    for r in subset:
        if is_limit_cycle(r["metrics"]):
            return float(r["dead_ms"])
    return None


def flight_score(m: dict) -> int:
    if not m:
        return 0
    hits = 0
    g = m.get("gyro_rms_deg_s", 0)
    f = m.get("dominant_hz_gyro_x") or 0
    if FLIGHT["ours_gyro"][0] <= g <= FLIGHT["ours_gyro"][1]:
        hits += 1
    if FLIGHT["ours_f"][0] <= f <= FLIGHT["ours_f"][1]:
        hits += 1
    return hits


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    budget = load_budget()
    all_results: list[dict] = []

    # Step 2: dead sweep × gyro LPF
    for dead in DEAD_MS:
        for lpf in (False, True):
            lat = {
                "cmd_dead_ms": dead,
                "gyro_lpf_hz": 80 if lpf else 0,
                "acc_lpf_hz": 30 if lpf else 0,
                "sensor_delay_ms": 0,
                "gyro_noise_deg_s": 0,
            }
            for base, role in ((OURS_BASE, "ours"), (OMAR, "omar")):
                cfg = {
                    **base,
                    "label": f"dead{dead}_lpf{int(lpf)}_{role}",
                    "latency": lat,
                }
                print("run", cfg["label"], flush=True)
                r = run_one(cfg)
                r["dead_ms"] = dead
                r["gyro_lpf"] = lpf
                r["role"] = role
                r["kr"] = 2400
                all_results.append(r)
                (OUT / "results_partial.json").write_text(json.dumps(all_results, indent=2))

    # Step 2b: delay margin vs kr (gyro LPF on)
    for kr, kw in KR_KW:
        for dead in DEAD_MS:
            lat = {
                "cmd_dead_ms": dead,
                "gyro_lpf_hz": 80,
                "acc_lpf_hz": 30,
                "sensor_delay_ms": 0,
                "gyro_noise_deg_s": 0,
            }
            cfg = {
                **OURS_BASE,
                "label": f"margin_kr{kr}_d{dead}",
                "indi_overrides": dict(kr=kr, kw=kw, fc_bw=206, filt_dt_us=1000, filt_prewarp=1),
                "latency": lat,
            }
            print("run", cfg["label"], flush=True)
            r = run_one(cfg)
            r["dead_ms"] = dead
            r["gyro_lpf"] = True
            r["role"] = "ours"
            r["kr"] = kr
            r["kw"] = kw
            all_results.append(r)
            (OUT / "results_partial.json").write_text(json.dumps(all_results, indent=2))

    # Omar margin at kr 2400 equivalent (embedded gains) — same dead sweep + LPF
    for dead in DEAD_MS:
        lat = {
            "cmd_dead_ms": dead,
            "gyro_lpf_hz": 80,
            "acc_lpf_hz": 30,
            "sensor_delay_ms": 0,
            "gyro_noise_deg_s": 0,
        }
        cfg = {**OMAR, "label": f"omar_margin_d{dead}", "latency": lat}
        print("run", cfg["label"], flush=True)
        r = run_one(cfg)
        r["dead_ms"] = dead
        r["gyro_lpf"] = True
        r["role"] = "omar"
        r["kr"] = None
        all_results.append(r)

    # Step 3: baseline with budget latency
    lat_nom = budget_latency(budget, gyro_lpf=True, noise=True)
    baseline_cases = [
        ("baseline_budget_ours", {**OURS_BASE, "latency": lat_nom}),
        ("baseline_budget_omar", {**OMAR, "latency": lat_nom}),
        ("baseline_budget_ours_spool125", {**OURS_BASE, "latency": {**lat_nom, "spool_asym": 1.25}}),
        ("baseline_budget_ours_spool130", {**OURS_BASE, "latency": {**lat_nom, "spool_asym": 1.30}}),
        ("baseline_dead2_lpf80_only", {**OURS_BASE, "latency": {"cmd_dead_ms": 2, "gyro_lpf_hz": 80, "acc_lpf_hz": 30, "gyro_noise_deg_s": 0}}),
    ]
    baseline_results = []
    for label, cfg in baseline_cases:
        cfg = {**cfg, "label": label}
        print("run", label, flush=True)
        r = run_one(cfg)
        baseline_results.append(r)
        all_results.append(r)

    # Step 4 grid (illustrative / partly validated)
    grid = []
    for rs in (+1, -1):
        for kr, kw in ((2400, 170), (987, 109), (483, 76)):
            for kp, kz, kvx, kvz, tag in (
                (64, 48, 5, 7, "flown"),
                (40, 30, 8, 10, "card_geom"),
                (26, 26, 3.2, 3.2, "card_matched"),
            ):
                lat = lat_nom
                cfg = {
                    **OURS_BASE,
                    "label": f"grid_rs{rs}_kr{kr}_{tag}",
                    "res_sign": rs,
                    "indi_overrides": dict(kr=kr, kw=kw, fc_bw=206, filt_dt_us=1000, filt_prewarp=1),
                    "kp_xy": kp,
                    "kp_z": kz,
                    "kv_xy": kvx,
                    "kv_z": kvz,
                    "latency": lat,
                }
                print("run", cfg["label"], flush=True)
                r = run_one(cfg)
                grid.append(r)
                all_results.append(r)

    # Lever: higher gyro LPF cutoff (sim-only firmware change)
    for fc_gyro in (80, 250):
        lat = {**lat_nom, "gyro_lpf_hz": fc_gyro}
        cfg = {**OURS_BASE, "label": f"lever_gyrolpf{fc_gyro}", "latency": lat}
        r = run_one(cfg)
        grid.append(r)
        all_results.append(r)

    margins = {kr: margin_ms(all_results, kr, True) for kr, _ in KR_KW}
    margin_2400 = margins.get(2400)
    hw_nom = budget.get("total_extra_dead_equiv_ms_nominal")
    hw_rng = budget.get("total_uncertainty_ms", [2, 8])

    best_baseline = None
    for r in baseline_results:
        if "metrics" not in r or not r.get("partner_ok", True):
            continue
        if best_baseline is None or flight_score(r["metrics"]) > flight_score(best_baseline["metrics"]):
            best_baseline = r

    repro = best_baseline and flight_score(best_baseline["metrics"]) >= 2
    partly = best_baseline and flight_score(best_baseline["metrics"]) == 1
    validity = "validated" if repro else ("partly_validated" if partly else "not_validated")

    summary = {
        "plant_validity": validity,
        "hardware_delay_ms_nominal": hw_nom,
        "hardware_delay_ms_range": hw_rng,
        "margin_at_kr2400_ms": margin_2400,
        "margins_per_kr": margins,
        "best_baseline_label": best_baseline.get("label") if best_baseline else None,
        "best_baseline_metrics": best_baseline.get("metrics") if best_baseline else None,
        "flight_repro_score_max": 2,
        "cmd_dead_method": "linear_interp_on_rpm_history",
    }
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2))
    (OUT / "results.json").write_text(json.dumps({"summary": summary, "results": all_results, "grid": grid}, indent=2))

    rows = ["dead_ms,lpf,role,gyro_rms,dom_hz,regime,z_err_cm,partner_gyro"]
    for r in all_results:
        if "metrics" not in r:
            continue
        m = r["metrics"]
        pm = r.get("partner_metrics") or {}
        rows.append(
            f"{r.get('dead_ms','')},{r.get('gyro_lpf','')},{r.get('role','')},"
            f"{m.get('gyro_rms_deg_s')},{m.get('dominant_hz_gyro_x')},{m.get('regime')},"
            f"{m.get('mean_z_err_cm')},{pm.get('gyro_rms_deg_s')}"
        )
    (OUT / "results.csv").write_text("\n".join(rows) + "\n")
    print("done", json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
