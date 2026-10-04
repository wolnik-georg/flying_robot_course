#!/usr/bin/env python3
"""Run SIL gap bisection: which missing harness elements excite limit cycle."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
ANALYSIS = Path(__file__).resolve().parent
OUT = ANALYSIS / "out" / "indi_harness_bisect"
SIM = ANALYSIS / "indi_harness_sil_gap_sim.py"
CS2_SIM = Path("/home/georg/Desktop/crazyswarm2/crazyflie_sim")

OURS = dict(
    label="ours_flown",
    bottom_controller="oot",
    ctrl_mode=3,
    res_sign=1,
    indi_overrides=dict(kr=2400, kw=170, fc_bw=206, filt_dt_us=1000, filt_prewarp=1),
    kp_xy=64, kp_z=48, kv_xy=5, kv_z=7, ki_z=0,
)

OMAR = dict(label="omar_c", bottom_controller="oot4", kp_xy=64, kp_z=48, kv_xy=5, kv_z=7)

FLIGHT = {"ours_gyro": (260, 290), "ours_f": (4.7, 5.7), "omar_gyro": (55, 97)}


def run_one(cfg: dict) -> dict:
    proc = subprocess.run(
        [sys.executable, str(SIM), json.dumps(cfg)],
        cwd=str(ANALYSIS),
        capture_output=True,
        text=True,
        env={**os.environ, "PYTHONPATH": f"{REPO / 'flying_drone_stack/firmware_app/host'}:/home/georg/Desktop/crazyflie-firmware/build:{CS2_SIM}"},
    )
    if proc.returncode != 0:
        return {"label": cfg.get("label"), "error": proc.stderr[-800:]}
    for ln in proc.stdout.splitlines():
        if ln.strip().startswith("{"):
            return json.loads(ln)
    return {"label": cfg.get("label"), "error": "no json"}


def score(m: dict) -> int:
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
    cases = [
        ("sil_baseline", {}),
        ("cmd_dead_2", {"cmd_dead_ticks": 2}),
        ("rpm_lag_1", {"rpm_lag_samples": 1}),
        ("dead2_rpm_lag1", {"cmd_dead_ticks": 2, "rpm_lag_samples": 1}),
        ("hold_500", {"output_hold_500hz": True}),
        ("spool_125", {"spool_asym": 1.25}),
        ("combo_harness_like", {"cmd_dead_ticks": 2, "rpm_lag_samples": 1, "spool_asym": 1.25}),
    ]
    results = []
    for name, gap in cases:
        for base, tag in ((OURS, "ours"), (OMAR, "omar")):
            cfg = {**base, "label": f"{name}_{tag}", "gap": gap}
            print("run", cfg["label"], flush=True)
            results.append(run_one(cfg))

    best = max(
        (r for r in results if "metrics" in r and "ours" in r["label"]),
        key=lambda r: r["metrics"]["gyro_rms_deg_s"],
        default=None,
    )
    validity = {
        "flight_reproduced": best and score(best["metrics"]) >= 2,
        "best_ours": best["label"] if best else None,
        "best_ours_gyro": best["metrics"]["gyro_rms_deg_s"] if best else None,
    }
    (OUT / "sil_gap_results.json").write_text(json.dumps({"validity": validity, "results": results}, indent=2))
    lines = ["| case | ours gyro | ours f | regime | omar gyro |", "|---|---:|---:|---|---:|"]
    by_case = {}
    for r in results:
        if "metrics" not in r:
            continue
        case = r["label"].rsplit("_", 1)[0]
        role = r["label"].split("_")[-1]
        by_case.setdefault(case, {})[role] = r["metrics"]
    for case, pair in by_case.items():
        o, m = pair.get("ours", {}), pair.get("omar", {})
        lines.append(
            f"| {case} | {o.get('gyro_rms_deg_s', float('nan')):.1f} | {o.get('dominant_hz_gyro_x', 0):.2f} | "
            f"{o.get('regime','?')} | {m.get('gyro_rms_deg_s', float('nan')):.1f} |"
        )
    (OUT / "sil_gap_summary.md").write_text("\n".join(lines) + "\n")
    print("validity", validity)


if __name__ == "__main__":
    main()
