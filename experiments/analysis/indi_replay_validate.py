#!/usr/bin/env python3
"""On-policy check: replay-ours thrust_si vs flown motor command (total thrust)."""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

ANALYSIS = Path(__file__).resolve().parent
sys.path.insert(0, str(ANALYSIS))
from indi_replay_command import commanded_total_thrust_N, radio_csv_for_meta, supply_voltage_on_usd_timeline  # noqa: E402
from indi_replay_worker import WARMUP_TICKS  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent / "out" / "indi_replay"
STEADY_Z = 0.35


def steady_mask(sp_z: np.ndarray, tick: int, warmup_start: int) -> bool:
    """Post warm-up and COMMANDED height at its plateau (>= 90% of the max setpoint z).

    Review fix 2026-10-06: the earlier rule used MEASURED pos_z > 0.35 m, which silently drops most
    of A1 (cf5 flew at ~0.24 m against a 0.5 m setpoint -- the known INDI height sag), leaving only
    1726 samples. The window must not depend on where the vehicle actually was."""
    if tick < warmup_start + WARMUP_TICKS:
        return False
    return float(sp_z[tick]) >= 0.9 * float(np.max(sp_z))


def validate_flight(tag: str, meta_path: Path) -> dict:
    z = np.load(OUT / tag / "inputs.npz")
    if "motor_pwm" not in z:
        return {"flight": tag, "error": "inputs.npz missing motor_pwm — rebuild inputs"}
    warmup_start = int(z["warmup_start"])
    sp_z = z["sp_pos"][:, 2]
    motor = z["motor_pwm"]
    radio = radio_csv_for_meta(meta_path)
    lag = float(z["lag_s"])
    vbat = supply_voltage_on_usd_timeline(z["t_usd"], lag, radio)
    csv_path = OUT / tag / "replay_ours_L0.csv"
    cmd_list, tr_list = [], []
    with csv_path.open() as f:
        for row in csv.DictReader(f):
            tick = int(row["tick"])
            if not steady_mask(sp_z, tick, warmup_start):
                continue
            cmd_list.append(float(commanded_total_thrust_N(motor[tick], vbat[tick])))
            tr_list.append(float(row["thrust_si"]))
    if not cmd_list:
        return {"flight": tag, "error": "no steady samples"}
    tc = np.asarray(cmd_list)
    tr = np.asarray(tr_list)
    mean_c = float(np.mean(tc))
    mean_r = float(np.mean(tr))
    err_pct = abs(mean_r - mean_c) / mean_c * 100 if mean_c else float("nan")
    corr = float(np.corrcoef(tc, tr)[0, 1]) if len(tc) > 2 else float("nan")
    slope, intercept = (float(np.polyfit(tc, tr, 1)[0]), float(np.polyfit(tc, tr, 1)[1])) if len(tc) > 2 else (float("nan"), float("nan"))
    return {
        "flight": tag,
        "flown_command_mean_N": mean_c,
        "replay_ours_mean_N": mean_r,
        "mean_err_pct": err_pct,
        "corr": corr,
        "regression_slope": slope,
        "regression_intercept_N": intercept,
        "pass_5pct": err_pct <= 5.0,
        "pass_corr": corr > 0.9,
        "n": int(len(tc)),
        "steady_definition": f"tick >= warmup_start+{WARMUP_TICKS} ({warmup_start}+{WARMUP_TICKS}) and commanded sp_z >= 0.9*max(sp_z)",
        "thrust_mapping": (
            "bat-comp inverse: V=(m/65535)*supplyVoltage_LPF(vbat); "
            "T_i=c0+c1*V+c2*V^2+c3*V^3 (CF21BL); sum T_i"
        ),
        "gate": "command-based (step 9)",
    }


def main() -> None:
    checks = [
        validate_flight("A8", REPO / "experiments/logs/A8_2026-10-02_19-09-54.meta.json"),
        validate_flight("A1", REPO / "experiments/logs/A1_2026-10-02_19-15-47.meta.json"),
    ]
    summary = {
        "on_policy": checks,
        "all_pass": all(c.get("pass_5pct") and c.get("pass_corr") for c in checks if "error" not in c),
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "on_policy_check.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
