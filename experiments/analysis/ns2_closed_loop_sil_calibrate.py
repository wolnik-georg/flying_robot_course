#!/usr/bin/env python3
"""Step 3 extended calibration + plausibility (2026-10-07 follow-up 4)."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np

REPO = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent / "out" / "ns2_closed_loop_sil"
ANALYSIS = Path(__file__).resolve().parent

for _rn in ("rclpy", "rclpy.node", "rclpy.time", "rosgraph_msgs", "rosgraph_msgs.msg"):
    sys.modules.setdefault(_rn, MagicMock())
sys.path.insert(0, str(ANALYSIS))

from ns2_closed_loop_sil_metrics import (  # noqa: E402
    find_crossing_times,
    steady_mask,
    tilt_from_quat_deg,
)
from ns2_closed_loop_sil_run import (  # noqa: E402
    HW_DIP_OFF_CM,
    HW_DIP_OFF_TOL_CM,
    HW_DIP_ON_CM,
    HW_DIP_ON_TOL_CM,
    N_CALIB_SEEDS,
    base_cfg,
    mean_dip,
    run_one,
)
from ns2_closed_loop_sil_sim import HW_MEAS_GYRO_STD_DEG_S, HW_MEAS_POS_STD_M  # noqa: E402

HW_DIP_RANGE_CM = (-7.2, -5.4)


def a8_cfg(
    scale: float,
    *,
    seed: int = 0,
    meas_noise: bool = False,
    rnn_en: bool = False,
    downwash_plant: str = "ns2",
) -> dict:
    c = {
        **base_cfg("A8"),
        "downwash": True,
        "downwash_scale": scale,
        "downwash_plant": downwash_plant,
        "skip_rnn_upload": not rnn_en,
        "rnn_en": rnn_en,
        "meas_noise": meas_noise,
        "seed": seed,
        "return_logs": True,
    }
    if downwash_plant == "bank":
        c["plant_weights"] = base_cfg("A8")["weights"]
    if rnn_en:
        c["skip_rnn_upload"] = False
    return c


def dip_run(
    scale: float,
    *,
    seed: int = 0,
    meas_noise: bool = False,
    force: bool = True,
    downwash_plant: str = "ns2",
) -> dict:
    label = f"calib2_{downwash_plant}_s{scale}_seed{seed}{'_n' if meas_noise else ''}"
    return run_one(
        {**a8_cfg(scale, seed=seed, meas_noise=meas_noise, downwash_plant=downwash_plant), "label": label},
        force=force,
    )


def fa_at_crossings(r: dict) -> float:
    logs = r.get("logs") or {}
    t = np.asarray(logs.get("t", []), float)
    fa = np.asarray(logs.get("fa_bot_az", []), float)
    pos = np.asarray(logs.get("pos_bot", []), float)
    if len(t) < 10 or len(fa) != len(t):
        return float("nan")
    tcross = find_crossing_times(t, pos[:, 1], n_expect=16)
    vals = []
    for tc in tcross[:8]:
        m = (t >= tc - 0.5) & (t <= tc + 0.5)
        if np.any(m):
            vals.append(float(np.mean(fa[m])))
    return float(np.mean(vals)) if vals else float("nan")


def fa_a1_hold(scale: float, *, force: bool = True, downwash_plant: str = "ns2") -> float:
    cfg = {
        **base_cfg("A1"),
        "downwash": True,
        "downwash_scale": scale,
        "downwash_plant": downwash_plant,
        "skip_rnn_upload": True,
        "return_logs": True,
        "duration_s": 22.0,
    }
    if downwash_plant == "bank":
        cfg["plant_weights"] = base_cfg("A1")["weights"]
    r = run_one(cfg, force=force)
    logs = r.get("logs") or {}
    t = np.asarray(logs.get("t", []), float)
    fa = np.asarray(logs.get("fa_bot_az", []), float)
    if len(fa) == 0:
        return float("nan")
    m = t >= 4.0
    return float(np.mean(fa[m]))


def bottom_tilt(r: dict) -> dict:
    logs = r.get("logs") or {}
    t = np.asarray(logs.get("t", []), float)
    qb = np.asarray(logs.get("quat_bot", []), float)
    pos = np.asarray(logs.get("pos_bot", []), float)
    mb = steady_mask(t, pos, scenario="A8", takeoff_s=4.0, n_crossings=16)
    tilts = np.array([tilt_from_quat_deg(q) for q in qb[mb]]) if np.any(mb) else np.array([])
    return {
        "p99_deg": float(np.percentile(tilts, 99)) if len(tilts) else float("nan"),
        "max_deg": float(np.max(tilts)) if len(tilts) else float("nan"),
    }


def search_scale(force: bool) -> tuple[list, float]:
    grid = [4.0, 4.5, 5.0, 6.0]
    rows = []
    for s in grid:
        r = dip_run(s, meas_noise=False, force=force)
        rows.append({"scale": s, "dip_cm_mean": mean_dip(r), "dip_each": r.get("crossing_dips", {}).get("dip_cm_each")})
    # bisection in [3.0, 6.0]
    lo, hi = 3.0, 6.0
    best_s, best_err, best_d = 4.0, 1e9, float("nan")
    for _ in range(8):
        mid = 0.5 * (lo + hi)
        r = dip_run(mid, meas_noise=False, force=force)
        d = mean_dip(r)
        rows.append({"scale": mid, "dip_cm_mean": d, "bisect": True})
        err = abs(d - HW_DIP_OFF_CM)
        if err < best_err:
            best_err, best_s, best_d = err, mid, d
        upper = HW_DIP_OFF_CM + HW_DIP_OFF_TOL_CM
        lower = HW_DIP_OFF_CM - HW_DIP_OFF_TOL_CM
        if d > upper:
            lo = mid
        elif d < lower:
            hi = mid
        else:
            best_s, best_d = mid, d
            break
    return rows, round(best_s, 2)


def main() -> None:
    force = "--force" in sys.argv
    downwash_plant = "bank" if "--plant" in sys.argv else "ns2"
    if "--plant" in sys.argv:
        i = sys.argv.index("--plant")
        if i + 1 < len(sys.argv) and not sys.argv[i + 1].startswith("-"):
            downwash_plant = sys.argv[i + 1]
    OUT.mkdir(parents=True, exist_ok=True)
    report: dict = {
        "downwash_plant": downwash_plant,
        "meas_noise": {
            "enabled_for_seeds": True,
            "pos_std_m": HW_MEAS_POS_STD_M.tolist(),
            "gyro_std_deg_s": HW_MEAS_GYRO_STD_DEG_S.tolist(),
            "source": "merged_A8_rnn0_2026-10-05_17-39-27 cf5 pos err & gyro std outside crossings",
        },
        "hardware_dip_target_cm": HW_DIP_OFF_CM,
        "hardware_dip_range_cm": list(HW_DIP_RANGE_CM),
    }

    if downwash_plant == "bank":
        scale = 1.0
        r0 = dip_run(1.0, meas_noise=False, force=force, downwash_plant="bank")
        report["dip_curve"] = [{"scale": 1.0, "dip_cm_mean": mean_dip(r0)}]
        report["calibrated_scale"] = scale
        report["note"] = "bank plant uses full_bank weights at scale 1.0 (no NS2 scalar search)"
    else:
        curve, scale = search_scale(force)
        report["dip_curve"] = curve
        report["calibrated_scale"] = scale

    seed_dips = []
    seed_each = []
    for seed in range(N_CALIB_SEEDS):
        r = dip_run(scale, seed=seed, meas_noise=True, force=force, downwash_plant=downwash_plant)
        cd = r.get("crossing_dips", {})
        seed_dips.append(mean_dip(r))
        seed_each.append(cd.get("dip_cm_each", []))
    report["seed_dips_cm"] = seed_dips
    report["seed_dips_spread_cm"] = {
        "mean": float(np.mean(seed_dips)),
        "std": float(np.std(seed_dips)),
        "min": float(np.min(seed_dips)),
        "max": float(np.max(seed_dips)),
    }

    in_band = all(abs(d - HW_DIP_OFF_CM) <= HW_DIP_OFF_TOL_CM for d in seed_dips if np.isfinite(d))

    r1 = dip_run(1.0, meas_noise=False, force=force, downwash_plant=downwash_plant)
    r_cal = dip_run(scale, meas_noise=False, force=force, downwash_plant=downwash_plant)
    plaus = {
        "scale_1.0": {
            "fa_az_crossings_m_s2": fa_at_crossings(r1),
            "fa_az_a1_hold_m_s2": fa_a1_hold(1.0, force=force, downwash_plant=downwash_plant),
        },
        f"scale_{scale}": {
            "fa_az_crossings_m_s2": fa_at_crossings(r_cal),
            "fa_az_a1_hold_m_s2": fa_a1_hold(scale, force=force, downwash_plant=downwash_plant),
        },
        "hardware_ref_m_s2": {"rnn_pred_z_crossings": -0.6, "rnn_pred_z_a1_stack": -1.6},
    }
    report["plausibility"] = plaus

    def _ok_plaus(s_key: str) -> bool:
        p = plaus[s_key]
        hw_c, hw_a = -0.6, -1.6
        for val, hw in ((p["fa_az_crossings_m_s2"], hw_c), (p["fa_az_a1_hold_m_s2"], hw_a)):
            if not np.isfinite(val) or hw == 0:
                return False
            if abs(val / hw) > 1.5 or abs(val / hw) < 1 / 1.5:
                return False
        return True

    plaus_ok = _ok_plaus(f"scale_{scale}") and _ok_plaus("scale_1.0")
    report["plausibility_ok_factor_1p5"] = plaus_ok

    tilt = bottom_tilt(r_cal)
    report["bottom_tilt_check"] = {**tilt, "hw_p99_deg": 23.0, "hw_max_deg": 28.0}
    tilt_ok = tilt.get("p99_deg", 999) <= 26 and tilt.get("max_deg", 999) <= 32

    step3_pass = in_band and tilt_ok and plaus_ok
    report["step3_pass"] = step3_pass

    step4 = None
    if step3_pass:
        s4_dips = []
        for seed in range(N_CALIB_SEEDS):
            r = run_one(
                {
                    **a8_cfg(scale, seed=seed, meas_noise=True, rnn_en=True, downwash_plant=downwash_plant),
                    "label": f"step4_s{scale}_seed{seed}",
                    "res_sign": 1,
                },
                force=force,
            )
            s4_dips.append(mean_dip(r))
        step4 = {
            "pass": all(abs(d - HW_DIP_ON_CM) <= HW_DIP_ON_TOL_CM for d in s4_dips if np.isfinite(d)),
            "dips_cm": s4_dips,
            "mean_cm": float(np.mean(s4_dips)),
            "target_cm": HW_DIP_ON_CM,
            "tol_cm": HW_DIP_ON_TOL_CM,
        }
        report["step4"] = step4

    out_name = "step3_calibrate_bank.json" if downwash_plant == "bank" else "step3_calibrate_extended.json"
    (OUT / out_name).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
