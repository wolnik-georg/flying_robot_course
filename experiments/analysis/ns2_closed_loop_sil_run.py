#!/usr/bin/env python3
"""Gated NS2 closed-loop SIL runner (subprocess per episode, cache per label)."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
ANALYSIS = Path(__file__).resolve().parent
OUT = ANALYSIS / "out" / "ns2_closed_loop_sil"
EP = OUT / "episodes"
SIM = ANALYSIS / "ns2_closed_loop_sil_sim.py"
WEIGHTS = REPO / "experiments/analysis/out/c2_e2e_2026-10-01/full_bank_c1_complete.npz"
HOST = REPO / "flying_drone_stack/firmware_app/host"
FW = Path("/home/georg/Desktop/crazyflie-firmware/build")
CS2_SIM = Path("/home/georg/Desktop/crazyswarm2/crazyflie_sim")

ENV = {
    **os.environ,
    "PYTHONPATH": f"{HOST}:{FW}:{CS2_SIM}:/home/georg/Desktop/crazyswarm2/crazyflie_examples",
}

HW_DIP_OFF_CM = -5.9
HW_DIP_OFF_TOL_CM = 0.8
HW_DIP_ON_CM = -10.9
HW_DIP_ON_TOL_CM = 1.5
N_CALIB_SEEDS = 3
HW_TOP_TILT_REF_DEG = 2.7
HW_TOP_TILT_SIL_MAX_DEG = HW_TOP_TILT_REF_DEG + 5.0


def base_cfg(scenario: str, **kw) -> dict:
    c = {
        "scenario": scenario,
        "weights": str(WEIGHTS),
        "anchor": [0.0, 0.0, 0.5],
        "dz": 0.5,
        "peer_hz": 100.0,
        "div": 10,
        "ki_z": 16.0,
        "res_sign": 1,
        "takeoff_s": 4.0,
        "land_s": 3.0,
        "log_hz": 100,
        "passes": 4,
        "settle_s": 2.0,
        "shuttle_duration": 6.0,
    }
    if scenario == "A1":
        c["hold_s"] = 15.0
        c["duration_s"] = 4.0 + 15.0 + 3.0
        c["n_crossings"] = 0
        c["setpoint_mode"] = "cmd_fd"
    else:
        c["duration_s"] = 4.0 + 26.0 + 3.0
        c["n_crossings"] = 16 if kw.get("passes_full") else 4
        c["setpoint_mode"] = "hlc"
    c.update(kw)
    return c


def run_one(cfg: dict, *, force: bool = False) -> dict:
    EP.mkdir(parents=True, exist_ok=True)
    label = cfg.get("label", "run")
    cache = EP / f"{label}.json"
    if cache.is_file() and not force:
        return json.loads(cache.read_text())
    proc = subprocess.run(
        [sys.executable, str(SIM), json.dumps(cfg)],
        cwd=str(ANALYSIS),
        capture_output=True,
        text=True,
        env=ENV,
    )
    if proc.returncode != 0:
        r = {"label": label, "error": (proc.stderr or proc.stdout)[-2000:]}
    else:
        r = None
        for ln in proc.stdout.splitlines():
            if ln.strip().startswith("{"):
                r = json.loads(ln)
                break
        if r is None:
            r = {"label": label, "error": "no json on stdout"}
    cache.write_text(json.dumps(r, indent=2))
    return r


def trace_max_diff(a: dict, b: dict) -> float:
    pa = np.asarray(a.get("trace_top_pos") or [], float)
    pb = np.asarray(b.get("trace_top_pos") or [], float)
    n = min(len(pa), len(pb))
    if n == 0:
        return float("nan")
    return float(np.max(np.abs(pa[:n] - pb[:n])))


def step1_isolation(*, force: bool = False) -> dict:
    """Bottom RNN on/off must not change top when downwash disabled."""
    common = dict(
        downwash=False,
        skip_rnn_upload=False,
        duration_s=12.0,
        save_trace=True,
        embed_trace=True,
        seed=0,
        label=None,
        setpoint_mode="hlc",
    )
    off = run_one({**base_cfg("A8", **common), "rnn_en": False, "label": "iso_rnn_off"}, force=force)
    on = run_one({**base_cfg("A8", **common), "rnn_en": True, "label": "iso_rnn_on"}, force=force)
    diff = trace_max_diff(off, on)
    verified = np.isfinite(diff) and diff < 1e-4 and off.get("partner_ok") and on.get("partner_ok")
    return {
        "pass": bool(verified),
        "top_pos_max_diff_m": diff,
        "off_top_gyro": off.get("top_gyro_rms_deg_s"),
        "on_top_gyro": on.get("top_gyro_rms_deg_s"),
        "criterion": "downwash off: |pos_top(rnn_en=0)-pos_top(rnn_en=1)| < 1e-4 m; partner_ok",
    }


def step2_baseline(*, force: bool = False) -> dict:
    """Geometric, no downwash, no network — both drones stable."""
    from ns2_closed_loop_sil_hw_ref import hardware_summary

    hw = hardware_summary()
    rows = []
    for scenario in ("A8", "A1"):
        r = run_one(
            {
                **base_cfg(scenario),
                "label": f"baseline_{scenario}_no_dw",
                "downwash": False,
                "skip_rnn_upload": True,
                "rnn_en": False,
            },
            force=force,
        )
        rows.append(r)
    # A8: compare cmd_fd vs HLC (12 s window) for reporting
    a8_compare = []
    for mode in ("cmd_fd", "hlc"):
        r = run_one(
            {
                **base_cfg("A8"),
                "label": f"a8_compare_{mode}_12s",
                "downwash": False,
                "skip_rnn_upload": True,
                "duration_s": 16.0,
                "setpoint_mode": mode,
                "n_crossings": 0,
            },
            force=force,
        )
        tb, tt = r.get("tracking_bottom_cm", {}), r.get("tracking_top_cm", {})
        a8_compare.append(
            {
                "mode": mode,
                "bot_lat_rms_cm": tb.get("rms_lateral_cm"),
                "top_lat_rms_cm": tt.get("rms_lateral_cm"),
                "bot_mean_z_cm": tb.get("mean_z_err_cm"),
                "top_max_tilt_deg": tt.get("max_tilt_deg"),
            }
        )

    def ok(r, scenario: str):
        if "error" in r:
            return False
        tb = r.get("tracking_bottom_cm", {})
        tt = r.get("tracking_top_cm", {})
        base = (
            r.get("partner_ok")
            and r.get("bottom_gyro_rms_deg_s", 999) < 25
            and r.get("top_gyro_rms_deg_s", 999) < 25
            and abs(tb.get("mean_z_err_cm", 999)) < 5
            and abs(tt.get("mean_z_err_cm", 999)) < 5
            and tb.get("rms_z_cm", 999) < 8
            and tt.get("rms_z_cm", 999) < 8
            and tb.get("rms_lateral_cm", 999) < 5
            and tt.get("rms_lateral_cm", 999) < 5
            and tt.get("max_tilt_deg", 999) <= HW_TOP_TILT_SIL_MAX_DEG
        )
        return base

    passed = all(ok(r, r.get("scenario", "")) for r in rows)
    return {
        "pass": passed,
        "hardware_reference": hw,
        "a8_setpoint_compare_12s": a8_compare,
        "criterion": (
            f"no dw/rnn; outside crossings: lat rms<5 cm, |mean z|<5 cm, rms_z<8 cm; "
            f"top max tilt<={HW_TOP_TILT_SIL_MAX_DEG} deg (hw top ~{HW_TOP_TILT_REF_DEG}); "
            "bottom tilt not gated at step 2"
        ),
        "runs": [
            {
                "label": r.get("label"),
                "partner_ok": r.get("partner_ok"),
                "bottom_mean_z_cm": r.get("tracking_bottom_cm", {}).get("mean_z_err_cm"),
                "bottom_rms_z_cm": r.get("tracking_bottom_cm", {}).get("rms_z_cm"),
                "bottom_rms_lat_cm": r.get("tracking_bottom_cm", {}).get("rms_lateral_cm"),
                "bottom_max_tilt_deg": r.get("tracking_bottom_cm", {}).get("max_tilt_deg"),
                "top_mean_z_cm": r.get("tracking_top_cm", {}).get("mean_z_err_cm"),
                "top_rms_z_cm": r.get("tracking_top_cm", {}).get("rms_z_cm"),
                "top_rms_lat_cm": r.get("tracking_top_cm", {}).get("rms_lateral_cm"),
                "top_max_tilt_deg": r.get("tracking_top_cm", {}).get("max_tilt_deg"),
                "bottom_gyro": r.get("bottom_gyro_rms_deg_s"),
                "top_gyro": r.get("top_gyro_rms_deg_s"),
                "final_pos_bot": r.get("final_pos_bot"),
                "final_pos_top": r.get("final_pos_top"),
            }
            for r in rows
        ],
    }


def mean_dip(r: dict) -> float:
    return float(r.get("crossing_dips", {}).get("dip_cm_mean", float("nan")))


def step3_calibrate(*, force: bool = False) -> dict:
    """One scalar on NS2 Fa; fit on rnn_en=0 only."""
    scales = [0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0]
    best = None
    rows = []
    for s in scales:
        dips = []
        for seed in range(N_CALIB_SEEDS):
            r = run_one(
                {
                    **base_cfg("A8"),
                    "label": f"calib_s{s}_seed{seed}",
                    "downwash": True,
                    "downwash_scale": s,
                    "rnn_en": False,
                    "skip_rnn_upload": True,
                    "seed": seed,
                },
                force=force,
            )
            dips.append(mean_dip(r))
        m = float(np.nanmean(dips))
        rows.append({"scale": s, "dip_cm_mean": m, "dips": dips})
        err = abs(m - HW_DIP_OFF_CM)
        if best is None or err < best[0]:
            best = (err, s, dips)
    assert best is not None
    _, scale, dips = best
    in_band = all(abs(d - HW_DIP_OFF_CM) <= HW_DIP_OFF_TOL_CM for d in dips if np.isfinite(d))
    tilt_check = {}
    if in_band:
        from ns2_closed_loop_sil_metrics import steady_mask, tilt_from_quat_deg

        r = run_one(
            {
                **base_cfg("A8"),
                "label": f"calib_tilt_check_s{scale}",
                "downwash": True,
                "downwash_scale": scale,
                "rnn_en": False,
                "skip_rnn_upload": True,
                "return_logs": True,
            },
            force=force,
        )
        if "logs" in r:
            t = np.asarray(r["logs"]["t"], float)
            qb = np.asarray(r["logs"]["quat_bot"], float)
            mb = steady_mask(t, np.asarray(r["logs"]["pos_bot"], float), scenario="A8", takeoff_s=4.0, n_crossings=16)
            tilts = np.array([tilt_from_quat_deg(q) for q in qb[mb]])
            tilt_check = {
                "bottom_tilt_p99_deg": float(np.percentile(tilts, 99)) if len(tilts) else float("nan"),
                "bottom_tilt_max_deg": float(np.max(tilts)) if len(tilts) else float("nan"),
                "hw_expect_p99_deg": 23.0,
                "hw_expect_max_deg": 28.0,
            }
    tilt_ok = (
        tilt_check.get("bottom_tilt_p99_deg", 999) <= 26.0
        and tilt_check.get("bottom_tilt_max_deg", 999) <= 32.0
    )
    return {
        "pass": bool(in_band and tilt_ok),
        "downwash_scale": scale,
        "target_cm": HW_DIP_OFF_CM,
        "tol_cm": HW_DIP_OFF_TOL_CM,
        "seed_dips_cm": dips,
        "grid": rows,
        "bottom_tilt_check": tilt_check,
        "criterion": f"dip {HW_DIP_OFF_CM}±{HW_DIP_OFF_TOL_CM} cm; bottom tilt p99~23 max~28 deg with DW on",
    }


def step4_test(scale: float) -> dict:
    dips = []
    for seed in range(N_CALIB_SEEDS):
        r = run_one(
            {
                **base_cfg("A8"),
                "label": f"test_en1_s{scale}_seed{seed}",
                "downwash_scale": scale,
                "rnn_en": True,
                "res_sign": 1,
                "seed": seed,
            }
        )
        dips.append(mean_dip(r))
    m = float(np.nanmean(dips))
    passed = all(abs(d - HW_DIP_ON_CM) <= HW_DIP_ON_TOL_CM for d in dips if np.isfinite(d))
    return {
        "pass": bool(passed),
        "downwash_scale": scale,
        "dip_cm_mean": m,
        "dips_cm": dips,
        "target_cm": HW_DIP_ON_CM,
        "tol_cm": HW_DIP_ON_TOL_CM,
        "criterion": f"network-on res_sign=+1: {HW_DIP_ON_CM}±{HW_DIP_ON_TOL_CM} cm, no retuning",
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=int, default=0, help="1..4 or 0=all gated")
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    report: dict = {"steps": {}}
    steps = [args.step] if args.step else [1, 2, 3, 4]

    if 1 in steps:
        print("step 1 isolation", flush=True)
        s1 = step1_isolation(force=args.force)
        report["steps"]["1_isolation"] = s1
        (OUT / "step1_isolation.json").write_text(json.dumps(s1, indent=2))
        if not s1["pass"] and not args.step:
            report["verdict"] = "not_validated"
            (OUT / "summary.json").write_text(json.dumps(report, indent=2))
            print(json.dumps(report, indent=2))
            return

    if 2 in steps:
        print("step 2 baseline", flush=True)
        s2 = step2_baseline(force=args.force)
        report["steps"]["2_baseline"] = s2
        (OUT / "step2_baseline.json").write_text(json.dumps(s2, indent=2))
        if not s2["pass"] and not args.step:
            report["verdict"] = "not_validated"
            (OUT / "summary.json").write_text(json.dumps(report, indent=2))
            print(json.dumps(report, indent=2))
            return

    scale = float(json.loads((OUT / "step3_calibrate.json").read_text())["downwash_scale"]) if (OUT / "step3_calibrate.json").is_file() and 3 not in steps else None

    if 3 in steps:
        print("step 3 calibrate", flush=True)
        s3 = step3_calibrate(force=args.force)
        report["steps"]["3_calibrate"] = s3
        (OUT / "step3_calibrate.json").write_text(json.dumps(s3, indent=2))
        scale = s3["downwash_scale"]
        if not s3["pass"] and not args.step:
            report["verdict"] = "not_validated"
            (OUT / "summary.json").write_text(json.dumps(report, indent=2))
            print(json.dumps(report, indent=2))
            return

    if 4 in steps:
        if scale is None:
            raise SystemExit("need step 3 scale")
        print("step 4 test +1", flush=True)
        s4 = step4_test(scale)
        report["steps"]["4_test_plus1"] = s4
        (OUT / "step4_test.json").write_text(json.dumps(s4, indent=2))
        report["verdict"] = "validated" if s4["pass"] else "not_validated"
        report["downwash_scale"] = scale
        report["isolation_verified"] = report.get("steps", {}).get("1_isolation", {}).get("pass", False)
        report["baseline_sanity_pass"] = report.get("steps", {}).get("2_baseline", {}).get("pass", False)
        (OUT / "summary.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
