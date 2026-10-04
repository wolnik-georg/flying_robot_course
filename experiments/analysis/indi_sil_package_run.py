#!/usr/bin/env python3
"""Launch CS2 SIL package grid (copied yaml scratch + subprocess-isolated episodes)."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
ANALYSIS = Path(__file__).resolve().parent
OUT = ANALYSIS / "out" / "indi_sil_package"
SCRATCH = OUT / "config_scratch"
SIM = ANALYSIS / "indi_sil_package_sim.py"

CS2_CFG = Path("/home/georg/Desktop/crazyswarm2/crazyflie/config")

FLIGHT_TARGETS = {
    "ours_oct02": {
        "gyro_rms_deg_s": (260, 290),
        "dominant_hz": (4.7, 5.7),
        "mean_z_err_cm": (-19, -17),
    },
    "omar_c_oct02": {
        "gyro_rms_deg_s": (55, 97),
        "dominant_hz": (3.2, 3.9),
        "mean_z_err_cm": (2, 4),
    },
}

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
)

GRID_EXTRA = [
    dict(
        label="val_2026-09-11_987_matched",
        bottom_controller="oot",
        ctrl_mode=3,
        res_sign=1,
        indi_overrides=dict(kr=987, kw=109, fc_bw=206, filt_dt_us=1000, filt_prewarp=1),
        kp_xy=26,
        kp_z=26,
        kv_xy=3.2,
        kv_z=3.2,
        ki_z=0,
    ),
    dict(
        label="val_2026-09-11_483_matched",
        bottom_controller="oot",
        ctrl_mode=3,
        res_sign=1,
        indi_overrides=dict(kr=483, kw=76, fc_bw=206, filt_dt_us=1000, filt_prewarp=1),
        kp_xy=13,
        kp_z=13,
        kv_xy=2.2,
        kv_z=2.2,
        ki_z=0,
    ),
    dict(
        label="package_never_flown_rs-1_483_geo",
        bottom_controller="oot",
        ctrl_mode=3,
        res_sign=-1,
        indi_overrides=dict(kr=483, kw=76, fc_bw=206, filt_dt_us=1000, filt_prewarp=1),
        kp_xy=40,
        kp_z=30,
        kv_xy=8,
        kv_z=10,
        ki_z=0,
    ),
    dict(
        label="val_2026-09-11_483_geo",
        bottom_controller="oot",
        ctrl_mode=3,
        res_sign=1,
        indi_overrides=dict(kr=483, kw=76, fc_bw=206, filt_dt_us=1000, filt_prewarp=1),
        kp_xy=40,
        kp_z=30,
        kv_xy=8,
        kv_z=10,
        ki_z=0,
    ),
]


def copy_scratch_configs() -> None:
    SCRATCH.mkdir(parents=True, exist_ok=True)
    names = [
        "server_sim_indi.yaml",
        "server_sim_omar_indi_dw.yaml",
        "server_sim_omar_indi_rust.yaml",
        "server_sim_dw.yaml",
        "crazyflies_sim_dw.yaml",
    ]
    for n in names:
        src = CS2_CFG / n
        if src.is_file():
            shutil.copy2(src, SCRATCH / n)
    note = SCRATCH / "README.txt"
    if not note.exists():
        note.write_text(
            "Copies of CS2 server/crazyflies yaml for traceability only.\n"
            "indi_sil_package_sim.py sets firmware_params via firm.cvar (same path as yaml).\n"
        )


def build_grid(include_full: bool) -> list[dict]:
    runs = [OURS_FLOWN, OMAR_C]
    if not include_full:
        return runs
    runs.extend(GRID_EXTRA)
    kr_kw = [(2400, 170), (987, 109), (483, 76)]
    pos_sets = [
        ("flown", 64, 48, 5, 7),
        ("geo_like", 40, 30, 8, 10),
        ("matched_low_kr", 26, 26, 3.2, 3.2),
    ]
    for rs in (+1, -1):
        for kr, kw in kr_kw:
            for pname, kpx, kpz, kvx, kvz in pos_sets:
                runs.append(
                    dict(
                        label=f"grid_rs{rs:+d}_kr{kr}_pos_{pname}",
                        bottom_controller="oot",
                        ctrl_mode=3,
                        res_sign=rs,
                        indi_overrides=dict(
                            kr=kr, kw=kw, fc_bw=206, filt_dt_us=1000, filt_prewarp=1
                        ),
                        kp_xy=kpx,
                        kp_z=kpz,
                        kv_xy=kvx,
                        kv_z=kvz,
                        ki_z=0,
                    )
                )
    return runs


def run_one(cfg: dict) -> dict:
    cs2_sim = Path("/home/georg/Desktop/crazyswarm2/crazyflie_sim")
    proc = subprocess.run(
        [sys.executable, str(SIM), json.dumps(cfg)],
        cwd=str(ANALYSIS),
        capture_output=True,
        text=True,
        env={
            **os.environ,
            "PYTHONPATH": f"{REPO / 'flying_drone_stack/firmware_app/host'}:"
            f"/home/georg/Desktop/crazyflie-firmware/build:{cs2_sim}",
        },
    )
    if proc.returncode != 0:
        return {"label": cfg.get("label"), "error": (proc.stderr or proc.stdout)[-2000:]}
    text = proc.stdout.strip()
    if not text:
        return {"label": cfg.get("label"), "error": proc.stderr[-2000:] or "empty stdout"}
    for ln in text.splitlines():
        if ln.strip().startswith("{"):
            return json.loads(ln)
    return {"label": cfg.get("label"), "error": text[-500:]}


def qualitative_match(row: dict, key: str) -> str:
    t = FLIGHT_TARGETS[key]
    m = row["metrics"]
    ok_g = t["gyro_rms_deg_s"][0] <= m["gyro_rms_deg_s"] <= t["gyro_rms_deg_s"][1]
    ok_f = t["dominant_hz"][0] <= m["dominant_hz_gyro_x"] <= t["dominant_hz"][1]
    ok_z = t["mean_z_err_cm"][0] <= m["mean_z_err_cm"] <= t["mean_z_err_cm"][1]
    hits = sum([ok_g, ok_f, ok_z])
    if hits >= 2:
        return "qualitative_match"
    if hits == 1:
        return "partial"
    return "not_reproduced"


def baseline_validity(results: list[dict]) -> dict:
    by = {r["label"]: r for r in results if "metrics" in r}
    out = {}
    if "baseline_ours_flown" in by:
        out["ours"] = qualitative_match(by["baseline_ours_flown"], "ours_oct02")
    if "baseline_omar_c" in by:
        out["omar_c"] = qualitative_match(by["baseline_omar_c"], "omar_c_oct02")
    out["grid_interpretation"] = (
        "exploratory"
        if out.get("ours") != "qualitative_match" and out.get("omar_c") != "qualitative_match"
        else "conditional"
    )
    return out


def write_summary_table(results: list[dict], validity: dict) -> None:
    lines = [
        "| label | regime | gyro RMS [°/s] | f_dom [Hz] | mean z err [cm] | lat RMS [cm] | roll/pitch RMS [°] |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for r in results:
        if "metrics" not in r:
            lines.append(f"| {r.get('label','?')} | ERROR | — | — | — | — | — |")
            continue
        m = r["metrics"]
        lines.append(
            f"| {r['label']} | {m['regime']} | {m['gyro_rms_deg_s']:.1f} | "
            f"{m['dominant_hz_gyro_x']:.2f} | {m['mean_z_err_cm']:.1f} | "
            f"{m['lateral_rms_cm']:.1f} | {m['roll_rms_deg']:.1f}/{m['pitch_rms_deg']:.1f} |"
        )
    (OUT / "summary_table.md").write_text(
        "Baseline validity: "
        + json.dumps(validity, indent=None)
        + "\n\n"
        + "\n".join(lines)
        + "\n"
    )


def plot_results(results: list[dict]) -> None:
    import matplotlib.pyplot as plt
    import numpy as np

    ok = [r for r in results if "metrics" in r]
    if not ok:
        return
    labels = [r["label"] for r in ok]
    gyro = [r["metrics"]["gyro_rms_deg_s"] for r in ok]
    freq = [r["metrics"]["dominant_hz_gyro_x"] for r in ok]
    zerr = [r["metrics"]["mean_z_err_cm"] for r in ok]

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    x = np.arange(len(labels))
    axes[0].bar(x, gyro, color="steelblue")
    axes[0].axhspan(260, 290, color="red", alpha=0.15, label="flight ours gyro")
    axes[0].axhspan(55, 97, color="green", alpha=0.15, label="flight Omar gyro")
    axes[0].set_ylabel("Gyro RMS [°/s]")
    axes[0].legend(fontsize=7)

    axes[1].bar(x, freq, color="darkorange")
    axes[1].axhspan(4.7, 5.7, color="red", alpha=0.15)
    axes[1].axhspan(3.2, 3.9, color="green", alpha=0.15)
    axes[1].set_ylabel("f_dom gyro_x [Hz]")

    axes[2].bar(x, zerr, color="purple")
    axes[2].axhspan(-19, -17, color="red", alpha=0.15)
    axes[2].axhspan(2, 4, color="green", alpha=0.15)
    axes[2].set_ylabel("mean z err [cm]")
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(labels, rotation=75, ha="right", fontsize=6)
    fig.tight_layout()
    fig.savefig(OUT / "fig_grid_metrics.png", dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-only", action="store_true")
    parser.add_argument("--full-grid", action="store_true")
    args = parser.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)
    copy_scratch_configs()

    configs = build_grid(include_full=args.full_grid and not args.baseline_only)
    results = []
    for i, cfg in enumerate(configs):
        print(f"Running ({i+1}/{len(configs)}) {cfg['label']}...", flush=True)
        results.append(run_one(cfg))
        (OUT / "results_partial.json").write_text(json.dumps(results, indent=2))

    validity = baseline_validity(results)
    payload = {"baseline_validity": validity, "results": results}
    (OUT / "results.json").write_text(json.dumps(payload, indent=2))
    write_summary_table(results, validity)
    try:
        plot_results(results)
    except (ImportError, AttributeError) as e:
        (OUT / "plot_skipped.txt").write_text(f"matplotlib unavailable: {e}\n")
    print(f"Wrote {OUT / 'results.json'}")
    print("Baseline validity:", validity)


if __name__ == "__main__":
    main()
