#!/usr/bin/env python3
"""Build replay inputs and run all controllers (subprocess per side)."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
ANALYSIS = Path(__file__).resolve().parent
BUILD = Path("/home/georg/Desktop/crazyflie-firmware/build")
OUT = ANALYSIS / "out" / "indi_replay"
WORKER = ANALYSIS / "indi_replay_worker.py"

FLIGHTS = {
    "A8": {
        "meta": REPO / "experiments/logs/A8_2026-10-02_19-09-54.meta.json",
        "usd": REPO / "experiments/logs/usd_raw/cf5_thesis69_2026-10-02_19-18-54.bin",
    },
    "A1": {
        "meta": REPO / "experiments/logs/A1_2026-10-02_19-15-47.meta.json",
        "usd": REPO / "experiments/logs/usd_raw/cf5_thesis72_2026-10-02_19-18-55.bin",
    },
}

SIDES = ("ours", "omar_c", "omar_rust")
LEVELS = ("L0", "L1", "L2", "L3")


def _host_defaults_snapshot() -> dict:
    sys.path.insert(0, str(BUILD))
    import cffirmware as fw  # noqa: E402

    fw.controllerOutOfTreeInit()
    cv = fw.cvar
    keys = [
        "g_controller_mode", "g_indi_mass", "g_indi_kr", "g_indi_kw", "g_indi_fc_bw",
        "g_kp_xy", "g_kp_z", "g_kv_xy", "g_kv_z", "g_ki_z", "g_indi_kt1",
    ]
    return {k: float(getattr(cv, k)) for k in keys if hasattr(cv, k)}


def env_build() -> dict:
    e = os.environ.copy()
    e["PYTHONPATH"] = str(BUILD) + ((":" + e["PYTHONPATH"]) if e.get("PYTHONPATH") else "")
    return e


def run_worker(side: str, level: str, npz: Path, csv: Path, meta: Path, debug_z_trim: bool) -> dict:
    cmd = [sys.executable, str(WORKER), side, level, str(npz), str(csv), str(meta), str(BUILD)]
    if debug_z_trim:
        cmd.append("--debug-z-trim")
    out = subprocess.run(cmd, capture_output=True, text=True, env=env_build(), check=False)
    if out.returncode != 0:
        raise RuntimeError(f"worker failed {side} {level}:\n{out.stderr}\n{out.stdout}")
    json_lines = [l for l in out.stdout.splitlines() if l.strip().startswith("{")]
    if not json_lines:
        raise RuntimeError(f"worker produced no JSON {side} {level}:\n{out.stderr}\n{out.stdout}")
    return json.loads(json_lines[-1])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--flight", choices=list(FLIGHTS), default="A8")
    ap.add_argument("--levels", default=",".join(LEVELS))
    ap.add_argument("--sides", default=",".join(SIDES))
    ap.add_argument("--skip-inputs", action="store_true")
    ap.add_argument("--force", action="store_true", help="re-run even if CSV exists")
    ap.add_argument(
        "--debug-z-trim",
        action="store_true",
        help="apply z_tracking_trim_m to setpoint z (debug only; not validated)",
    )
    args = ap.parse_args()

    sys.path.insert(0, str(ANALYSIS))
    from indi_replay_inputs import build_replay_npz  # noqa: E402

    spec = FLIGHTS[args.flight]
    tag = args.flight
    run_dir = OUT / tag
    run_dir.mkdir(parents=True, exist_ok=True)
    npz = run_dir / "inputs.npz"
    from indi_replay_config import flight_ours_config, load_yaml_flight_params  # noqa: E402

    meta_obj = json.loads(spec["meta"].read_text())
    yaml_cache = load_yaml_flight_params()
    flight_cfg = flight_ours_config(meta_obj, yaml_cache=yaml_cache)
    (run_dir / "flight_config.json").write_text(json.dumps(flight_cfg, indent=2))

    if not args.skip_inputs:
        pack = build_replay_npz(spec["usd"], spec["meta"], role="bottom", out_npz=npz)
        meta_out = {
            "flight": tag,
            "lag_s": float(pack["lag_s"]),
            "warmup_start": int(pack["warmup_start"]),
            "z_tracking_trim_m": float(pack["z_tracking_trim_m"]),
            "n_ticks_1khz": int(len(pack["pos"])),
            "usd": str(spec["usd"]),
            "meta": str(spec["meta"]),
        }
        (run_dir / "inputs_meta.json").write_text(json.dumps(meta_out, indent=2))

    levels = [x.strip() for x in args.levels.split(",") if x.strip()]
    sides = [x.strip() for x in args.sides.split(",") if x.strip()]
    jobs = []
    for level in levels:
        for side in sides:
            csv = run_dir / f"replay_{side}_{level}.csv"
            if not args.force and csv.is_file() and csv.stat().st_size > 1000:
                jobs.append((side, level, csv, None))
            else:
                jobs.append((side, level, csv, "run"))

    manifest: dict = {
        "flight": tag,
        "flight_config_path": str(run_dir / "flight_config.json"),
        "host_defaults_before_apply": _host_defaults_snapshot(),
        "runs": [],
        "params_by_side_level": {},
    }
    pending = [j for j in jobs if j[3] == "run"]
    with ThreadPoolExecutor(max_workers=3) as pool:
        futs = {
            pool.submit(run_worker, s, lv, npz, csv, spec["meta"], args.debug_z_trim): (s, lv, csv)
            for s, lv, csv, flag in pending
        }
        for fut in as_completed(futs):
            info = fut.result()
            manifest["runs"].append(info)
            if "params_used" in info:
                manifest["params_by_side_level"][f"{info['side']}_{info['level']}"] = info["params_used"]
    for s, lv, csv, flag in jobs:
        if flag is None:
            manifest["runs"].append({"side": s, "level": lv, "rows": "cached", "out": str(csv)})
    manifest["debug_z_trim"] = args.debug_z_trim
    (run_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(json.dumps({"flight": tag, "out": str(run_dir), "runs": len(manifest["runs"])}))


if __name__ == "__main__":
    main()
