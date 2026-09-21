#!/usr/bin/env python3
"""End-to-end desk pipeline for 2026-09-19 A8 session (uSD + radio).

Raw uSD under experiments/logs/usd_raw/ is never modified. Successful formation
flights are packaged under experiments/logs/a8_2026-09-19_successful/<stamp>/ with
symlinks to the archive, copied radio/meta, merged uSD CSV, metrics, and plots.

    python3 experiments/analysis/run_a8_2026_09_19_suite.py
"""
from __future__ import annotations

import csv
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / "flying_drone_stack" / "tools"
LOGS = ROOT / "experiments" / "logs"
OUT_ROOT = LOGS / "a8_2026-09-19_successful"
AN_OUT = ROOT / "experiments" / "analysis" / "out" / "a8_2026-09-19"
PYENV = Path.home() / ".pyenv/versions/flying_robots/bin/python"

sys.path.insert(0, str(Path(__file__).resolve().parent))
import metrics as M  # noqa: E402

sys.path.insert(0, str(Path("/home/georg/Desktop/crazyswarm2/crazyflie_examples")))

# Pairing: thesis counter order on THESIS1 (cf5) + THESIS2 (cf_second), verified
# with merge_usd_logs.py --meta --roles (2026-09-19 desk pass).
FLIGHTS = [
    ("2026-09-19_14-58-02", "thesis00", "thesis07", "stock_lee", True),
    ("2026-09-19_14-59-47", "thesis06", "thesis08", "stock_lee", False),  # false start
    ("2026-09-19_15-00-55", "thesis29", "thesis18", "stock_lee", True),
    ("2026-09-19_15-01-33", "thesis31", "thesis32", "stock_lee", True),
    ("2026-09-19_15-04-01", "thesis53", "thesis33", "geometric", True),
    ("2026-09-19_15-04-41", "thesis54", "thesis34", "geometric", True),
    ("2026-09-19_15-06-38", "thesis55", "thesis35", "full_indi", True),
    ("2026-09-19_15-07-11", "thesis56", "thesis36", "full_indi", True),
    ("2026-09-19_15-12-08", None, None, "stock_indi", False),  # radio only — no unused THESIS1 file
]

USD_BOTTOM = LOGS / "usd_raw/2026-09-19_THESIS1"
USD_TOP = LOGS / "usd_raw/2026-09-19_THESIS2"

# `controller` column in metrics CSV — must NOT alias stock Lee to geometric (C.4 grouping).
CTRL_LABEL = {
    "stock_lee": "stock_lee",
    "geometric": "geometric",
    "full_indi": "indi",
    "stock_indi": "stock_indi",
}


def symlink_usd(raw: Path, dest: Path, drone: str, stamp: str, card_stem: str):
    """Name symlinks so merge_usd_logs drone_name_from() yields cf5 / cf_second."""
    dest.parent.mkdir(parents=True, exist_ok=True)
    link = dest / f"{drone}_A8_{card_stem}_{stamp}.usd"
    if link.exists() or link.is_symlink():
        link.unlink()
    link.symlink_to(raw.resolve())
    return link


def rename_merged_columns(path: Path, old_new: dict[str, str]):
    text = path.read_text()
    for old, new in old_new.items():
        text = text.replace(f"{old}.", f"{new}.")
        text = text.replace(f"rel.{old}_", f"rel.{new}_")
        text = text.replace(f"_{old}_", f"_{new}_")
    path.write_text(text)


def radio_rows(stamp: str) -> dict[str, int]:
    out = {}
    for drone in ("cf5", "cf_second"):
        p = LOGS / f"A8_{drone}_{stamp}.csv"
        n = sum(1 for line in open(p) if line.strip() and not line.startswith("#") and
                "time_s" not in line.split(",")[0])
        # subtract header row counted above — recount properly
        with open(p) as fh:
            hdr = None
            n = 0
            for line in fh:
                if line.startswith("#"):
                    continue
                if hdr is None:
                    hdr = True
                    continue
                if line.strip():
                    n += 1
        out[drone] = n
    return out


def phase_metrics(merged: Path, meta_path: Path, ctrl: str) -> list[dict]:
    from crazyflie_examples.formations import scenarios as S

    meta = json.load(open(meta_path))
    sc = S.build(meta["scenario"], **meta["params"])
    vehicles = M.load_merged_csv(merged)
    anchor = np.array(meta["anchor"])
    t0 = float(meta["t_start_sim"])
    timescale = float(meta.get("timescale", 1.0))
    rows = []
    for i, name in enumerate(meta["names"]):
        if name not in vehicles:
            continue
        v = vehicles[name]
        pos_des = None
        if v.pos_des is None:
            pos_des = M.commanded_from_scenario(sc, i, anchor, t0, timescale, v.t)
        rows.extend(M.vehicle_metrics_by_phase(v, sc, meta["scenario"], ctrl,
                                               len(meta["names"]), pos_des))
    rows.append(M.formation_row(meta["scenario"], ctrl,
                                [vehicles[n] for n in meta["names"] if n in vehicles],
                                meta["params"].get("dz")))
    for r in rows:
        r["run"] = meta_path.stem
        r["study_controller"] = ctrl
    return rows


def main():
    AN_OUT.mkdir(parents=True, exist_ok=True)
    all_phase_rows = []
    manifest_all = {"flights": [], "raw_usd_archive_untouched": True}

    for stamp, bot, top, study, success_flag in FLIGHTS:
        meta_src = LOGS / f"A8_{stamp}.meta.json"
        flight_dir = OUT_ROOT / stamp
        flight_dir.mkdir(parents=True, exist_ok=True)

        pkg = dict(
            stamp=stamp,
            study_controller=study,
            designated_success=success_flag,
            bottom_usd_raw=str((USD_BOTTOM / bot).resolve()) if bot else None,
            top_usd_raw=str((USD_TOP / top).resolve()) if top else None,
            bottom_thesis=bot,
            top_thesis=top,
            radio_rows=radio_rows(stamp),
        )

        for name in (f"A8_{stamp}.meta.json", f"A8_cf5_{stamp}.csv", f"A8_cf_second_{stamp}.csv"):
            shutil.copy2(LOGS / name, flight_dir / name)

        if bot is None or top is None:
            pkg["status"] = "excluded_no_usd_match"
            (flight_dir / "package_manifest.json").write_text(json.dumps(pkg, indent=2) + "\n")
            manifest_all["flights"].append(pkg)
            print(f"[excluded no uSD] {stamp}")
            continue

        b_link = symlink_usd(USD_BOTTOM / bot, flight_dir, "cf5", stamp, bot)
        t_link = symlink_usd(USD_TOP / top, flight_dir, "cf_second", stamp, top)

        merged = flight_dir / f"A8_{stamp}_merged_usd.csv"
        cmd = [
            sys.executable, str(TOOLS / "merge_usd_logs.py"),
            str(b_link), str(t_link),
            "--meta", str(flight_dir / f"A8_{stamp}.meta.json"),
            "--roles", "bottom", "top",
            "-o", str(merged),
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        pkg["merge_ok"] = proc.returncode == 0
        pkg["merge_log_tail"] = (proc.stdout + proc.stderr).splitlines()[-8:]
        if not pkg["merge_ok"]:
            pkg["status"] = "failed_merge"
            (flight_dir / "package_manifest.json").write_text(json.dumps(pkg, indent=2) + "\n")
            manifest_all["flights"].append(pkg)
            print(f"[skip merge fail] {stamp}")
            continue

        # Auto quality gates on radio
        rr = pkg["radio_rows"]
        if rr["cf5"] < 400 or rr["cf_second"] < 400:
            pkg["status"] = "failed_short_radio"
            success_flag = False
        if not success_flag:
            pkg["status"] = pkg.get("status") or "excluded_by_design"
        else:
            pkg["status"] = "success"

        ctrl = CTRL_LABEL[study]
        sidecar = flight_dir / f"A8_{stamp}.meta.json"
        subprocess.run([
            sys.executable, str(Path(__file__).parent / "run_analysis.py"),
            "--scenario", "A8", "--ctrl", ctrl,
            "--logs", str(merged),
            "--dz-cmd", "0.25",
            "--sidecar", str(sidecar),
            "--source", "hardware",
            "--out", str(flight_dir / "analysis"),
        ], check=True)

        if PYENV.exists():
            subprocess.run([
                str(PYENV), str(Path(__file__).parent / "plot_flight.py"),
                "--scenario", "A8", "--ctrl", ctrl,
                "--logs", str(merged),
                "--dz-cmd", "0.25",
                "--sidecar", str(sidecar),
                "--source", "hardware",
                "--out", str(flight_dir / "analysis"),
            ], check=False)
            subprocess.run([
                str(PYENV), str(Path(__file__).parent / "plot_interaction.py"),
                str(merged), "--bottom", "cf5", "--top", "cf_second",
                "--out", str(flight_dir / "analysis" / f"A8_{stamp}_interaction.png"),
            ], check=False)

        if pkg["status"] == "success":
            rows = phase_metrics(merged, flight_dir / f"A8_{stamp}.meta.json", ctrl)
            for r in rows:
                r["study_controller"] = study
            all_phase_rows.extend(rows)
            with open(flight_dir / "analysis" / f"A8_{stamp}_phase_metrics.csv", "w", newline="") as fh:
                if rows:
                    w = csv.DictWriter(fh, fieldnames=sorted({k for r in rows for k in r}))
                    w.writeheader()
                    w.writerows(rows)

        (flight_dir / "package_manifest.json").write_text(json.dumps(pkg, indent=2) + "\n")
        manifest_all["flights"].append(pkg)
        print(f"[{pkg['status']}] {stamp} {study} {bot}+{top}")

    (OUT_ROOT / "session_manifest.json").write_text(json.dumps(manifest_all, indent=2) + "\n")

    if all_phase_rows:
        phase_path = AN_OUT / "a8_2026-09-19_phase_metrics_all.csv"
        with open(phase_path, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=sorted({k for r in all_phase_rows for k in r}))
            w.writeheader()
            w.writerows(all_phase_rows)
        print(f"wrote {phase_path} ({len(all_phase_rows)} rows)")

        # P3 / P4 on successful flights only (scenario phase, bottom vehicle)
        import pandas as pd
        df = pd.read_csv(phase_path)
        veh = df[(df["vehicle_id"] == "cf5") & (df["phase"] == "scenario")].copy()
        veh = veh[veh["study_controller"].isin(("geometric", "full_indi"))]
        veh["controller"] = veh["study_controller"].map({
            "geometric": "geometric",
            "full_indi": "indi",
        })
        veh.to_csv(AN_OUT / "a8_2026-09-19_compare_input.csv", index=False)

        if PYENV.exists() and len(veh["study_controller"].unique()) > 1:
            agg_py = str(Path(__file__).parent / "aggregate.py")
            cmp_csv = str(AN_OUT / "a8_2026-09-19_compare_input.csv")
            r = subprocess.run(
                [str(PYENV), agg_py, cmp_csv,
                 "--metric", "pos_rmse_m", "--baseline", "geometric", "--treatment", "indi"],
                cwd=str(Path(__file__).parent), capture_output=True, text=True,
            )
            if r.stdout:
                print(r.stdout, end="")
            if r.stderr:
                print(r.stderr, end="", file=sys.stderr)
            subprocess.run([
                str(PYENV), str(Path(__file__).parent / "plot_comparison.py"),
                str(AN_OUT / "a8_2026-09-19_compare_input.csv"),
                "--metrics", "pos_rmse_m,a_res_z_rms,motor_rms_ratio,min_sep_m",
                "-o", str(AN_OUT / "figures"),
            ], check=False)

    print(f"Done. Packages: {OUT_ROOT}")


if __name__ == "__main__":
    main()
