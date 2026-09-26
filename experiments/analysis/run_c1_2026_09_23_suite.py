#!/usr/bin/env python3
"""Desk analysis pipeline for 2026-09-23 C.1 verified merges.

Reads manifest_2026-09-23_c1.json only — does not re-merge or modify merged CSVs.
Filters: merge_status == \"merged\" AND training_eligible == true (19 flights as of desk pass).
All flights are geometric on cf5 (study); phase-metrics rows use controller=geometric throughout.

    ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/run_c1_2026_09_23_suite.py
"""
from __future__ import annotations

import csv
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "experiments/logs/c1_2026-09-23_merged/manifest_2026-09-23_c1.json"
MERGED_ROOT = ROOT / "experiments/logs/c1_2026-09-23_merged"
META_FALLBACK = ROOT / "experiments/logs"
AN_OUT = ROOT / "experiments/analysis/out/c1_2026-09-23"
PYENV = Path.home() / ".pyenv/versions/flying_robots/bin/python"
CTRL = "geometric"
DATE = "2026-09-23"

sys.path.insert(0, str(Path(__file__).resolve().parent))
import metrics as M  # noqa: E402

sys.path.insert(0, str(Path("/home/georg/Desktop/crazyswarm2/crazyflie_examples")))


def resolve_vehicle_names(raw: dict, meta: dict) -> dict:
    """Map meta names (cf5, cf_second) to merge_usd suffixed columns."""
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


def phase_metrics(merged: Path, meta_path: Path) -> list[dict]:
    from crazyflie_examples.formations import scenarios as S

    import numpy as np

    meta = json.load(open(meta_path))
    sc = S.build(meta["scenario"], **meta["params"])
    vehicles = resolve_vehicle_names(M.load_merged_csv(merged), meta)
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
        rows.extend(
            M.vehicle_metrics_by_phase(
                v, sc, meta["scenario"], CTRL, len(meta["names"]), pos_des
            )
        )
    if len(meta["names"]) >= 2:
        rows.append(
            M.formation_row(
                meta["scenario"],
                CTRL,
                [vehicles[n] for n in meta["names"] if n in vehicles],
                meta["params"].get("dz"),
            )
        )
    for r in rows:
        r["run"] = meta_path.stem
        r["study_controller"] = CTRL
        r["controller"] = CTRL
        vid = r.get("vehicle_id", "")
        if isinstance(vid, str):
            if vid.startswith("cf5_"):
                r["vehicle_id"] = "cf5"
            elif vid.startswith("cf_second_"):
                r["vehicle_id"] = "cf_second"
    return rows


def dz_cmd_from_meta(meta: dict) -> float | None:
    p = meta.get("params") or {}
    if "dz" in p:
        return float(p["dz"])
    return None


def meta_path_for(entry: dict) -> Path:
    sc, stamp = entry["scenario"], entry["stamp"]
    fdir = MERGED_ROOT / f"{sc}_{DATE}_{stamp}"
    p = fdir / f"{sc}_{DATE}_{stamp}.meta.json"
    if p.is_file():
        return p
    p2 = META_FALLBACK / f"{sc}_{DATE}_{stamp}.meta.json"
    return p2


def main() -> int:
    if not MANIFEST.is_file():
        sys.exit(f"missing manifest: {MANIFEST}")
    all_entries = json.loads(MANIFEST.read_text())
    entries = [
        e
        for e in all_entries
        if e.get("merge_status") == "merged" and e.get("training_eligible") is True
    ]
    print(f"[c1_2026_09_23] {len(entries)} training-eligible merged flights")
    AN_OUT.mkdir(parents=True, exist_ok=True)
    all_phase: list[dict] = []
    session = {"flights": [], "manifest": str(MANIFEST), "n_entries": len(entries)}

    analysis_py = Path(__file__).parent / "run_analysis.py"
    plot_flight = Path(__file__).parent / "plot_flight.py"
    plot_ix = Path(__file__).parent / "plot_interaction.py"

    failures: list[str] = []
    ok_count = 0

    for entry in entries:
        sc, stamp = entry["scenario"], entry["stamp"]
        merged = ROOT / entry["path"]
        meta_path = meta_path_for(entry)
        fdir = merged.parent
        analysis_out = fdir / "analysis"
        analysis_out.mkdir(parents=True, exist_ok=True)

        pkg = {
            "scenario": sc,
            "stamp": stamp,
            "merged": str(merged),
            "meta": str(meta_path),
        }
        if not merged.is_file():
            pkg["status"] = "missing_merged_csv"
            failures.append(f"{sc}_{stamp}: missing merged CSV")
            session["flights"].append(pkg)
            continue
        if not meta_path.is_file():
            pkg["status"] = "missing_meta"
            failures.append(f"{sc}_{stamp}: missing meta.json")
            session["flights"].append(pkg)
            continue

        meta = json.loads(meta_path.read_text())
        dz = dz_cmd_from_meta(meta)
        pkg["dz_cmd"] = dz
        n_names = len(meta.get("names") or [])

        run_args = [
            sys.executable,
            str(analysis_py),
            "--scenario",
            sc,
            "--ctrl",
            CTRL,
            "--logs",
            str(merged),
        ]
        if dz is not None:
            run_args.extend(["--dz-cmd", str(dz)])
        run_args.extend(
            [
                "--sidecar",
                str(meta_path),
                "--source",
                "hardware",
                "--out",
                str(analysis_out),
            ]
        )

        r = subprocess.run(run_args, capture_output=True, text=True)
        pkg["run_analysis_ok"] = r.returncode == 0
        if r.returncode != 0:
            pkg["run_analysis_tail"] = (r.stdout + r.stderr)[-1500:]
            failures.append(f"{sc}_{stamp}: run_analysis failed")

        if PYENV.is_file():
            pf_args = [
                str(PYENV),
                str(plot_flight),
                "--scenario",
                sc,
                "--ctrl",
                CTRL,
                "--logs",
                str(merged),
                "--sidecar",
                str(meta_path),
                "--source",
                "hardware",
                "--out",
                str(analysis_out),
            ]
            if dz is not None:
                pf_args.extend(["--dz-cmd", str(dz)])
            pr = subprocess.run(pf_args, capture_output=True, text=True)
            pkg["plot_flight_ok"] = pr.returncode == 0
            if n_names >= 2:
                pi = subprocess.run(
                    [
                        str(PYENV),
                        str(plot_ix),
                        str(merged),
                        "--bottom",
                        "cf5",
                        "--top",
                        "cf_second",
                        "--out",
                        str(analysis_out / f"{sc}_{DATE}_{stamp}_interaction.png"),
                    ],
                    capture_output=True,
                    text=True,
                )
                pkg["plot_interaction_ok"] = pi.returncode == 0
            else:
                pkg["plot_interaction_ok"] = None
                pkg["plot_interaction_skipped"] = "solo"
        else:
            failures.append(f"{sc}_{stamp}: pyenv flying_robots missing — no plots")

        dash = list(analysis_out.glob("*_dashboard.png"))
        pkg["dashboards"] = [p.name for p in dash]
        if not dash:
            failures.append(f"{sc}_{stamp}: no dashboard png produced")

        try:
            rows = phase_metrics(merged, meta_path)
            if not rows and n_names >= 1:
                failures.append(f"{sc}_{stamp}: phase_metrics returned 0 rows (vehicle resolve?)")
            all_phase.extend(rows)
            pm = analysis_out / f"{sc}_{DATE}_{stamp}_phase_metrics.csv"
            with open(pm, "w", newline="") as fh:
                if rows:
                    w = csv.DictWriter(fh, fieldnames=sorted({k for r in rows for k in r}))
                    w.writeheader()
                    w.writerows(rows)
            pkg["phase_metrics_rows"] = len(rows)
            analysis_ok = pkg.get("run_analysis_ok") and len(rows) > 0
            pkg["status"] = "ok" if analysis_ok else "analysis_failed"
            if analysis_ok:
                ok_count += 1
        except Exception as e:
            pkg["status"] = "phase_metrics_failed"
            pkg["phase_metrics_error"] = str(e)
            failures.append(f"{sc}_{stamp}: phase_metrics: {e}")

        session["flights"].append(pkg)
        print(f"[{pkg.get('status')}] {sc} {stamp} dz={dz} vehicles={n_names}")

    summary = {
        "n_entries": len(entries),
        "n_ok": ok_count,
        "n_fail": len(entries) - ok_count,
        "failures": failures,
    }
    (AN_OUT / "session_manifest.json").write_text(json.dumps(session, indent=2) + "\n")
    (AN_OUT / "run_summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    if all_phase:
        phase_path = AN_OUT / "c1_2026-09-23_phase_metrics_all.csv"
        with open(phase_path, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=sorted({k for r in all_phase for k in r}))
            w.writeheader()
            w.writerows(all_phase)
        print(f"wrote {phase_path} ({len(all_phase)} rows)")

    print(f"\nSummary: {ok_count}/{len(entries)} flights OK")
    if failures:
        print("\nFailures / warnings:")
        for f in failures:
            print(f"  - {f}")
        (AN_OUT / "failures.txt").write_text("\n".join(failures) + "\n")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
