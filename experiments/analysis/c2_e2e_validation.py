#!/usr/bin/env python3
"""Stages A, B, D of docs/40_C2_Residual_Pipeline_E2E_Validation_Plan.md."""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
RES = ROOT / "flying_drone_stack/tools/residual"
MANIFEST = ROOT / "experiments/logs/c1_2026-09-21_merged/manifest_2026-09-21_c1.json"
OUT = ROOT / "experiments/analysis/out/c2_e2e_2026-09-21"
PY = sys.executable

sys.path.insert(0, str(RES))
import dataset  # noqa: E402
import model as M  # noqa: E402
from model import firmware_forward  # noqa: E402


def c1_paths():
    m = json.loads(MANIFEST.read_text())
    return {
        f"{e['scenario']}_{e['stamp']}": ROOT / e["path"]
        for e in m
        if e["scenario"] != "A8"
    }


def stage_a(flights: dict) -> dict:
    rows = {}
    for name, path in flights.items():
        try:
            _, _, _, y, stats = dataset.build([str(path)], verbose=False)
            n = int(len(y))
        except SystemExit:
            n = 0
            stats = {"note": "dataset.build rejected (0 usable rows)"}
        rows[name] = {"path": str(path), "rows_kept": n, "stats": stats}
    usable = [k for k, v in rows.items() if v["rows_kept"] > 0]
    return {"per_flight": rows, "usable_flights": usable, "n_usable": len(usable)}


def eval_holdout(w_path: Path, holdout: Path, mass: float = M.DEFAULT_MASS) -> dict:
    from eval_model import eval_slice, predict_batch  # noqa: E402

    w = np.load(w_path)["weights"].astype(np.float64)
    rel, mask, ground, y, _ = dataset.build([str(holdout)], verbose=False)
    if len(y) == 0:
        return {"n": 0, "note": "no rows"}
    pred = predict_batch(w, rel, mask, ground, mass)
    return eval_slice(y, pred)


def stage_b(flights: dict, usable: list[str], epochs: int = 50) -> dict:
    OUT.mkdir(parents=True, exist_ok=True)
    wdir = OUT / "loo_weights"
    wdir.mkdir(exist_ok=True)
    folds = {}
    for hold in usable:
        train = [str(flights[k]) for k in usable if k != hold]
        w_path = wdir / f"train_without_{hold}.npz"
        subprocess.run(
            [PY, str(RES / "train.py"), *train, "-o", str(w_path), "--epochs", str(epochs)],
            cwd=str(ROOT), check=True,
        )
        metrics = eval_holdout(w_path, flights[hold])
        folds[hold] = {"weights": str(w_path), "held_out": hold, "metrics": metrics}
        m = metrics
        print(f"LOO holdout {hold}: n={m.get('n')} RMSE={m.get('rmse_mps2', float('nan')):.4f} "
              f"baseline_zero={m.get('baseline_zero_rmse', float('nan')):.4f} "
              f"pct={m.get('pct_reduction_vs_zero', float('nan')):.1f}%")

    a3_keys = [k for k in usable if k.startswith("A3_")]
    a1_keys = [k for k in usable if k.startswith("A1_")]
    cross = {}
    if a3_keys and a1_keys:
        for tag, train_keys, test_keys in (
            ("A3_train_A1_test", a3_keys, a1_keys),
            ("A1_train_A3_test", a1_keys, a3_keys),
        ):
            w_path = wdir / f"{tag}.npz"
            train_paths = [str(flights[k]) for k in train_keys]
            subprocess.run(
                [PY, str(RES / "train.py"), *train_paths, "-o", str(w_path),
                 "--epochs", str(epochs)],
                cwd=str(ROOT), check=True,
            )
            test_metrics = []
            for tk in test_keys:
                test_metrics.append({"flight": tk, **eval_holdout(w_path, flights[tk])})
            cross[tag] = {"weights": str(w_path), "test_flights": test_metrics}
    return {"leave_one_out": folds, "cross_scenario": cross}


def stage_d_synthetic_sweep(w_path: Path) -> dict:
    """Overhead neighbour: dp = peer - own, dp_z = dz separation (world frame)."""
    w = np.load(w_path)["weights"].astype(np.float64)
    mass = M.DEFAULT_MASS
    dz_grid = np.linspace(0.05, 1.2, 25)
    own_z = 0.8
    own_v = np.zeros(3)
    preds = []
    for dz in dz_grid:
        dp = np.array([0.0, 0.0, dz], np.float32)
        dv = np.zeros(3, np.float32)
        a, _ = firmware_forward(w, [(dp, dv)], own_z, own_v, mass=mass, apply_clamp=False)
        preds.append(float(a))
    preds = np.array(preds)
    corr = float(np.corrcoef(dz_grid, np.abs(preds))[0, 1])
    # Physical downwash: magnitude usually largest when dz is small (neighbour closer overhead).
    abs_decreases = bool(np.abs(preds[0]) >= np.abs(preds[-1]))
    return {
        "weights": str(w_path),
        "note": "Synthetic grid; thin C.1 data — extrapolation can look anti-physical.",
        "dz_separation_m": dz_grid.tolist(),
        "a_res_z_mps2": preds.tolist(),
        "abs_larger_at_small_dz": abs_decreases,
        "correlation_abs_a_vs_dz": corr,
    }


def stage_d_loo_binned(flights: dict, loo: dict) -> list[dict]:
    from eval_model import binned_error, predict_batch, neighbour_geometry  # noqa: E402

    rows = []
    for hold, fold in loo.items():
        w_path = Path(fold["weights"])
        w = np.load(w_path)["weights"].astype(np.float64)
        rel, mask, ground, y, _ = dataset.build([str(flights[hold])], verbose=False)
        if len(y) == 0:
            rows.append({"held_out": hold, "note": "no rows"})
            continue
        pred = predict_batch(w, rel, mask, ground, M.DEFAULT_MASS)
        err = pred - y
        dy, dz = neighbour_geometry(rel, mask)
        rows.append({
            "held_out": hold,
            "weights": str(w_path),
            "binned_error_dy": binned_error(dy, dz, err, "dy", np.linspace(0, 0.25, 6)),
            "binned_error_dz": binned_error(dy, dz, err, "dz", np.linspace(0.05, 1.0, 8)),
        })
    return rows


def stage_c_subprocess() -> dict:
    script = RES / "test_real_data_pipeline.py"
    r = subprocess.run([PY, str(script)], cwd=str(ROOT), capture_output=True, text=True)
    import re

    max_diff = None
    max_diff_vel = None
    for line in (r.stdout or "").splitlines():
        m = re.search(r"max \|NumPy firmware_forward - compiled\| = ([0-9.e+-]+)", line)
        if m and "differenced" not in line.lower():
            max_diff = float(m.group(1))
        if "differenced peer velocity" in line:
            m2 = re.search(r"= ([0-9.e+-]+) m/s", line)
            if m2:
                max_diff_vel = float(m2.group(1))
    note_parts = []
    if max_diff is not None:
        note_parts.append(f"max |NumPy−compiled| = {max_diff:.2e} m/s² (position / zero peer dv path)")
    if max_diff_vel is not None:
        note_parts.append(f"max |NumPy−compiled| = {max_diff_vel:.2e} m/s² (differenced peer dv)")
    return {
        "exit_code": r.returncode,
        "max_diff_mps2": max_diff,
        "max_diff_differenced_peer_vel_mps2": max_diff_vel,
        "note": "; ".join(note_parts) if note_parts else None,
        "limitation": (
            "Path A: loader positions, NumPy peer dv forced to 0 (original check). "
            "Path B: measured rel velocity via differenced oot_set_peer (100 ms). "
            "Ego velocity in Path A is 0 in the harness — see test_real_data_pipeline.py."
        ),
        "stdout_tail": r.stdout[-4000:] if r.stdout else "",
        "stderr_tail": r.stderr[-2000:] if r.stderr else "",
        "pass": r.returncode == 0,
    }


def main():
    flights = c1_paths()
    report = {"stage_a": stage_a(flights)}
    usable = report["stage_a"]["usable_flights"]
    print(f"Stage A: {report['stage_a']['n_usable']} usable flights (expect 4, not 5)")
    for k, v in report["stage_a"]["per_flight"].items():
        print(f"  {k}: {v['rows_kept']} rows")

    if len(usable) < 2:
        sys.exit("Need at least 2 usable flights for Stage B")

    report["stage_b"] = stage_b(flights, usable)

    print("Stage C: real-data loader vs compiled firmware …")
    report["stage_c"] = stage_c_subprocess()
    if not report["stage_c"]["pass"]:
        print(report["stage_c"]["stderr_tail"])
        sys.exit("Stage C failed — fix before D/E")

    a3_w = report["stage_b"]["leave_one_out"].get("A3_13-00-57", {}).get("weights")
    if not a3_w:
        a3_w = list(report["stage_b"]["leave_one_out"].values())[0]["weights"]
    report["stage_d"] = {
        "synthetic_overhead_sweep": stage_d_synthetic_sweep(Path(a3_w)),
        "held_out_binned_error": stage_d_loo_binned(
            flights, report["stage_b"]["leave_one_out"]),
    }

    stage_e_script = ROOT / "experiments/analysis/c2_stage_e_dryrun.sh"
    if stage_e_script.is_file():
        print("Stage E: CS2 SIL dry run (predict vs compensate) …")
        er = subprocess.run(["bash", str(stage_e_script)], cwd=str(ROOT),
                            capture_output=True, text=True, timeout=900)
        report["stage_e"] = {
            "exit_code": er.returncode,
            "stdout_tail": er.stdout[-6000:] if er.stdout else "",
            "stderr_tail": er.stderr[-2000:] if er.stderr else "",
            "pass": er.returncode == 0,
        }
    else:
        report["stage_e"] = {"skipped": True, "reason": f"missing {stage_e_script}"}

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "c2_validation_report.json").write_text(json.dumps(report, indent=2, default=str) + "\n")
    write_markdown_report(report)
    print(f"Wrote {OUT / 'c2_validation_report.json'}")
    return 0


def write_markdown_report(report: dict) -> None:
    """Human-readable summary for thesis / doc 30."""
    lines = [
        "# C.2 E2E validation (2026-09-21 C.1 data)",
        "",
        "Four usable flights (A1 dz 0.75 + A3×3). A1_13-25-10 contributes **0 rows**.",
        "",
        "## Stage B — LOO (headline)",
        "",
    ]
    for hold, fold in report.get("stage_b", {}).get("leave_one_out", {}).items():
        m = fold["metrics"]
        lines.append(
            f"- **{hold}**: RMSE {m['rmse_mps2']:.3f} vs predict-zero "
            f"{m['baseline_zero_rmse']:.3f} ({m['pct_reduction_vs_zero']:.1f}%)"
        )
    cross = report.get("stage_b", {}).get("cross_scenario", {})
    if cross:
        lines.extend(["", "## Stage B — cross-scenario (headline)", ""])
        a1a3 = cross.get("A1_train_A3_test", {})
        for tf in a1a3.get("test_flights", []):
            m = tf.get("metrics", tf)
            lines.append(
                f"- **Train A1 only → test {tf['flight']}**: RMSE {m['rmse_mps2']:.2f} vs "
                f"predict-zero {m['baseline_zero_rmse']:.3f} "
                f"({m['pct_reduction_vs_zero']:.0f}% — ~**{abs(m['pct_reduction_vs_zero']):.0f}% worse "
                f"than predicting zero**)"
            )
        a3a1 = cross.get("A3_train_A1_test", {})
        for tf in a3a1.get("test_flights", []):
            m = tf.get("metrics", tf)
            lines.append(
                f"- **Train A3 only → test {tf['flight']}**: RMSE {m['rmse_mps2']:.2f} vs "
                f"predict-zero {m['baseline_zero_rmse']:.3f} ({m['pct_reduction_vs_zero']:.0f}%)"
            )
    sc = report.get("stage_c", {})
    lines.extend([
        "",
        f"## Stage C — loader↔firmware: {'PASS' if sc.get('pass') else 'FAIL'}",
        "",
    ])
    if sc.get("note"):
        lines.append(f"- {sc['note']}")
    elif sc.get("max_diff_mps2") is not None:
        lines.append(f"- max |NumPy−compiled| = {sc['max_diff_mps2']:.2e} m/s²")
    if sc.get("limitation"):
        lines.append(f"- *Scope:* {sc['limitation']}")
    sd = report.get("stage_d", {}).get("synthetic_overhead_sweep", {})
    if sd:
        lines.extend([
            "",
            "## Stage D — physical plausibility",
            "",
            f"- Synthetic overhead sweep (LOO A3 weights): |a| larger at small dz = "
            f"{sd.get('abs_larger_at_small_dz')}; "
            f"corr(|a|, dz) = {sd.get('correlation_abs_a_vs_dz', float('nan')):.3f}",
            "- Held-out binned error: see JSON `stage_d.held_out_binned_error`.",
        ])
    se = report.get("stage_e", {})
    if se.get("skipped"):
        lines.append(f"\n## Stage E — skipped ({se.get('reason')})")
    elif "pass" in se:
        lines.append(f"\n## Stage E — SIL gate: {'PASS' if se['pass'] else 'INCONCLUSIVE/FAIL'}")
        if se.get("inconclusive_reason"):
            lines.append(f"- {se['inconclusive_reason']}")
        if se.get("formation_log_path_blocker"):
            lines.append(f"- {se['formation_log_path_blocker']}")
        lines.append(
            f"- Weights uploaded in sim; compare logs under `experiments/sim_validation/c2_e2e_*.csv`."
        )
    lines.extend([
        "",
        "Full JSON: `c2_validation_report.json`. Plan: `docs/40_C2_Residual_Pipeline_E2E_Validation_Plan.md`.",
    ])
    (OUT / "C2_VALIDATION_REPORT.md").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    sys.exit(main())
