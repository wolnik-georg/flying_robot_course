#!/usr/bin/env python3
"""Equal-parameter input replay: ours, Omar C and the Tal & Karaman controllers on the SAME logged inputs, parameters harmonised step by step.

Why: docs/64 compared the three controllers with their native gains and filters, and docs/64 closure 2 harmonised only the mass and kt. If the outputs
still differ when every settable gain, constant and filter of one controller is set to the other's value, the remaining difference is the law itself
(structure / implementation). This script walks that ladder one parameter group at a time and measures the distance to Omar C after every step.

  python3 experiments/analysis/indi_replay_equal.py --inputs experiments/analysis/out/indi_replay/A8 [--jobs 8] [--tag A8]

Output (out/indi_replay_equal/<tag>/): runs/*.csv, summary.json, summary.md.  Reference = Omar C native (attitude integral K_I 0.03 as shipped) and Omar C without K_I.
System python3 (host bindings are the 3.10 extension), PYTHONPATH handled inside the worker."""
from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
WORKER = HERE / "indi_replay_equal_worker.py"

# --- Omar C constants (controller_omar_indi.c, printed from the host binding) ---
OM_MASS = 0.0427
OM_KT = 4.2899225838333166e-10          # oot_omar_kt_equiv()
OM_J = (16.571710e-6, 16.655602e-6, 29.261652e-6)
OM_KR = (0.007, 0.007, 0.008)           # N m / rad
OM_KW = (0.00115, 0.00115, 0.002)       # N m s / rad
OM_KPOS_P, OM_KPOS_D = 7.0, 4.0
# --- ours (lib.rs) ---
OURS_J = (23.951e-6, 23.951e-6, 32.347e-6)
# --- Tal & Karaman C / Rust inertia (controller_tk_indi.c, tk_indi.rs) ---
TK_J = (23.951e-6, 23.951e-6, 32.347e-6)

J_SCALE_XY = float(np.mean([OM_J[0] / OURS_J[0], OM_J[1] / OURS_J[1]]))     # 0.693 (z would need 0.905, one scalar only)
KR_XY = float(np.mean([OM_KR[0] / OM_J[0], OM_KR[1] / OM_J[1]]))            # s^-2, torque-equivalent with Omar's J
KW_XY = float(np.mean([OM_KW[0] / OM_J[0], OM_KW[1] / OM_J[1]]))
KR_Z, KW_Z = OM_KR[2] / OM_J[2], OM_KW[2] / OM_J[2]


def ours_ladder() -> list[tuple[str, dict]]:
    """Cumulative overrides on top of the flown configuration."""
    steps: list[tuple[str, dict]] = [("ours_L0_native", {})]
    cum: dict = {}

    def add(name: str, upd: dict) -> None:
        cum.update(upd)
        steps.append((name, dict(cum)))

    add("ours_L1_mass", {"g_indi_mass": OM_MASS})
    add("ours_L2_kt", {f"g_indi_kt{i}": OM_KT for i in range(1, 5)})
    add("ours_L3_position_gains", {"g_kp_xy": OM_KPOS_P, "g_kp_z": OM_KPOS_P, "g_kv_xy": OM_KPOS_D, "g_kv_z": OM_KPOS_D, "g_ki_z": 0.0})
    add("ours_L4_residual_sign", {"g_indi_res_sign": -1})
    add("ours_L5_attitude_gains", {"g_indi_kr": KR_XY, "g_indi_kw": KW_XY, "g_indi_kr_z": KR_Z, "g_indi_kw_z": KW_Z})
    add("ours_L6_inertia", {"g_indi_j_scale": J_SCALE_XY})
    add("ours_L7_filters", {"g_indi_fc_bw": 30.0, "g_indi_res_fc": 30.0, "g_indi_filt_order": 0, "g_indi_notch_en": 0})
    add("ours_L8_limits_off", {"g_indi_clamp_en": 0, "g_indi_tau_clamp": 0.0, "g_indi_res_clamp": 0.0})
    return steps


def tk_variants(flight_indi: dict | None = None) -> list[tuple[str, str, dict]]:
    # "native" = the paper gains and 30 Hz with the FLOWN mass and kt of the drone (the host defaults 0.0364 kg / 1.48e-10 are not what the yaml sets)
    fl = {}
    if flight_indi:
        fl = {"g_indi_mass": float(flight_indi["mass"]), **{f"g_indi_kt{i}": float(flight_indi[f"kt{i}"]) for i in range(1, 5)}}
    omar_common = {"g_indi_mass": OM_MASS, **{f"g_indi_kt{i}": OM_KT for i in range(1, 5)},
                   "g_tk_kx_xy": OM_KPOS_P, "g_tk_kx_z": OM_KPOS_P, "g_tk_kv_xy": OM_KPOS_D, "g_tk_kv_z": OM_KPOS_D,
                   "g_tk_ka_xy": 0.0, "g_tk_ka_z": 0.0, "g_tk_jrz": 0.0, "g_tk_fc": 30.0}
    torque = {"g_tk_kxi_xy": float(np.mean([OM_KR[0], OM_KR[1]])) / TK_J[0], "g_tk_kom_xy": float(np.mean([OM_KW[0], OM_KW[1]])) / TK_J[0],
              "g_tk_kxi_z": OM_KR[2] / TK_J[2], "g_tk_kom_z": OM_KW[2] / TK_J[2]}
    accel = {"g_tk_kxi_xy": KR_XY, "g_tk_kom_xy": KW_XY, "g_tk_kxi_z": KR_Z, "g_tk_kom_z": KW_Z}
    return [("tk_rust_native", "tk_rust", dict(fl)), ("tk_c_native", "tk_c", dict(fl)),
            ("tk_rust_omar_values_torque", "tk_rust", {**omar_common, **torque}), ("tk_c_omar_values_torque", "tk_c", {**omar_common, **torque}),
            ("tk_rust_omar_values_accel", "tk_rust", {**omar_common, **accel})]


def run_jobs(jobs: list[dict], inputs: Path, runs: Path, flight_cfg: Path, n_jobs: int) -> None:
    runs.mkdir(parents=True, exist_ok=True)

    def one(j: dict) -> str:
        out = runs / f"{j['name']}.csv"
        if out.is_file() and not j.get("force"):
            return f"{j['name']}: cached"
        ovp = runs / f"{j['name']}.overrides.json"
        ovp.write_text(json.dumps(j["overrides"], indent=1))
        r = subprocess.run([sys.executable, str(WORKER), j["side"], str(inputs / "inputs.npz"), str(out), str(ovp), str(flight_cfg)], capture_output=True, text=True)
        if r.returncode != 0:
            return f"{j['name']}: FAILED {r.stderr.strip()[-300:]}"
        return f"{j['name']}: ok"

    with ThreadPoolExecutor(max_workers=n_jobs) as ex:
        for msg in ex.map(one, jobs):
            print(msg, flush=True)


def load(p: Path) -> dict[int, np.ndarray]:
    with p.open() as f:
        return {int(r["tick"]): np.array([float(r["thrust_si"]), float(r["tau_x"]), float(r["tau_y"]), float(r["tau_z"])]) for r in csv.DictReader(f)}


def compare(a: dict, b: dict) -> dict:
    ks = sorted(set(a) & set(b))
    A = np.array([a[k] for k in ks]); B = np.array([b[k] for k in ks])
    out = {"n": len(ks)}
    for i, nm in enumerate(("thrust_N", "tau_x", "tau_y", "tau_z")):
        d = A[:, i] - B[:, i]
        c = float(np.corrcoef(A[:, i], B[:, i])[0, 1]) if A[:, i].std() > 0 and B[:, i].std() > 0 else float("nan")
        out[nm] = {"rms_diff": float(np.sqrt(np.mean(d ** 2))), "mean_diff": float(d.mean()), "corr": c, "rms_a": float(np.sqrt(np.mean(A[:, i] ** 2))), "rms_b": float(np.sqrt(np.mean(B[:, i] ** 2)))}
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--inputs", required=True, type=Path)
    ap.add_argument("--tag", default=None)
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--flight-config", type=Path, default=None)
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()
    tag = a.tag or a.inputs.name
    base = HERE / "out" / "indi_replay_equal" / tag
    runs = base / "runs"
    fcfg = a.flight_config or (a.inputs / "flight_config.json")

    jobs: list[dict] = [
        {"name": "omar_c_native", "side": "omar_c", "overrides": {}},
        {"name": "omar_c_noKI", "side": "omar_c", "overrides": {"omar": {"KI": [0.0, 0.0, 0.0]}}},
        {"name": "omar_c_with_ours_gains", "side": "omar_c", "overrides": {"omar": {
            "mass": 0.041, "J": list(OURS_J), "Kpos_P": [64.0, 64.0, 48.0], "Kpos_D": [5.0, 5.0, 7.0],
            "KR": [2400.0 * OURS_J[0], 2400.0 * OURS_J[1], 2400.0 * OURS_J[2]], "Komega": [170.0 * OURS_J[0], 170.0 * OURS_J[1], 170.0 * OURS_J[2]], "KI": [0.0, 0.0, 0.0]}}},
        {"name": "omar_rust_native", "side": "omar_rust", "overrides": {}},
    ]
    for nm, ov in ours_ladder():
        jobs.append({"name": nm, "side": "ours", "overrides": {"ours_native": True, "cvar": ov}})
    fcfg_json = json.loads(fcfg.read_text())
    for nm, side, ov in tk_variants(fcfg_json["indi"]):
        jobs.append({"name": nm, "side": side, "overrides": {"cvar": ov}})
    for j in jobs:
        j["force"] = a.force
    run_jobs(jobs, a.inputs, runs, fcfg, a.jobs)

    data = {j["name"]: load(runs / f"{j['name']}.csv") for j in jobs if (runs / f"{j['name']}.csv").is_file()}
    summary = {"inputs": str(a.inputs), "constants": {"omar_mass": OM_MASS, "omar_kt_equiv": OM_KT, "j_scale_xy": J_SCALE_XY, "kr_xy": KR_XY, "kw_xy": KW_XY, "kr_z": KR_Z, "kw_z": KW_Z},
               "vs_omar_c_native": {}, "vs_omar_c_noKI": {}, "pairs": {}}
    for nm, d in data.items():
        summary["vs_omar_c_native"][nm] = compare(d, data["omar_c_native"])
        summary["vs_omar_c_noKI"][nm] = compare(d, data["omar_c_noKI"])
    for x, y in (("omar_c_native", "omar_rust_native"), ("tk_rust_native", "tk_c_native"), ("tk_rust_omar_values_torque", "tk_c_omar_values_torque"), ("ours_L0_native", "ours_L8_limits_off")):
        if x in data and y in data:
            summary["pairs"][f"{x} vs {y}"] = compare(data[x], data[y])
    (base / "summary.json").write_text(json.dumps(summary, indent=1))

    def row(nm: str, ref: str) -> str:
        c = summary[ref][nm]
        t = c["thrust_N"]
        return "| %s | %.4f | %+.4f | %.2f | %.5f / %.5f / %.5f | %.2f / %.2f / %.2f |" % (
            nm, t["rms_diff"], t["mean_diff"], t["corr"], c["tau_x"]["rms_diff"], c["tau_y"]["rms_diff"], c["tau_z"]["rms_diff"], c["tau_x"]["corr"], c["tau_y"]["corr"], c["tau_z"]["corr"])

    lines = ["# Equal-parameter replay (generated)", "", f"Inputs: `{a.inputs}`; open loop, same logged inputs for every controller; differences are *controller output − Omar C output*.", "",
             "| run | thrust RMS diff [N] | thrust mean diff [N] | thrust corr | torque RMS diff x / y / z [N m] | torque corr x / y / z |", "|---|---|---|---|---|---|"]
    for ref, title in (("vs_omar_c_native", "reference: Omar C native (K_I 0.03)"), ("vs_omar_c_noKI", "reference: Omar C with K_I = 0")):
        lines += ["", f"**{title}**", "", "| run | thrust RMS diff [N] | thrust mean diff [N] | thrust corr | torque RMS diff x / y / z [N m] | torque corr x / y / z |", "|---|---|---|---|---|---|"]
        for nm in data:
            lines.append(row(nm, ref))
    (base / "summary.md").write_text("\n".join(lines) + "\n")
    print("wrote", base / "summary.md")


if __name__ == "__main__":
    main()
