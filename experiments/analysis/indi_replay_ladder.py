#!/usr/bin/env python3
"""On-policy diagnostic ladder (rung 0: a_res path; rung 1: geometric command)."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
ANALYSIS = Path(__file__).resolve().parent
BUILD = Path("/home/georg/Desktop/crazyflie-firmware/build")
sys.path.insert(0, str(ANALYSIS))
sys.path.insert(0, str(BUILD))

from indi_replay_command import (
    commanded_total_thrust_N,
    flight_mean_vbat,
    radio_csv_for_meta,
    supply_voltage_on_usd_timeline,
)
from indi_replay_config import apply_ours_globals, flight_ours_config, load_yaml_flight_params
from indi_replay_inputs import build_replay_npz
from indi_replay_worker import WARMUP_TICKS

OUT = ANALYSIS / "out" / "indi_replay_ladder"

HIGH_SIGNAL_STD = 0.15  # m/s^2

RUNG0_FLIGHTS = [
    {
        "label": "A8_17-39-27",
        "meta": REPO / "experiments/logs/A8_2026-10-05_17-39-27.meta.json",
        "usd": REPO / "experiments/logs/usd_raw/cf5_A8_thesis00_2026-10-05_17-44-01.bin",
        "yaml_rev": "c13546a",
    },
    {
        "label": "A8_17-41-09",
        "meta": REPO / "experiments/logs/A8_2026-10-05_17-41-09.meta.json",
        "usd": REPO / "experiments/logs/usd_raw/cf5_A8_thesis01_2026-10-05_17-44-01.bin",
        "yaml_rev": "c13546a",
    },
]

RUNG2_FLIGHTS = [
    {
        "label": "A8_19-09-54",
        "meta": REPO / "experiments/logs/A8_2026-10-02_19-09-54.meta.json",
        "usd": REPO / "experiments/logs/usd_raw/cf5_thesis69_2026-10-02_19-18-54.bin",
        "yaml_rev": "1a43568",
        "tag": "A8",
    },
    {
        "label": "A1_19-15-47",
        "meta": REPO / "experiments/logs/A1_2026-10-02_19-15-47.meta.json",
        "usd": REPO / "experiments/logs/usd_raw/cf5_thesis72_2026-10-02_19-18-55.bin",
        "yaml_rev": "1a43568",
        "tag": "A1",
    },
]


def commanded_thrust_on_ticks(z, meta_path: Path, ticks: np.ndarray) -> np.ndarray:
    meta = json.loads(meta_path.read_text())
    radio = radio_csv_for_meta(meta_path)
    lag = float(z["lag_s"])
    vbat = supply_voltage_on_usd_timeline(z["t_usd"], lag, radio)
    return commanded_total_thrust_N(z["motor_pwm"][ticks], vbat[ticks])


def thrust_gate_metrics(cmd: np.ndarray, rep: np.ndarray, ki_z: float, mass: float) -> dict:
    mean_l, mean_r = float(np.mean(cmd)), float(np.mean(rep))
    err_pct = abs(mean_r - mean_l) / (abs(mean_l) + 1e-12) * 100
    corr = float(np.corrcoef(cmd, rep)[0, 1]) if len(cmd) > 2 else float("nan")
    slope, intercept = (
        (float(np.polyfit(cmd, rep, 1)[0]), float(np.polyfit(cmd, rep, 1)[1])) if len(cmd) > 2 else (float("nan"), float("nan"))
    )
    mean_off = mean_r - mean_l
    out = {
        "n": int(len(cmd)),
        "mean_command_N": mean_l,
        "mean_replay_N": mean_r,
        "mean_offset_N": mean_off,
        "mean_err_pct": err_pct,
        "corr": corr,
        "regression_slope": slope,
        "regression_intercept_N": intercept,
        "implied_extra_f_d_z_mps2": mean_off / mass,
    }
    if ki_z > 0:
        out["implied_i_ez_observation_s_m"] = (mean_off / mass) / ki_z
        out["pass"] = corr > 0.95 and 0.9 <= slope <= 1.1
        out["criterion"] = "ki_z>0: corr>0.95, slope in [0.9,1.1]; mean offset reported not gated"
    else:
        out["pass"] = err_pct <= 3.0 and corr > 0.98
        out["criterion"] = "ki_z=0: mean err<=3%, corr>0.98"
    return out


def load_yaml_at_rev(rev: str) -> dict:
    import subprocess
    import yaml

    raw = subprocess.check_output(
        ["git", "-C", str(Path("/home/georg/Desktop/crazyswarm2")), "show", f"{rev}:crazyflie/config/crazyflies.yaml"],
        text=True,
    )
    doc = yaml.safe_load(raw)

    def flat(d, p=""):
        o = {}
        for k, v in (d or {}).items():
            if isinstance(v, dict):
                o.update(flat(v, f"{p}.{k}" if p else k))
            else:
                o[f"{p}.{k}" if p else k] = v
        return o

    all_p = flat(doc.get("all", {}).get("firmware_params", {}))
    cf5_p = flat(doc.get("robots", {}).get("cf5", {}).get("firmware_params", {}))
    return {**all_p, **cf5_p}


def flight_config_from_meta(meta: dict, yaml_rev: str | None = None) -> dict:
    yaml_p = load_yaml_at_rev(yaml_rev) if yaml_rev else load_yaml_flight_params()
    cfg = flight_ours_config(meta, yaml_cache=yaml_p)
    cfg["res_sign"] = int(yaml_p.get("indi_gains.res_sign", meta["per_drone"]["cf5"].get("res_sign", 1)))
    cfg["yaml_rev"] = yaml_rev or cfg["yaml_rev"]
    cfg["yaml_params_reference"] = {
        k: yaml_p[k]
        for k in sorted(yaml_p)
        if any(x in k for x in ("indi_gains", "pos_gains", "rnn", "stabilizer", "res_sign"))
    }
    return cfg


def run_ours_ticks(
    npz_path: Path,
    flight_cfg: dict,
    *,
    cache_path: Path | None = None,
    force: bool = False,
) -> dict:
    if cache_path is not None and cache_path.exists() and not force:
        zc = np.load(cache_path)
        return {
            "thrust_si": zc["thrust_si"],
            "a_res": zc["a_res"],
            "tau": zc["tau"],
            "ticks": zc["ticks"],
            "globals_applied": json.loads(str(zc["globals_applied"])),
        }

    import cffirmware as fw  # noqa: E402

    z = np.load(npz_path)
    n = len(z["pos"])
    start = int(z["warmup_start"]) + WARMUP_TICKS
    start = min(start, n - 1)

    globals_applied = apply_ours_globals(fw, flight_cfg, "L0")
    fw.controllerOutOfTreeInit()

    sp = fw.setpoint_t()
    sens = fw.sensorData_t()
    st = fw.state_t()
    ctl = fw.control_t()

    thrust_r, a_res_r, tau_r = [], [], []
    for tick in range(n):
        i = tick
        p = z["pos"][i]
        v = z["vel"][i]
        q = z["quat"][i]
        spp = z["sp_pos"][i]
        sp.position.x, sp.position.y, sp.position.z = float(spp[0]), float(spp[1]), float(spp[2])
        sp_vel = z["sp_vel"][i]
        sp.velocity.x, sp.velocity.y, sp.velocity.z = float(sp_vel[0]), float(sp_vel[1]), float(sp_vel[2])
        sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = (
            float(z["sp_acc"][i, 0]),
            float(z["sp_acc"][i, 1]),
            float(z["sp_acc"][i, 2]),
        )
        sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs
        sp.mode.yaw = fw.modeAbs
        sp.attitude.yaw = math.degrees(float(z["yaw_d_rad"][i]))

        sens.gyro.x, sens.gyro.y, sens.gyro.z = (
            float(z["gyro_deg_s"][i, 0]),
            float(z["gyro_deg_s"][i, 1]),
            float(z["gyro_deg_s"][i, 2]),
        )
        ag = z["acc_g"][i]
        sens.acc.x, sens.acc.y, sens.acc.z = float(ag[0]), float(ag[1]), float(ag[2])

        st.position.x, st.position.y, st.position.z = float(p[0]), float(p[1]), float(p[2])
        st.velocity.x, st.velocity.y, st.velocity.z = float(v[0]), float(v[1]), float(v[2])
        st.attitudeQuaternion.w = float(q[0])
        st.attitudeQuaternion.x = float(q[1])
        st.attitudeQuaternion.y = float(q[2])
        st.attitudeQuaternion.z = float(q[3])

        rpms = z["rpm"][i]
        fw.oot_set_rpm(int(rpms[0]), int(rpms[1]), int(rpms[2]), int(rpms[3]))
        fw.controllerOutOfTree(ctl, sp, sens, st, tick)

        if tick < start:
            continue
        thrust_r.append(float(ctl.thrustSi))
        a_res_r.append([fw.oot_get_a_res(j) for j in range(3)])
        tau_r.append([float(ctl.torqueX), float(ctl.torqueY), float(ctl.torqueZ)])

    out = {
        "thrust_si": np.asarray(thrust_r),
        "a_res": np.asarray(a_res_r),
        "tau": np.asarray(tau_r),
        "ticks": np.arange(start, n),
        "globals_applied": globals_applied,
    }
    if cache_path is not None:
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            cache_path,
            thrust_si=out["thrust_si"],
            a_res=out["a_res"],
            tau=out["tau"],
            ticks=out["ticks"],
            globals_applied=json.dumps(globals_applied),
        )
    return out


def axis_gate(logged: np.ndarray, replay: np.ndarray) -> dict:
    if len(logged) < 3:
        return {
            "n": len(logged),
            "gate": "insufficient",
            "pass": False,
            "corr": float("nan"),
            "std_log": float("nan"),
            "mean_abs_err": float("nan"),
            "rms_err": float("nan"),
            "mean_err_pct": float("nan"),
        }
    mean_l = float(np.mean(logged))
    std_l = float(np.std(logged))
    mean_abs_err = float(np.mean(np.abs(replay - logged)))
    rms_err = float(np.sqrt(np.mean((replay - logged) ** 2)))
    corr = float(np.corrcoef(logged, replay)[0, 1])
    mean_err_pct = abs(float(np.mean(replay)) - mean_l) / (abs(mean_l) + 1e-12) * 100
    mean_bias = abs(float(np.mean(replay)) - mean_l)
    if std_l >= HIGH_SIGNAL_STD:
        passed = corr > 0.99 and mean_err_pct <= 2.0
        gate = "high_signal"
    else:
        passed = mean_bias <= 0.02 and rms_err <= 0.12
        gate = "low_signal"
    return {
        "n": int(len(logged)),
        "gate": gate,
        "pass": passed,
        "std_log": std_l,
        "mean_log": mean_l,
        "mean_rep": float(np.mean(replay)),
        "corr": corr,
        "mean_abs_err": mean_abs_err,
        "mean_bias_mps2": mean_bias,
        "rms_err": rms_err,
        "mean_err_pct": mean_err_pct,
    }


def steady_indices(z: np.lib.npyio.NpzFile, ticks: np.ndarray) -> np.ndarray:
    sp_z = z["sp_pos"][:, 2]
    warmup_start = int(z["warmup_start"])
    mask = []
    for t in ticks:
        if t < warmup_start + WARMUP_TICKS:
            continue
        if float(sp_z[t]) >= 0.9 * float(np.max(sp_z)):
            mask.append(int(t))
    return np.asarray(mask, int)


def ensure_npz(spec: dict, run_dir: Path) -> Path:
    run_dir.mkdir(parents=True, exist_ok=True)
    npz = run_dir / "inputs.npz"
    meta = json.loads(spec["meta"].read_text())
    if not npz.exists():
        build_replay_npz(spec["usd"], spec["meta"], role="bottom", out_npz=npz)
    z = np.load(npz)
    if "a_res_log" not in z.files:
        sys.path.insert(0, str(REPO / "flying_drone_stack/tools"))
        from decode_usd_log import load as load_usd  # noqa: E402

        d = load_usd(str(spec["usd"]))
        ax = d.get("a_res_x")
        if ax is not None:
            lag = float(z["lag_s"])
            t = np.asarray(d["t"], float)
            total = float(meta["duration"])
            margin = 3.0
            lo, hi = lag - margin, lag + total + margin
            mask = (t >= lo) & (t <= hi)
            a_log = np.stack([d["a_res_x"], d["a_res_y"], d["a_res_z"]], axis=1)
            a_log = np.repeat(a_log[mask], 2, axis=0)
            if len(a_log) != len(z["pos"]):
                raise RuntimeError(f"a_res length {len(a_log)} != npz {len(z['pos'])}")
            np.savez_compressed(npz, **{k: z[k] for k in z.files}, a_res_log=a_log)
            z = np.load(npz)
    return npz


def rung0_one(spec: dict, *, force_replay: bool = False) -> dict:
    meta = json.loads(spec["meta"].read_text())
    run_dir = OUT / "rung0" / spec["label"]
    npz = ensure_npz(spec, run_dir)
    flight_cfg = flight_config_from_meta(meta, spec.get("yaml_rev"))
    (run_dir / "flight_config.json").write_text(json.dumps(flight_cfg, indent=2))

    cache = run_dir / "replay_cache.npz"
    rep = run_ours_ticks(npz, flight_cfg, cache_path=cache, force=force_replay)
    z = np.load(npz)
    ticks = rep["ticks"]
    steady = steady_indices(z, ticks)
    idx_map = {int(t): i for i, t in enumerate(ticks)}
    steady_500 = steady[steady % 2 == 0]
    sel = [idx_map[t] for t in steady_500 if t in idx_map]
    log_a = z["a_res_log"][steady_500]
    rep_a = rep["a_res"][sel]

    axes = {name: axis_gate(log_a[:, j], rep_a[:, j]) for j, name in enumerate("xyz")}
    pass_rung = all(axes[a]["pass"] for a in "xyz")

    return {
        "label": spec["label"],
        "ctrl_mode": flight_cfg["ctrl_mode"],
        "globals_applied": rep["globals_applied"],
        "yaml_rev": flight_cfg.get("yaml_rev"),
        "yaml_params_reference": flight_cfg.get("yaml_params_reference"),
        "meta_pos": meta["per_drone"]["cf5"]["pos"],
        "meta_indi": {k: meta["per_drone"]["cf5"]["indi"][k] for k in meta["per_drone"]["cf5"]["indi"] if k in ("mass", "kr", "kw", "fc_bw", "rpm_source", "kt1")},
        "res_sign": flight_cfg.get("res_sign"),
        "steady_n": int(len(steady)),
        "axes": axes,
        "pass": pass_rung,
        "criterion": f"high-signal (std>={HIGH_SIGNAL_STD}): corr>0.99, mean err<=2%; low-signal: |mean err|<=0.02 m/s^2, RMS<=0.12 m/s^2",
    }


def rpm_implied_thrust(rpm: np.ndarray, kt: np.ndarray) -> float:
    return float(np.sum(kt * (rpm.astype(float) ** 2)))


def rung1_one(spec: dict, *, force_replay: bool = False) -> dict:
    run_dir = OUT / "rung0" / spec["label"]
    r0 = rung0_one(spec, force_replay=force_replay)

    meta = json.loads(spec["meta"].read_text())
    npz = run_dir / "inputs.npz"
    z = np.load(npz)
    flight_cfg = json.loads((run_dir / "flight_config.json").read_text())
    rep = run_ours_ticks(npz, flight_cfg, cache_path=run_dir / "replay_cache.npz", force=False)

    meta_cf5 = meta["per_drone"]["cf5"]
    kt = np.array([meta_cf5["indi"][f"kt{i}"] for i in range(1, 5)])

    ticks = rep["ticks"]
    steady = steady_indices(z, ticks)
    idx_map = {int(t): i for i, t in enumerate(ticks)}
    sel = [idx_map[t] for t in steady if t in idx_map]

    cmd_th = commanded_thrust_on_ticks(z, spec["meta"], steady)
    rep_th = rep["thrust_si"][sel]
    rpm_th = np.array([rpm_implied_thrust(z["rpm"][t], kt) for t in steady])

    ki_z = float(meta_cf5.get("pos", {}).get("ki_z") or 0.0)
    mass = float(meta_cf5["indi"]["mass"])
    thrust = thrust_gate_metrics(cmd_th, rep_th, ki_z, mass)
    radio = radio_csv_for_meta(spec["meta"])
    lag = float(z["lag_s"])
    dur = float(meta["duration"])
    thrust["vbat_flight_mean_V"] = flight_mean_vbat(radio, lag, lag + dur)
    thrust["vbat_sensitivity_N"] = {
        "vbat_minus_0.1V_mean_cmd": float(np.mean(commanded_total_thrust_N(z["motor_pwm"][steady], thrust["vbat_flight_mean_V"] - 0.1))),
        "vbat_plus_0.1V_mean_cmd": float(np.mean(commanded_total_thrust_N(z["motor_pwm"][steady], thrust["vbat_flight_mean_V"] + 0.1))),
    }

    sys.path.insert(0, str(REPO / "flying_drone_stack/tools"))
    from decode_usd_log import load as load_usd  # noqa: E402

    d = load_usd(str(spec["usd"]))
    tau_log = np.stack([d["tau_x"], d["tau_y"], d["tau_z"]], axis=1)
    t = np.asarray(d["t"], float)
    lag = float(z["lag_s"])
    total = float(meta["duration"])
    margin = 3.0
    mask = (t >= lag - margin) & (t <= lag + total + margin)
    tau_log = np.repeat(tau_log[mask], 2, axis=0)[steady]

    rep_tau = rep["tau"][sel]
    tau_corrs = {}
    for j, name in enumerate("xyz"):
        tau_corrs[name] = float(np.corrcoef(tau_log[:, j], rep_tau[:, j])[0, 1]) if len(steady) > 2 else float("nan")

    pass_rung = thrust["pass"]

    return {
        "label": spec["label"],
        "ki_z": ki_z,
        "thrust_vs_command": thrust,
        "command_vs_rpm_implied_ratio_mean": float(np.mean(cmd_th / (rpm_th + 1e-12))),
        "command_vs_rpm_implied_ratio_median": float(np.median(cmd_th / (rpm_th + 1e-12))),
        "tau_corr": tau_corrs,
        "tau_note": "logged indi.tau_* often zero on these uSD bins",
        "pass": pass_rung,
        "rung0": r0,
    }


def rung2_one(spec: dict, *, force_replay: bool = False) -> dict:
    from indi_replay_rung2_forensic import git_audit, run_forensic_ticks, a1_segment_metrics

    run_dir = OUT / "rung2" / spec["label"]
    run_dir.mkdir(parents=True, exist_ok=True)
    npz = ensure_npz(spec, run_dir)
    meta = json.loads(spec["meta"].read_text())
    flight_cfg = flight_config_from_meta(meta, spec.get("yaml_rev"))
    (run_dir / "flight_config.json").write_text(json.dumps(flight_cfg, indent=2))

    cache = run_dir / "replay_cache.npz"
    rep = run_ours_ticks(npz, flight_cfg, cache_path=cache, force=force_replay)
    z = np.load(npz)
    ticks = rep["ticks"]
    steady = steady_indices(z, ticks)
    idx_map = {int(t): i for i, t in enumerate(ticks)}
    sel = [idx_map[t] for t in steady if t in idx_map]

    cmd_th = commanded_thrust_on_ticks(z, spec["meta"], steady)
    rep_th = rep["thrust_si"][sel]
    ki_z = float(meta["per_drone"]["cf5"].get("pos", {}).get("ki_z") or 0.0)
    mass = float(meta["per_drone"]["cf5"]["indi"]["mass"])
    thrust = thrust_gate_metrics(cmd_th, rep_th, ki_z, mass)

    forensics = run_forensic_ticks(spec, npz, flight_cfg, run_dir)
    segments = a1_segment_metrics(spec, z, rep, steady) if spec["label"].startswith("A1") else None

    return {
        "label": spec["label"],
        "ctrl_mode": flight_cfg["ctrl_mode"],
        "ki_z": ki_z,
        "git_audit": git_audit(),
        "thrust_vs_command": thrust,
        "forensics": forensics,
        "a1_segments": segments,
        "pass": thrust["pass"],
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rung", type=int, choices=(0, 1, 2), default=0)
    ap.add_argument("--force-replay", action="store_true")
    ap.add_argument("--no-stop", action="store_true", help="Continue ladder after a failed flight")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    results = []
    flights = RUNG0_FLIGHTS if args.rung in (0, 1) else RUNG2_FLIGHTS
    for spec in flights:
        if args.rung == 0:
            results.append(rung0_one(spec, force_replay=args.force_replay))
        elif args.rung == 1:
            results.append(rung1_one(spec, force_replay=args.force_replay))
        else:
            results.append(rung2_one(spec, force_replay=args.force_replay))
        print(json.dumps(results[-1], indent=2))
        if not results[-1].get("pass") and not args.no_stop:
            print(f"STOP: rung {args.rung} failed on {spec['label']}", file=sys.stderr)
            break
    (OUT / f"rung{args.rung}_summary.json").write_text(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
