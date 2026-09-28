#!/usr/bin/env python3
"""Count indi_gains/pos_gains setParam writes in apply('takeoff', ...) per drone.

Mirrors run_formation.py resolve() + apply() gain loops (pre-f25470a vs post-fix).
Uses the installed crazyflies.yaml (same source as run_formation on the lab PC).
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

_RAMP_CONTROLLER = 6
GEOMETRIC = {"kp_xy": 40.0, "kv_xy": 8.0, "kp_z": 48.0, "kv_z": 7.0}


def load_config():
    path = Path(get_package_share_directory("crazyflie")) / "config" / "crazyflies.yaml"
    with open(path) as f:
        cfg = yaml.safe_load(f)
    fp = cfg.get("all", {}).get("firmware_params", {})
    stab, indi = fp.get("stabilizer", {}), fp.get("indi_gains", {})
    gains = {
        k: indi[k]
        for k in (
            "kr", "kw", "kr_z", "kw_z", "kr_geo", "kw_geo", "kr_z_geo", "kw_z_geo",
            "fc_bw", "mass", "kt1", "kt2", "kt3", "kt4", "j_scale", "notch_f0", "notch_bw",
        )
        if k in indi
    }
    pos = fp.get("pos_gains", {})
    pos_gains = {k: pos[k] for k in ("kp_xy", "kp_z", "kv_xy", "kv_z") if k in pos}
    per_robot = {}
    for name, robot in cfg.get("robots", {}).items():
        rfp = robot.get("firmware_params", {})
        flat = {}
        ctrl = rfp.get("stabilizer", {}).get("controller")
        if ctrl is not None:
            flat["stabilizer.controller"] = ctrl
        for k, v in rfp.get("indi_gains", {}).items():
            flat[f"indi_gains.{k}"] = v
        for k, v in rfp.get("pos_gains", {}).items():
            flat[f"pos_gains.{k}"] = v
        if flat:
            per_robot[name] = flat
    return int(stab["controller"]), int(indi["ctrl_mode"]), gains, pos_gains, per_robot, path


def resolve(name, ctrl, mode_, pgains, per_robot):
    overrides = per_robot.get(name, {})
    eff_ctrl = int(overrides.get("stabilizer.controller", ctrl))
    eff_mode = int(overrides.get("indi_gains.ctrl_mode", mode_))
    pos = dict(pgains) if pgains else None
    if pos is not None and eff_ctrl == _RAMP_CONTROLLER and eff_mode == 0:
        pos = dict(GEOMETRIC)
    return eff_ctrl, eff_mode, pos


def count_gain_writes(name, ctrl, mode_, gains, pgains, per_robot, *, skip_non_oot6: bool):
    overrides = per_robot.get(name, {})
    _, _, pg = resolve(name, ctrl, mode_, pgains, per_robot)
    eff_ctrl, _, _ = resolve(name, ctrl, mode_, pgains, per_robot)
    push = (eff_ctrl == _RAMP_CONTROLLER) if skip_non_oot6 else True
    n = 0
    keys = []
    if push:
        for k in (gains or {}):
            key = f"indi_gains.{k}"
            if key not in overrides:
                n += 1
                keys.append(key)
        for k in (pg or {}):
            key = f"pos_gains.{k}"
            if key not in overrides:
                n += 1
                keys.append(key)
    return n, keys, eff_ctrl


def main():
    ctrl, mode_, indi_gains, pos_gains, per_robot, path = load_config()
    phase_ctrl, phase_mode = _RAMP_CONTROLLER, 0
    drones = ["cf5", "cf_second"]

    print(f"yaml: {path}")
    print(f"apply('takeoff', {phase_ctrl}, {phase_mode})  shared indi keys={len(indi_gains)} pos keys={len(pos_gains)}")
    print()

    for label, skip in [("BEFORE fix (f25470a^)", False), ("AFTER fix (f25470a)", True)]:
        print(f"=== {label} ===")
        for name in drones:
            n, keys, eff = count_gain_writes(
                name, phase_ctrl, phase_mode, indi_gains, pos_gains, per_robot,
                skip_non_oot6=skip,
            )
            print(f"  {name:12s} eff_controller={eff}  gain setParam writes={n}")
        print()

    # Controllers 7/8: same yaml override shape as 9 (only controller + ctrl_mode + rpm, no per-key indi)
    for alt in (7, 8):
        pr = dict(per_robot)
        pr["cf5"] = dict(pr.get("cf5", {}))
        pr["cf5"]["stabilizer.controller"] = alt
        n_old, _, eff = count_gain_writes(
            "cf5", phase_ctrl, phase_mode, indi_gains, pos_gains, pr, skip_non_oot6=False,
        )
        n_new, _, _ = count_gain_writes(
            "cf5", phase_ctrl, phase_mode, indi_gains, pos_gains, pr, skip_non_oot6=True,
        )
        print(f"=== cf5 @ controller={alt} (hypothetical yaml pin, same keys as c=9) ===")
        print(f"  eff_controller={eff}  BEFORE={n_old}  AFTER={n_new}")
        indi_only = [k for k in pr["cf5"] if k.startswith("indi_gains.") and k != "indi_gains.ctrl_mode"]
        print(f"  per-key indi overrides besides ctrl_mode: {indi_only or '(none)'}")
        print()

    return 0


if __name__ == "__main__":
    sys.exit(main())
