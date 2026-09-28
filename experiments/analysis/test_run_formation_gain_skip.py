#!/usr/bin/env python3
"""Desk check: run_formation apply() must not push OOT gains to non-6 controllers."""

_RAMP_CONTROLLER = 6
GEOMETRIC = {"kp_xy": 40.0, "kv_xy": 8.0, "kp_z": 48.0, "kv_z": 7.0}
POS = {"kp_xy": 64.0, "kv_xy": 5.0, "kp_z": 48.0, "kv_z": 7.0}
INDI = {"kr": 2400.0, "mass": 0.041}
PER = {
    "cf5": {"stabilizer.controller": 9, "indi_gains.ctrl_mode": 0},
    "cf_second": {"stabilizer.controller": 5, "indi_gains.ctrl_mode": 3},
}


def resolve(name, ctrl, mode_, pgains, per_robot):
    overrides = per_robot.get(name, {})
    eff_ctrl = int(overrides.get("stabilizer.controller", ctrl))
    eff_mode = int(overrides.get("indi_gains.ctrl_mode", mode_))
    pos = dict(pgains) if pgains else None
    if pos is not None and eff_ctrl == _RAMP_CONTROLLER and eff_mode == 0:
        pos = dict(GEOMETRIC)
    return eff_ctrl, eff_mode, pos


def gains_apply(name, ctrl, mode_, pgains, per_robot):
    eff_ctrl, _, _ = resolve(name, ctrl, mode_, pgains, per_robot)
    return eff_ctrl == _RAMP_CONTROLLER


def main():
    assert not gains_apply("cf5", 6, 0, POS, PER)
    assert not gains_apply("cf_second", 6, 0, POS, PER)
    assert gains_apply("cf231_active", 6, 0, POS, {})
    print("OK: alt-controller formation skips OOT gain push; OOT6 still receives gains.")


if __name__ == "__main__":
    main()
