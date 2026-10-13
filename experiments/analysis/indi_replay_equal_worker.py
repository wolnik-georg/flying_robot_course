#!/usr/bin/env python3
"""One controller, one parameter set, one input NPZ -> output CSV (subprocess entry, equal-parameter replay).

  python3 indi_replay_equal_worker.py <side> <inputs.npz> <out.csv> <overrides.json> [flight_config.json]

side: ours | omar_c | omar_rust | tk_rust | tk_c
overrides.json: {"cvar": {name: value, ...}, "omar": {"mass": x, "Kpos_P": [x,y,z], ...}, "omar_indi": 3, "ours_native": true|false}
  - ours:   `ours_native` true applies the flown configuration first (apply_ours_globals), then the cvar overrides (before AND after Init, as indi_replay_harmonize.py does)
  - omar_c: fields of controllerOmarIndi_t set after controllerOmarIndiInit (vectors as 3-lists)
  - tk_*:   cvar overrides (g_tk_*, g_indi_mass, g_indi_kt1..4) before Init
Same input handling as indi_replay_worker.py (1 kHz ticks, the logged state, gyro, accelerometer, RPM, rebuilt setpoint).
Host bindings are the Python 3.10 extension: run with system python3, PYTHONPATH=<crazyflie-firmware>/build."""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from indi_replay_config import apply_ours_globals  # noqa: E402

WARMUP_TICKS = 800


def main() -> None:
    side, npz_path, out_csv, ov_path = sys.argv[1:5]
    flight_cfg_path = sys.argv[5] if len(sys.argv) > 5 else str(Path(npz_path).parent / "flight_config.json")
    sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/build")
    import cffirmware as fw  # noqa: E402

    ov = json.loads(Path(ov_path).read_text())
    cvar_ov = ov.get("cvar", {})
    z = np.load(npz_path)
    n = len(z["pos"])
    start = min(int(z["warmup_start"]) + WARMUP_TICKS, n - 1)
    sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
    applied = {}

    def set_cvar():
        for k, v in cvar_ov.items():
            setattr(fw.cvar, k, v)
            applied[k] = v

    if side == "ours":
        if ov.get("ours_native", True):
            cfg = json.loads(Path(flight_cfg_path).read_text())
            apply_ours_globals(fw, cfg, "L0")
        set_cvar()
        fw.controllerOutOfTreeInit()
        set_cvar()
        step = lambda tick: fw.controllerOutOfTree(ctl, sp, sens, st, tick)
    elif side == "omar_c":
        omar = fw.controllerOmarIndi_t()
        fw.controllerOmarIndiInit(omar)
        omar.indi = int(ov.get("omar_indi", 3))
        for k, v in ov.get("omar", {}).items():
            if isinstance(v, (list, tuple)):
                vec = getattr(omar, k)
                vec.x, vec.y, vec.z = map(float, v)
                setattr(omar, k, vec)
            else:
                setattr(omar, k, float(v))
            applied["omar." + k] = v
        step = lambda tick: fw.controllerOmarIndi(omar, ctl, sp, sens, st, tick)
    elif side == "omar_rust":
        fw.controllerOutOfTree5Init()
        fw.omar_indi_rust_set_indi(int(ov.get("omar_indi", 3)))
        step = lambda tick: fw.controllerOutOfTree5(ctl, sp, sens, st, tick)
    elif side in ("tk_rust", "tk_c"):
        set_cvar()
        if side == "tk_rust":
            fw.controllerOutOfTree6Init()
            step = lambda tick: fw.controllerOutOfTree6(ctl, sp, sens, st, tick)
        else:
            fw.controllerOutOfTree7Init()
            step = lambda tick: fw.controllerOutOfTree7(ctl, sp, sens, st, tick)
        set_cvar()
    else:
        raise ValueError(side)

    rows = []
    for tick in range(n):
        p, v, q = z["pos"][tick], z["vel"][tick], z["quat"][tick]
        spp = z["sp_pos"][tick]
        sp.position.x, sp.position.y, sp.position.z = float(spp[0]), float(spp[1]), float(spp[2])
        sv = z["sp_vel"][tick]
        sp.velocity.x, sp.velocity.y, sp.velocity.z = float(sv[0]), float(sv[1]), float(sv[2])
        sa = z["sp_acc"][tick]
        sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = float(sa[0]), float(sa[1]), float(sa[2])
        sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs
        sp.mode.yaw = fw.modeAbs
        sp.attitude.yaw = math.degrees(float(z["yaw_d_rad"][tick]))
        g = z["gyro_deg_s"][tick]
        sens.gyro.x, sens.gyro.y, sens.gyro.z = float(g[0]), float(g[1]), float(g[2])
        a = z["acc_g"][tick]
        sens.acc.x, sens.acc.y, sens.acc.z = float(a[0]), float(a[1]), float(a[2])
        st.position.x, st.position.y, st.position.z = float(p[0]), float(p[1]), float(p[2])
        st.velocity.x, st.velocity.y, st.velocity.z = float(v[0]), float(v[1]), float(v[2])
        st.attitudeQuaternion.w, st.attitudeQuaternion.x, st.attitudeQuaternion.y, st.attitudeQuaternion.z = map(float, q)
        # Omar C reads the ESTIMATOR acceleration state->acc (world frame, Gs, gravity removed: kalman_core.c externalize: R·acc_body − [0,0,1]).
        # The earlier replay harness never set it (zero), so Omar's measured acceleration was missing. Ours reads the body accelerometer (sens.acc) and ignores it.
        qw, qx, qy, qz = (float(x) for x in q)
        Rm = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                       [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                       [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
        aw = Rm @ np.array([float(a[0]), float(a[1]), float(a[2])]) - np.array([0.0, 0.0, 1.0])
        st.acc.x, st.acc.y, st.acc.z = float(aw[0]), float(aw[1]), float(aw[2])
        r = z["rpm"][tick]
        fw.oot_set_rpm(int(r[0]), int(r[1]), int(r[2]), int(r[3]))
        step(tick)
        if tick >= start:
            rows.append((tick, float(ctl.thrustSi), float(ctl.torqueX), float(ctl.torqueY), float(ctl.torqueZ)))

    out = Path(out_csv)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tick", "thrust_si", "tau_x", "tau_y", "tau_z"])
        w.writerows(rows)
    sys.stdout.write(json.dumps({"side": side, "rows": len(rows), "applied": applied}) + "\n")


if __name__ == "__main__":
    main()
