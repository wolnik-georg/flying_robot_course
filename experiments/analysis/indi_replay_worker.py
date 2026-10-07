#!/usr/bin/env python3
"""One controller, one level, one input NPZ -> output CSV (subprocess entry)."""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

WARMUP_TICKS = 800  # 0.8 s at 1 kHz after hover entry

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from indi_replay_config import (  # noqa: E402
    apply_ours_globals,
    flight_ours_config,
    harmonize_omar_constants,
    omar_native_snapshot,
)


def main() -> None:
    side, level, npz_path, out_csv, meta_path = sys.argv[1:6]
    build_dir = sys.argv[6] if len(sys.argv) > 6 else "/home/georg/Desktop/crazyflie-firmware/build"
    debug_z_trim = "--debug-z-trim" in sys.argv[7:]
    sys.path.insert(0, build_dir)
    import cffirmware as fw  # noqa: E402

    meta = json.loads(Path(meta_path).read_text())
    cfg_path = Path(npz_path).parent / "flight_config.json"
    if cfg_path.is_file():
        flight_cfg = json.loads(cfg_path.read_text())
    else:
        flight_cfg = flight_ours_config(meta)
    params_used: dict = {"side": side, "level": level, "flight_yaml_rev": flight_cfg["yaml_rev"]}

    z = np.load(npz_path)
    z_trim = 0.0
    if debug_z_trim and "z_tracking_trim_m" in z:
        z_trim = float(z["z_tracking_trim_m"])
    params_used["debug_z_trim"] = debug_z_trim
    params_used["z_tracking_trim_m_applied"] = z_trim
    n = len(z["pos"])
    start = int(z["warmup_start"]) + WARMUP_TICKS
    start = min(start, n - 1)

    sp = fw.setpoint_t()
    sens = fw.sensorData_t()
    st = fw.state_t()
    ctl = fw.control_t()

    if side == "ours":
        params_used["ours"] = apply_ours_globals(fw, flight_cfg, level)
        fw.controllerOutOfTreeInit()
        step = lambda tick: fw.controllerOutOfTree(ctl, sp, sens, st, tick)
        extra = lambda: (
            fw.oot_get_a_res(0),
            fw.oot_get_a_res(1),
            fw.oot_get_a_res(2),
            fw.oot_get_e_r_norm(),
        )
    elif side == "omar_c":
        params_used["omar_native"] = omar_native_snapshot(fw)
        if level in ("L1", "L2", "L3"):
            params_used["L1_constants"] = harmonize_omar_constants(meta)
        omar = fw.controllerOmarIndi_t()
        fw.controllerOmarIndiInit(omar)
        omar.indi = 3
        step = lambda tick: fw.controllerOmarIndi(omar, ctl, sp, sens, st, tick)
        extra = lambda: (0.0, 0.0, 0.0, 0.0)
    elif side == "omar_rust":
        params_used["omar_native"] = omar_native_snapshot(fw)
        if level in ("L1", "L2", "L3"):
            params_used["L1_constants"] = harmonize_omar_constants(meta)
        fw.controllerOutOfTree5Init()
        fw.omar_indi_rust_set_indi(3)
        step = lambda tick: fw.controllerOutOfTree5(ctl, sp, sens, st, tick)
        extra = lambda: (0.0, 0.0, 0.0, 0.0)
    else:
        raise ValueError(side)

    rows = []
    for tick in range(n):
        i = tick
        p = z["pos"][i]
        v = z["vel"][i]
        q = z["quat"][i]
        spp = z["sp_pos"][i]
        sp.position.x, sp.position.y = float(spp[0]), float(spp[1])
        sp.position.z = float(spp[2]) + z_trim
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

        step(tick)

        if tick < start:
            continue
        ar = extra()
        rows.append(
            {
                "tick": tick,
                "thrust_si": float(ctl.thrustSi),
                "tau_x": float(ctl.torqueX),
                "tau_y": float(ctl.torqueY),
                "tau_z": float(ctl.torqueZ),
                "a_res_x": float(ar[0]),
                "a_res_y": float(ar[1]),
                "a_res_z": float(ar[2]),
                "e_r_norm": float(ar[3]),
            }
        )

    out_path = Path(out_csv)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["tick", "thrust_si", "tau_x", "tau_y", "tau_z", "a_res_x", "a_res_y", "a_res_z", "e_r_norm"]
    with out_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    payload = {"side": side, "level": level, "rows": len(rows), "out": str(out_path), "params_used": params_used}
    sys.stdout.write(json.dumps(payload) + "\n")
    sys.stdout.flush()
    sys.stderr.write("")  # ensure stderr empty on success (orchestrator parses stdout only)


if __name__ == "__main__":
    main()
