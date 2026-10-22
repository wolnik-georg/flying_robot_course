#!/usr/bin/env python3
"""Stage A of the NN-vs-INDI check (supervisor feedback 2026-10-22): per pilot flight, bottom drone (cf5), from the raw uSD log:
  - strategy flags (INDI on? NS2 applied? -> ns2_applied_check.fit),
  - the controller's own INDI estimate rebuilt for ANY strategy by running the real Omar C code over the log (as omar_labels.py), but this time also the
    UNFILTERED a_rpm / a_imu (members `a_rpm`, `a_imu`) so that stage B can smooth the label offline with a zero-delay filter,
  - the logged NN output (rnn.pred_x/y/z).
Writes experiments/results/nn_vs_indi_2026-10-22/cache/<flight>.npz (not committed).
  PYTHONPATH=<crazyflie-firmware>/build python3 experiments/analysis/nn_vs_indi_raw.py"""
import json, sys
from pathlib import Path
import numpy as np
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "flying_drone_stack/tools")); sys.path.insert(0, str(REPO / "flying_drone_stack/tools/residual")); sys.path.insert(0, str(REPO / "experiments/analysis"))
sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/build")
import decode_usd_log as D                 # noqa: E402
import ns2_applied_check as N              # noqa: E402
import omar_labels as OL                   # noqa: E402
import cffirmware as fw                    # noqa: E402

BANKS = ["bank_v2val_2026-10-19", "bank_a1clean_2026-10-19", "bank_pilot2_2026-10-20", "bank_pilot3_2026-10-20", "bank_pilot4_2026-10-20",
         "bank_pilot5_2026-10-20", "bank_pilot6_2026-10-20", "bank_video_2026-10-20"]
BANKS = sys.argv[1:] or BANKS                                              # optional: bank folder names under experiments/logs/
OUT = REPO / "experiments/results/nn_vs_indi_2026-10-22/cache"; OUT.mkdir(parents=True, exist_ok=True)


def raw_estimates(d):
    """-> a_rpm_raw, a_imu_raw (N,3) and the filtered pair, running the real controller with indi = 3 (rpm source: deck, as the labels)."""
    n = len(d["t"]); q = OL.quaternions(d)
    omar = fw.controllerOmarIndi_t(); fw.controllerOmarIndiInit(omar); omar.indi = 3
    sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
    sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs; sp.mode.yaw = fw.modeAbs
    raw_r, raw_i, f_r, f_i = (np.zeros((n, 3)) for _ in range(4))
    for i in range(n):
        qw, qx, qy, qz = (float(v) for v in q[i])
        st.position.x, st.position.y, st.position.z = float(d["x"][i]), float(d["y"][i]), float(d["z"][i])
        st.velocity.x, st.velocity.y, st.velocity.z = float(d["vx"][i]), float(d["vy"][i]), float(d["vz"][i])
        st.attitudeQuaternion.w, st.attitudeQuaternion.x, st.attitudeQuaternion.y, st.attitudeQuaternion.z = qw, qx, qy, qz
        sp.position.x, sp.position.y, sp.position.z = st.position.x, st.position.y, st.position.z
        sens.gyro.x, sens.gyro.y, sens.gyro.z = float(d["gyro_x"][i]), float(d["gyro_y"][i]), float(d["gyro_z"][i])
        a = np.array([d["acc_x"][i], d["acc_y"][i], d["acc_z"][i]], float)
        sens.acc.x, sens.acc.y, sens.acc.z = float(a[0]), float(a[1]), float(a[2])
        aw = OL._rot(qw, qx, qy, qz) @ a - np.array([0.0, 0.0, 1.0])
        st.acc.x, st.acc.y, st.acc.z = float(aw[0]), float(aw[1]), float(aw[2])
        fw.oot_set_rpm(*[int(max(0, round(float(d[k][i])))) for k in ("rpm_m1", "rpm_m2", "rpm_m3", "rpm_m4")])
        fw.controllerOmarIndi(omar, ctl, sp, sens, st, 2 * i)
        for arr, v in ((raw_r, omar.a_rpm), (raw_i, omar.a_imu), (f_r, omar.a_rpm_filtered), (f_i, omar.a_imu_filtered)):
            arr[i] = (v.x, v.y, v.z)
    return raw_r, raw_i, f_r, f_i


def main():
    done = 0
    for bank in BANKS:
        man = json.load(open(REPO / "experiments/logs" / bank / "training_manifest.json"))
        for fl in man["flights"]:
            name = fl["folder"]; out = OUT / f"{name}.npz"
            if out.exists():
                continue
            binp = REPO / "experiments/logs/usd_raw" / fl["cf5"]
            if not binp.exists():
                print("missing", binp.name); continue
            try:
                f = N.fit(str(binp)); d = D.load(str(binp))
                if f is None or "rpm_m1" not in d:
                    print("skip (short / no rpm)", name); continue
                rr, ri, fr, fi = raw_estimates(d)
                np.savez_compressed(out, t=np.asarray(d["t"], float), z=np.asarray(d["z"]), x=np.asarray(d["x"]), y=np.asarray(d["y"]),
                                    pred=np.stack([d["rnn_pred_x"], d["rnn_pred_y"], d["rnn_pred_z"]], 1), a_rpm_raw=rr, a_imu_raw=ri, a_rpm_f=fr, a_imu_f=fi,
                                    logged_rpm_f=np.stack([d["omar_a_rpm_fx"], d["omar_a_rpm_fy"], d["omar_a_rpm_fz"]], 1),
                                    logged_imu_f=np.stack([d["omar_a_imu_fx"], d["omar_a_imu_fy"], d["omar_a_imu_fz"]], 1),
                                    indi=f["indi"], coef_pred=f["coef_pred"], scenario=fl["scenario"], bank=bank, folder=name)
                done += 1; print("ok", name, "indi" if f["indi"] else "geo", "NS2" if N.verdict(f) == "NS2 APPLIED" else "noNS2", flush=True)
            except Exception as e:                                     # noqa: BLE001
                print("FAILED", name, repr(e))
    print("flights written:", done)


if __name__ == "__main__":
    main()
