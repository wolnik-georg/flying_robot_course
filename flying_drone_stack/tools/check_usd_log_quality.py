#!/usr/bin/env python3
"""Quality check of a decoded uSD log (usd_study_config.txt superset, 2026-10-11).

usage: check_usd_log_quality.py file.bin [file.bin ...]

Reports, per file: sample gaps (the card/ring buffer could not keep up at 500 Hz), channels that arrived
completely constant (e.g. an unused controller's group), and which channel groups carry data. Run it after the first
flight with a new firmware/config; exit code 1 if gaps or missing columns are found.
"""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import decode_usd_log as dec

GROUPS = {
    "state": ["x", "y", "z", "vx", "vy", "vz", "roll_deg", "pitch_deg", "yaw_deg", "est_qx", "est_qy", "est_qz", "est_qw", "gyro_x", "acc_x"],
    "reference": ["ctrltarget_x", "ctrltarget_vx", "ctrltarget_ax"],
    "motors": ["motor_m1", "motor_m1_rpm", "rpm_m1"],
    "battery": ["vbat"],
    "mocap raw": ["locsrv_x", "locsrv_tick"],
    "ours (indi.*)": ["a_res_x", "tau_x", "alp_x", "e_r_x", "e_r_norm"],
    "NS2 (rnn.*)": ["rnn_pred_x", "rnn_clamped"],
    "Omar C (ctrlOmarIndi.*)": ["omar_thrust_si", "omar_tau_x", "omar_a_rpm_fx", "omar_rpy_x", "omar_rpyd_x", "omar_err_pos_x"],
    "session tag": ["run_tag"],
}


def main():
    bad = False
    for f in sys.argv[1:]:
        r = dec.load(f)
        t = r["t"]; dt = np.diff(t) * 1000.0
        print(f"\n{Path(f).name}: {len(t)} samples, {t[-1]-t[0]:.1f} s, median dt {np.median(dt):.2f} ms")
        late = int(np.sum(dt > 3.0)); big = float(dt.max())
        print(f"  gaps (dt > 3 ms): {late} ({100*late/len(dt):.3f} %), max gap {big:.1f} ms" + ("   <-- card cannot keep up" if late > 0.001 * len(dt) else ""))
        bad |= late > 0.001 * len(dt)
        n_vars = sum(1 for k in r if k != "t")
        print(f"  channels decoded: {n_vars}")
        for g, keys in GROUPS.items():
            have = [k for k in keys if k in r]
            if len(have) < len(keys):
                print(f"  {g:26s} MISSING columns: {[k for k in keys if k not in r]}"); bad = True; continue
            live = [k for k in keys if np.nanstd(r[k]) > 0]
            print(f"  {g:26s} {'carries data' if live else 'constant / unused (expected if that controller is not running)'}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
