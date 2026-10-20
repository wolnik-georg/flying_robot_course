#!/usr/bin/env python3
"""FBL gain tuning in the C-in-the-loop plant with command delay, motor lag, sensor noise (c_in_the_loop_real.py). Bundle gains are scaled: a = position loop (kp, kv, ki), b = rate loop (kr).
  CFFIRMWARE_DIR=<firmware>/build python3 experiments/fbl/stage3_port/gain_tuning.py --stage grid|robust --tag NAME
Output experiments/fbl/out/stage3/gain_tuning_<tag>.json (system python3, host bindings)."""
import argparse, json, sys
from pathlib import Path
sys.argv, _argv = sys.argv[:1], sys.argv[1:]
sys.path.insert(0, str(Path(__file__).parent)); import c_in_the_loop_real as C  # noqa: E402
REPO = Path(__file__).resolve().parents[3]
B = dict(kp_xy=56, kp_z=45, kv_xy=38, kv_z=21.6, ki_xy=48, ki_z=3, kr_xy=238, kr_z=140)
def gains(a, b): return {k: v * (b if k.startswith("kr") else a) for k, v in B.items()}
def real(lag=44, delay=4, eff=1.0): return dict(delay_ms=delay, motor_tau_ms=lag, gyro_noise_dps=0.5, pos_noise_m=0.0005, vel_noise=0.005, thrust_eff=eff)
ap = argparse.ArgumentParser(); ap.add_argument("--stage", default="grid"); ap.add_argument("--tag", default="x"); a_ = ap.parse_args(_argv); out = {}
def go(key, mode, speed, g=None, **rk):
    r = C.run(mode, speed, real=real(**rk), gains=g); out[key] = r
    print(f"{key:48s} z_rms {r['z_rms_cm']:6.2f} dip {r['z_dip_cm']:7.2f} tilt {r['tilt_max_deg']:5.1f} tripped {r['tripped']}", flush=True)
if a_.stage == "grid":
    for sp in (0.1, 0.3, 0.5):
        go(f"omar|v{sp}", "omar", sp)
        for a in (0.18, 0.25, 0.35):
            for b in (0.25, 0.5):
                go(f"fbl_rom|v{sp}|a{a}|b{b}", "fbl_rom", sp, gains(a, b))
        go(f"fbl_none|v{sp}|a0.25|b0.5", "fbl_none", sp, gains(0.25, 0.5))
elif a_.stage == "scale":      # bundle gains scaled, realistic plant (lag 44 ms, delay 4 ms, noise), v 0.3
    for b in (1, 0.5, 0.25, 0.12):
        for a in (1, 0.5, 0.35, 0.25, 0.18, 0.12): go(f"fbl_rom|v0.3|a{a}|b{b}", "fbl_rom", 0.3, gains(a, b))
elif a_.stage == "eff":        # thrust efficiency x z-integral gain, tuned gains
    for eff in (1.0, 0.95, 0.9, 0.85, 0.8):
        for kiz in (0.75, 3, 8, 16): go(f"fbl_rom|v0.3|eff{eff}|kiz{kiz}", "fbl_rom", 0.3, {**gains(0.25, 0.25), "ki_z": kiz}, eff=eff)
else:
    for lag in (20, 30, 44, 60):
        for eff in (1.0, 0.7):
            go(f"fbl_rom|v0.3|a0.25|b0.25|lag{lag}|eff{eff}", "fbl_rom", 0.3, gains(0.25, 0.25), lag=lag, eff=eff)
json.dump(out, open(REPO / f"experiments/fbl/out/stage3/gain_tuning_{a_.tag}.json", "w"), indent=1)
