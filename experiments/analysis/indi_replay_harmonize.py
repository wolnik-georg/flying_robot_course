#!/usr/bin/env python3
"""Attribution step the L1-L3 'levels' never performed: make OURS use Omar's mass / kt (settable on the host)
and see how much of the ours-vs-Omar-C thrust difference disappears. Same inputs, open loop.
Usage: indi_replay_harmonize.py A8|A1  (system python3, PYTHONPATH=firmware build)"""
import csv, json, math, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/build")
import cffirmware as fw
from indi_replay_config import apply_ours_globals

tag = sys.argv[1]
base = Path(__file__).resolve().parent / "out" / "indi_replay" / tag
z = np.load(base / "inputs.npz"); cfg = json.loads((base / "flight_config.json").read_text())
n = len(z["pos"]); start = min(int(z["warmup_start"]) + 800, n - 1)
om = {int(r["tick"]): float(r["thrust_si"]) for r in csv.DictReader(open(base / "replay_omar_c_L0.csv"))}
omass, okt = float(fw.oot_omar_mass()), float(fw.oot_omar_kt_equiv())
print(tag, "Omar mass %.4f kt_equiv %.3e" % (omass, okt))

def run(over):
    apply_ours_globals(fw, cfg, "L0")
    for k, v in over.items(): setattr(fw.cvar, k, v)
    fw.controllerOutOfTreeInit()
    for k, v in over.items(): setattr(fw.cvar, k, v)
    sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
    out = {}
    for i in range(n):
        sp.position.x, sp.position.y, sp.position.z = map(float, z["sp_pos"][i])
        sp.velocity.x, sp.velocity.y, sp.velocity.z = map(float, z["sp_vel"][i])
        sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = map(float, z["sp_acc"][i])
        sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs; sp.mode.yaw = fw.modeAbs
        sp.attitude.yaw = math.degrees(float(z["yaw_d_rad"][i]))
        sens.gyro.x, sens.gyro.y, sens.gyro.z = map(float, z["gyro_deg_s"][i])
        sens.acc.x, sens.acc.y, sens.acc.z = map(float, z["acc_g"][i])
        st.position.x, st.position.y, st.position.z = map(float, z["pos"][i])
        st.velocity.x, st.velocity.y, st.velocity.z = map(float, z["vel"][i])
        q = z["quat"][i]; st.attitudeQuaternion.w, st.attitudeQuaternion.x, st.attitudeQuaternion.y, st.attitudeQuaternion.z = map(float, q)
        fw.oot_set_rpm(*[int(x) for x in z["rpm"][i]])
        fw.controllerOutOfTree(ctl, sp, sens, st, i)
        if i >= start: out[i] = float(ctl.thrustSi)
    return out

res = {}
for name, over in (("ours native", {}),
                   ("ours + Omar mass", {"g_indi_mass": omass}),
                   ("ours + Omar mass + kt", {"g_indi_mass": omass, **{f"g_indi_kt{j}": okt for j in range(1, 5)}})):
    o = run(over); ks = sorted(set(o) & set(om))
    a = np.array([o[k] for k in ks]); b = np.array([om[k] for k in ks])
    r = dict(rms=float(np.sqrt(np.mean((a - b) ** 2))), mean_diff=float(np.mean(a - b)), corr=float(np.corrcoef(a, b)[0, 1]), mean_ours=float(a.mean()), mean_omar=float(b.mean()), n=len(ks))
    res[name] = r; print("%-24s rms %.4f N  mean(ours-omar) %+.4f  corr %.3f  (ours %.3f omar %.3f, n=%d)" % (name, r["rms"], r["mean_diff"], r["corr"], r["mean_ours"], r["mean_omar"], r["n"]))
json.dump(res, open(base / "harmonize_summary.json", "w"), indent=1)
