#!/usr/bin/env python3
"""Diagnostic: which applied position gain (override) lets the full-INDI replay match the flown thrust?
Usage (system python3 + PYTHONPATH=firmware build): indi_replay_gain_sweep.py A1|A8 '{"g_kp_z":12}'
Constant gains only; no data-derived trim. Reports gate metrics vs bat-comp flown command."""
import json, sys, math
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/build")
import cffirmware as fw
import indi_replay_ladder as L
from indi_replay_config import apply_ours_globals

tag, ov = sys.argv[1], json.loads(sys.argv[2])
spec = [s for s in L.RUNG2_FLIGHTS if s["tag"] == tag][0]
run_dir = L.OUT / "rung2" / spec["label"]
npz = run_dir / "inputs.npz"
meta = json.loads(spec["meta"].read_text())
cfg = L.flight_config_from_meta(meta, spec.get("yaml_rev"))
z = np.load(npz)
n = len(z["pos"]); start = min(int(z["warmup_start"]) + L.WARMUP_TICKS, n - 1)
apply_ours_globals(fw, cfg, "L0")
for k, v in ov.items():
    setattr(fw.cvar, k, v)
fw.controllerOutOfTreeInit()
for k, v in ov.items():           # Init may reset: re-apply
    setattr(fw.cvar, k, v)
sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
th = []
for i in range(n):
    p, v, q, spp = z["pos"][i], z["vel"][i], z["quat"][i], z["sp_pos"][i]
    sp.position.x, sp.position.y, sp.position.z = map(float, spp)
    sp.velocity.x, sp.velocity.y, sp.velocity.z = map(float, z["sp_vel"][i])
    sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = map(float, z["sp_acc"][i])
    sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs; sp.mode.yaw = fw.modeAbs
    sp.attitude.yaw = math.degrees(float(z["yaw_d_rad"][i]))
    sens.gyro.x, sens.gyro.y, sens.gyro.z = map(float, z["gyro_deg_s"][i])
    sens.acc.x, sens.acc.y, sens.acc.z = map(float, z["acc_g"][i])
    st.position.x, st.position.y, st.position.z = map(float, p)
    st.velocity.x, st.velocity.y, st.velocity.z = map(float, v)
    st.attitudeQuaternion.w, st.attitudeQuaternion.x, st.attitudeQuaternion.y, st.attitudeQuaternion.z = map(float, q)
    fw.oot_set_rpm(*[int(x) for x in z["rpm"][i]])
    fw.controllerOutOfTree(ctl, sp, sens, st, i)
    if i >= start: th.append(float(ctl.thrustSi))
ticks = np.arange(start, n)
steady = L.steady_indices(z, ticks)
sel = steady - start
cmd = L.commanded_thrust_on_ticks(z, spec["meta"], steady)
rep = np.asarray(th)[sel]
m = L.thrust_gate_metrics(cmd, rep, 0.0, 0.041)
print(tag, ov, "cmd %.3f rep %.3f err %.1f%% corr %.3f slope %.2f" % (m["mean_command_N"], m["mean_replay_N"], m["mean_err_pct"], m["corr"], m["regression_slope"]))

# lag scan: replay vs flown command, shift replay by k ticks (1 kHz)
allcmd = L.commanded_thrust_on_ticks(z, spec["meta"], ticks)
allrep = np.asarray(th)
msk = np.isin(ticks, steady)
best = []
for k in range(-150, 151, 5):
    a = np.roll(allrep, k)           # positive k: replay delayed
    c = np.corrcoef(allcmd[msk], a[msk])[0, 1]
    best.append((c, k))
best.sort(reverse=True)
print("lag scan top:", [(round(c, 3), k) for c, k in best[:5]], " corr@0 %.3f" % np.corrcoef(allcmd[msk], allrep[msk])[0, 1])
