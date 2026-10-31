#!/usr/bin/env python3
"""Equivalence of diff/s0_torch.py (S0 controller + mixer) with the firmware (host bindings): same random closed-loop-like input sequence through both,
compare thrustSi, torque and the four motor forces every 500 Hz tick. System python3 + CFFIRMWARE_DIR=~/Desktop/crazyflie-firmware/build."""
import os, sys
from pathlib import Path
import numpy as np, torch
sys.path.insert(0, str(Path(__file__).parent.parent)); sys.path.insert(0, str(Path(__file__).parent))
import plant_core as P
import s0_torch as T
from scipy.spatial.transform import Rotation as Rot
fw = P.fw
rng = np.random.default_rng(int(os.environ.get('SEED', 0)))
gains = dict(Kpos_P=(6.6, 6.6, 4.9), Kpos_D=(2.7, 2.7, 4.0), Kpos_Iz=4.2, KR=(.0066, .0066, .0069), Komega=(.0008, .0008, .0015), KI=(.118, .118, .118), Kpos_I_limit=2.0)
omar = fw.controllerOmarIndi_t(); fw.controllerOmarIndiInit(omar); omar.mass = P.M; omar.indi = 0; omar.fbl.en = 0; fw.cvar.g_rnn_en = 0
omar.Kpos_P.x, omar.Kpos_P.y, omar.Kpos_P.z = gains['Kpos_P']; omar.Kpos_D.x, omar.Kpos_D.y, omar.Kpos_D.z = gains['Kpos_D']; omar.Kpos_I.z = gains['Kpos_Iz']
omar.KR.x, omar.KR.y, omar.KR.z = gains['KR']; omar.Komega.x, omar.Komega.y, omar.Komega.z = gains['Komega']; omar.KI.x, omar.KI.y, omar.KI.z = gains['KI']
sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs; sp.mode.yaw = fw.modeAbs; sp.attitude.yaw = 0.0
ctrl = T.S0(1, gains, mass=P.M)
N = int(os.environ.get('N', 4000)); mx = dict(thrust=0, torque=0, force=0)
pos = np.array([0.0, 0, 0.5]); vel = np.zeros(3); eul = np.zeros(3); gy = np.zeros(3)
for k in range(N):
    pos = pos + rng.normal(0, 0.002, 3) ; vel = vel * 0.99 + rng.normal(0, 0.02, 3); eul = np.clip(eul * 0.99 + rng.normal(0, 1.0, 3), -35, 35); gy = gy * 0.9 + rng.normal(0, 15, 3)
    pd = np.array([0.3 * np.sin(k / 300), 0.3 * np.cos(k / 400), 0.5 + 0.1 * np.sin(k / 200)]); vd = rng.normal(0, 0.1, 3); ad = rng.normal(0, 0.3, 3); jd = rng.normal(0, 1.0, 3)
    q = Rot.from_euler('xyz', eul, degrees=True).as_quat()
    sp.position.x, sp.position.y, sp.position.z = map(float, pd); sp.velocity.x, sp.velocity.y, sp.velocity.z = map(float, vd)
    sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = map(float, ad); sp.jerk.x, sp.jerk.y, sp.jerk.z = map(float, jd)
    st.position.x, st.position.y, st.position.z = map(float, pos); st.velocity.x, st.velocity.y, st.velocity.z = map(float, vel)
    st.attitudeQuaternion.x, st.attitudeQuaternion.y, st.attitudeQuaternion.z, st.attitudeQuaternion.w = map(float, q)
    sens.gyro.x, sens.gyro.y, sens.gyro.z = map(float, gy)
    if k % 2: continue
    fw.controllerOmarIndi(omar, ctl, sp, sens, st, k)
    Fc = P.mixer_forces(ctl)
    f = lambda a: torch.tensor(a, dtype=torch.float64)[None]
    th, u = ctrl.step(f(pd), f(vd), f(ad), f(jd), f(pos), f(vel), f(q), f(gy))
    Ft = T.mixer(th, u)[0].numpy()
    mx['thrust'] = max(mx['thrust'], abs(float(th) - ctl.thrustSi)); mx['torque'] = max(mx['torque'], np.abs(u[0].numpy() - np.array([ctl.torqueX, ctl.torqueY, ctl.torqueZ])).max())
    d_=np.abs(Ft - Fc).max(); mx['force'] = max(mx['force'], d_)
    if d_>1e-4 and mx.setdefault('n',0)<4: mx['n']+=1; print('k',k,'thrust',float(th),'u',u[0].numpy(),'torch F',Ft.round(4),'fw F',Fc.round(4))
print('max abs diff over %d ticks: thrustSi %.2e N  torque %.2e N m  motor force %.2e N (uint16 quantisation step: %.1e N)' % (N // 2, mx['thrust'], mx['torque'], mx['force'], P.THRUST_MAX / 65535))
