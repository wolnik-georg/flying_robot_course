#!/usr/bin/env python3
"""Estimate of the FBL step time on the drone from host timings: the NS2 evaluation takes 526 us on the drone (measured, rnn.us_max); the ratio of host times (FBL step extra vs NS2 evaluation extra, same host build and state) scales that to the FBL step.
  CFFIRMWARE_DIR=<fw>/build python3 experiments/fbl/stage3_port/timing_estimate.py"""
import os, sys, time
from pathlib import Path
from unittest.mock import MagicMock
import numpy as np
for _m in ("rclpy", "rclpy.node", "rclpy.time", "rosgraph_msgs", "rosgraph_msgs.msg"): sys.modules.setdefault(_m, MagicMock())
sys.path.insert(0, os.environ.get("CFFIRMWARE_DIR", "/home/georg/Desktop/crazyflie-firmware/build"))
import cffirmware as fw
def make(fbl, ns2):
    o = fw.controllerOmarIndi_t(); fw.controllerOmarIndiInit(o); o.mass = 0.0427; o.indi = 0; o.fbl.en = int(fbl); o.fbl.c_d = 0.43
    fw.cvar.g_rnn_en = int(ns2); fw.cvar.g_rnn_div = 1
    return o
sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs; sp.position.x, sp.position.y, sp.position.z = 0.0, 0.0, 0.5
st.position.x, st.position.y, st.position.z = 0.05, 0.02, 0.5; st.attitudeQuaternion.w = 1.0
fw.oot_set_peer_count(1)
def bench(o, n=40000):
    t0 = time.perf_counter()
    for i in range(n):
        fw.oot_set_peer(0, 0.05 + 0.0001 * (i % 7), 0.0, 1.0, i + 1)      # a neighbour above and close: the ROM / network are evaluated
        fw.controllerOmarIndi(o, ctl, sp, sens, st, i)
    return (time.perf_counter() - t0) / n * 1e6
base = min(bench(make(0, 0)) for _ in range(3)); ns2 = min(bench(make(0, 1)) for _ in range(3)); fbl = min(bench(make(1, 0)) for _ in range(3))
print(f"host us per step: baseline {base:.2f}, with NS2 every step {ns2:.2f}, with FBL {fbl:.2f}  (the loop itself includes the python call overhead)")
dn, df = ns2 - base, fbl - base
print(f"extra host time: NS2 evaluation {dn:.2f} us, FBL step {df:.2f} us -> ratio {df / dn:.2f}")
if dn > 0.5: print(f"estimated FBL step on the drone: about {df / dn * 526:.0f} us (NS2 evaluation 526 us there); controller slot 2000 us")
