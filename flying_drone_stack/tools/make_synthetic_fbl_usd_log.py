#!/usr/bin/env python3
"""Turn a REAL uSD log (study config, 96 variables) into a synthetic one with the FBL config (usd_fbl_config.txt): the 11 INDI/oot5 variables are dropped, the 11 ctrlFbl.* variables
are added with plausible values (active while z > 0.15 m, steps counting up, no trips/faults). Everything else stays real. Purpose: test decode / merge / metrics / fbl_log_check on the
FBL layout BEFORE the first FBL flight.   python3 flying_drone_stack/tools/make_synthetic_fbl_usd_log.py <real.bin> <out.bin>"""
import struct, sys
from zlib import crc32
import numpy as np
DROP = {"indi.oot5_branch", "indi.oot5_thrust_si", "indi.oot5_sp_mode_z", "indi.oot5_sp_thrust", "indi.oot5_sp_mode_x", "indi.oot5_sp_mode_y", "indi.tau_x", "indi.tau_y", "indi.tau_z", "indi.alp_x", "indi.alp_y"}
ADD = [("ctrlFbl.active", "B"), ("ctrlFbl.tripped", "B"), ("ctrlFbl.steps", "I"), ("ctrlFbl.faults", "I"), ("ctrlFbl.res_az", "f"), ("ctrlFbl.res_jz", "f"), ("ctrlFbl.thrust", "f"), ("ctrlFbl.acc_z", "f"),
       ("ctrlFbl.w_des_x", "f"), ("ctrlFbl.w_des_y", "f"), ("ctrlFbl.w_des_z", "f")]
def name(d, i):
    e = i
    while d[e] != 0: e += 1
    return d[i:e].decode(), e + 1
src, dst = sys.argv[1:3]; d = open(src, "rb").read(); assert d[0] == 0xBC
version, nev = struct.unpack("HH", d[1:5]); assert version == 2 and nev == 1
i = 5; eid, = struct.unpack("H", d[i:i + 2]); i += 2; ename, i = name(d, i); nv, = struct.unpack("H", d[i:i + 2]); i += 2
vars_ = []
for _ in range(nv): s, i = name(d, i); vars_.append((s[:-3], s[-2]))
fmt = "<" + "".join(t for _, t in vars_); nb = struct.calcsize(fmt); recs = []
while i < len(d) - 4:
    eid_, ts = struct.unpack("<HQ", d[i:i + 10]); i += 10; recs.append((ts, struct.unpack(fmt, d[i:i + nb]))); i += nb
col = {n: k for k, (n, _) in enumerate(vars_)}; keep = [k for k, (n, _) in enumerate(vars_) if n not in DROP]; ts = np.array([r[0] for r in recs]); data = np.array([r[1] for r in recs], dtype=object)
z = np.array([r[1][col["stateEstimate.z"]] for r in recs], float); act = (z > 0.15).astype(int); steps = np.cumsum(act)
thr = np.array([r[1][col["ctrlOmarIndi.thrustSi"]] for r in recs], float)
new_vars = [vars_[k] for k in keep]; ui = [n for n, _ in new_vars].index("usd.runTag"); new_vars = new_vars[:ui] + ADD + new_vars[ui:]
fmt2 = "<" + "".join(t for _, t in new_vars); hdr = bytearray(b"\xbc" + struct.pack("HH", 2, 1) + struct.pack("H", eid) + b"fixedFrequency\0" + struct.pack("H", len(new_vars)))
for n, t in new_vars: hdr += f"{n}({t})".encode() + b"\0"
body = bytearray()
for r, (t, vals) in enumerate(recs):
    row = [vals[k] for k in keep]; fb = [int(act[r]), 0, int(steps[r]), 0, 0.0, 0.0, float(thr[r]), 0.0, 0.0, 0.0, 0.0]
    row = row[:ui] + fb + row[ui:]; body += struct.pack("<HQ", eid, int(t)) + struct.pack(fmt2, *row)
out = bytes(hdr + body); out += struct.pack("I", crc32(out)); open(dst, "wb").write(out); print(f"{dst}: {len(recs)} samples, {len(new_vars)} variables")
