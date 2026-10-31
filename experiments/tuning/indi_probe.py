import sys
from multiprocessing import Pool
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent)); import plant_core as P, numpy as np
cfgs = []
for st, so in (("S0", dict(indi=0, ns2=0)), ("S1", dict(indi=3, ns2=0))):
    for gn in (0.5, 2.0, 8.0):
        for rn in (0.0, 100.0):
            c = P.plant(dict(gn=gn, rpmn=rn, J="omar")); c.update(scen="A1", sc=dict(dz=0.45, hold=15.0), gains={}, seed=0, tree=True, nm=f"probe_{st}_{gn}_{rn}", tag="probe", **so); cfgs.append(c)
with Pool(12) as pool: res = pool.map(P.run, cfgs, chunksize=1)
for c, r in zip(cfgs, res):
    ez = np.array(r["ez"]); ts = np.array(r["ts"]); w = (ts >= 2) & (ts <= 15)
    print("A1 indi %d gyro noise %.1f rpm noise %.0f | stable %s tilt max %.1f | mean ez %.1f cm  z rms %.1f" % (c["indi"], c["gn"], c["rpmn"], r["stable"], r["tilt_max_deg"], 100 * ez[w].mean() if w.any() else float('nan'), r["z_rms_cm"]), flush=True)
