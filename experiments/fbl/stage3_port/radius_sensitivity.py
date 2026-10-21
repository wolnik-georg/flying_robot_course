#!/usr/bin/env python3
"""Sensitivity of the ROM downwash model to the propeller radius (the bundle hard-codes 45 mm / 50 mm in two places of force_batch_fo.py): the bundle is copied to a scratch directory (the original is never edited),
both radii are scaled by k = 0.9 / 1.1, the ROM acceleration per unit C_D is evaluated on a (z0, r0) grid and compared with k = 1. out/stage3/radius_sensitivity.json
  ~/.pyenv/versions/flying_robots/bin/python experiments/fbl/stage3_port/radius_sensitivity.py"""
import json, os, re, shutil, subprocess, sys, tempfile
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent; BUNDLE = Path(os.environ.get("FBL_BUNDLE", "~/Desktop/learned_fbl_core")).expanduser()
Z = [0.25, 0.5, 0.75, 1.0]; Rr = [0.0, 0.03, 0.06, 0.1, 0.15, 0.2, 0.3]
CODE = """
import sys, json, numpy as np
sys.path.insert(0, %r); sys.path.insert(0, %r)
import rom_table_lib as L
zz, rr = np.meshgrid(%r, %r, indexing='ij')
a = L.rom_exact(zz.ravel(), rr.ravel())[:, 0].reshape(zz.shape)
print(json.dumps(a.tolist()))
"""
def run(k):
    tmp = Path(tempfile.mkdtemp()) / "bundle"; shutil.copytree(BUNDLE, tmp, ignore=shutil.ignore_patterns("__pycache__"))
    f = tmp / "KNODE/Training/Velocity_Field/force_batch_fo.py"; t = f.read_text()
    t2 = t.replace("r_prop_vel = 0.045 / 2.0", f"r_prop_vel = {k} * 0.045 / 2.0").replace("r_prop_int = 0.05 / 2.0", f"r_prop_int = {k} * 0.05 / 2.0")
    assert t2 != t; f.write_text(t2)
    out = subprocess.run([sys.executable, "-c", CODE % (str(HERE), str(HERE.parent), Z, Rr)], env={**os.environ, "FBL_BUNDLE": str(tmp)}, capture_output=True, text=True)
    if out.returncode: sys.exit(out.stderr[-600:])
    shutil.rmtree(tmp.parent); return np.array(json.loads(out.stdout.strip().splitlines()[-1]))
base, lo, hi = run(1.0), run(0.9), run(1.1); res = {}
for name, a in (("0.9", lo), ("1.1", hi)):
    m = (np.array(Rr)[None, :] <= 0.2) & (np.array(Z)[:, None] >= 0.25)
    s = float((a[m] * base[m]).sum() / (base[m] ** 2).sum())          # best scale factor (equivalent C_D change) over the gate region
    res[name] = dict(ratio=(a / np.where(np.abs(base) > 1e-9, base, np.nan)).tolist(), equivalent_cd_scale=s, residual_rel=float(np.sqrt(((a[m] - s * base[m]) ** 2).sum() / (base[m] ** 2).sum())))
json.dump(dict(z=Z, r=Rr, base=base.tolist(), lo=lo.tolist(), hi=hi.tolist(), summary=res), open(HERE.parent / "out/stage3/radius_sensitivity.json", "w"), indent=1)
print("base a1 [m/s^2 per unit C_D] at z0=0.5:", np.round(base[1], 2))
for n, r in res.items(): print(f"radius x{n}: equivalent C_D scale {r['equivalent_cd_scale']:.3f}, shape residual {100 * r['residual_rel']:.1f} % of the signal (gate region r0 <= 0.2 m)")
