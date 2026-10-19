#!/usr/bin/env python3
"""Test vectors for the C residual module: random and edge cases with the expected acc / jerk from the Python reference (fbl_rom_runtime.py, float32).
  ~/.pyenv/versions/flying_robots/bin/python gen_test_vectors.py  [table tag, default v1]   -> test_vectors.txt"""
import sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE.parent))
import fbl_rom_runtime as RT  # noqa: E402
tag = sys.argv[1] if len(sys.argv) > 1 else "v1"
tab = dict(np.load(HERE.parents[1] / f"out/stage3/rom_table_{tag}/rom_table.npz")); CD = 0.43
rng = np.random.default_rng(7); cases = []
def case(n=None, **kw):
    n = n or int(rng.integers(1, 4)); p = rng.uniform(-1, 1, 3).astype(np.float32); p[2] = rng.uniform(0.1, 1.0); v = rng.uniform(-1, 1, 3).astype(np.float32); pn, vn = [], []
    for k in range(n):
        z0 = kw.get("z0", rng.choice([rng.uniform(-0.2, 0.06), rng.uniform(0.05, 1.45), rng.uniform(0.05, 1.45), rng.uniform(1.1, 1.5)]))
        r0 = kw.get("r0", rng.choice([0.0, rng.uniform(0, 0.05), rng.uniform(0, 0.3), rng.uniform(0, 0.7)])); ph = rng.uniform(0, 2 * np.pi)
        pn.append((p + np.array([r0 * np.cos(ph), r0 * np.sin(ph), z0])).astype(np.float32)); vn.append(rng.uniform(-1, 1, 3).astype(np.float32))
    cases.append((p, v, pn, vn))
for _ in range(3000): case()
for z0 in (0.05, 0.0500001, 0.0499, 0.1, 0.5, 1.2, 1.3, 1.399, 1.4, 1.5, 2.0):
    for r0 in (0.0, 1e-7, 0.05, 0.3, 0.5999, 0.6, 0.8): case(1, z0=z0, r0=r0)
with open(HERE / "test_vectors.txt", "w") as f:
    f.write(f"{CD}\n{len(cases)}\n")
    for p, v, pn, vn in cases:
        acc = np.zeros(3, np.float32); jerk = np.zeros(3, np.float32); na = 0
        for q, w in zip(pn, vn):
            a, j = RT.residual(tab, CD, p, v, q, w); acc += a; jerk += j; na += int(abs(a[2]) > 0 or abs(j[2]) > 0 or (RT.Z_LO < float(-(p - q)[2]) < RT.Z_FADE1 and float(np.hypot(*(p - q)[:2])) < RT.R_MAX_USE))
        f.write(" ".join(f"{x:.9g}" for x in [len(pn), *p, *v] + [c for q, w in zip(pn, vn) for c in (*q, *w)] + [*acc, *jerk]) + f" {na}\n")
print(len(cases), "cases ->", HERE / "test_vectors.txt")
