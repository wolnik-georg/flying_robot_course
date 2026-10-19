#!/usr/bin/env python3
"""Test vectors for fbl_law.c from the AUTHORS' code (bundle compute_fbl_command and FBLControl._position_commands, imported, unmodified).
  ~/.pyenv/versions/flying_robots/bin/python gen_law_vectors.py   -> law_vectors_command.txt, law_vectors_position.txt"""
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE.parents[1]))
from common import bundle as B  # noqa: E402
B.load(); sys.path.insert(0, str(B.BUNDLE))
from simulator_multi_python.Controllers.fbl_control import FBLControl, compute_fbl_command  # noqa: E402
rng = np.random.default_rng(11); J = np.diag([1.43e-5, 1.43e-5, 2.89e-5]); K = np.diag([238., 238., 140.]); m, g = 0.03, 9.81
with open(HERE / "law_vectors_command.txt", "w") as f:
    N = 3000; f.write(f"{N}\n")
    for k in range(N):
        tilt = rng.uniform(0, 0.8 if k % 3 else 0.2); ax = rng.normal(size=3); ax[2] = 0; ax /= np.linalg.norm(ax) + 1e-9
        R = (Rot.from_rotvec(ax * tilt) * Rot.from_euler("z", rng.uniform(-np.pi, np.pi))).as_matrix()
        if k % 50 == 0: R = Rot.from_euler("zyx", [0.3, np.pi / 2 - 1e-7, 0]).as_matrix()     # yaw singularity (x axis perpendicular to the heading)
        w = rng.normal(0, 2, 3); acc = rng.normal(0, 2, 3); jerk = rng.normal(0, 8, 3); yaw = rng.uniform(-np.pi, np.pi); yr = rng.normal(0, 0.5)
        ra = np.array([0, 0, -rng.uniform(0, 3)]); rj = np.array([0, 0, rng.normal(0, 5)]); rw = np.zeros(3)
        if k % 40 == 0: acc = np.array([0, 0, -g + rng.uniform(-0.2, 0.2)])      # thrust near zero: floor of the rate inversion
        th, wd, tau = compute_fbl_command(R, w, acc, jerk, yaw, yr, ra, rj, rw, m, g, J, K)
        f.write(" ".join(f"{x:.9g}" for x in [*R.ravel(), *w, *acc, *jerk, yaw, yr, *ra, *rj, *rw, th, *wd, *tau]) + "\n")
# position law: sequences through the bundle's FBLControl (acceleration estimator + integral); t starts at 0 as the bundle assumes
with open(HERE / "law_vectors_position.txt", "w") as f:
    nseq, nstep = 6, 400; f.write(f"{nseq} {nstep}\n")
    for s in range(nseq):
        c = FBLControl(100, 10, False, False, False, 0.0, 0.0, 0, "bottom", 1)
        dt = [0.002, 0.004, 0.01, 0.002, 0.005, 0.002][s]; t = 0.0; pos = rng.normal(0, 0.3, 3); vel = rng.normal(0, 0.3, 3)
        for i in range(nstep):
            vel = vel + rng.normal(0, 0.05, 3); pos = pos + vel * dt
            fo = dict(x=rng.normal(0, 0.3, 3) * 0 + np.array([0.1 * np.sin(t), 0.1 * np.cos(t), 0.5]), x_dot=rng.normal(0, 0.1, 3), x_ddot=rng.normal(0, 0.5, 3), x_dddot=rng.normal(0, 2, 3))
            if i % 97 == 0 and i > 0: t_use = t                       # repeated time stamp: estimator skips the update
            acc, jerk = c._position_commands(t, pos, vel, fo)
            f.write(" ".join(f"{x:.9g}" for x in [t, *pos, *vel, *fo["x"], *fo["x_dot"], *fo["x_ddot"], *fo["x_dddot"], *acc, *jerk]) + "\n")
            if i % 97 != 96: t += dt                                  # every 97th step the time stamp repeats once
print("done")
