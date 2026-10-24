#!/usr/bin/env python3
"""Test vectors for fbl_law_position_dt2 (ctrlFbl.abg_on_new = 1) from the AUTHORS' code: FBLControl._position_commands called every 2 ms controller
tick (integral at the tick rate), its alpha-beta-gamma estimator updated ONLY when the velocity measurement changes (irregular, like the EKF on the drone:
mostly every 2-4 ticks), i.e. the bundle estimator driven at the measurement rate. t starts at 0 as the bundle assumes.
  ~/.pyenv/versions/flying_robots/bin/python gen_law_vectors_abg.py   -> law_vectors_abg.txt"""
import sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE.parents[1]))
from common import bundle as B  # noqa: E402
B.load(); sys.path.insert(0, str(B.BUNDLE))
from simulator_multi_python.Controllers.fbl_control import FBLControl  # noqa: E402


class OnNew:
    """forward update() to the bundle estimator only for a new velocity sample"""
    def __init__(self, est): self.est = est; self.last = None
    def update(self, t, vel):
        if self.last is None or not np.array_equal(vel, self.last): self.last = vel.copy(); return self.est.update(t, vel)
        return self.est.acc_estimate


rng = np.random.default_rng(23)
with open(HERE / "law_vectors_abg.txt", "w") as f:
    nseq, nstep = 6, 600; f.write(f"{nseq} {nstep}\n")
    for s in range(nseq):
        c = FBLControl(100, 10, False, False, False, 0.0, 0.0, 0, "bottom", 1); c.acc_estimator = OnNew(c.acc_estimator)
        dt = 0.002; pos = rng.normal(0, 0.3, 3); vel = rng.normal(0, 0.3, 3).astype(np.float32).astype(float); next_new = 0
        for i in range(nstep):
            t = i * dt
            if i >= next_new:                                    # new EKF velocity sample after 1-4 ticks (median ~3)
                vel = (vel + rng.normal(0, 0.05, 3)).astype(np.float32).astype(float); next_new = i + int(rng.choice([1, 2, 3, 3, 3, 4]))
            pos = pos + vel * dt
            fo = dict(x=np.array([0.1 * np.sin(t), 0.1 * np.cos(t), 0.5]), x_dot=rng.normal(0, 0.1, 3), x_ddot=rng.normal(0, 0.5, 3), x_dddot=rng.normal(0, 2, 3))
            acc, jerk = c._position_commands(t, pos, vel, fo)
            f.write(" ".join(f"{x:.9g}" for x in [t, *pos, *vel, *fo["x"], *fo["x_dot"], *fo["x_ddot"], *fo["x_dddot"], *acc, *jerk]) + "\n")
print("done")
