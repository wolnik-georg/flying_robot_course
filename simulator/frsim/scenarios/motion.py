"""Reference motion of both vehicles of a scenario: position, velocity, acceleration, jerk, yaw as functions of the scenario time."""
import numpy as np

from .. import config
from . import poly4d
from . import scenarios as _scenarios


class Motion:
    """A scenario cell evaluated along time. Robot 0 is the bottom (tuned) vehicle, robot 1 the top vehicle that causes the downwash."""

    def __init__(self, scenario, cell):
        self.scenario = scenario
        self.plan = getattr(_scenarios, scenario)(**cell)
        self.anchor = np.array(config.SCENARIO_ANCHOR_M, float)
        self.duration_s = min(r.curve.duration for r in self.plan.robots)
        self._cache = {}

    def kin(self, robot, t):
        """(position, velocity, acceleration, jerk, yaw) of a robot at scenario time t (s); results are cached (pure function of t)."""
        key = (robot, t)
        value = self._cache.get(key)
        if value is None:
            rp = self.plan.robots[robot]
            d = poly4d._derivs(rp.curve, min(t, rp.curve.duration))
            value = self._cache[key] = (self.anchor + rp.slot + d[0, :3], d[1, :3], d[2, :3], d[3, :3], d[0, 3])
        return value
