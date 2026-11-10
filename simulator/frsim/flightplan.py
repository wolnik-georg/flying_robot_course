"""Flight plan of one episode: smooth takeoff from the ground to the scenario start, hold, the scenario, smooth landing, time on the ground.

All ramps are smoothsteps (quintic) so velocity, acceleration and jerk setpoints are continuous; the scenario part comes from the vendored
scenario code (frsim.scenarios).
"""
import numpy as np

from . import config


def _smoothstep(u):
    return 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5


def _smoothstep_derivatives(u, duration):
    d1 = (30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4) / duration
    d2 = (60 * u - 180 * u ** 2 + 120 * u ** 3) / duration ** 2 if 0 < u < 1 else 0.0
    d3 = (60 - 360 * u + 360 * u ** 2) / duration ** 3 if 0 < u < 1 else 0.0
    return d1, d2, d3


class FlightPlan:
    def __init__(self, motion):
        self.motion = motion
        self.scenario_s = motion.duration_s
        self.total_s = config.TAKEOFF_S + config.HOLD_S + self.scenario_s + config.LANDING_S + config.GROUND_S
        self.start_position = motion.kin(0, 0.0)[0]
        self.end_position = None

    def scenario_time(self, t):
        """Time since the scenario start (negative during takeoff and hold)."""
        return t - config.TAKEOFF_S - config.HOLD_S

    def setpoint(self, t):
        """(position, velocity, acceleration, jerk) setpoints of the bottom vehicle at time t."""
        ts = self.scenario_time(t)
        if ts < 0:                                              # takeoff: ground -> scenario start, then hold
            u = min(max(t / config.TAKEOFF_S, 0.0), 1.0)
            s = _smoothstep(u)
            ds, dds, ddds = _smoothstep_derivatives(u, config.TAKEOFF_S)
            h = self.start_position[2] - 0.0
            p0 = self.start_position
            return (np.array([p0[0], p0[1], h * s]), np.array([0, 0, h * ds]), np.array([0, 0, h * dds]), np.array([0, 0, h * ddds]))
        if ts <= self.scenario_s:
            pr, vr, ar, jr, _ = self.motion.kin(0, ts)
            return pr, vr, ar, jr
        if self.end_position is None:
            self.end_position = self.motion.kin(0, self.scenario_s)[0]
        u = min(max((ts - self.scenario_s) / config.LANDING_S, 0.0), 1.0)     # landing: last scenario point -> ground
        s = _smoothstep(u)
        ds, dds, ddds = _smoothstep_derivatives(u, config.LANDING_S)
        h = self.end_position[2]
        p1 = self.end_position
        return (np.array([p1[0], p1[1], h * (1 - s)]), np.array([0, 0, -h * ds]), np.array([0, 0, -h * dds]), np.array([0, 0, -h * ddds]))

    def top_vehicle(self, t):
        """(position, velocity) of the top vehicle (it follows the scenario and stays at its start position before it)."""
        tp, tv, *_ = self.motion.kin(1, max(self.scenario_time(t), 0.0))
        return tp, tv
