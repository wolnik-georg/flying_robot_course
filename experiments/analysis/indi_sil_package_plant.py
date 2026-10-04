#!/usr/bin/env python3
"""Quadrotor plant excerpt from CS2 np.py (no ROS dependency)."""

from __future__ import annotations

import numpy as np
import rowan


class Quadrotor:
    def __init__(self, state, params=None):
        self.mass = 0.034
        self.J = np.array([16.571710e-6, 16.655602e-6, 29.261652e-6])
        arm_length = 0.046
        arm = 0.707106781 * arm_length
        t2t = 0.006
        self.B0 = np.array(
            [
                [1, 1, 1, 1],
                [-arm, -arm, arm, arm],
                [-arm, arm, arm, -arm],
                [-t2t, t2t, -t2t, t2t],
            ]
        )
        self.g = 9.81
        self.inv_J = 1 / self.J
        self.kt = None
        self.drag = None
        self.motor_tau = None
        self._rpm = None
        if params:
            self.mass = float(params.get("mass", self.mass))
            if "inertia" in params:
                self.J = np.array(params["inertia"], dtype=float)
                self.inv_J = np.linalg.pinv(self.J) if self.J.shape == (3, 3) else 1 / self.J
            if "kt" in params:
                kt = params["kt"]
                self.kt = np.full(4, float(kt)) if np.isscalar(kt) else np.array(kt, dtype=float)
            if params.get("motor_tau"):
                self.motor_tau = float(params["motor_tau"])
            arm_length = float(params.get("arm_length", 0.046))
            t2t = float(params.get("t2t", 0.006))
            arm = 0.707106781 * arm_length
            self.B0 = np.array(
                [
                    [1, 1, 1, 1],
                    [-arm, -arm, arm, arm],
                    [-arm, arm, arm, -arm],
                    [-t2t, t2t, -t2t, t2t],
                ]
            )
        self.state = state

    def step(self, action, dt, f_a=np.zeros(3)):
        def rpm_to_force(rpm):
            if self.kt is not None:
                return self.kt * np.square(np.asarray(rpm, dtype=float))
            p = [2.55077341e-08, -4.92422570e-05, -1.51910248e-01]
            force_in_grams = np.polyval(p, rpm)
            return np.maximum(force_in_grams * 9.81 / 1000.0, 0)

        rpm = np.asarray(action.rpm, dtype=float)
        if self.motor_tau:
            if self._rpm is None:
                self._rpm = rpm.copy()
            alpha = dt / (self.motor_tau + dt)
            self._rpm = self._rpm + alpha * (rpm - self._rpm)
            rpm = self._rpm

        force = rpm_to_force(rpm)
        eta = np.dot(self.B0, force)
        f_u = np.array([0, 0, eta[0]])
        tau_u = np.array([eta[1], eta[2], eta[3]])

        pos_next = self.state.pos + self.state.vel * dt
        vel_next = self.state.vel + (
            np.array([0, 0, -self.g]) + (rowan.rotate(self.state.quat, f_u) + f_a) / self.mass
        ) * dt

        omega_global = rowan.rotate(self.state.quat, self.state.omega)
        q_next = rowan.normalize(rowan.calculus.integrate(self.state.quat, omega_global, dt))
        omega_next = self.state.omega + (self.inv_J * (np.cross(self.J * self.state.omega, self.state.omega) + tau_u)) * dt
        acc_body = (f_u + rowan.rotate(rowan.inverse(self.state.quat), f_a)) / (self.mass * self.g)

        self.state.pos = pos_next
        self.state.vel = vel_next
        self.state.quat = q_next
        self.state.omega = omega_next
        self.state.acc = acc_body
        self.state.rpm = np.asarray(rpm, dtype=float)

        if self.state.pos[2] < 0:
            self.state.pos[2] = 0
            self.state.vel = [0, 0, 0]
            self.state.omega = [0, 0, 0]
