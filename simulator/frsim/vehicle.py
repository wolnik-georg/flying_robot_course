"""Rigid-body vehicle: firmware mixer, command delay, motor lag, thrust law, wrench, and the equations of motion
(explicit Euler, 1 ms)."""

import numpy as np
from scipy.spatial.transform import Rotation

from . import config


def mixer_forces(fw, control):
    """Per-motor force command (N) from the firmware power distribution (force / torque command -> capped 16-bit motor
    command)."""
    uncapped = fw.motors_thrust_uncapped_t()
    fw.powerDistribution(control, uncapped)
    pwm = fw.motors_thrust_pwm_t()
    fw.powerDistributionCap(uncapped, pwm)
    command = np.array([pwm.motors.m1, pwm.motors.m2, pwm.motors.m3, pwm.motors.m4], float)
    return command / 65535.0 * config.THRUST_MAX_N


def wrench(forces):
    """Total thrust (N) and body torque (N m) from the four motor forces."""
    arm, t2t = config.ARM_TORQUE_M, config.THRUST_TO_TORQUE_M
    return forces.sum(), np.array(
        [
            arm * (-forces[0] - forces[1] + forces[2] + forces[3]),
            arm * (-forces[0] + forces[1] + forces[2] - forces[3]),
            t2t * (-forces[0] + forces[1] - forces[2] + forces[3]),
        ]
    )


def real_thrust(command_forces, max_force):
    """Real motor force from the battery-compensated command: quadratic law calibrated at hover, limited by the
    available force."""
    law = config.THRUST_LAW_GAIN_N * (command_forces / config.THRUST_LAW_HOVER_COMMAND) ** 2
    return np.minimum(law, max_force)


def cross(a, b):
    """Cross product of two 3-vectors."""
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


class RigidBody:
    """State: position x, velocity v (world), attitude quaternion q (x, y, z, w), body rates w (rad/s); motor forces
    F."""

    def __init__(self, start_xy):
        self.x = np.array([start_xy[0], start_xy[1], 0.0])
        self.v = np.zeros(3)
        self.q = np.array([0, 0, 0, 1.0])
        self.w = np.zeros(3)
        self.forces = np.full(4, config.MASS_KG * config.GRAVITY / 4)
        self.previous_acceleration = np.zeros(3)

    def step(self, rotation, rotation_obj, thrust, torque, external_force, lateral_thrust_scale, dt):
        """Advance one tick. `rotation` = body-to-world matrix at the start of the tick. Returns nothing; updates x, v,
        q, w."""
        mass, g = config.MASS_KG, config.GRAVITY
        v_old = self.v.copy()
        scale = np.array([lateral_thrust_scale, lateral_thrust_scale, 1.0])
        self.v = self.v + (np.array([0, 0, -g]) + rotation[:, 2] * scale * thrust / mass + external_force / mass) * dt
        self.x = self.x + self.v * dt
        self.previous_acceleration = (self.v - v_old) / dt
        if self.x[2] <= 0.0:  # ground contact
            self.x[2] = 0.0
            self.v[2] = max(self.v[2], 0.0)
            self.v[:2] *= 0.0 if thrust < mass * g else 1.0
        j = config.INERTIA_DIAG
        self.w = self.w + ((torque - cross(self.w, j * self.w)) / j) * dt
        self.q = (Rotation.from_rotvec(rotation @ self.w * dt) * rotation_obj).as_quat()
