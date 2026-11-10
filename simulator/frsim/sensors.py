"""Sensor model: noise on position, velocity, gyro, accelerometer and rpm telemetry, and the state delay.

One random generator per episode (explicit seed). The DRAW ORDER per 1 ms tick is fixed (it defines the numbers):
position (3), velocity (3),
gyro (3), then for S0 - S3 accelerometer (3) and rpm (4, zero standard deviation but drawn).
"""

import numpy as np

from . import config


class Sensors:
    """Noise and delay of all sensors of one episode (see the module docstring for the draw order)."""
    def __init__(self, seed, plant, variant, tree):
        self.rng = np.random.default_rng(seed)
        self.plant = plant
        self.gyro_noise = (
            plant.gyro_noise_deg_s
            if variant.gyro_noise_scale is None
            else plant.gyro_noise_deg_s * variant.gyro_noise_scale
        )
        self.delay_ms = int(variant.state_delay_ms)
        self.tree = tree
        self._state_queue = []
        self._rpm_queue = []

    def state(self, position, velocity):
        """Noisy position and velocity, delayed by the state delay (ms; 1 sample per ms)."""
        xm = position + self.rng.normal(0, config.STATE_NOISE_POSITION_M, 3)
        vm = velocity + self.rng.normal(0, config.STATE_NOISE_VELOCITY_M_S, 3)
        if self.delay_ms:
            self._state_queue.append((xm.copy(), vm.copy()))
            if len(self._state_queue) > self.delay_ms + 1:
                self._state_queue.pop(0)
            xm, vm = self._state_queue[0]
        return xm, vm

    def gyro(self, angular_velocity):
        """Gyro (deg/s) with vibration noise."""
        return np.degrees(angular_velocity) + self.rng.normal(0, self.gyro_noise, 3)

    def accelerometer(self, specific_force_g):
        """Accelerometer reading (g) = true specific force + noise."""
        return specific_force_g + self.rng.normal(0, self.plant.accelerometer_noise_g, 3)

    def rpm(self, motor_forces):
        """Rpm telemetry from the true motor forces, delayed by the rpm delay (ms); integer values like the DShot
        telemetry."""
        rpm = np.sqrt(np.maximum(motor_forces, 0.0) / config.MOTOR_RPM_TO_FORCE) * 60.0 / (2 * np.pi) + self.rng.normal(
            0, 0.0, 4
        )
        self._rpm_queue.append(rpm.copy())
        if len(self._rpm_queue) > int(self.plant.rpm_delay_ms) + 1:
            self._rpm_queue.pop(0)
        return [int(max(0, min(65535, value))) for value in self._rpm_queue[0]]
