"""Downwash truth: the force the top vehicle's propeller wash exerts on the bottom vehicle, and the torque of the wash on the rotors.

* Force: the NS2 v2 network (deep-set network trained on 73 flights; z force only), evaluated in float32 on the relative position and velocity of the
  top vehicle, scaled by the calibrated downwash scale. Weights: data/ns2_v2.npz. Network layout as in the firmware / training code.
* Torque: fitted Gaussian lift loss per rotor under the top vehicle's axis (plume model), calibrated amplitude and width.
"""
import numpy as np

from . import config

# Network layout (phi: neighbour / ground encoders, rho: decoder), ReLU layers and a linear output layer
HIDDEN = 20
PHI_IN_NEIGHBOUR = 6          # relative [dx, dy, dz, dvx, dvy, dvz]
PHI_IN_GROUND = 4             # ground effect [0 - z, -vx, -vy, -vz]
RHO_LAYERS = [(HIDDEN, 40), (40, 40), (40, 40), (40, 1)]
MAX_NEIGHBOURS = 3
OUTPUT_CLAMP = 8.0            # |force| clamp in the units of the network output
GATE_DXY = 0.2                # only neighbours closer than this horizontally (m) and slower than GATE_DVX (m/s) relative are used
GATE_DVX = 1.5
GRAMS_TO_NEWTONS = 9.81 / 1000.0


def _phi_layers(input_dim):
    return [(input_dim, 25), (25, 40), (40, 40), (40, HIDDEN)]


_N_PHI_NEIGHBOUR = sum(i * o + o for i, o in _phi_layers(PHI_IN_NEIGHBOUR))
_N_PHI_GROUND = sum(i * o + o for i, o in _phi_layers(PHI_IN_GROUND))
_N_RHO = sum(i * o + o for i, o in RHO_LAYERS)
# contiguous weight layout: phi_S | phi_L | phi_G | rho_S | rho_L
OFFSET_PHI_NEIGHBOUR = 0
OFFSET_PHI_GROUND = 2 * _N_PHI_NEIGHBOUR
OFFSET_RHO = OFFSET_PHI_GROUND + _N_PHI_GROUND


def _forward(weights, offset, layers, x, last_linear=True):
    x = np.asarray(x, np.float32)
    for k, (n_in, n_out) in enumerate(layers):
        w = weights[offset:offset + n_out * n_in].reshape(n_out, n_in)
        offset += n_out * n_in
        b = weights[offset:offset + n_out]
        offset += n_out
        x = (w @ x + b).astype(np.float32)
        if not (last_linear and k == len(layers) - 1):
            x = np.maximum(x, 0.0).astype(np.float32)
    return x


class Ns2Truth:
    """Downwash force network (NS2 v2)."""

    def __init__(self, weights_file=None):
        path = weights_file or (config.DATA_DIR / 'ns2_v2.npz')
        self.weights = np.asarray(np.load(path)['weights'], np.float32)

    def acceleration(self, neighbours, own_z, own_velocity, mass):
        """Acceleration (m/s^2, vector with only z non-zero) of the bottom vehicle caused by the neighbours.
        neighbours: list of (relative position, relative velocity) of the vehicles above, may be empty."""
        ground = [0.0 - own_z, -own_velocity[0], -own_velocity[1], -own_velocity[2]]
        rho_input = _forward(self.weights, OFFSET_PHI_GROUND, _phi_layers(PHI_IN_GROUND), ground)
        for dp, dv in neighbours[:MAX_NEIGHBOURS]:
            if not (abs(dp[0]) < GATE_DXY and abs(dp[1]) < GATE_DXY and abs(dv[0]) < GATE_DVX):
                continue
            x12 = [dp[0], dp[1], dp[2], dv[0], dv[1], dv[2]]
            rho_input = rho_input + _forward(self.weights, OFFSET_PHI_NEIGHBOUR, _phi_layers(PHI_IN_NEIGHBOUR), x12)
        force_grams = float(_forward(self.weights, OFFSET_RHO, RHO_LAYERS, rho_input)[0])
        az = force_grams * GRAMS_TO_NEWTONS / mass
        if not np.isfinite(az):
            return np.zeros(3, np.float32)
        if abs(az) > OUTPUT_CLAMP:
            return np.array([0.0, 0.0, np.sign(az) * OUTPUT_CLAMP], np.float32)
        return np.array([0.0, 0.0, az], np.float32)


def plume_torque(position, rotation, top_position, plant):
    """Torque (N m, x / y components) from the lift lost by each rotor inside the top vehicle's plume.
    position, rotation: bottom vehicle position and body-to-world rotation matrix; top_position: top vehicle position."""
    torque = np.zeros(3)
    for rx, ry in config.ROTOR_POSITIONS_M:
        rotor = position + rotation @ np.array([rx, ry, 0.0])
        rho2 = (rotor[0] - top_position[0]) ** 2 + (rotor[1] - top_position[1]) ** 2
        lift_change = -plant.plume_amplitude_n * (0.4 / max(top_position[2] - rotor[2], 0.05)) ** plant.plume_exponent \
            * np.exp(-rho2 / (2 * plant.plume_width_m ** 2))
        torque += np.array([ry * lift_change, -rx * lift_change, 0.0])
    return torque
