"""Robustness variants: changes of the plant that the tuning used to make gains robust and to test them.

md = measurement (state) delay in ms, ffs = scale of the acceleration / jerk feedforward (hardware behaves like about
0.4 in A8),
lag = motor time constant in ms, gn = gyro vibration noise scale, kt = scale of the lateral thrust components.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class Variant:
    """A change of the plant (delay, motor lag, noise, feedforward, lateral thrust)."""
    name: str
    description: str
    motor_lag_ms: Optional[float] = None
    gyro_noise_scale: Optional[float] = None
    state_delay_ms: int = 0
    feedforward_scale: Optional[float] = None
    lateral_thrust_scale: float = 1.0


NOMINAL = Variant('nominal', 'calibrated plant')
VARIANTS = {
    v.name: v
    for v in (
        NOMINAL,
        Variant('gn1.7', 'gyro vibration noise x1.7', gyro_noise_scale=1.7),
        Variant('md10', 'state delay 10 ms (measured on the drone)', state_delay_ms=10),
        Variant('md40', 'state delay 40 ms', state_delay_ms=40),
        Variant('md60', 'state delay 60 ms', state_delay_ms=60),
        Variant('ffs0.4', 'feedforward x0.4', feedforward_scale=0.4),
        Variant('ffs0.0', 'no feedforward', feedforward_scale=0.0),
        Variant('lag45', 'motor lag 45 ms', motor_lag_ms=45),
        Variant('lag60', 'motor lag 60 ms', motor_lag_ms=60),
        Variant('kt0.5', 'lateral thrust x0.5', lateral_thrust_scale=0.5),
        Variant('ffs0.4+md40', 'feedforward x0.4 and state delay 40 ms', state_delay_ms=40, feedforward_scale=0.4),
    )
}

# The variants of the tuning search, of the 9-variant selection stage, and of the test error (names as in VARIANTS)
SEARCH_VARIANTS = ['nominal', 'gn1.7', 'md40', 'ffs0.4', 'lag45']
SELECTION_VARIANTS = ['nominal', 'gn1.7', 'md40', 'ffs0.4', 'lag60', 'md60', 'ffs0.0', 'kt0.5', 'ffs0.4+md40']
TEST_VARIANTS = ['nominal', 'ffs0.4', 'md10', 'md40', 'md60']


def get(name_or_variant):
    """Variant by name (KeyError listing the valid names) or the variant itself."""
    if isinstance(name_or_variant, Variant):
        return name_or_variant
    if name_or_variant not in VARIANTS:
        raise KeyError('unknown variant %r, choose one of %s' % (name_or_variant, ', '.join(VARIANTS)))
    return VARIANTS[name_or_variant]


def with_delay(delay_ms):
    """Variant with an arbitrary state delay (ms)."""
    return Variant('md%d' % delay_ms, 'state delay %d ms' % delay_ms, state_delay_ms=int(delay_ms))
