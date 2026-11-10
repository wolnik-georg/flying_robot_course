"""Error measures. Units: cm. An episode that crashed counts as 100 cm.

episode error  = 3-D RMS of the position error (x, y, z) over the scoring window (from 2 s after the scenario start to
its end), or 100 if crashed
selection error = mean over noise seeds of the mean episode error on the TUNING cells x 9 robustness variants (used to
choose gains)
test error     = per robustness variant, mean over the study cells and the noise seeds (cells never used for tuning)
"""

import numpy as np

CRASH_ERROR_CM = 100.0


def episode_error(result):
    """Episode error (cm): 3-D RMS, or 100 for a crashed episode."""
    if not result.stable:
        return CRASH_ERROR_CM
    return float(np.sqrt(result.x_rms_cm**2 + result.y_rms_cm**2 + result.z_rms_cm**2))


def rms3d(result):
    """3-D RMS error (cm) of an episode, without the crash rule."""
    return (result.x_rms_cm**2 + result.y_rms_cm**2 + result.z_rms_cm**2) ** 0.5


def selection_error(errors_by_seed):
    """errors_by_seed: {seed: [episode errors in variant-major, cell-minor order]} -> (selection error, number of
    crashed episodes)."""
    value = float(np.mean([np.mean(v) for v in errors_by_seed.values()]))
    crashes = int(sum(x >= CRASH_ERROR_CM for v in errors_by_seed.values() for x in v))
    return value, crashes


def test_error(errors_by_seed):
    """errors_by_seed: {seed: [episode errors over the study cells]} -> {seed: mean over cells}, mean over seeds."""
    per_seed = {s: float(np.mean(x)) for s, x in errors_by_seed.items()}
    return per_seed, float(np.mean(list(per_seed.values())))
