"""frsim: the thesis simulator. Real firmware controller code (S0, S1, S2, S3, FBL) in a calibrated model of our
brushless Crazyflie 2.1 with downwash.

Quick use:
    from frsim import run_episode, EpisodeConfig
    result = run_episode(EpisodeConfig('S0', 'A8', variant='md40', seed=0))
    print(result.stable, result.x_rms_cm, result.y_rms_cm, result.z_rms_cm)
Needs the compiled firmware bindings (simulator/firmware/build_bindings.sh; location via the environment variable
FRSIM_FIRMWARE_BUILD).
"""

from .episode import EpisodeConfig, EpisodeResult, run_episode  # noqa: F401

__version__ = '1.0'
