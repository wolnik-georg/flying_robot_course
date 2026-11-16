"""Stage 5 B1: the new fractional rpm-delay option of frsim (simulator/frsim/sensors.py, plant value rpm_delay_fractional, off by default).
 (1) option off = old behaviour: the quick and full equivalence tests (tests/test_equivalence.py, 199 / 199 episodes bit-identical, logs/equivalence_full.log) and the check below;
 (2) option on with a whole-ms delay (8.0 ms) = the whole-ms code path, trace bit-identical; (3) option on with 4.0502 ms differs from the truncated 4 ms (the interpolation acts)."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1])); sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import reference  # noqa
from frsim import config
from frsim.episode import EpisodeConfig, run_episode
from frsim.scenarios import tuning_cells
import loop_lib as L
g = L.gains_of(np.zeros(10)); cell = tuning_cells('A8')[0]
def tr(over, ctrl='S1'):
    return run_episode(EpisodeConfig(ctrl, 'A8', cell, g, 'nominal', 1000, plant=config.plant_with(over), noise=True, record_state=True)).state_trace
a = tr({}); b = tr({'rpm_delay_fractional': 0})
print('1 option off explicitly vs default: identical', np.array_equal(a, b))
c = tr({'rpm_delay_ms': 8.0}); d = tr({'rpm_delay_ms': 8.0, 'rpm_delay_fractional': 1})
print('2 8.0 ms whole-ms path vs fractional path: identical', np.array_equal(c, d), 'max |diff| %.3g' % np.abs(c - d).max())
e = tr({'rpm_delay_fractional': 1})
print('3 calibrated 4.0502 ms fractional vs truncated 4 ms: max position diff %.3f mm (must be > 0, small)' % (1000 * np.abs(e[:, 1:4] - a[:, 1:4]).max()))
f = tr({'rpm_delay_ms': 8.5, 'rpm_delay_fractional': 1}); print('4 8.5 ms fractional lies between 8 ms and 9 ms: ', end='')
g9 = tr({'rpm_delay_ms': 9.0}); d8 = np.abs(c[:, 1:4] - f[:, 1:4]).max(); d9 = np.abs(g9[:, 1:4] - f[:, 1:4]).max(); d89 = np.abs(c[:, 1:4] - g9[:, 1:4]).max(); print('|8-8.5| %.3f mm, |9-8.5| %.3f mm, |8-9| %.3f mm' % (1000 * d8, 1000 * d9, 1000 * d89))
