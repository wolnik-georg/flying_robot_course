"""Self-test of the frame mapping for the turned figure-8 flight file: a fake merged file whose logged command is the rotated flight file with a 80 ms lead and a (0.1, -0.2) m anchor shift;
usd_ingest.estimate_frame must return theta = -90 deg (turning it back), lag 80 ms and the shift, with a ~0 residual."""
import sys, tempfile
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'flightfiles'))
import make_flightfiles as M, usd_ingest as U, fake_usd as F, reference as R  # noqa
from frsim.scenarios import tuning_cells  # noqa
cell = tuning_cells('FIG8')[0]; ref = R.build('FIG8', cell); pf = M.read_csv(HERE.parent / 'flightfiles' / 'figure8_frsim_rot90_tuning_mode1.csv')
t = np.arange(-4.5, ref.scenario_s + 3.0, 0.002); lead = 0.080; shift = np.array([0.1, -0.2, 0.0])
tc = np.clip(t + lead, 0, ref.scenario_s)
p = M.sample(pf, tc) + np.array([0, 0, 1.0]) + shift
tr = np.zeros((len(t), 23)); tr[:, 0] = t; tr[:, 1:4] = p; tr[:, 7] = 1e-4 * np.sin(t); tr[:, 10] = np.sqrt(1 - tr[:, 7] ** 2); tr[:, 14:17] = p
f = tempfile.mktemp(suffix='.csv'); F.write_merged(f, tr, 'FIG8', cell)
fl = U.load_merged(f); fr = U.estimate_frame(fl, ref); print(fr)
g = U.ingest_flight(f, 'FIG8', cell); print('aligned max diff %.2e m' % g['reference_check']['max_abs_scored_m'], 'real cost %.2e cm (perfect tracking of the shifted file)' % g['rms3d_cm'])
