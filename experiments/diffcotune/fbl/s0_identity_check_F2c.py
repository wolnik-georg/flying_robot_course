"""Stage F2c.1 check (a): an S0 frsim episode with noise (A1, seed 1000, calibration plant; and A8) is bit-identical with the F2c sensors.py (FBL gyro low-pass) and with the sensors.py before F2c (simulator/frsim/sensors.py.before_F2c). Max abs difference of the full state traces."""
import importlib.util, sys, os
from pathlib import Path
H = Path(__file__).resolve().parent; sys.path.insert(0, str(H.parent)); sys.path.insert(0, str(H))
import numpy as np
import forward_check_fbl as FC
from frsim import config, episode
import frsim.sensors as NEW
from importlib.machinery import SourceFileLoader
ldr = SourceFileLoader('frsim.sensors_old', str(Path(config.__file__).parent / 'sensors.py.before_F2c')); spec = importlib.util.spec_from_loader('frsim.sensors_old', ldr)
old = importlib.util.module_from_spec(spec); old.__package__ = 'frsim'; ldr.exec_module(old)
for ctrl in ('S0', 'S2'):
    for scen in ('A1', 'A8'):
        trs = []
        for mod in (old, NEW):
            episode.sensors.Sensors = mod.Sensors
            r = episode.run_episode(episode.EpisodeConfig(ctrl, scen, FC.CELLS[scen][1], None, 'nominal', 1000, noise=True, record_state=True)); trs.append(r.state_trace)
        print(ctrl, scen, 'rows', len(trs[0]), 'max abs diff', float(np.max(np.abs(trs[0] - trs[1]))), flush=True)
