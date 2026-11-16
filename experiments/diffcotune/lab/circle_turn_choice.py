#!/usr/bin/env python3
"""Stage 4d: which turn of the circle file fits the current cf5 pad best (+90 or -90 deg), and the pad windows of all flight files on the corrected basis (path about the hover point = file - file(0),
crazyflie-firmware planner.c:335-348). Uses flightfiles/make_flightfiles.py (build, volume, pads). Output lab/results/circle_turn_choice.json. No pool, one process."""
import json, sys
sys.dont_write_bytecode = True
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'flightfiles'))
import make_flightfiles as M  # noqa
from frsim.scenarios import tuning_cells, study_cells, safety  # noqa
pad, pad2 = M.pads(); fs = safety.FLIGHT_SPACE; out = {'pad_cf5': pad.tolist(), 'pad_cf_second': pad2.tolist(), 'cases': {}}
for name, scen, cell, rot in (('CIRCLE unturned', 'CIRCLE', tuning_cells('CIRCLE')[0], 0.0), ('CIRCLE +90', 'CIRCLE', tuning_cells('CIRCLE')[0], 90.0), ('CIRCLE -90', 'CIRCLE', tuning_cells('CIRCLE')[0], -90.0),
                              ('FIG8 +90', 'FIG8', tuning_cells('FIG8')[0], 90.0), ('OVAL unturned', 'OVAL', study_cells('OVAL')[0], 0.0)):
    m, T, pieces = M.build(scen, cell, rot); t = np.arange(0, int(round(T * 1000)) + 1) / 1000.0; t = t[t <= T]
    got = M.sample(pieces, t) + np.array([0, 0, M.HEIGHT]); v = M.volume(got, pad, pad2, fs); out['cases'][name] = v
    print('%-16s about hover: x %.2f..%.2f y %.2f..%.2f | pad window x [%.2f, %.2f] y [%.2f, %.2f] | margin at pad %s -> min %.2f | to cf_second pad %.2f m | fits %s' % (name, v['path_about_hover_min'][0], v['path_about_hover_max'][0],
          v['path_about_hover_min'][1], v['path_about_hover_max'][1], *v['pad_window']['x'], *v['pad_window']['y'], {k: round(x, 2) for k, x in v['margin_at_pad_m'].items() if k in 'xy'}, v['min_margin_at_pad_m'], v['min_horizontal_distance_to_cf_second_pad_m'], v['fits_at_pad']))
json.dump(out, open(HERE / 'results' / 'circle_turn_choice.json', 'w'), indent=1)
