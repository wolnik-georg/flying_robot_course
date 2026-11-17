"""Stage F1.3 information: how much does frsim itself move in the forward-check cells that failed the 0.5 % gate, under a 1e-6 relative change of the thrust-law gain (the diagnostic of Stage 3d, dryrun/stage3d/noise_sensitivity.py)?"""
import json, os, sys
from pathlib import Path
os.environ.setdefault('OMP_NUM_THREADS', '1')
sys.path.insert(0, str(Path(__file__).resolve().parents[1])); sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
from dataclasses import replace
import forward_check_fbl as FC
from frsim import config
from frsim.episode import EpisodeConfig, run_episode
SRC = sys.argv[1] if len(sys.argv) > 1 else 'forward_check_fbl.json'; DST = sys.argv[2] if len(sys.argv) > 2 else 'sensitivity_failed_cells.json'
tab = json.load(open(Path(__file__).resolve().parent / SRC))['table']
bad = [r for r in tab if r['pass'] is False]
out = []
for r in bad:
    sc = FC.SCEN_OF.get(r['scenario'], r['scenario']); cell = FC.CELLS[r['scenario']][1]; os.environ['FRSIM_FBL_VARIANT'] = r['variant']
    base = config.plant_with(FC.ROWS[r['row']]); pert = replace(base, thrust_law_gain_n=base.thrust_law_gain_n * (1 + 1e-6))
    res = []
    for p in (base, pert):
        tr = run_episode(EpisodeConfig('FBL', sc, cell, {}, 'nominal', 0, plant=p, noise=False, record_state=True)).state_trace
        from reference import build
        ref = build(sc, cell); res.append(FC.win_rms(tr[:, 0], tr[:, 1:4], tr[:, 14:17], ref.scenario_s))
    e = dict(variant=r['variant'], scenario=r['scenario'], row=r['row'], rel_diff_model_vs_frsim_pct=r['rel_diff_pct'], frsim_rms_cm=res[0], frsim_rms_perturbed_cm=res[1], frsim_change_pct=100 * abs(res[1] - res[0]) / res[0], crash_frsim=r['crash_frsim'], crash_model=r['crash_model'])
    out.append(e); print(e, flush=True)
json.dump(out, open(Path(__file__).resolve().parent / DST, 'w'), indent=1)
