#!/usr/bin/env python3
"""Stage 4c, P1: the exact lab command for every S0 cell of the tuning loop and the check that it reproduces the frsim cell.
A1, A8, A4: the command line is parsed with the lab's own parser (crazyflie_examples.run_formation.build_parser, imported read-only from ~/Desktop/crazyswarm2, no bytecode written, nothing run, nothing
written there), the scenario is built (scenarios.build) and compiled to Poly4D in memory (poly4d.compile_curve, evaluated with poly4d.evaluate = what the drone flies), then compared with the frsim reference of the cell
(`reference.build`): (a) bottom drone path relative to its start, turned back by -rotate, 1 ms grid over the scenario time 0 .. scenario_s; (b) top minus bottom vector (dz, offset, passes) the same way;
(c) bottom start height (--height + slot z) against the frsim start z. Also: safety.check at the centred anchor ('checks passed' of the script), the same scenario time, and the rotate_deg the ingest needs.
CIRCLE, FIG8, OVAL: no run_formation scenario exists for them; the flight files of experiments/diffcotune/flightfiles/ are compared in checks.json (Stage 4a) and here against the duration of every existing
data/<shape>_mode1_kt*.csv of simple_flight.py. Output lab/results/cmd_check.json. No pool, one process."""
import json, os, sys
sys.dont_write_bytecode = True
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
CS2 = Path.home() / 'Desktop' / 'crazyswarm2' / 'crazyflie_examples'
sys.path.insert(0, str(CS2))
import reference as R  # noqa
from frsim.scenarios import study_cells, tuning_cells  # noqa
from crazyflie_examples import run_formation as RF  # noqa
from crazyflie_examples.formations import scenarios as SC, poly4d, safety  # noqa

PREFIX = ['--auto-center', '--yes']
CELLS = [  # (name, role, frsim scenario, cell, command arguments)
    ('A1', 'tuning', 'A1', tuning_cells('A1')[0], ['--scenario', 'A1', '--dz', '0.375', '--hold', '15.0', '--height', '0.5']),
    ('A8', 'tuning', 'A8', tuning_cells('A8')[0], ['--scenario', 'A8', '--dz', '0.375', '--duration', '8.0', '--passes', '2', '--height', '0.5']),
    ('A4 cell 1', 'test', 'A4', study_cells('A4')[0], ['--scenario', 'A4', '--dz', '0.4', '--speed', '0.35', '--offset', '0.125', '--motion', 'line', '--rotate', '90', '--height', '0.5']),
    ('A4 cell 2', 'test', 'A4', study_cells('A4')[1], ['--scenario', 'A4', '--dz', '0.4', '--speed', '0.35', '--offset', '0.075', '--motion', 'line', '--rotate', '90', '--height', '0.5']),
]
EXTRA = [('A8 as flown 23 Oct (study cell, --passes 4)', 'A8', study_cells('A8')[0], ['--scenario', 'A8', '--dz', '0.4', '--speed', '0.35', '--height', '0.5', '--passes', '4']),
         ('A8 tuning cell with --speed instead of --duration', 'A8', tuning_cells('A8')[0], ['--scenario', 'A8', '--dz', '0.375', '--speed', '0.2734375', '--passes', '2', '--height', '0.5'])]


def table_pos(tables, t):
    return np.array([poly4d.evaluate(tables, float(x))[:3] for x in t])


def check(name, sc_name, cell, argv):
    args = RF.build_parser().parse_args(argv + PREFIX)
    sc = SC.build(args.scenario, **RF.scenario_params(args))
    rot = float(args.rotate or 0.0)
    ref = R.build(sc_name, cell); t = ref.time_s; m = (t >= 0) & (t <= ref.scenario_s + 1e-9); tt = t[m]
    i0 = int(np.argmin(np.abs(t)))
    tabs = [poly4d.compile_curve(r.curve) for r in sc.robots]
    th = np.radians(-rot); Rz = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1.0]])
    lab_b = table_pos(tabs[0], tt); lab_t = table_pos(tabs[1], tt) if len(tabs) > 1 else None
    rel_b = (lab_b - lab_b[0]) @ Rz.T; ref_b = ref.pos[m] - ref.pos[i0]
    out = dict(name=name, command='ros2 run crazyflie_examples run_formation ' + ' '.join(argv + PREFIX), rotate_arg_deg=rot, ingest_rotate_deg=-rot + 0.0,
               lab_duration_s=float(sc.duration), frsim_scenario_s=float(ref.scenario_s), params=dict(sc.params), realised=dict(sc.realised),
               max_diff_bottom_path_mm=float(1e3 * np.abs(rel_b - ref_b).max()))
    if lab_t is not None and ref.has_top:
        d_lab = (lab_t - lab_b + np.array(sc.robots[1].slot) - np.array(sc.robots[0].slot) + (np.array(sc.robots[1].curve(0.0))[:3] * 0 if False else 0))
        # table_pos is relative to each drone's own slot: add the slot difference to get the top-minus-bottom vector
        d_lab = (lab_t - lab_b) + (np.array(sc.robots[1].slot) - np.array(sc.robots[0].slot))
        d_ref = ref.top_pos[m] - ref.pos[m]
        out['max_diff_top_minus_bottom_mm'] = float(1e3 * np.abs(d_lab @ Rz.T - d_ref).max())
        out['dz_start_lab_m'] = float((d_lab[0] @ Rz.T)[2]); out['dz_start_frsim_m'] = float(d_ref[0][2])
    out['start_z_lab_m'] = float(args.height + sc.robots[0].slot[2] + lab_b[0][2]); out['start_z_frsim_m'] = float(ref.pos[i0][2])
    lim = RF.make_limits(args); base = RF.centred_anchor(sc, lim, args.height)
    # Stage 4d: run_formation takes off to anchor + slot, goTo's there, then startTrajectory with relative=True (default): the firmware shifts each trajectory so that its FIRST point sits at the
    # current position (planner.c:335-348), so the absolute path is slot + (curve - curve(0)) about the anchor. Volume at the centred anchor (the pad xy only matters for the vertical takeoff and the goTo).
    first = [np.array(poly4d.evaluate(tb, 0.0)[:3]) for tb in tabs]; fs = safety.FLIGHT_SPACE; allp = []
    for r, tb, f0 in zip(sc.robots, tabs, first):
        pp = (table_pos(tb, tt) - f0 + np.array(r.slot) + np.array(base)); allp.append(pp)
    allp = np.concatenate(allp); lo, hi = allp.min(0), allp.max(0)
    out['curve_first_point_mm'] = [(1e3 * f0).round(4).tolist() for f0 in first]
    out['start_is_relative_with_shift'] = True
    out['absolute_path_min'] = lo.round(3).tolist(); out['absolute_path_max'] = hi.round(3).tolist()
    out['margin_to_flight_space_m'] = dict(x=float(min(lo[0] - fs['x'][0], fs['x'][1] - hi[0])), y=float(min(lo[1] - fs['y'][0], fs['y'][1] - hi[1])), z_floor=float(lo[2] - fs['z'][0]), z_ceiling=float(fs['z'][1] - hi[2]))
    out['safety_problems'] = safety.check(sc, lim, base=base); out['spec_problems'] = SC.check_spec(sc); out['anchor_xy_centred'] = base[:2].round(3).tolist()
    return out


if __name__ == '__main__':
    res = []
    for name, role, scn, cell, argv in CELLS:
        o = check(name, scn, cell, argv); o['role'] = role; o['cell'] = cell; res.append(o)
    for name, scn, cell, argv in EXTRA:
        o = check(name, scn, cell, argv); o['role'] = 'extra'; o['cell'] = cell; res.append(o)
    files = {}
    checks = json.load(open(HERE.parent / 'flightfiles' / 'checks.json'))
    for shape, key, dur in (('circle', 'circle_frsim_rot90_tuning_mode1.csv', None), ('figure8', 'figure8_frsim_rot90_tuning_mode1.csv', None), ('oval', 'oval_frsim_study_mode1.csv', None)):
        d_cell = checks[key]['duration_s']; ds = {}
        for f in sorted((CS2 / 'crazyflie_examples' / 'data').glob('%s_mode1_kt*.csv' % shape)):
            if 'onboard' in f.name: continue
            tab = np.loadtxt(f, delimiter=',', skiprows=1, usecols=range(33)); ds[f.name] = float(tab[:, 0].sum())
        near = min(ds.items(), key=lambda kv: abs(kv[1] - d_cell))
        files[shape] = dict(lab_cell=(shape != 'oval'), note=('OVAL dropped from the lab test set (user, 13 Nov)' if shape == 'oval' else ''), pad_window=checks[key]['pad_window'], fits_at_pad=checks[key]['fits_at_pad'], file=key, cell_duration_s=d_cell, max_pos_diff_mm=checks[key]['max_abs_position_diff_mm'], rotate_deg_in_file=checks[key]['rotate_deg'], n_existing_files=len(ds),
                            nearest_existing_file=near[0], nearest_existing_duration_s=near[1], nearest_duration_difference_s=abs(near[1] - d_cell), exact_duration_match=any(abs(v - d_cell) < 1e-3 for v in ds.values()))
    json.dump(dict(run_formation=res, solo_files=files), open(HERE / 'results' / 'cmd_check.json', 'w'), indent=1, default=str)
    for o in res:
        print('   first point of the curves (mm) %s | absolute path x %s..%s y %s..%s z %s..%s | margins %s' % (o['curve_first_point_mm'], o['absolute_path_min'][0], o['absolute_path_max'][0], o['absolute_path_min'][1], o['absolute_path_max'][1], o['absolute_path_min'][2], o['absolute_path_max'][2], {k: round(v, 3) for k, v in o['margin_to_flight_space_m'].items()}))
        print('%-48s dur lab %.3f / frsim %.3f s | bottom path %.4f mm | top-bottom %s mm | dz %s/%s | z0 %.3f/%.3f | safety %s spec %s | ingest rotate_deg %.0f' % (o['name'], o['lab_duration_s'], o['frsim_scenario_s'], o['max_diff_bottom_path_mm'],
              '%.4f' % o['max_diff_top_minus_bottom_mm'] if 'max_diff_top_minus_bottom_mm' in o else '-', '%.4f' % o.get('dz_start_lab_m', np.nan), '%.4f' % o.get('dz_start_frsim_m', np.nan), o['start_z_lab_m'], o['start_z_frsim_m'], o['safety_problems'] or 'ok', o['spec_problems'] or 'ok', o['ingest_rotate_deg']))
    print(json.dumps(files, indent=1))
