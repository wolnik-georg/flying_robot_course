#!/usr/bin/env python3
"""Stage 4c / 4d (proposal, NOT tested with ROS or on hardware): fly the Stage 4a flight files with the unchanged `simple_flight.py`, without copying anything into crazyswarm2.
simple_flight finds its file as DATA_DIR / '<trajectory>_mode1_kt<kt>.csv' (`_csv_path`, trajectory in figure8|circle|oval|hover). This wrapper replaces that one function in memory by a lookup of
experiments/diffcotune/flightfiles/<file>, sets `--trajectory` and a kt LABEL that no existing file uses (the label only names the log file / meta), and calls simple_flight.main().
Flight volume (Stage 4d fix): simple_flight hovers at initial_position + --height and calls startTrajectory with relative=True (crazyflie_py default, crazyflie.py:417-419). The firmware then shifts the
trajectory so that its FIRST point sits at the current position: shift = start_from - traj_eval(t_begin).pos (crazyflie-firmware planner.c:335-348). The path about the hover point is therefore
(file - file(0)), NOT the file itself (Stage 4c took the file origin as the hover point; that was wrong for files that start at the edge, e.g. the circle: x -1.5..0 about the hover point).
  python3 fly_file.py --cell CIRCLE --check [--pad X Y] [--other-pad X Y]   no ROS: file, pieces (<= 31, HLC_MAX_PIECES), duration, path about the hover point, pad window per axis, z window, margins at
                                                         the pad, smallest horizontal distance to the pad of the disabled cf_second
  python3 fly_file.py --cell FIG8 [-- <more simple_flight args>]   needs the CS2 / ROS environment of the lab PC, flies at --height 1.0 (refuses when the check fails)
Cells: CIRCLE (turned by +90 deg, user decision 13 Nov), FIG8 (turned by +90 deg). OVAL is dropped from the lab test set (user decision 13 Nov) and refused here. Default pads: cf5 and cf_second `initial_position`
read from ~/Desktop/crazyswarm2/crazyflie/config/crazyflies.yaml (read only; the lab PC's yaml decides at lab time)."""
import argparse, sys
sys.dont_write_bytecode = True
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent; FILES = HERE.parent / 'flightfiles'
CELLS = {'CIRCLE': ('circle', 'circle_frsim_rot90_tuning_mode1.csv', 0.0007), 'FIG8': ('figure8', 'figure8_frsim_rot90_tuning_mode1.csv', 0.0008)}
DROPPED = {'OVAL': 'OVAL is dropped from the lab test set (user decision 13 Nov); the file does not fit the pad (y window +-0.2 m, x window [0.31, 1.0] m)'}
YAML = Path.home() / 'Desktop' / 'crazyswarm2' / 'crazyflie' / 'config' / 'crazyflies.yaml'
FLIGHT_SPACE = {'x': (-1.0, 1.0), 'y': (-2.0, 2.0), 'z': (0.0, 1.70)}      # protocol draft / safety.FLIGHT_SPACE of crazyswarm2 (read from there at lab time if it differs)
HLC_MAX_PIECES = 31


def table(path):
    return np.loadtxt(path, delimiter=',', skiprows=1, usecols=range(33))


def sample(tab, n=4000):
    ts = np.linspace(0, tab[:, 0].sum(), n); ends = np.cumsum(tab[:, 0]); out = []
    for t in ts:
        k = min(int(np.searchsorted(ends, t, side='left')), len(tab) - 1); tl = t - (ends[k] - tab[k, 0])
        out.append([sum(tab[k, 1 + 8 * a + p] * tl ** p for p in range(8)) for a in range(3)])
    return np.array(out)


def yaml_pads():
    try:
        import yaml
        r = yaml.safe_load(open(YAML))['robots']; return list(r['cf5']['initial_position'][:2]), list(r['cf_second']['initial_position'][:2])
    except Exception:
        return [0.0, 0.3], [0.0, -0.3]


def check(cell, pad, other, height=1.0):
    shape, f, kt = CELLS[cell]; tab = table(FILES / f); xyz = sample(tab); rel = xyz - xyz[0]; lo, hi = rel.min(0), rel.max(0)
    print('%s: file %s, trajectory %s, kt label %g, %d pieces (limit %d), duration %.3f s' % (cell, f, shape, kt, len(tab), HLC_MAX_PIECES, tab[:, 0].sum()))
    print('  file frame: first point %s; path about the hover point (file - file(0)): x %.3f .. %.3f, y %.3f .. %.3f, z %.3f .. %.3f' % (xyz[0].round(3), lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
    ok = len(tab) <= HLC_MAX_PIECES
    for i, ax in enumerate('xy'):
        wlo, whi = FLIGHT_SPACE[ax][0] - lo[i], FLIGHT_SPACE[ax][1] - hi[i]
        inside = wlo <= pad[i] <= whi; ok &= inside
        print('  pad %s window [%.3f, %.3f] m; pad %s = %.3f -> %s (margin %.3f m)' % (ax, wlo, whi, ax, pad[i], 'inside' if inside else 'OUTSIDE the flight space', min(pad[i] - wlo, whi - pad[i])))
    zl, zh = height + lo[2], height + hi[2]; zin = FLIGHT_SPACE['z'][0] <= zl and zh <= FLIGHT_SPACE['z'][1]; ok &= zin
    print('  z at --height %.2f: %.3f .. %.3f m (flight space %.2f .. %.2f) -> %s' % (height, zl, zh, *FLIGHT_SPACE['z'], 'inside' if zin else 'OUTSIDE'))
    d = np.linalg.norm(np.array(pad)[None, :] + rel[:, :2] - np.array(other)[None, :], axis=1).min()
    print('  smallest horizontal distance of the path to the pad of the disabled cf_second %s: %.2f m (that drone is on the floor, the path is at z %.2f m; no limit is stated)' % (other, d, zl))
    print('CHECK %s' % ('ok' if ok else 'FAILED')); return 0 if ok else 1


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--yaml', help='crazyflies.yaml of the lab PC for the pads (default: the desk PC path)'); ap.add_argument('--cell', required=True, choices=sorted(list(CELLS) + list(DROPPED))); ap.add_argument('--check', action='store_true'); ap.add_argument('--pad', nargs=2, type=float, default=None); ap.add_argument('--other-pad', nargs=2, type=float, default=None)
    a, rest = ap.parse_known_args()
    if a.cell in DROPPED: sys.exit('refusing: ' + DROPPED[a.cell])
    if a.yaml: YAML = Path(a.yaml)
    yp, yo = yaml_pads(); a.pad = a.pad or yp; a.other_pad = a.other_pad or yo
    if a.check: sys.exit(check(a.cell, a.pad, a.other_pad))
    shape, f, kt = CELLS[a.cell]; path = FILES / f
    if check(a.cell, a.pad, a.other_pad): sys.exit('refusing to fly: check failed (pad position, see above)')
    sys.argv = ['simple_flight', '--trajectory', shape, '--kt', str(kt), '--height', '1.0'] + [r for r in rest if r != '--']
    from crazyflie_examples import simple_flight as SF
    SF._csv_path = lambda trajectory, kt_: path
    SF.main()
