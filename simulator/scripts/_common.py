"""Shared helpers of the command-line scripts: path setup, gain files, trace output."""
import csv
import sys
from pathlib import Path

SIMULATOR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SIMULATOR))

TRACE_COLUMNS = ['time_s', 'error_x_m', 'error_y_m', 'error_z_m', 'yaw_deg', 'tilt_deg', 'fbl_active', 'fbl_tripped', 'x_m', 'y_m', 'z_m']


def load_gains(controller, scenario, spec):
    """Gains dictionary from `spec`: 'default' (None), 'tuned' (gains/tuned/<controller>.yaml, scenario entry) or a YAML file.
    A file may be a flat mapping of gain names or the tuned format (scenario -> {gains: {...}})."""
    import yaml
    if spec in (None, 'default'):
        return None, 'default gains'
    if spec == 'tuned':
        path = SIMULATOR / 'gains' / 'tuned' / ('%s.yaml' % controller)
        if not path.exists():
            raise SystemExit('no tuned gains for %s yet (gains/tuned has S0 - S3); use --gains default or a file' % controller)
    else:
        path = Path(spec)
    data = yaml.safe_load(open(path))
    if scenario in data and isinstance(data[scenario], dict) and 'gains' in data[scenario]:
        entry = data[scenario]
        return dict(entry['gains']), 'tuned gains from %s (%s, seed %s)' % (path.name, entry.get('method', '?'), entry.get('seed', '?'))
    return dict(data), 'gains from %s' % path


def write_trace(path, trace):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(TRACE_COLUMNS)
        w.writerows(trace.tolist())


def try_plot(traces, out, title=None, labels=None):
    """Plot trace CSV files with scripts/plot_trace.py in a subprocess (so a broken matplotlib cannot disturb the simulation).
    Python tried: environment variable FRSIM_PLOT_PYTHON, then the current python. Returns the plot path or None."""
    import os
    import subprocess
    cmd_tail = [str(Path(__file__).parent / 'plot_trace.py')] + [str(t) for t in traces] + ['--out', str(out)]
    if title:
        cmd_tail += ['--title', title]
    if labels:
        cmd_tail += ['--labels'] + list(labels)
    for py in [os.environ.get('FRSIM_PLOT_PYTHON'), sys.executable]:
        if not py:
            continue
        done = subprocess.run([py] + cmd_tail, capture_output=True, text=True)
        if done.returncode == 0:
            return out
    print('plot skipped: no python with a working matplotlib found; set FRSIM_PLOT_PYTHON or run scripts/plot_trace.py on the trace CSV files')
    return None
