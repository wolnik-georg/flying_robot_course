#!/usr/bin/env python3
"""Plot trace CSV files written by run_scenario.py: position error (x, y, z) and tilt over the scenario time.
  python3 scripts/plot_trace.py trace_S0_A8_nominal_cell0.csv [more.csv ...] [--out plot.png]
Needs numpy and matplotlib only (no firmware): use any python where matplotlib works. Several files are overlaid (one colour per file)."""
import argparse
import csv

import numpy as np

COLORS = ['#1f6feb', '#d1481f', '#2a9d5c', '#8a5cc2', '#8c6d1f']


def read(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    head = rows[0]
    return head, np.array(rows[1:], float)


def plot(paths, out, title=None, labels=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(4, 1, figsize=(8, 8), sharex=True)
    for k, path in enumerate(paths):
        head, d = read(path)
        t = d[:, 0]
        keep = t >= 0
        label = (labels[k] if labels else path.split('/')[-1].replace('trace_', '').replace('.csv', ''))
        c = COLORS[k % len(COLORS)]
        for j, name in enumerate(['x', 'y', 'z']):
            ax[j].plot(t[keep], 100 * d[keep, 1 + j], color=c, lw=1.2, label=label)
        ax[3].plot(t[keep], d[keep, 5], color=c, lw=1.2)
    for j, name in enumerate(['x error (cm)', 'y error (cm)', 'z error (cm)', 'tilt (deg)']):
        ax[j].set_ylabel(name)
        ax[j].grid(alpha=0.3)
    ax[3].set_xlabel('scenario time (s)')
    if len(paths) > 1 or labels:
        ax[0].legend(loc='upper right', fontsize=8, ncol=2)
    if title:
        ax[0].set_title(title)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('traces', nargs='+')
    ap.add_argument('--out', default='trace.png')
    ap.add_argument('--title', default=None)
    ap.add_argument('--labels', nargs='+', default=None, help='legend names, one per file')
    a = ap.parse_args()
    print(plot(a.traces, a.out, a.title, a.labels))
