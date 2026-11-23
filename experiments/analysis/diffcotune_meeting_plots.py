#!/usr/bin/env python3
"""Plots for the meeting doc 2026-11-26: DiffCoTune on hardware, S0-S3 (real flights, uSD logs only).

Run with the flying_robots pyenv:  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/diffcotune_meeting_plots.py
Inputs: experiments/logs/usd_merged/rounds/<S>/round_<l>/ (flights.json + merged csv), experiments/diffcotune/lab/<S>/ (gains, desk logs).
Flight pick per scenario = round.py's rule (first usable attempt, not crashed, no floor contact); ingest call identical to round.py.
"""
import json
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
LAB = ROOT / 'experiments/diffcotune/lab'
LOGS = ROOT / 'experiments/logs/usd_merged/rounds'
OUT = ROOT / 'docs/meetings/assets/2026-11-26'
OUT.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(LAB.parent)); sys.path.insert(0, str(LAB))
import usd_ingest as U  # noqa: E402
import round as RD  # noqa: E402

C = {'S0': '#2a6fdb', 'S1': '#d9480f', 'S2': '#0f9d76', 'S3': '#8e44c9'}
NAME = {'S0': 'S0 geometric', 'S1': 'S1 INDI', 'S2': 'S2 geometric + NS2', 'S3': 'S3 INDI + NS2'}
SCEN = ['A1', 'A8', 'A4', 'CIRCLE', 'FIG8']
def SD(s):
    """state dir of protocol v1 (frozen 23 Nov as lab/<S>_v1 by the protocol change); falls back to lab/<S>."""
    return LAB / (s + '_v1') if (LAB / (s + '_v1')).is_dir() else LAB / s


plt.rcParams.update({'font.size': 11, 'axes.spines.top': False, 'axes.spines.right': False, 'axes.grid': True, 'grid.alpha': .3})


def desk_costs(s):
    """real cost per flown round and per-flight costs, as logged by the desk step (desk_<l>.log)."""
    cost, flights = {}, {}
    for f in sorted(SD(s).glob('desk_*.log')):
        l = int(f.stem.split('_')[1]); txt = f.read_text()
        m = re.search(r'real 3-D RMS ([0-9.]+) cm', txt)
        if m: cost[l] = float(m.group(1))
        per = {}
        for sc, att, crashed, floor, v in re.findall(r'ingest \S+\s+(\w+) attempt (\d+): gate ok, crashed (\w+) floor (\w+), real ([0-9.]+) cm', txt):
            if crashed == 'False' and floor == 'False' and sc not in per: per[sc] = float(v)
        flights[l] = per
    return cost, flights


def predicted(s):
    """desk_<l> previews the set of round l+1: {l+1: predicted cost}."""
    out = {}
    for f in SD(s).glob('desk_*.log'):
        m = re.findall(r'new set ([0-9.]+) cm', f.read_text())
        if m: out[int(f.stem.split('_')[1]) + 1] = float(m[-1])
    return out


def gains(s, l):
    t = (SD(s) / ('round_%d' % l) / 'gains.yaml').read_text()
    return {k: float(v) for k, v in re.findall(r'(\w+):\s*([-0-9.e]+)\s*$', t, re.M)}


def trace(s, l, scen):
    folder = LOGS / s / ('round_%d' % l)
    man = json.load(open(folder / 'flights.json'))
    for m in sorted([m for m in man if m['scenario'] == scen and m['role'] == 'tuning'], key=lambda m: m['attempt']):
        g = U.ingest_flight(folder / m['file'], scen, RD.cell_of(scen, 'tuning', 0), top_from_log=scen in RD.TOP_FROM_LOG,
                            rotate_deg=m.get('rotate_deg'), lag_s=m.get('lag_s'))
        if g.get('usable') and not g['flags']['crashed'] and not g['flags']['floor']:
            tr = g['trace']; e = tr[:, 1:4] - tr[:, 14:17]
            return tr[:, 0], e, g['flags']['scored_from_s'], g['flags']['scored_to_s'], g['rms3d_cm']
    return None


D = {s: desk_costs(s) for s in C}
P = {s: predicted(s) for s in C}
FLIP = {'S0': 5, 'S2': 5}
BEST = {s: min(D[s][0], key=D[s][0].get) for s in C}

# 1. real cost per round ------------------------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(8, 4.6))
for s in ['S0', 'S2', 'S1', 'S3']:
    r = sorted(D[s][0]); v = [D[s][0][l] for l in r]
    ax.plot(r, v, '-o', color=C[s], lw=2.4, label=NAME[s])
    ax.plot(BEST[s], D[s][0][BEST[s]], 'o', ms=13, mfc='none', mec=C[s], mew=2)
    ax.annotate('%.2f' % D[s][0][BEST[s]], (BEST[s], D[s][0][BEST[s]]), textcoords='offset points', xytext={'S1': (12, -14), 'S3': (12, 6)}.get(s, (10, -4)), color=C[s], fontweight='bold')
    if s in FLIP: ax.plot(FLIP[s], 0.6 if s == 'S0' else 1.3, 'x', color=C[s], ms=12, mew=3)
ax.text(5, 2.0, 'round 5:\nflipped', ha='center', fontsize=9, color='0.35')
ax.set_xticks(range(6)); ax.set_xticklabels(['round %d' % i for i in range(6)])
ax.set_ylabel('real cost [cm]'); ax.set_ylim(0, 11.5)
ax.set_title('Real cost per round (5 tuning flights each); circle = best set so far')
ax.legend(frameon=False, loc='upper right')
fig.tight_layout(); fig.savefig(OUT / 'cost_per_round.png', dpi=150); plt.close(fig)

# 2. per-flight: round 0 vs best ----------------------------------------------------------------------------------
fig, axs = plt.subplots(1, 4, figsize=(15, 4), sharey=True)
x = np.arange(len(SCEN))
for ax, s in zip(axs, ['S0', 'S2', 'S1', 'S3']):
    f0 = D[s][1][0]; fb = D[s][1][BEST[s]] if BEST[s] != 0 else None
    last = max(D[s][1]); fl = D[s][1][last]
    ax.bar(x - .2, [f0.get(k, np.nan) for k in SCEN], .4, color='0.75', label='round 0 (default gains)')
    if fb is not None:
        ax.bar(x + .2, [fb.get(k, np.nan) for k in SCEN], .4, color=C[s], label='best: round %d' % BEST[s])
    else:
        ax.bar(x + .2, [fl.get(k, np.nan) for k in SCEN], .4, color=C[s], alpha=.45, label='round %d (worse)' % last)
    ax.set_xticks(x); ax.set_xticklabels(SCEN, fontsize=9); ax.set_title(NAME[s], color=C[s], fontweight='bold')
    ax.legend(frameon=False, fontsize=9)
axs[0].set_ylabel('real 3-D RMS [cm]')
fig.suptitle('Per flight: default gains vs tuned gains', y=1.0)
fig.tight_layout(); fig.savefig(OUT / 'per_flight.png', dpi=150, bbox_inches='tight'); plt.close(fig)

# 3. error over time, round 0 vs best, S0 and S2, all 5 tuning scenarios (2 rows of 3 / 2 per controller) ----------------------
fig, axs = plt.subplots(4, 3, figsize=(15, 13))
for i, s in enumerate(['S0', 'S2']):
    for j, sc in enumerate(SCEN):
        ax = axs[2 * i + j // 3, j % 3]
        for l, col, lab in [(0, '0.6', 'round 0'), (BEST[s], C[s], 'round %d' % BEST[s])]:
            r = trace(s, l, sc)
            if r is None: continue
            t, e, a, b, rms = r; m = (t >= a) & (t <= b)
            en = 100 * np.linalg.norm(e[m], axis=1); en = np.convolve(en, np.ones(10) / 10, mode='same')          # 20 ms moving mean: position and reference are held for several 2 ms log rows and step at different times (display only; RMS unchanged)
            ax.plot(t[m][5:-5], en[5:-5], color=col, lw=1.4, label='%s (%.1f cm)' % (lab, rms))
        ax.set_title('%s · %s' % (NAME[s], sc), color=C[s], fontsize=11)
        ax.legend(frameon=False, fontsize=9, loc='upper right')
        ax.set_ylabel('3-D position error [cm]'); ax.set_xlabel('time [s]')
    axs[2 * i + 1, 2].axis('off')
fig.suptitle('Position error over the scored part of the flight (bottom drone, uSD; 20 ms moving mean, RMS in legend unsmoothed)', y=1.0)
fig.tight_layout(); fig.savefig(OUT / 'error_over_time.png', dpi=150, bbox_inches='tight'); plt.close(fig)

# 4. gains: final / default, and KR/Kw per round ------------------------------------------------------------------
G = ['Kpos_Px', 'Kpos_Pz', 'Kpos_Dx', 'Kpos_Dz', 'Kpos_Iz', 'KR_x', 'KR_z', 'Kw_x', 'Kw_z', 'KI_x']
GL = ['pos P xy', 'pos P z', 'pos D xy', 'pos D z', 'pos I z', 'att P xy', 'att P z', 'rate P xy', 'rate P z', 'att I xy']
fig, a2 = plt.subplots(figsize=(8, 4.4))          # 26 Nov doc: relative-gain bars removed (gains are a table in the doc)
for s in ['S0', 'S2', 'S1', 'S3']:
    rr = sorted(set(D[s][0]) | ({FLIP[s]} if s in FLIP else set()))          # flown rounds only (unflown next-round gains left out)
    v = [gains(s, l)['KR_x'] / gains(s, l)['Kw_x'] for l in rr]
    a2.plot(rr, v, '-o', color=C[s], lw=2, label=s)
    if s in FLIP: a2.plot(FLIP[s], v[rr.index(FLIP[s])], 'X', color='crimson', ms=12, zorder=5)
    if s == 'S1' and 2 in rr: a2.plot(2, v[rr.index(2)], 'X', color='darkorange', ms=12, zorder=5); a2.text(2.1, v[rr.index(2)] - .6, 'oscillated', color='darkorange', fontsize=9)
a2.text(4.95, 15.1, 'flipped', color='crimson', ha='right', fontsize=9)
a2.set_xticks(range(6)); a2.set_xlabel('round'); a2.set_ylabel('KR / Kw (xy)')
a2.set_title('Attitude P / rate P ratio per round'); a2.legend(frameon=False, fontsize=9)
fig.tight_layout(); fig.savefig(OUT / 'gains.png', dpi=150); plt.close(fig)

# 5. predicted (simulator preview) vs real ------------------------------------------------------------------------
fig, axs = plt.subplots(1, 4, figsize=(15, 3.8), sharey=True)
for ax, s in zip(axs, ['S0', 'S2', 'S1', 'S3']):
    rr = sorted(P[s]); real = [D[s][0].get(l, np.nan) for l in rr]
    ax.plot(rr, [P[s][l] for l in rr], 's--', color=C[s], alpha=.6, label='simulator prediction')
    ax.plot(rr, real, 'o-', color=C[s], lw=2.2, label='real flight')
    for l in rr:
        if s in FLIP and l == FLIP[s]: ax.plot(l, 0.4, 'X', color='crimson', ms=11)
    ax.set_title(NAME[s], color=C[s], fontweight='bold'); ax.set_xticks(range(1, 6)); ax.set_xlabel('round')
    ax.legend(frameon=False, fontsize=9)
axs[0].set_ylabel('cost [cm]')
fig.suptitle('Simulator preview vs real flight (X = flipped on hardware, preview had passed)', y=1.02)
fig.tight_layout(); fig.savefig(OUT / 'predicted_vs_real.png', dpi=150, bbox_inches='tight'); plt.close(fig)

print('written to', OUT)
for s in C: print(s, 'cost', D[s][0], 'best', BEST[s], 'pred', P[s])

# ================================================================================================================
# additions 2026-11-22: z error, top-down x-y, simulator parameters, flight log
from functools import lru_cache


@lru_cache(maxsize=None)
def ftrace(s, l, scen):
    """(time, position, reference position, scored window, rms3d) of the flight round.py used (first usable attempt)."""
    folder = LOGS / s / ('round_%d' % l)
    man = json.load(open(folder / 'flights.json'))
    for m in sorted([m for m in man if m['scenario'] == scen and m['role'] == 'tuning'], key=lambda m: m['attempt']):
        g = U.ingest_flight(folder / m['file'], scen, RD.cell_of(scen, 'tuning', 0), top_from_log=scen in RD.TOP_FROM_LOG,
                            rotate_deg=m.get('rotate_deg'), lag_s=m.get('lag_s'))
        if g.get('usable') and not g['flags']['crashed'] and not g['flags']['floor']:
            tr = g['trace']; f = g['flags']
            return tr[:, 0], tr[:, 1:4], tr[:, 14:17], f['scored_from_s'], f['scored_to_s'], g['rms3d_cm']
    return None


def cmp_round(s):
    """round compared with round 0: best set, or the last flown round if round 0 is still best."""
    return BEST[s] if BEST[s] != 0 else max(D[s][0])


def zstats(s, l, scen):
    r = ftrace(s, l, scen)
    if r is None: return np.nan, np.nan
    t, p, ref, a, b, _ = r; m = (t >= a) & (t <= b); ez = 100 * (p[m, 2] - ref[m, 2])
    return float(np.sqrt(np.mean(ez ** 2))), float(np.mean(ez))


ORDER = ['S0', 'S2', 'S1', 'S3']
# 6. z RMS per scenario, round 0 vs tuned ---------------------------------------------------------------------------
fig, axs = plt.subplots(1, 4, figsize=(15, 4), sharey=True); x = np.arange(len(SCEN)); ZT = {}
for ax, s in zip(axs, ORDER):
    l = cmp_round(s); z0 = [zstats(s, 0, k) for k in SCEN]; z1 = [zstats(s, l, k) for k in SCEN]; ZT[s] = (l, z0, z1)
    ax.bar(x - .2, [v[0] for v in z0], .4, color='0.75', label='round 0 (default gains)')
    ax.bar(x + .2, [v[0] for v in z1], .4, color=C[s], alpha=1 if l == BEST[s] else .45, label=('best: round %d' if l == BEST[s] else 'round %d (worse)') % l)
    ax.set_xticks(x); ax.set_xticklabels(SCEN, fontsize=9); ax.set_title(NAME[s], color=C[s], fontweight='bold'); ax.legend(frameon=False, fontsize=9)
axs[0].set_ylabel('z RMS error [cm]')
fig.suptitle('Height (z) error per flight: default vs tuned gains', y=1.0)
fig.tight_layout(); fig.savefig(OUT / 'z_rms.png', dpi=150, bbox_inches='tight'); plt.close(fig)

# 7. signed z error over time, A1 / A8 / A4, all four controllers --------------------------------------------------
fig, axs = plt.subplots(4, 3, figsize=(15, 11), sharex='col', sharey=True)
for i, s in enumerate(ORDER):
    l = cmp_round(s)
    for j, sc in enumerate(['A1', 'A8', 'A4']):
        ax = axs[i, j]; ax.axhline(0, color='k', lw=.8)
        for rr, col, al in [(0, '0.6', 1), (l, C[s], 1 if l == BEST[s] else .6)]:
            r = ftrace(s, rr, sc)
            if r is None: continue
            t, p, ref, a, b, _ = r; m = (t >= a) & (t <= b); ez = 100 * (p[m, 2] - ref[m, 2])
            ax.plot(t[m], ez, color=col, alpha=al, lw=1.3, label='round %d (mean %+.1f cm)' % (rr, ez.mean()))
        ax.legend(frameon=False, fontsize=8, loc='lower right')
        if i == 0: ax.set_title(sc, fontweight='bold')
        if j == 0: ax.set_ylabel('%s\nz error [cm]' % NAME[s], color=C[s])
        if i == 3: ax.set_xlabel('time [s]')
axs[0, 0].set_ylim(-28, 14)
fig.suptitle('Height error over time (bottom drone, below the top drone): default (grey) vs tuned', y=1.0)
fig.tight_layout(); fig.savefig(OUT / 'z_error_over_time.png', dpi=150, bbox_inches='tight'); plt.close(fig)

# 8. top-down x-y + z error over time, CIRCLE / FIG8, S0 and S2 ----------------------------------------------------
fig = plt.figure(figsize=(12, 13.5))
gs = fig.add_gridspec(4, 2, height_ratios=[3, 1, 3, 1], hspace=.38, wspace=.18)
for i, s in enumerate(['S0', 'S2']):
    for j, sc in enumerate(['CIRCLE', 'FIG8']):
        ax = fig.add_subplot(gs[2 * i, j]); az = fig.add_subplot(gs[2 * i + 1, j])
        for rr, col, lw in [(0, '0.6', 1.3), (BEST[s], C[s], 1.6)]:
            t, p, ref, a_, b_, rms = ftrace(s, rr, sc)
            mv = np.where(np.linalg.norm(np.diff(ref[:, :2], axis=0), axis=1) > 1e-6)[0]
            jmp = np.where(np.linalg.norm(np.diff(ref[:, :2], axis=0), axis=1) > 0.02)[0]          # end: the reference snaps to the post-trajectory point (one-sample jump), not flown
            m = (t >= max(t[mv[0]], 0)) & (t <= (t[jmp[0]] if len(jmp) else t[mv[-1] + 1]))          # whole flown trajectory (reference moving), not only the scored window
            ms = (t >= a_) & (t <= b_); zr = 100 * np.sqrt(np.mean((p[ms, 2] - ref[ms, 2]) ** 2))
            if rr == 0: ax.plot(100 * ref[m, 0], 100 * ref[m, 1], 'k--', lw=1.2, label='reference')
            ax.plot(100 * p[m, 0], 100 * p[m, 1], color=col, lw=lw)
            az.plot(t[m], 100 * (p[m, 2] - ref[m, 2]), color=col, lw=lw, label='round %d: 3-D %.1f, z %.1f cm' % (rr, rms, zr))
        az.axvspan(a_, b_, color='0.92', zorder=0); az.axhline(0, color='k', lw=.8)
        ax.set_aspect('equal'); ax.set_xlabel('x [cm]'); ax.set_ylabel('y [cm]')
        ax.set_title('%s · %s' % (NAME[s], sc), color=C[s], fontweight='bold')
        az.set_xlabel('time [s]'); az.set_ylabel('z error [cm]'); az.legend(frameon=False, fontsize=8.5, loc='upper right', ncol=2)
fig.suptitle('Solo trajectories, whole flight: top-down path (dashed = reference) and height error; grey band = scored part, RMS over it', y=.92)
fig.savefig(OUT / 'xy_topdown.png', dpi=150, bbox_inches='tight'); plt.close(fig)

# 9. fitted simulator parameters per round --------------------------------------------------------------------------
PN = ['thrust gain', 'motor lag', 'downwash scale', 'plume scale', 'RPM delay (INDI only)']
fig, axs = plt.subplots(1, 5, figsize=(17, 3.8), sharex=True); FIT = {}
for s in ORDER:
    last = max(int(p.name.split('_')[1]) for p in SD(s).glob('round_*'))
    hist = json.load(open(SD(s) / ('round_%d' % last) / 'state.json'))['history']
    rr = [h['round'] for h in hist if 'psi_after_fit' in h]; ps = np.exp(np.array([h['psi_after_fit'] for h in hist if 'psi_after_fit' in h]))
    FIT[s] = dict(zip(rr, ps.round(3).tolist()))
    for k in range(ps.shape[1]): axs[k].plot(rr, ps[:, k], '-o', color=C[s], lw=2, label=s)
for k in range(5):
    axs[k].axhline(1, color='k', lw=.8); axs[k].set_title(PN[k]); axs[k].set_xlabel('after round'); axs[k].set_xticks(range(5))
axs[0].set_ylabel('factor vs calibrated model'); axs[0].legend(frameon=False, fontsize=9)
fig.suptitle('Simulator parameters fitted to the real flights after each round (1 = calibrated start model)', y=1.03)
fig.tight_layout(); fig.savefig(OUT / 'sim_params.png', dpi=150, bbox_inches='tight'); plt.close(fig)

# 10. flight log (from the desk logs: every flight the desk step ingested) ------------------------------------------
print('\nFLIGHT LOG')
for s in ORDER:
    tot = used = 0; bad = {}
    for f in sorted(SD(s).glob('desk_*.log')):
        for sc, att, gate, crashed, floor in re.findall(r'ingest \S+\s+(\w+) attempt (\d+): gate (ok|\[[^\]]*\]), crashed (\w+) floor (\w+)', f.read_text()):
            tot += 1
            why = 'gate' if gate != 'ok' else 'crash' if crashed == 'True' else 'floor' if floor == 'True' else None
            if why: bad[why] = bad.get(why, 0) + 1
            else: used += 1
    print(s, 'rounds', sorted(D[s][0]), 'ingested', tot, 'usable', used, 'not usable', bad)
print('\nZ TABLE (rms, mean) round 0 -> compared round')
for s in ORDER:
    l, z0, z1 = ZT[s]; print(s, l, ' | '.join('%s %.1f/%+.1f -> %.1f/%+.1f' % (k, a[0], a[1], b[0], b[1]) for k, a, b in zip(SCEN, z0, z1)))
print('\nFIT', json.dumps(FIT))
