#!/usr/bin/env python3
"""Validation of the S3 versions (2026-10-18), part 2: crossing-aligned profiles of the A8 flights of 2026-10-15.

Crossing = relative y (top - bottom) passes zero (as tree_validation_analysis.py). For each strategy, the mean over all
crossings of: bottom z error (relative to its steady offset), measured residual a_res (offline, the controller's
definition), NS2 prediction, commanded thrust / m and RPM thrust / m. Also the lead of the NS2 prediction over the
measured residual (cross-correlation, every strategy; NS2 is evaluated in all of them) and per-crossing dip and peak.
Reads out/validation_s3/aligned_*.npz (run validation_s3_c2_gates.py first). Writes out/validation_s3/events.npz,
events_summary.csv and fig_events.png.
"""
import glob, os
import numpy as np
import pandas as pd

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
OUT = os.path.join(ROOT, 'experiments', 'analysis', 'out', 'validation_s3')
MASS, G, DT = 0.0427, 9.81, 0.01
W = np.arange(-1.0, 1.0 + 1e-9, DT)


def crossings(t, dy):
    s = np.sign(dy); idx = np.where(np.diff(s) != 0)[0]
    out = []
    for i in idx:
        tc = t[i] - dy[i] * (t[i+1]-t[i]) / (dy[i+1]-dy[i])
        if not out or tc - out[-1] > 1.5:
            out.append(tc)
    return out


def main():
    prof, rows = {}, []
    for f in sorted(glob.glob(os.path.join(OUT, 'aligned_tree_S*_A8_*.npz'))):
        name = os.path.basename(f)[8:-4]; strat = name.split('_')[1]
        d = dict(np.load(f)); t = d['t']; ez = d['z'] - d['zref']
        ok = np.isfinite(d['dy'])
        tcs = [tc for tc in crossings(t[ok], d['dy'][ok]) if t[0] + 1.3 < tc < t[-1] - 1.3]
        far = np.ones_like(t, bool)
        for tc in tcs:
            far &= np.abs(t - tc) > 1.2
        off = np.median(ez[far & (d['z'] > 0.3)])
        sig = dict(ez=(ez - off)*100, ares=d['ares_off'], pred=d['pred'],
                   thr=d['thrust_cmd']/MASS - G, rpm=d['f_rpm_off']/MASS - G)
        for tc in tcs:
            seg = {k: np.interp(tc + W, t, v) for k, v in sig.items()}
            prof.setdefault(strat, []).append(seg)
            w = np.abs(W) <= 0.8
            rows.append(dict(flight=name, strategy=strat, t_cross=tc, dip_cm=seg['ez'][w].min(), peak_cm=seg['ez'][w].max(),
                             ares_min=seg['ares'][w].min(), ares_int=np.sum(seg['ares'][w])*DT,
                             pred_min=seg['pred'][w].min(), pred_int=np.sum(seg['pred'][w])*DT, vbat=np.median(d['vbat'])))
        # lead of the prediction over the measured residual, whole flight in the air
        air = d['z'] > 0.3
        a = d['ares_off'][air] - d['ares_off'][air].mean(); p = d['pred'][air] - d['pred'][air].mean()
        lags = np.arange(-30, 31)
        xc = [np.corrcoef(np.roll(p, L), a)[0, 1] for L in lags]
        rows[-1]['lead_ms_flight'] = float(lags[int(np.argmax(xc))]*10)
        rows[-1]['xcorr_max'] = float(np.max(xc))
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(OUT, 'events_summary.csv'), index=False)
    mean = {s: {k: np.mean([seg[k] for seg in v], 0) for k in v[0]} for s, v in prof.items()}
    np.savez(os.path.join(OUT, 'events.npz'), W=W, **{f'{s}_{k}': v for s, m in mean.items() for k, v in m.items()})
    pd.set_option('display.width', 220)
    print(df.groupby('strategy')[['dip_cm', 'peak_cm', 'ares_min', 'ares_int', 'pred_min', 'pred_int']].agg(['mean', 'std']).round(2))
    print(df.dropna(subset=['lead_ms_flight'])[['flight', 'lead_ms_flight', 'xcorr_max']].to_string(index=False))
    for s, m in sorted(mean.items()):
        i0 = int(np.argmin(m['ez'])); ia = int(np.argmin(m['ares'])); ip = int(np.argmin(m['pred']))
        print(f"{s}: mean-profile dip {m['ez'].min():+.1f} cm at {W[i0]*1000:+.0f} ms; a_res min {m['ares'].min():+.2f} at {W[ia]*1000:+.0f} ms; "
              f"pred min {m['pred'].min():+.2f} at {W[ip]*1000:+.0f} ms; a_res FWHM {np.sum(m['ares'] < m['ares'].min()/2)*DT*1000:.0f} ms; "
              f"thrust cmd/m peak rise {m['thr'].max()-m['thr'][:20].mean():+.2f}, rpm/m rise {m['rpm'].max()-m['rpm'][:20].mean():+.2f} m/s^2")
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(7, 8), sharex=True)
        for s, m in sorted(mean.items()):
            ax[0].plot(W, m['ez'], label=s); ax[1].plot(W, m['ares'], label=s); ax[2].plot(W, m['pred'], label=s)
        ax[0].set_ylabel('z error [cm]'); ax[1].set_ylabel('a_res [m/s^2]'); ax[2].set_ylabel('NS2 pred [m/s^2]')
        ax[2].set_xlabel('time from crossing [s]'); ax[0].legend(); [a.grid(alpha=.3) for a in ax]
        fig.tight_layout(); fig.savefig(os.path.join(OUT, 'fig_events.png'), dpi=120)
    except Exception as e:  # plotting is optional
        print('plot skipped:', e)


if __name__ == '__main__':
    main()
