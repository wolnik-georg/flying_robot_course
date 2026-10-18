#!/usr/bin/env python3
"""Validation of the S3 versions (2026-10-18), part 4: what Version B adds to the thrust, on the logged signals.

Version B adds f30(a_nn) to the INDI term while a_d -= a_nn stays, so the net NS2 contribution is the high-pass part
h = a_nn - f30(a_nn). a_nn is a 100 Hz zero-order hold sampled by the 500 Hz controller: h consists of (i) the smooth
high-pass of the prediction (its lead over the 30 Hz-filtered measurement) and (ii) a decaying pulse at every hold
step (the staircase artefact). Splits the two on the 500 Hz uSD logs of all A8/A1 flights (the prediction is logged
under every strategy) and compares them with the INDI term's own sample-to-sample activity.
Writes out/validation_s3/staircase.csv.
"""
import glob, os, sys
import numpy as np
import pandas as pd
from scipy.signal import butter, lfilter

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'flying_drone_stack', 'tools'))
import decode_usd_log  # noqa: E402
OUT = os.path.join(ROOT, 'experiments', 'analysis', 'out', 'validation_s3')
B, A = butter(2, 30.0, fs=500.0)


def main():
    rows = []
    for f in sorted(glob.glob(os.path.join(ROOT, 'experiments', 'logs', 'usd_raw', 'cf5_tree_S*_A[18]_*.bin'))):
        u = decode_usd_log.load(f)
        z = np.asarray(u['z'], float); air = z > 0.3
        nn = np.asarray(u['rnn_pred_z'], float)
        h = nn - lfilter(B, A, nn)                        # B's net NS2 contribution (high-pass of the hold signal)
        # smooth version: the prediction without the hold steps (linear interpolation between hold updates)
        steps = np.flatnonzero(np.diff(nn) != 0) + 1
        if len(steps) > 10:
            sm = np.interp(np.arange(len(nn)), steps, nn[steps])
        else:
            sm = nn.copy()
        h_smooth = sm - lfilter(B, A, sm)
        h_stair = h - h_smooth
        indi = np.asarray(u['omar_a_rpm_fz'], float) - np.asarray(u['omar_a_imu_fz'], float)
        indi_on = np.any(np.abs(indi) > 1e-6)
        name = os.path.basename(f).split('_thesis')[0].replace('cf5_', '')
        step_sizes = np.abs(np.diff(nn))[np.diff(nn) != 0]
        rows.append(dict(flight=name, hold_step_rate_hz=len(steps)/(len(nn)*0.002),
                         step_abs_p50=np.median(step_sizes) if len(step_sizes) else np.nan,
                         step_abs_p99=np.percentile(step_sizes, 99) if len(step_sizes) else np.nan,
                         h_total_rms=np.sqrt(np.mean(h[air]**2)), h_total_absmax=np.max(np.abs(h[air])),
                         h_smooth_rms=np.sqrt(np.mean(h_smooth[air]**2)), h_stair_rms=np.sqrt(np.mean(h_stair[air]**2)),
                         h_stair_absmax=np.max(np.abs(h_stair[air])),
                         indi_term_diff_rms=np.sqrt(np.mean(np.diff(indi[air])**2)) if indi_on else np.nan,
                         indi_term_rms=np.std(indi[air]) if indi_on else np.nan))
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(OUT, 'staircase.csv'), index=False)
    pd.set_option('display.width', 250)
    print(df.round(3).to_string(index=False))


if __name__ == '__main__':
    main()
