#!/usr/bin/env python3
"""Stage 4b item 5.2: real tilt per 23 Oct flight over the SCORED window only (flags scored_from_s .. scored_to_s of lab/results/ingest_oct23.json, i.e. without take-off and landing), bottom and top vehicle,
and the abort rule of docs/comparative_study_protocol.md S4 (|roll| or |pitch| > 25 deg for 0.5 s). Three readings of the rule are reported because the wording does not say whether 'for 0.5 s' is continuous:
longest continuous run, total time above 25 deg in any 1-s window (max), total fraction. Output lab/results/tilt_abort_check.json. No torch, no pool."""
import json
from pathlib import Path
import numpy as np, pandas as pd
HERE = Path(__file__).resolve().parent; BANK = HERE.parents[1] / 'logs' / 'bank_unseen_S0_2026-10-23'
meta = {r['folder']: r for r in json.load(open(HERE / 'results' / 'ingest_oct23.json')) if r['top'] == 'planned'}


def runs(mask, dt):
    best = cur = 0
    for m in mask:
        cur = cur + 1 if m else 0; best = max(best, cur)
    return best * dt


out = []
for folder, r in meta.items():
    df = pd.read_csv(BANK / folder / (folder + '_merged_usd.csv'), comment='#'); t = df['t'].values; dt = float(np.median(np.diff(t)))
    a, b = r['flags']['scored_from_s'], r['flags']['scored_to_s']; sm = (t >= a) & (t <= b); res = dict(folder=folder, scenario=r['scenario'], scored_s=[a, b])
    for veh in ('cf5_unseen', 'cf_second_unseen'):
        ro = df[veh + '.roll_deg'].values; pi = df[veh + '.pitch_deg'].values; tilt = np.maximum(abs(ro), abs(pi))[sm]; gy = abs(df[veh + '.gyro_y'].values)[sm]; gx = abs(df[veh + '.gyro_x'].values)[sm]
        over = (tilt > 25).astype(float); w = int(round(1.0 / dt)); roll1 = np.convolve(over, np.ones(w), 'valid') * dt
        res[veh] = dict(max_abs_roll=float(abs(ro[sm]).max()), max_abs_pitch=float(abs(pi[sm]).max()), pitch_p99_abs=float(np.percentile(abs(pi[sm]), 99)), tilt_p99=float(np.percentile(tilt, 99)),
                        frac_over_25=float(over.mean()), longest_continuous_over_25_s=runs(tilt > 25, dt), max_time_over_25_in_1s_window_s=float(roll1.max()), rule_fires_continuous_0p5s=bool(runs(tilt > 25, dt) >= 0.5),
                        max_abs_gyro_y_dps=float(gy.max()), max_abs_gyro_x_dps=float(gx.max()))
    out.append(res)
    bb, tt_ = res['cf5_unseen'], res['cf_second_unseen']
    print('%-24s scored %.0f-%.0f s | bottom: max roll %.1f pitch %.1f, pitch|.| p99 %.1f, >25: %.2f of time, longest run %.2f s, max in 1 s %.2f s, fires(cont. 0.5 s) %s, gyro_y max %.0f dps | top max roll %.2f pitch %.2f' % (
        folder, a, b, bb['max_abs_roll'], bb['max_abs_pitch'], bb['pitch_p99_abs'], bb['frac_over_25'], bb['longest_continuous_over_25_s'], bb['max_time_over_25_in_1s_window_s'], bb['rule_fires_continuous_0p5s'], bb['max_abs_gyro_y_dps'], tt_['max_abs_roll'], tt_['max_abs_pitch']))
json.dump(out, open(HERE / 'results' / 'tilt_abort_check.json', 'w'), indent=1)
