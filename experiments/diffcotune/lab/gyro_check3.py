#!/usr/bin/env python3
"""Stage 4b item 2, part (a) completed: euler columns (roll, pitch with the legacy sign, yaw) -> body rates by the ZYX kinematic relation
p = phi' - psi' sin(theta); q = theta' cos(phi) + psi' sin(phi) cos(theta); r = psi' cos(phi) cos(theta) - theta' sin(phi), against the logged gyro: correlation and scale per axis.
(The plain derivative of the euler angles is not the body rate when the drone is tilted: the yaw axis then correlates badly, not an error of the log.) Output lab/results/gyro_check3.json."""
import csv, json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import usd_ingest as U  # noqa
import test_ingest_oct23 as T, gyro_check as G  # noqa

if __name__ == '__main__':
    rows = [r for r in csv.DictReader(open(T.PF)) if r['bank'] == 'bank_unseen_S0_2026-10-23']; out = []
    for r in rows:
        fl = U.load_merged(T.BANK / r['folder'] / (r['folder'] + '_merged_usd.csv')); c = fl.cols[fl.bottom]; t = fl.t; dt = float(np.median(np.diff(t)))
        e = np.unwrap(np.radians(np.stack([c['roll_deg'], -c['pitch_deg'], c['yaw_deg']], 1)), axis=0); d = np.zeros_like(e); k = 10; d[k:-k] = (e[2 * k:] - e[:-2 * k]) / (2 * k * dt)
        ph, th = e[:, 0], e[:, 1]
        b = np.stack([d[:, 0] - d[:, 2] * np.sin(th), d[:, 1] * np.cos(ph) + d[:, 2] * np.sin(ph) * np.cos(th), d[:, 2] * np.cos(ph) * np.cos(th) - d[:, 1] * np.sin(ph)], 1)
        gy = np.radians(np.stack([c['gyro_x'], c['gyro_y'], c['gyro_z']], 1)); m = (t > 5) & (t < t[-1] - 2)
        cs = G.corr_scale(G.ma(gy, 10), G.ma(b, 10), m); out.append(dict(folder=r['folder'], axes=cs))
        print('%-24s corr %s slope %s' % (r['folder'], [round(a['corr'], 3) for a in cs], [round(a['slope'], 2) for a in cs]))
    json.dump(out, open(HERE / 'results' / 'gyro_check3.json', 'w'), indent=1)
