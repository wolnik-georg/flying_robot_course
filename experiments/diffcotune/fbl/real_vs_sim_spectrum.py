"""F2b.2: dominant frequency of the tilt signal in the real A1 FBL flights (complete uSD logs, cf5) and in the frsim noisy A1 runs (a1_noise_diag_traces.json). Window: scenario time 2 ... 11 s; detrended; peak of the FFT above 0.3 Hz."""
import glob, json, sys
from pathlib import Path
import numpy as np, pandas as pd
H = Path(__file__).resolve().parent; L = Path.home() / 'Desktop/flying_robot_course/experiments/logs'
def peak(t, x, lo=0.3):
    w = (t > 2) & (t <= 11); x = x[w] - x[w].mean(); dt = np.median(np.diff(t[w])); F = np.abs(np.fft.rfft(x * np.hanning(len(x)))); f = np.fft.rfftfreq(len(x), dt); m = f > lo
    return float(f[m][np.argmax(F[m])]), float(x.std())
for b, fl in [('bank_fbl2_2026-10-24', 'A1_2026-10-24_14-30-39'), ('bank_fbl3_2026-10-24', 'A1_2026-10-24_14-57-11'), ('bank_fbl_b1_ours_2026-10-26', 'A1_2026-10-26_17-28-19'), ('bank_fbl_b3_cd055_2026-10-26', 'A1_2026-10-26_18-08-05'), ('bank_a1clean_2026-10-19', 'A1_2026-10-19_19-16-08')]:
    d = pd.read_csv(glob.glob(str(L / b / fl / '*.csv'))[0], comment='#'); p = sorted({c.split('.')[0] for c in d.columns if c.startswith('cf5')})[0]; t = d['t'].to_numpy()
    q = d[[p + '.est_qx', p + '.est_qy']].to_numpy(); tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q ** 2).sum(1), -1, 1)))
    print('real  %-30s %-24s tilt peak %.2f Hz  std %.1f deg' % (b, fl, *peak(t, tilt)))
d = json.load(open(H / 'a1_noise_diag_traces.json'))
for k in ('calibration_1000', 'calibration_1001', 'P1_1000'):
    t = np.array(d[k]['t']); tilt = np.array(d[k]['tilt']); print('frsim FBL base A1 %-18s tilt peak %.2f Hz  std %.1f deg (P1: crash after %.1f s, window cut)' % (k, *peak(t, tilt), 0 if d[k]['summary']['crash_time_s'] is None else d[k]['summary']['crash_time_s']))
