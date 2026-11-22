"""Step 3 hover check of the DShot image from a MERGED uSD csv (one cf5 file, merge_usd_logs.py --roles bottom, or a round file).
  python3 hover_report.py <merged.csv> [t_from t_to]      (hover window in scenario seconds, default 6 16)
Prints, for the hover window: motor.mN_rpm range, thrust from those rpm (MOTORRPM2FORCE 3.91194e-8, rad/s), ctrlOmarIndi a_rpm_fz, a_imu_fz and their
difference (expect about +2.7 m/s2, Step 1), and the logged f_rpm. NOTE: the log variable ctrlOmarIndi.f_rpm is NEVER assigned in
controller_omar_indi.c (the local float f_rpm shadows self->f_rpm), so it reads 0 on every image; it is not an image test."""
import sys
import numpy as np

path = sys.argv[1]
t0, t1 = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (6.0, 16.0)
rows = [l for l in open(path) if not l.startswith('#')]
h = rows[0].strip().split(',')
D = np.array([[float(v) if v not in ('', 'nan') else np.nan for v in r.strip().split(',')] for r in rows[1:]])
pre = h[1].split('.')[0] + '.'
col = lambda n: D[:, h.index(pre + n)]
t = D[:, 0]
m = (t > t0) & (t < t1)
rpm = np.array([col(f'motor_m{i}_rpm') for i in (1, 2, 3, 4)]).T
k = 3.91194027330774e-08
valid = ((rpm > 0) & (rpm < 28000)).all(axis=1)   # DShot spikes / 0xFFFF sentinel are rejected by the firmware filter; here they are only counted
f = (k * (rpm * 2 * np.pi / 60.0) ** 2).sum(axis=1)
a_r, a_i = col('omar_a_rpm_fz'), col('omar_a_imu_fz')
print(f'{path}  drone {pre[:-1]}  hover window {t0}-{t1} s: {int(m.sum())} samples')
bad = float(np.mean(~valid[m]))
print('motor.mN_rpm hover median:', np.round(np.nanmedian(rpm[m], axis=0)), ' 1-99 % range', int(np.nanpercentile(rpm[m], 1)), '-', int(np.nanpercentile(rpm[m], 99)),
      ' samples outside 0 < rpm < 28000: %.2f %%' % (100 * bad))
print('thrust from DShot rpm in hover (valid samples): %.3f N (expect about 0.54 N = m*(g + 2.7); m*g = 0.419 N)' % np.nanmean(f[m & valid]))
print('ctrlOmarIndi a_rpm_fz %.2f  a_imu_fz %.3f  difference %.2f m/s2 (expect about +2.7)' % (np.nanmean(a_r[m]), np.nanmean(a_i[m]), np.nanmean(a_r[m] - a_i[m])))
print('logged ctrlOmarIndi.f_rpm mean %.3f (always 0, never assigned: not a test)' % np.nanmean(col('omar_f_rpm')[m]))
ok = (np.nanpercentile(rpm[m], 1) > 1000) and (bad < 0.01) and abs(np.nanmean(a_r[m] - a_i[m]) - 2.7) < 1.0
print('RESULT:', 'OK' if ok else 'CHECK / STOP')
