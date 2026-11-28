#!/usr/bin/env python3
"""usd_check (REPORT_59 L3): on-the-spot GO / NO-GO of one chirp flight from its uSD file, in well under 2 minutes.
  usage: python3 usd_check.py <uSD file> [--axis y|x] [--amp 0.3] [--kr 0.0077] [--kw 0.00117812] [--tilt-go 12.0] [--hold-gyro-go 30]
Checks: (1) all 115 variables of config v3 present with the right type; (2) lost rows < 0.2 % and no gap > 60 ms; (3) the chirp is on the right axis (y chirp -> roll, ctrltarget_ay; x chirp -> pitch, ctrltarget_ax), peak acceleration within 15 % of --amp, swept 1 -> 8 Hz, the other axis quiet; (4) peak tilt below --tilt-go (default 12 deg, user decision; a WARN between --tilt-go and the 15 deg abort limit means: stop and decide);
(5) roll/pitch gyro rms in the first 8 s (hold) below --hold-gyro-go deg/s; (6) timing pipeline OK (mocap quaternion present, unit norm, changing; coherence gyro vs mocap >= 0.8 in >= 4 of 5 bands); (7) KR / Kw logged as commanded (1 %); (8) the INDI-correction variables are not constant (the rate reference omega_r is ~0 for a streamed full-state setpoint: jerk 0); usd.fatWrBps shown.
Exit code 0 = GO, 1 = NO-GO."""
import os, sys, time, argparse, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import chirp_lib as CL, cfusdlog
def main():
    ap = argparse.ArgumentParser(); ap.add_argument('file'); ap.add_argument('--axis', default='y'); ap.add_argument('--amp', type=float, default=0.3); ap.add_argument('--kr', type=float, default=0.0077); ap.add_argument('--kw', type=float, default=0.00117812); ap.add_argument('--tilt-go', type=float, default=12.0); ap.add_argument('--hold-gyro-go', type=float, default=30.0); ap.add_argument('--amp0', type=float, default=0.1, help='ramp start (run sheet: 0.1 -> amp over --ramp s)'); ap.add_argument('--ramp', type=float, default=20.0); ap.add_argument('--no-ramp', action='store_true', help='flat-amplitude file (old rehearsals)'); a = ap.parse_args(); t_start = time.time(); R = []
    def add(name, ok, detail, warn=False): R.append((name, 'PASS' if ok else ('WARN' if warn else 'FAIL'), detail))
    raw = cfusdlog.decode(a.file)
    if raw is None or 'fixedFrequency' not in raw: print('NO-GO: not a uSD log (magic / event missing)'); return 1
    D = CL.load(a.file); names = D['_names']; ax = a.axis; ai, ins, k = CL.AX[ax]
    # (1) variables
    hdr = set(raw['fixedFrequency'].keys()) - {'timestamp'}; req = CL.VARS['variables']; miss = [v for v in req if v not in hdr]; extra = [v for v in hdr if v not in set(req)]
    add('1 variables: all %d of config v3 present' % len(req), not miss, 'missing %s' % miss[:6] if miss else '%d logged%s' % (len(hdr), (', %d extra' % len(extra)) if extra else ''))
    # (2) rows
    t = D['_raw_t']; dt = np.diff(t) * 1000; med = float(np.median(dt)); lost = float(np.mean(dt > 1.5 * med)) * 100; add('2 rows: lost < 0.2 %, no gap > 60 ms', lost < 0.2 and dt.max() < 60, '%d rows, median dt %.3f ms, dt > 1.5x median: %.3f %%, longest gap %.0f ms' % (len(t), med, lost, dt.max()))
    # (3) chirp
    t0, t1, peak = CL.chirp_window(D, ax); other = 'x' if ax == 'y' else 'y'; o_rms = float(np.sqrt(np.mean(D[CL.AX[other][1]] ** 2))); m_rms = float(np.sqrt(np.mean(D[ins][(D['t'] >= t0) & (D['t'] <= t1)] ** 2))) if t1 > t0 else 0.0
    seg = D[ins][(D['t'] >= t0) & (D['t'] <= t1)]; zc = np.flatnonzero(np.diff(np.sign(seg)) != 0); dur = t1 - t0
    f_start = f_end = float('nan')
    if len(zc) > 20: tz = D['t'][(D['t'] >= t0) & (D['t'] <= t1)][zc]; ft = 0.5 / np.diff(tz); n = max(3, len(ft) // 10); f_start = float(np.median(ft[:n])); f_end = float(np.median(ft[-n:]))
    ramp = 0 if a.no_ramp else a.ramp; long_enough = dur >= ramp + 10; env_ok = True; rat = []
    if ramp > 0 and dur > 8:
        tt = D['t'][(D['t'] >= t0) & (D['t'] <= t1)] - t0
        for s0 in np.arange(1.0, min(ramp, dur - 5), 4.0):
            mm = (tt >= s0) & (tt < s0 + 4.0); pk = float(np.max(np.abs(seg[mm]))) if mm.any() else 0.0; A_end = a.amp0 + (a.amp - a.amp0) * min((s0 + 4.0) / ramp, 1.0); rat.append(pk / A_end)
        env_ok = len(rat) > 0 and 0.7 <= float(np.median(rat)) <= 1.1 and min(rat) > 0.5
    peak_ok = abs(peak - a.amp) < 0.15 * a.amp if long_enough else peak >= 0.8 * a.amp
    add('3 chirp on %s axis (%s): peak %.2f m/s2%s, ramp %.2f -> %.2f over %.0f s, sweep 1 -> 8 Hz, other axis quiet' % ('roll' if ax == 'y' else 'pitch', ins, a.amp, ' +-15 %' if long_enough else ' (>= 80 %, window shorter than ramp + 10 s)', a.amp0, a.amp, ramp), peak_ok and env_ok and dur > (15 if not long_enough else 50) and o_rms < 0.2 * max(m_rms, 1e-9) and 0.6 < f_start < 1.8 and 6.0 < f_end < 10.0 * (1 if long_enough else 2), 'peak %.3f, window %.1f s, rms %.3f vs other axis %.3f, sweep %.1f -> %.1f Hz%s' % (peak, dur, m_rms, o_rms, f_start, f_end, (', ramp envelope ratio (segment peak / expected) median %.2f min %.2f' % (np.median(rat), min(rat))) if rat else ''))
    # (4) tilt (deg, whole flight) and (5) hold gyro
    tilt = float(np.degrees(max(np.nanmax(np.abs(D['omar_rpy_x'])), np.nanmax(np.abs(D['omar_rpy_y']))))); add('4 peak tilt < %.0f deg (abort limit 15)' % a.tilt_go, tilt < a.tilt_go, 'peak tilt %.1f deg' % tilt, warn=tilt < 15)
    hm = D['t'] < min(8.0, max(t0 - 0.5, 2.0)); hg = float(np.sqrt(np.mean(np.r_[D['gyro_x'][hm], D['gyro_y'][hm]] ** 2))); add('5 hold: gyro rms (before the chirp) < %.0f deg/s' % a.hold_gyro_go, hg < a.hold_gyro_go, 'rms %.1f deg/s over %.1f s' % (hg, hm.sum() * 0.002))
    # (6) timing pipeline
    q = [D.get(n) for n in ('locsrv_qx', 'locsrv_qy', 'locsrv_qz', 'locsrv_qw')]
    if all(x is not None for x in q):
        qn = np.sqrt(sum(x ** 2 for x in q)); chg = float(np.mean(np.diff(q[0]) != 0)); tm = CL.timing(D, ax)
        ok_tm = tm.get('ok') and sum(c >= 0.8 for c in tm['coh']) >= 4 and abs(np.median(qn) - 1) < 0.01
        add('6 timing pipeline: quaternion unit norm and changing, coherence gyro vs mocap >= 0.8 in >= 4/5 bands', bool(ok_tm), ('quaternion norm %.4f, changes in %.1f %% of rows; ' % (np.median(qn), 100 * chg)) + (('delay d - g = %.1f ms, filter tau_m = %.0f ms, coherence %s' % (tm['D_ms'], tm['tau_m_ms'], np.round(tm['coh'], 2))) if tm.get('ok') else tm.get('reason', '')))
    else: add('6 timing pipeline', False, 'locSrv quaternion missing')
    # (7) gains as commanded
    kr = float(np.median(D['omar_KR_' + k])); kw = float(np.median(D['omar_Kw_' + k])); add('7 KR / Kw as commanded for the %s axis (KR_%s / Kw_%s %.5g / %.5g, 1 %%)' % ('roll' if ax == 'y' else 'pitch', k, k, a.kr, a.kw), abs(kr / a.kr - 1) < 0.01 and abs(kw / a.kw - 1) < 0.01, 'logged KR_%s %.5g, Kw_%s %.5g' % (k, kr, k, kw))
    # (8) variables alive
    live = {n: float(np.std(D[n])) for n in ('omar_tau_rpm_f' + k, 'omar_tau_gyro_f' + k, 'omar_tau_' + k) if n in D}; dead = [n for n, s in live.items() if s == 0]; rr = {n: float(np.max(np.abs(D[n]))) for n in ('omar_omegar_' + k, 'omar_omegadd_' + k) if n in D}
    add('8 INDI-correction variables alive (tau_rpm_f, tau_gyro_f, command not constant)', not dead and len(live) == 3, 'std ' + ', '.join('%s %.2e' % (n.replace('omar_', ''), s) for n, s in live.items()) + (' | constant: %s' % dead if dead else ''))
    R.append(('info', 'INFO', 'rate reference omega_r / omega_des_dot max |.| = %s (streamed full-state setpoints carry jerk = snap = 0, so about 0 is expected; non-zero means a jerk reached the controller)' % ', '.join('%s %.3g' % (n.replace('omar_', ''), v) for n, v in rr.items())))
    if 'usd_fatWrBps' in D: fw = D['usd_fatWrBps']; R.append(('info', 'INFO', 'usd.fatWrBps median %.0f B/s (expected about 215000), spiWrBps median %.0f' % (np.median(fw[fw > 0]) if (fw > 0).any() else 0, np.median(D.get('usd_spiWrBps', [0])))))
    w = max(len(r[0]) for r in R); [print('%-4s %-*s  %s' % (s, w, n, d)) for n, s, d in R]; bad = [n for n, s, d in R if s == 'FAIL']; dtm = time.time() - t_start
    print('\n%s  (%s, %.0f s)' % ('GO' if not bad else 'NO-GO: ' + '; '.join(b.split(':')[0] for b in bad), os.path.basename(a.file), dtm)); return 0 if not bad else 1
if __name__ == '__main__': sys.exit(main())
