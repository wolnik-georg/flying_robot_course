#!/usr/bin/env python3
"""usd_analyse (REPORT_59 L4a): post-lab analysis of the chirp flights, one command.
  usage: python3 usd_analyse.py --out <dir> F1=<uSD file>:y F2=<file>:y F3=<file>:y F5=<file>:y F6=<S3 file>:y [x-axis flights as F2x=<file>:x]  (F4 dropped; F6 = S3 v21 r1 as flown, reported separately)
For every flight: E_full loop estimate (Ms, PM, GM, gain / phase per band 1.5-8 Hz, bootstrap 5-95 % over windows), timing (gyro delay minus mocap latency, mocap filter time constant), INDI split A / B / R per band 1-5 Hz, the 25-32 Hz line (f0, strength, Q). Then the KR trend between the flights (KR taken from the log).
Writes <dir>/analysis.json (read by sim_block.py) and prints a summary. Reference values (frsim, S1 v21 r1 flown gains, REPORT_50): Ms 10.9 dB, PM 18 deg, GM 4.8 dB; known resolution at 0.3 m/s2: Ms +-0.7 dB, PM +-1.9 deg, GM +-0.1 dB, gain(3.5 Hz) +-1.3 dB, phase +-5 deg, timing +-2 ms."""
import os, sys, json, argparse, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import chirp_lib as CL
def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('flights', nargs='+'); a = ap.parse_args(); os.makedirs(a.out, exist_ok=True); R = {}
    for spec in a.flights:
        lab, rest = spec.split('=', 1); path, ax = rest.rsplit(':', 1); D = CL.load(path); k = CL.AX[ax][2]; ef = CL.efull(D, ax); tm = CL.timing(D, ax); sp = CL.split(D, ax); ln = CL.line(D, 'y' if ax == 'y' else 'x')
        R[lab] = dict(file=path, axis=ax, KR=float(np.median(D['omar_KR_' + k])), Kw=float(np.median(D['omar_Kw_' + k])), efull=ef, timing=tm, split=sp, line=ln)
        print('== %s%s (%s, axis %s, KR %.5g, Kw %.5g)' % (lab, '  [S3 CHECK, reported separately, same analysis as F2]' if lab.startswith('F6') else '', os.path.basename(path), ax, R[lab]['KR'], R[lab]['Kw']))
        if ef.get('ok'):
            c = ef['ci']; fm = lambda x, d=1: 'None' if x is None else ('%.*f' % (d, x)); print('   loop: Ms %.1f dB [%.1f, %.1f] at %.1f Hz | PM %s [%s, %s] | GM %s [%s, %s] | gain(3.5 Hz) %.1f dB, phase %.0f deg | windows %d, peak acc %.2f' % (ef['Ms_db'], *c['Ms_db'], ef['Ms_f'], fm(ef['pm'], 0), fm(c['pm'][0], 0), fm(c['pm'][1], 0), fm(ef['gm_db']), fm(c['gm_db'][0]), fm(c['gm_db'][1]), ef['gain35_db'], ef['phase35'], ef['n_windows'], ef['peak_acc']))
            print('   L per band %s: %s' % (['%.1f' % f for f in ef['f']], ' '.join('%.1f/%.0f' % (g, p) for g, p in zip(ef['gain_db'], ef['phase_deg']))))
        else: print('   loop: not available (%s)' % ef.get('reason'))
        print('   timing: ' + (('gyro delay minus mocap latency %.1f ms, mocap filter %.0f ms, coherence %s' % (tm['D_ms'], tm['tau_m_ms'], np.round(tm['coh'], 2))) if tm.get('ok') else tm.get('reason', '')))
        print('   INDI split (bands 1-2 / 2-3 / 3-4 / 4-5 Hz): A ' + ' '.join('%.2f/%.0f' % (s['A_mag'], s['A_ph']) for s in sp) + ' | B ' + ' '.join('%.2f/%.0f' % (s['B_mag'], s['B_ph']) for s in sp) + ' | R ' + ' '.join('%.2f' % s['R_mag'] for s in sp) + ' | |C| ' + ' '.join('%.2f' % s['C_mag'] for s in sp))
        print('   line: f0 %.1f Hz, strength %.0f x floor, Q %.0f' % (ln['f0'], ln['strength'], ln['Q']))
    ok = [l for l in R if R[l]['efull'].get('ok') and R[l]['axis'] == 'y' and not l.startswith('F6')]   # F6 = the S3 check, reported separately, never mixed into the S1 KR trend
    if len(ok) >= 2:
        kr = np.array([R[l]['KR'] for l in ok]); ms = np.array([R[l]['efull']['Ms_db'] for l in ok]); pm = np.array([R[l]['efull']['pm'] if R[l]['efull']['pm'] is not None else np.nan for l in ok])
        o = np.argsort(kr); print('\nKR trend (y-chirp flights): ' + ' | '.join('%s KR %.5g: Ms %.1f dB, PM %s' % (ok[i], kr[i], ms[i], '%.0f' % pm[i] if np.isfinite(pm[i]) else 'None') for i in o))
        if kr.max() > kr.min() * 1.02:
            x = 20 * np.log10(kr); g = np.isfinite(pm); print('   dMs per +1 dB of KR: %.2f dB' % np.polyfit(x, ms, 1)[0] + ('; dPM per +1 dB of KR: %.1f deg' % np.polyfit(x[g], pm[g], 1)[0] if g.sum() > 1 else ''))
    json.dump(R, open(os.path.join(a.out, 'analysis.json'), 'w'), indent=1); print('\nwrote', os.path.join(a.out, 'analysis.json'))
if __name__ == '__main__': main()
