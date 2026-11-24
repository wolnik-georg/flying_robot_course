#!/usr/bin/env python3
"""Protocol v2.1 short summary for the planning agent: per controller round costs (round.py values), keep-best, next round gain set (steps vs the last flown round, KR/Kw, PREVIEW, fit J_sysId before/after).
Usage: v21_summary.py S3 S0 S2 (each controller's last desk-stepped round is detected from round_<k>/report.json)."""
import glob, json, sys, yaml
from pathlib import Path
H = Path(__file__).resolve().parent
g = lambda p: list(yaml.safe_load(open(p))['robots']['cf5']['firmware_params'].values())[0]
out = ['# v2.1 summary (round.py values: real cost = pos + att, 3-D position RMS)\n']
for S in sys.argv[1:]:
    reps = sorted(glob.glob(str(H / 'v21' / S / 'round_*' / 'report.json')), key=lambda p: int(Path(p).parent.name.split('_')[1]))
    rows = [(int(Path(p).parent.name.split('_')[1]), json.load(open(p))) for p in reps]
    k, r = rows[-1]
    out.append('## %s' % S)
    out.append('| round | real cost | pos RMS cm | crashed/floor |\n|---|---|---|---|')
    for l, j in rows: out.append('| %d | %.3g | %.2f | %s/%s |' % (l, j['real']['cost'], j['real']['rms3d_cm'], j['real']['crashed'], j['real']['floor']))
    kb = r['keep_best']; out.append('\nKeep-best: round %d (cost %.3g, %.2f cm).' % (kb['round'], kb['cost'], kb['rms3d_cm']))
    a = g(H / 'v21' / S / ('round_%d' % k) / 'gains.yaml'); b = g(H / 'v21' / S / ('round_%d' % (k + 1)) / 'gains.yaml')
    st = {n: b[n] / a[n] for n in a}
    out.append('Round %d gain set (steps vs round %d): ' % (k + 1, k) + ', '.join('%s x%.3f' % (n, v) for n, v in st.items()))
    out.append('At the +-10 %% limit: %s' % (', '.join(n for n, v in st.items() if abs(v - 1) > 0.0995) or 'none'))
    out.append('KR_x/Kw_x %.2f -> %.2f; KR_z/Kw_z %.2f -> %.2f' % (a['KR_x'] / a['Kw_x'], b['KR_x'] / b['Kw_x'], a['KR_z'] / a['Kw_z'], b['KR_z'] / b['Kw_z']))
    out.append('Fit J_sysId %.3g -> %.3g; PREVIEW %s (new %.2f cm vs best flown %.2f cm)\n' % (r['fit']['J0'], r['fit']['J_best'], r['preview']['verdict'], r['preview']['mean_rms_cm']['new'], r['preview']['mean_rms_cm']['best']))
(H / 'v21' / 'SUMMARY_round4.md').write_text('\n'.join(out)); print('\n'.join(out))
