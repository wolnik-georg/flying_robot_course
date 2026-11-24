#!/usr/bin/env python3
"""Protocol v2.1: round l+1 gain report (usage: v21_round1_report.py <round l> <S..>; default round 0, all four) for the planning agent, from lab/v21/<S>/round_0/report.json, round_0|1/gains.yaml, results/v21_fit_parts_<S>.json. Writes v21/REPORT_round1.md."""
import json, math, sys, yaml
from pathlib import Path
H = Path(__file__).resolve().parent; RND = int(sys.argv[1]) if len(sys.argv) > 1 else 0; CTRLS = sys.argv[2:] or ['S0', 'S1', 'S2', 'S3']; SUF = '' if RND == 0 else '_r%d' % RND; out = ['# v2.1 (ours21, linear +-10 %%) round %d gain sets from round %d, for the planning agent check\n' % (RND + 1, RND),
 'Cost = position + attitude (att = 2(1-<q,q*>^2) ~ angle^2/2); attitude RMS deg = sqrt(2 x att). Nothing is flown before your check.\n']
def gains(p):
    d = yaml.safe_load(open(p))['robots']['cf5']['firmware_params']; return d[next(iter(d))]
for S in CTRLS:
    r = json.load(open(H / 'v21' / S / ('round_%d' % RND) / 'report.json')); fp = json.load(open(H / 'results' / ('v21_fit_parts_%s%s.json' % (S, SUF))))
    g0 = gains(H / 'v21' / S / ('round_%d' % RND) / 'gains.yaml'); g1 = gains(H / 'v21' / S / ('round_%d' % (RND + 1)) / 'gains.yaml')
    out.append('## %s\n' % S); rl = r['real']
    out.append('Round cost (mean of 5): total %.4g (pos %.4g + att %.4g); 3-D position RMS %.2f cm; attitude RMS %.2f deg; crashed %s floor %s\n' % (rl['cost'], rl['pos_cost'], rl['att_cost'], rl['rms3d_cm'], math.degrees(math.sqrt(2 * rl['att_cost'])), rl['crashed'], rl['floor']))
    out.append('| flight | pos RMS cm | pos cost | att cost | att RMS deg | total cost | max tilt deg | gate |\n|---|---|---|---|---|---|---|---|')
    for f in r['flights']:
        if not f.get('used'): continue
        out.append('| %s | %.2f | %.4g | %.4g | %.2f | %.4g | %.1f | %s |' % (f['scenario'], f['rms3d_cm'], f['pos_cost_m2'], f['att_cost'], math.degrees(math.sqrt(2 * f['att_cost'])), f['cost_m2'], f['tilt_max_deg'], 'ok' if f['gate_ok'] else 'FAIL'))
    b, a = fp['before'], fp['after']
    out.append('\nJ_sysId per part (mean over 5 flights, unweighted), before -> after the beta fit: ' + ', '.join('%s %.4g -> %.4g' % (k, b['mean'][k], a['mean'][k]) for k in 'pvqw') + '; total %.4g -> %.4g (fit epochs %s, best epoch %s)\n' % (b['total'], a['total'], r['fit']['epochs'], r['fit']['best_epoch']))
    out.append('Fitted beta factors (x default): ' + ', '.join('%s %.3f' % kv for kv in fp['beta_after'].items()) + '\n')
    out.append('| gain | round l | round l+1 | step x | at limit |\n|---|---|---|---|---|')
    for k in g0:
        s = g1[k] / g0[k]; out.append('| %s | %.5g | %.5g | %.4f | %s |' % (k, g0[k], g1[k], s, 'yes' if abs(s - 1) > 0.0999 else ''))
    out.append('\nKR_x/Kw_x %.2f -> %.2f; KR_z/Kw_z %.2f -> %.2f' % (g0['KR_x'] / g0['Kw_x'], g1['KR_x'] / g1['Kw_x'], g0['KR_z'] / g0['Kw_z'], g1['KR_z'] / g1['Kw_z']))
    out.append('Tuning step: J %.4g -> %.4g (best epoch %s); gain-limit hits: %s\n' % (r['tune']['J0'], r['tune']['J_best'], r['tune']['best_epoch'], r['tune'].get('hits')))
    out.append('PREVIEW:\n```\n%s\n```\n' % r['preview']['text'])
(H / 'v21' / ('REPORT_round%d.md' % (RND + 1))).write_text('\n'.join(out)); print('\n'.join(out))
