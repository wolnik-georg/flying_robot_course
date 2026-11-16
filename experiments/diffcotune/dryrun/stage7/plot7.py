import json, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
R = 'results/'; S3 = json.load(open(R + 'diag7_S3.json')); S1 = json.load(open(R + 'diag7_S1.json')); fx = json.load(open(R + 'diag7_fix_S3.json')); fx1 = json.load(open(R + 'diag7_fix_S1.json'))
def rough(J):
    d = np.diff(J, axis=0); d2 = np.diff(J, 2, axis=0); return [int((np.diff(np.sign(d[:, i])) != 0).sum()) for i in range(J.shape[1])], [float(np.std(d2[:, i]) / (np.ptp(J[:, i]) + 1e-12)) for i in range(J.shape[1])]
fig, ax = plt.subplots(2, 3, figsize=(13, 6.5)); c = {'A1': '#D55E00', 'A8': '#0072B2'}
names = ['Pxy', 'Dz', 'KRxy']
for k, nm in enumerate(names):
    for r, (D, lab) in enumerate(((S3, 'S3'), (S1, 'S1'))):
        b = D['b'][nm]; J = np.array(b['J_scen']); u = np.array(b['u']); a = ax[r, k]
        a.plot(u, J[:, 0] / J[10, 0], '-o', ms=3, color=c['A1'], label='A1'); a.plot(u, J[:, 1] / J[10, 1], '-o', ms=3, color=c['A8'], label='A8')
        sc, rg = rough(J); a.set_title('%s, gain %s: direction changes A1 %d, A8 %d' % (lab, nm, sc[0], sc[1]), fontsize=9); a.set_xlabel('log change of the gain'); a.set_ylabel('J noisy / J at the start'); a.grid(alpha=.3)
ax[0, 0].legend(fontsize=8); plt.tight_layout(); plt.savefig('../stage7/figs/landscape_S3_S1.png', dpi=130)
# table of roughness
rows = []
for lab, D in (('S3', S3), ('S1', S1)):
    for nm in [k for k in D['b'] if k != 'neg_g']:
        J = np.array(D['b'][nm]['J_scen']); sc, rg = rough(J); rows.append((lab, nm, sc[0], sc[1], sc[2], sc[3], rg[0], rg[1]))
md = ['| controller | gain | direction changes in 20 steps A1 / A8 / CIRCLE / FIG8 | roughness A1 / A8 |', '|---|---|---|---|'] + ['| %s | %s | %d / %d / %d / %d | %.2f / %.2f |' % r for r in rows]
open(R + 'landscape_table.md', 'w').write('\n'.join(md)); print('\n'.join(md))
fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
for a, (F, lab) in zip(ax, ((fx, 'S3'), (fx1, 'S1'))):
    f = F['rounds'][0]['fd_dir']; hs = sorted(f, key=float, reverse=True); a.plot([float(h) for h in hs], [f[h]['fd_dir_deriv_per_scen'][0] for h in hs], 'o-', color='#D55E00', label='finite difference, A1 noisy'); a.axhline(f[hs[0]]['fwd_pred'][0], color='k', ls='--', label='forward = reverse gradient')
    a.set_xscale('log'); a.set_yscale('symlog', linthresh=1e-2); a.set_xlabel('step h'); a.set_ylabel('directional derivative'); a.set_title('%s, round-0 point, A1' % lab, fontsize=9); a.legend(fontsize=7); a.grid(alpha=.3)
plt.tight_layout(); plt.savefig('../stage7/figs/fd_A1_S3_S1.png', dpi=130)
