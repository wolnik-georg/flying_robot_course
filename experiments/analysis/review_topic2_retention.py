#!/usr/bin/env python3
"""Independent review (Topic 2): robustness of the pre-registered retention R = RI_unseen / RI_seen (geometric pair S0 -> S2).
Inputs = per-flight metrics from unseen_block_analysis.py run on the unseen banks and (dry run) on the seen pilot banks; the seen side
reproduces experiments/results/unified_overview_2026-10-20/summary.json exactly (same metric code; the 2 drained S3 A8 flights excluded as there).
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/review_topic2_retention.py <unseen per_flight.csv> <seen per_flight.csv>"""
import sys
import numpy as np, pandas as pd
U = pd.read_csv(sys.argv[1]); S = pd.read_csv(sys.argv[2])
S = S[~S.folder.str.endswith(("17-55-03", "17-56-35"))]                   # DRAIN rule of unified_overview.py
UC = ["A1 dz0.45", "A4 dz0.4 off0.075 v0.35", "A4 dz0.4 off0.125 v0.35", "A8 dz0.4 dur6.25"]
SEEN = {"A1": "A1 dz0.5", "A8": "A8 dz0.5 dur4.375", 0.05: "A4 dz0.5 off0.05 v0.5", 0.10: "A4 dz0.5 off0.1 v0.5", 0.15: "A4 dz0.5 off0.15 v0.5"}
class _V(np.ndarray):
    def mean(self, *a, **k): return abs(float(np.asarray(self).mean()))      # |mean of the signed metric|, as unseen_block_analysis.py
def vals(D, cell, s): return D[(D.cell == cell) & (D.strategy == s)].metric.values.view(_V)
def seen_mean(Sd, cell, s, ref):
    if cell.startswith("A1"): return vals(Sd, SEEN["A1"], s).mean()
    if cell.startswith("A8"): return vals(Sd, SEEN["A8"], s).mean()
    off = 0.075 if "0.075" in cell else 0.125
    if ref == "interp":
        o = [0.05, 0.10, 0.15]; v = [vals(Sd, SEEN[k], s).mean() for k in o]; return float(np.interp(off, o, v))
    if ref == "trained_only":                                             # nearest TRAINED pilot offset(s): 0.10 for 0.075; 0.10/0.15 interp for 0.125
        return vals(Sd, SEEN[0.10], s).mean() if off == 0.075 else float(np.interp(off, [0.10, 0.15], [vals(Sd, SEEN[k], s).mean() for k in (0.10, 0.15)]))
def RI(a, b): return (a - b) / a
def R_cells(Ud, Sd, ref="interp", base="S0", ns="S2"):
    out = {}
    for c in UC:
        ru = RI(vals(Ud, c, base).mean(), vals(Ud, c, ns).mean()); rs = RI(seen_mean(Sd, c, base, ref), seen_mean(Sd, c, ns, ref)); out[c] = (ru, rs, ru / rs if rs > 0 else np.nan)
    return out
def pooled(d, cells=UC): return float(np.nanmean([d[c][2] for c in cells]))
base = R_cells(U, S); print("pre-registered:", {c[:24]: tuple(round(x, 3) for x in v) for c, v in base.items()}, "pooled %.3f" % pooled(base))
# 1) leave one flight out (unseen and seen flights, each removed in turn)
lo = []
for D, tag in ((U, "unseen"), (S, "seen")):
    for i in D.index:
        if D.loc[i, "strategy"] not in ("S0", "S2"): continue
        d = R_cells(D.drop(i), S) if tag == "unseen" else R_cells(U, D.drop(i)); lo.append((tag, D.loc[i, "folder"], D.loc[i, "strategy"], pooled(d), min(d[c][2] for c in UC)))
L = pd.DataFrame(lo, columns=["side", "flight", "strat", "pooled", "min_cell"]); print("LOFO pooled R: min %.3f max %.3f (n=%d); worst single cell min %.3f" % (L.pooled.min(), L.pooled.max(), len(L), L.min_cell.min()))
print(L.sort_values("pooled").head(4).to_string(index=False))
# 2) bootstrap: resample flights with replacement within every (cell, strategy), both sides
rng = np.random.default_rng(0); B = []; Bc = []
def rs(D):
    return pd.concat([g.sample(len(g), replace=True, random_state=int(rng.integers(1 << 31))) for _, g in D.groupby(["cell", "strategy"])])
for _ in range(4000):
    d = R_cells(rs(U), rs(S)); B.append(pooled(d)); Bc.append([d[c][2] for c in UC])
B = np.array(B); Bc = np.array(Bc)
print("bootstrap pooled R: median %.2f, 95%% CI [%.2f, %.2f], P(R<0.7) = %.3f" % (np.median(B), *np.percentile(B, [2.5, 97.5]), (B < 0.7).mean()))
for j, c in enumerate(UC): print("   cell %-26s 95%% CI [%.2f, %.2f]  P(R<0.7) %.3f" % (c, *np.nanpercentile(Bc[:, j], [2.5, 97.5]), np.nanmean(Bc[:, j] < 0.7)))
# 3) alternatives (reported separately, the rule itself is not changed)
alt = R_cells(U, S, ref="trained_only"); print("A4 reference = trained pilot offsets only:", {c[:24]: round(v[2], 3) for c, v in alt.items()}, "pooled %.3f" % pooled(alt))
print("pooled without A1 (soft cell): %.3f; without A1 and A4 0.075 (half-unseen reference): %.3f" % (pooled(base, UC[1:]), pooled(base, UC[2:])))
print("pooled as ratio of mean RI: %.3f; median of cells: %.3f" % (np.mean([base[c][0] for c in UC]) / np.mean([base[c][1] for c in UC]), np.median([base[c][2] for c in UC])))
gu = {c: vals(U, c, "S0").mean() - vals(U, c, "S2").mean() for c in UC}; print("absolute gain unseen S0-S2 [cm]:", {c[:24]: round(v, 2) for c, v in gu.items()})
ind = R_cells(U, S, base="S1", ns="S3"); print("INDI pair RI unseen/seen:", {c[:24]: (round(v[0], 2), round(v[1], 2)) for c, v in ind.items()})
S7 = pd.read_csv(sys.argv[2]); ind7 = R_cells(U, S7, base="S1", ns="S3"); print("INDI pair, seen A8 S3 incl. the 2 drained flights: RI seen %.2f" % ind7["A8 dz0.4 dur6.25"][1])
