import numpy as np
import matplotlib.pyplot as plt

# Standard drone
std_geo = [5.4, 5.9, 6.1, 6.8, 6.3, 6.5, 6.6, 6.6, 6.1, 5.1]
std_indi = [3.6, 3.7, 3.7, 4.2, 3.6, 3.7, 4.0, 4.0, 4.3, 3.9]

# Upgraded drone (thrust upgrade kit), n=11, INDI only
upg_indi = [3.9, 3.8, 3.9, 3.9, 3.2, 3.6, 4.0, 3.7, 3.7, 3.6, 3.6]

# Brushless (CF21BL), n=12, INDI only, locked config kr=2400/kw=170/kp=64/kv=5
bl_indi = [2.6, 2.7, 2.3, 2.6, 2.4, 2.3, 2.5, 2.5, 2.4, 2.3, 2.6, 2.5]

def stats(x):
    return np.mean(x), np.std(x)

groups = [
    ("Standard\nGeometric", std_geo, "#9fb2c2"),
    ("Standard\nINDI", std_indi, "#1f6f8b"),
    ("Upgraded\nINDI", upg_indi, "#c96f2c"),
    ("Brushless\nINDI", bl_indi, "#3f8f5f"),
]

means = [stats(g[1])[0] for g in groups]
stds  = [stats(g[1])[1] for g in groups]
labels = [g[0] for g in groups]
colors = [g[2] for g in groups]
ns = [len(g[1]) for g in groups]

# --- Plot 1: per-flight bars for all 4 groups (like the geo_vs_indi flight-by-flight panel) ---
fig, axes = plt.subplots(1, 4, figsize=(18, 4.5), sharey=True)
for ax, (label, vals, color), n in zip(axes, groups, ns):
    x = np.arange(1, len(vals) + 1)
    ax.bar(x, vals, color=color, width=0.6)
    ax.axhline(np.mean(vals), color='black', linestyle='--', linewidth=1)
    ax.set_title(f"{label.replace(chr(10),' ')} (n={n})\navg {np.mean(vals):.2f} cm", fontsize=11)
    ax.set_xlabel("flight #")
    ax.set_xticks(x)
    ax.grid(axis='y', alpha=0.3)
axes[0].set_ylabel("XY RMSE [cm]")
fig.suptitle("Figure-8 tracking — per-flight XY RMSE, all four configurations", fontsize=13)
fig.tight_layout(rect=[0, 0, 1, 0.94])
fig.savefig("/tmp/claude-1000/-home-georg-Desktop-flying-robot-course/776e626c-a26d-45e0-9fad-41689550ff42/scratchpad/4way_per_flight.png", dpi=150, bbox_inches="tight")
plt.close(fig)

# --- Plot 2: mean +/- std comparison bar ---
fig, ax = plt.subplots(figsize=(8, 5.5))
bars = ax.bar(labels, means, yerr=stds, capsize=6, color=colors,
               error_kw=dict(elinewidth=1.5, ecolor='black'))
for i, (m, s, n) in enumerate(zip(means, stds, ns)):
    ax.text(i, m + s + 0.15, f"{m:.2f} cm\n(n={n})", ha='center', fontsize=10, fontweight='bold')

# annotate % improvement vs standard geometric baseline
base = means[0]
for i in range(1, len(means)):
    pct = (means[i] - base) / base * 100
    ax.annotate(f"{pct:.0f}%", xy=(i, means[i]/2), ha='center', color='darkred', fontsize=10, fontweight='bold')

ax.set_ylabel("mean XY RMSE [cm]")
ax.set_title("Figure-8 tracking error — mean ± std, all platforms/controllers")
ax.set_ylim(0, max(means) + max(stds) + 1.2)
ax.grid(axis='y', alpha=0.3)
fig.tight_layout()
fig.savefig("/tmp/claude-1000/-home-georg-Desktop-flying-robot-course/776e626c-a26d-45e0-9fad-41689550ff42/scratchpad/4way_mean_comparison.png", dpi=150, bbox_inches="tight")
plt.close(fig)

print("done")
for l, v in zip(labels, groups):
    print(l.replace(chr(10),' '), "mean=%.2f"%np.mean(v[1]), "std=%.2f"%np.std(v[1]), "n=%d"%len(v[1]))
