#!/usr/bin/env python3
"""3D trajectory + orientation-triad plot for the best inverted-loop flight
(commit ba25316, v5): loop_mode1_kt0.15_2026-07-28_19-33-11.csv.

Matches the style of the project's other `_3d_orientation.png` plots
(analyze_flight.py: _draw_triad / _euler_deg_to_rot) -- body-axis triads
(x=red, y=green, z=blue) sampled along the flown path, flown-path color
"#1f77b4" matching the project convention. Event/marker colors are kept off
red/green/blue (those are reserved for the axis triad) -- black, varying
marker shape, so the color coding stays unambiguous.

Every callout marker ("pin") is numbered in time order (#1, #2, ...) both on
the plot and in the printed summary below, so individual pins can be removed
by number -- edit the `kind == "..."` entries in MARKER_SPECS.

Run: ~/.pyenv/versions/flying_robots/bin/python Controls/plot_loop_best_flight.py
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
from analyze_flight import _draw_triad, _euler_deg_to_rot  # noqa: E402

LOG_DIR = Path(__file__).parent / "logs"
FNAME = "loop_mode1_kt0.15_2026-07-28_19-33-11.csv"

df = pd.read_csv(LOG_DIR / FNAME, comment="#")
# time_s resets when the onboard trajectory itself arms -- keep only the
# post-reset segment (the actual loop maneuver, not the pre-trajectory climb).
t_raw = df["time_s"].values
reset_idx = np.where(np.diff(t_raw) < 0)[0]
if len(reset_idx):
    df = df.iloc[reset_idx[0] + 1 :].reset_index(drop=True)

t = df["time_s"].values
x, y, z = df["x"].values, df["y"].values, df["z"].values
roll, pitch, yaw = df["roll_deg"].values, df["pitch_deg"].values, df["yaw_deg"].values

# Crop to the actual maneuver window (apex through the exit-ramp tumble and
# initial re-settle) -- excludes the later commanded-landing sequence, which
# is a separate, unrelated event and not part of this flight's story.
window = t < 4.5
t, x, y, z = t[window], x[window], y[window], z[window]
roll, pitch, yaw = roll[window], pitch[window], yaw[window]

# Key events (from investigation_inverted_loop_2026-07-27.md section 9.3)
apex_search = t < 2.5
i_apex = int(np.where(apex_search)[0][np.argmax(np.abs(roll[apex_search]))])
i_graze = i_apex + int(np.argmin(z[i_apex : i_apex + 150]))
level_mask = np.abs(roll[i_graze : i_graze + 150]) < 30
i_recover = i_graze + int(np.where(level_mask)[0][np.argmax(z[i_graze : i_graze + 150][level_mask])])
tumble_mask = (t > 3.3) & (t < 3.5)
i_tumble = int(np.where(tumble_mask)[0][np.argmax(np.abs(roll[tumble_mask]))])

t_apex = t[i_apex]


def nearest(dt):
    return int(np.argmin(np.abs(t - (t_apex + dt))))


# All callout markers ("pins"), unnumbered here -- numbered by time below.
# Remove an entry from this list to drop that pin from the plot.
MARKER_SPECS = [
    (0, "Start", "start"),
    (nearest(-0.20), "Flip in (-0.20s)", "flip"),
    (nearest(-0.10), "Flip in (-0.10s)", "flip"),
    (nearest(-0.05), "Flip in (-0.05s)", "flip"),
    (i_apex, "Apex (near-full inversion)", "event"),
    (nearest(+0.05), "Flip out (+0.05s)", "flip"),
    (nearest(+0.10), "Flip out (+0.10s)", "flip"),
    (nearest(+0.20), "Flip out (+0.20s)", "flip"),
    (i_graze, "Floor graze (not a crash)", "event"),
    (i_recover, "Recovery peak", "event"),
    (i_tumble, "Exit-ramp tumble (unresolved)", "event"),
    (len(t) - 1, "End", "end"),
]

# Sort by time and number in order -- this is the numbering to reference
# when asking for a pin to be removed ("remove pin 4").
MARKER_SPECS.sort(key=lambda m: t[m[0]])
markers = [(n + 1, idx, label, kind) for n, (idx, label, kind) in enumerate(MARKER_SPECS)]

STYLE = {
    "start": dict(marker="^", s=90, alpha=0.9, triad_scale=1.0),
    "end": dict(marker="s", s=90, alpha=0.9, triad_scale=1.0),
    "event": dict(marker="o", s=70, alpha=1.0, triad_scale=1.4),
    "flip": dict(marker="D", s=40, alpha=0.9, triad_scale=1.1),
}

fig = plt.figure(figsize=(11, 9))
ax = fig.add_subplot(111, projection="3d")

ax.plot(x, y, z, color="#1f77b4", lw=1.9)

span_x = float(np.nanmax(x) - np.nanmin(x))
span_y = float(np.nanmax(y) - np.nanmin(y))
span_z = float(np.nanmax(z) - np.nanmin(z))
diag = max(np.sqrt(span_x**2 + span_y**2 + span_z**2), 0.3)
axis_len = 0.07 * diag
ax.set_box_aspect([max(span_x, 1e-3), max(span_y, 1e-3), max(span_z, 1e-3)])

# Sparse, evenly-spaced orientation triads along the whole path -- same
# marker-count convention as the project's other 3D orientation plots
# (np.linspace(t0, t1, 6-10)), not a dense trail.
sample_times = np.linspace(t[0], t[-1], 10)
for t_m in sample_times:
    i = int(np.argmin(np.abs(t - t_m)))
    rc = _euler_deg_to_rot(roll[i], pitch[i], yaw[i])
    _draw_triad(ax, x[i], y[i], z[i], rc, axis_len, alpha=0.5)
    ax.scatter([x[i]], [y[i]], [z[i]], c="#1f77b4", s=12, alpha=0.5)

# Numbered pins.
for n, idx, label, kind in markers:
    sty = STYLE[kind]
    rc = _euler_deg_to_rot(roll[idx], pitch[idx], yaw[idx])
    _draw_triad(ax, x[idx], y[idx], z[idx], rc, axis_len * sty["triad_scale"], alpha=sty["alpha"])
    face = "dimgray" if kind == "flip" else "black"
    ax.scatter([x[idx]], [y[idx]], [z[idx]], c=face, s=sty["s"], zorder=6,
               marker=sty["marker"], edgecolors="white")
    ax.text(x[idx], y[idx], z[idx], f"  #{n} {label}\n  t={t[idx]:.2f}s", fontsize=7.5, color="#222222")

ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_zlabel("z [m]")
ax.set_title(
    "Best inverted-loop flight --- 3D trajectory + orientation triads\n"
    f"{FNAME}",
    fontsize=11,
)

legend_handles = [
    Line2D([0], [0], color="#1f77b4", lw=1.9, label="Flown path"),
    Line2D([0], [0], color="#d62728", lw=2, label="Body x-axis"),
    Line2D([0], [0], color="#2ca02c", lw=2, label="Body y-axis"),
    Line2D([0], [0], color="#1f77b4", lw=2, label="Body z-axis"),
    Line2D([0], [0], marker="o", color="none", markerfacecolor="black",
           markeredgecolor="white", markersize=8, label="Key event (#n)"),
    Line2D([0], [0], marker="D", color="none", markerfacecolor="dimgray",
           markeredgecolor="white", markersize=7, label="Flip-bracket pin (#n)"),
    Line2D([0], [0], marker="^", color="none", markerfacecolor="black",
           markeredgecolor="white", markersize=8, label="Start (#n)"),
    Line2D([0], [0], marker="s", color="none", markerfacecolor="black",
           markeredgecolor="white", markersize=8, label="End (#n)"),
]
ax.legend(handles=legend_handles, loc="upper left", fontsize=8, framealpha=0.9)

out = LOG_DIR / "loop_best_flight_3d.png"
fig.tight_layout()
fig.savefig(out, dpi=140)
print(f"saved {out}")

print("\nPin summary (time order -- use these numbers to request removals):")
for n, idx, label, kind in markers:
    print(f"  #{n:2d} [{kind:5s}] {label:32s} t={t[idx]:.3f}s  z={z[idx]:.3f}m  roll={roll[idx]:.1f}deg")


# ── Interactive Plotly version, same data/pins, matching the project's
# existing interactive-3D-orientation convention (plot_3d_orientation_multi_interactive) ──
def build_interactive():
    import plotly.graph_objects as go

    fig_p = go.Figure()
    fig_p.add_trace(go.Scatter3d(
        x=x, y=y, z=z, mode="lines", line=dict(color="#1f77b4", width=4),
        name="Flown path", hoverinfo="skip",
    ))

    # Exact same hex values as _draw_triad (analyze_flight.py) / the static PNG --
    # NOT the plain "red"/"green"/"blue" the project's other interactive plots use,
    # since that made the z-axis color visibly not match the flown-path "#1f77b4".
    triad_colors = {"x": "#d62728", "y": "#2ca02c", "z": "#1f77b4"}
    marker_symbols = {"start": "diamond", "end": "square", "event": "circle", "flip": "diamond-open"}
    marker_sizes = {"start": 7, "end": 7, "event": 8, "flip": 5}

    def add_triad(ox, oy, oz, rc, length, group, opacity):
        r00, r01, r02, r10, r11, r12, r20, r21, r22 = rc
        axes = {"x": (r00, r10, r20), "y": (r01, r11, r21), "z": (r02, r12, r22)}
        for name, (ax_, ay_, az_) in axes.items():
            fig_p.add_trace(go.Scatter3d(
                x=[ox, ox + length * ax_], y=[oy, oy + length * ay_], z=[oz, oz + length * az_],
                mode="lines", line=dict(color=triad_colors[name], width=4 if group == "pin" else 2),
                opacity=opacity, legendgroup=group, showlegend=False, hoverinfo="skip",
            ))

    # Background sparse triads (same 10 samples as the static plot).
    for t_m in sample_times:
        i = int(np.argmin(np.abs(t - t_m)))
        rc = _euler_deg_to_rot(roll[i], pitch[i], yaw[i])
        add_triad(x[i], y[i], z[i], rc, axis_len, "background", 0.35)

    # Numbered pins: triad + labeled marker, hover shows full detail.
    by_kind = {}
    for n, idx, label, kind in markers:
        sty = STYLE[kind]
        rc = _euler_deg_to_rot(roll[idx], pitch[idx], yaw[idx])
        add_triad(x[idx], y[idx], z[idx], rc, axis_len * sty["triad_scale"], "pin", 0.9)
        by_kind.setdefault(kind, {"x": [], "y": [], "z": [], "text": [], "hover": []})
        by_kind[kind]["x"].append(x[idx])
        by_kind[kind]["y"].append(y[idx])
        by_kind[kind]["z"].append(z[idx])
        by_kind[kind]["text"].append(f"#{n}")
        by_kind[kind]["hover"].append(f"#{n} {label}<br>t={t[idx]:.2f}s  z={z[idx]:.2f}m  roll={roll[idx]:.1f}deg")

    kind_names = {"start": "Start", "end": "End", "event": "Key event", "flip": "Flip-bracket pin"}
    for kind, d in by_kind.items():
        fig_p.add_trace(go.Scatter3d(
            x=d["x"], y=d["y"], z=d["z"], mode="markers+text",
            marker=dict(size=marker_sizes[kind], color="black", symbol=marker_symbols[kind],
                        line=dict(color="white", width=1)),
            text=d["text"], textposition="top center", textfont=dict(size=10, color="#222222"),
            hovertext=d["hover"], hoverinfo="text", name=kind_names[kind],
        ))

    # Legend-only entries for the triad axis colors (the triad line traces
    # themselves are all showlegend=False to avoid one row per sample/pin).
    for name, color in [("Body x-axis", "#d62728"), ("Body y-axis", "#2ca02c"), ("Body z-axis", "#1f77b4")]:
        fig_p.add_trace(go.Scatter3d(
            x=[None], y=[None], z=[None], mode="lines",
            line=dict(color=color, width=4), name=name, hoverinfo="skip",
        ))

    fig_p.update_layout(
        title=(
            "Best inverted-loop flight — 3D trajectory + orientation triads<br>"
            f"<sub>{FNAME} — x=red, y=green, z=blue; numbered pins match the static plot — "
            "drag to orbit, scroll to zoom, click legend to toggle</sub>"
        ),
        scene=dict(xaxis_title="x [m]", yaxis_title="y [m]", zaxis_title="z [m]", aspectmode="data"),
        height=900, width=1200,
    )
    out_html = LOG_DIR / "loop_best_flight_3d_interactive.html"
    fig_p.write_html(out_html, include_plotlyjs="cdn")
    print(f"saved {out_html}")


build_interactive()
