import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT = "out/indi_comparison"

# ── Plot 1: A1 dz=0.30 same-day 3-way comparison, real hardware result ──
labels = ["Our geometric\n(controller=6)", "Omar geometric\n(indi=0)", "Omar INDI\n(indi=3)"]
errors_m = [0.17, 0.59, 0.04]
colors = ["#4FB39A", "#9A6B12", "#1F5C4D"]
fig, ax = plt.subplots(figsize=(7, 5))
bars = ax.bar(labels, errors_m, color=colors)
ax.axhline(0, color="black", lw=0.8)
ax.set_ylabel("separation error vs 0.30m commanded (m)")
ax.set_title("A1 dz=0.30 — same-day, same-conditions comparison (2026-09-29)")
for b, v in zip(bars, errors_m):
    ax.text(b.get_x() + b.get_width()/2, v + 0.01, f"{v:.2f}m", ha="center", fontsize=10)
fig.tight_layout()
fig.savefig(f"{OUT}/docs41_a1_three_way_comparison.png", dpi=130)
print("wrote a1 comparison plot")

# ── Plot 2: controller=10 failed flight — single-motor-dominant PWM pattern (real logged data) ──
import csv
rows = []
with open("/home/georg/Desktop/flying_robot_course/experiments/logs/usd_raw/A1_controller10_2026-09-29_19-02-37_merged.csv") as f:
    hdr = None
    for line in f:
        if line.startswith("t,"):
            hdr = line.strip().split(",")
        elif hdr and line.strip() and not line.startswith("#"):
            rows.append(dict(zip(hdr, line.strip().split(","))))

t = np.array([float(r["t"]) for r in rows])
m1 = np.array([float(r["cf5.motor_m1"]) for r in rows])
m2 = np.array([float(r["cf5.motor_m2"]) for r in rows])
m3 = np.array([float(r["cf5.motor_m3"]) for r in rows])
m4 = np.array([float(r["cf5.motor_m4"]) for r in rows])
z = np.array([float(r["cf5.z"]) for r in rows])
target_z = np.array([float(r["cf5.ctrltarget_z"]) for r in rows])

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True, height_ratios=[2, 1])
ax1.plot(t, m1, label="motor 1", color="#B4341C", lw=1.4)
ax1.plot(t, m2, label="motor 2", color="#4FB39A", lw=1.0, alpha=0.7)
ax1.plot(t, m3, label="motor 3", color="#1F5C4D", lw=1.0, alpha=0.7)
ax1.plot(t, m4, label="motor 4", color="#9A6B12", lw=1.0, alpha=0.7)
ax1.set_ylabel("motor PWM command")
ax1.set_title("controller=10 first hardware attempt — single-motor-dominant pattern (2026-09-29, A1 19:02:37)")
ax1.legend(loc="upper right")
ax1.grid(alpha=0.3)

ax2.plot(t, z, label="actual z (cf5)", color="#1F5C4D", lw=1.6)
ax2.plot(t, target_z, label="commanded ctrltarget_z", color="gray", ls="--", lw=1.2)
ax2.set_xlabel("time (s)")
ax2.set_ylabel("z (m)")
ax2.legend(loc="right")
ax2.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(f"{OUT}/docs41_controller10_single_motor_pattern.png", dpi=130)
print("wrote controller10 motor pattern plot")
