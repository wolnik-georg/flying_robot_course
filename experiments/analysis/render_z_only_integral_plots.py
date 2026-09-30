import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

with open("out/z_only_integral/z_integral_plot_data.json") as f:
    data = json.load(f)

fig, ax = plt.subplots(figsize=(9, 5))
colors = {"8": "#1F5C4D", "16": "#4FB39A", "32": "#B4341C"}
for kz, color in colors.items():
    d = data["task4"][kz]
    ax.plot(d["t"], d["err_mm"], label=f"ki_z={kz}", color=color, lw=1.6)
ax.axvline(5.0, color="gray", ls="--", lw=1, label="disturbance onset (t=5s)")
ax.set_xlabel("time (s)")
ax.set_ylabel("z error (mm)")
ax.set_title("Task 4 — disturbance mid-flight: peak dip unchanged by gain, only recovery speed differs\n(-200mN step at t=5s)")
ax.legend()
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig("out/z_only_integral/z_integral_task4_disturbance_onset.png", dpi=130)
print("wrote task4 plot")

fig, ax = plt.subplots(figsize=(9, 5))
colors5 = {"0": "#6E7773", "8": "#1F5C4D", "16": "#4FB39A", "32": "#B4341C"}
for kz, color in colors5.items():
    d = data["task5"][kz]
    label = "OFF (ki_z=0)" if kz == "0" else f"ki_z={kz}"
    ax.plot(d["t"], d["err_mm"], label=label, color=color, lw=1.6)
ax.set_xlabel("time (s)")
ax.set_ylabel("z error (mm)")
ax.set_title("Task 5 — persistent bias present from t=0 (A1-scale ~17-19% sag), realistic 8s hold\nki_z=16 closes most of the gap within the hold")
ax.legend()
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig("out/z_only_integral/z_integral_task5_persistent_bias.png", dpi=130)
print("wrote task5 plot")
