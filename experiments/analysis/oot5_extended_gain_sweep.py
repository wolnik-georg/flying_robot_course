#!/usr/bin/env python3
"""Extended one-knob gain sweeps for controller=10 desk prep (docs/41 §20.5).

Widens §20.3 grids — still **one knob at a time**, not combinatorial. Height uses **indi=3**
+ **IMU-bias proxy** (§20.2). Figure8 uses **`figure8_mode1_kt0.05.csv`** (verified path).

Run: /usr/bin/python3.10 oot5_extended_gain_sweep.py
"""
from __future__ import annotations

import sys
from pathlib import Path

_ANALYSIS = Path(__file__).resolve().parent
sys.path.insert(0, str(_ANALYSIS))

import oot5_bounded_gain_sweep as s  # noqa: E402

assert s.TRAJ_CSV.endswith("figure8_mode1_kt0.05.csv"), s.TRAJ_CSV

KPOS_I_GRID = (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0)
KPOS_P_GRID = (7.0, 8.4, 9.1, 10.0, 11.0, 12.0, 14.0, 16.0)
ATT_SCALES = (0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75)  # ±75% around Omar


def _print_height_table(title, grid, runner):
    print(f"\n{title}")
    print(f"{'value':>8} {'mean err (mm)':>14} {'z std (mm)':>12}")
    for v in grid:
        m = runner(v)
        print(f"{v:8.2f} {m['err_mm']:14.2f} {m['std_mm']:12.2f}")


def main():
    print(f"TRAJ_CSV OK: {s.TRAJ_CSV}")
    acc = s.calibrate_acc_bias_g()
    print(f"IMU acc_z bias [g]: {acc:+.5f} (calibrated to +{s.HW_BIAS_MM:.0f} mm @ Omar P/D/I)")

    def hover_i(ki):
        return s.run_hover(indi=3, acc_z_bias_g=acc, kpos_i_z=ki)

    def hover_p(kp):
        return s.run_hover(indi=3, acc_z_bias_g=acc, kpos_p_z=kp)

    _print_height_table(
        "Kpos_I.z extended (indi=3, IMU-bias proxy; P/D at Omar defaults)",
        KPOS_I_GRID,
        hover_i,
    )
    _print_height_table(
        "Kpos_P.z extended (indi=3, IMU-bias proxy; I=0)",
        KPOS_P_GRID,
        hover_p,
    )

    base = s.run_figure8()
    print(
        f"\nFigure8 baseline Omar (kt=0.05): roll std {base['roll_std']:.2f}° "
        f"peak {base['roll_peak']:.1f}° (hardware {s.HW_ROLL_STD}° / {s.HW_ROLL_PEAK}°)"
    )

    for label, attr in (
        ("KR scale", "kr_scale"),
        ("KOMEGA scale", "kom_scale"),
        ("KI_ATT scale", "ki_scale"),
    ):
        print(f"\n{label} extended (×Omar) -> roll std [deg]")
        print(f"{'scale':>8} {'roll std':>10} {'roll peak':>10} {'Δstd vs 1.0':>12}")
        ref_std = s.run_figure8(**{attr: 1.0})["roll_std"]
        for sc in ATT_SCALES:
            r = s.run_figure8(**{attr: sc})
            delta = r["roll_std"] - ref_std
            print(f"{sc:8.2f} {r['roll_std']:10.2f} {r['roll_peak']:10.1f} {delta:+12.2f}")


if __name__ == "__main__":
    main()
