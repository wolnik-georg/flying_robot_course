"""Flown motor command → total thrust (battery-compensated inverse).

Logged `motor.m1..4` are PWM ratios **after** `motorsCompensateBatteryVoltage` in
`stabilizer.c` (CONFIG_ENABLE_THRUST_BAT_COMPENSATED). Inverse (CF21BL):

  V_i = (motor_m_i / 65535) * supplyVoltage
  T_i = c0 + c1*V + c2*V² + c3*V³
  F_cmd_total = sum_i T_i

`supplyVoltage` matches firmware: LPF on pm.vbat with b=0.01 (stabilizer.c L212-213).
Load vbat from the flight radio CSV (`time_s`, `vbat`), align to uSD via `lag_s`, interpolate
to the 1 kHz replay timeline, then LPF.

Legacy linear THRUST_MAX mapping (pre-bat-comp) kept for labelled comparisons only.
"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

PWM_MAX = 65535.0
THRUST_MAX_PER_MOTOR_N = 0.2  # CF21BL nominal (not the bat-comp inverse)

# platform_defaults_cf21bl.h VMOTOR2THRUST*
C0 = -0.014058926705279723
C1 = 0.04265273261724981
C2 = 0.0018327760144017432
C3 = 0.0020576974784587178

VBAT_LPF_B = 0.01  # stabilizer.c batteryCompensation


def per_motor_thrust_N_bat_comp(motor_pwm: np.ndarray, supply_voltage_V: np.ndarray | float) -> np.ndarray:
    """motor_pwm: (..., 4); supply_voltage_V broadcastable to (...) ."""
    m = np.asarray(motor_pwm, float) / PWM_MAX
    v = np.asarray(supply_voltage_V, float)
    if v.ndim == 0:
        v = np.full(m.shape[:-1], float(v))
    elif v.shape != m.shape[:-1]:
        v = np.broadcast_to(v, m.shape[:-1])
    vm = m * v[..., np.newaxis]
    return C0 + C1 * vm + C2 * vm**2 + C3 * vm**3


def commanded_total_thrust_N(
    motor_pwm: np.ndarray,
    supply_voltage_V: np.ndarray | float | None = None,
) -> np.ndarray:
    """Total commanded thrust [N] from logged motor PWM (bat-comp inverse).

    If supply_voltage_V is None, uses 4.2 V (legacy callers must pass vbat explicitly).
    """
    if supply_voltage_V is None:
        supply_voltage_V = 4.2
    return np.sum(per_motor_thrust_N_bat_comp(motor_pwm, supply_voltage_V), axis=-1)


def commanded_total_thrust_linear_legacy(motor_pwm: np.ndarray) -> np.ndarray:
    """Pre-Round-3 linear inverse: sum(m/65535)*THRUST_MAX — NOT flown mapping."""
    return (np.asarray(motor_pwm, float) / PWM_MAX * THRUST_MAX_PER_MOTOR_N).sum(axis=-1)


def lpf_supply_voltage(vbat_raw: np.ndarray, dt: float = 0.001, b: float = VBAT_LPF_B, seed: float = 4.2) -> np.ndarray:
    out = np.empty_like(vbat_raw, dtype=float)
    sv = float(seed)
    for i, v in enumerate(np.asarray(vbat_raw, float)):
        sv += b * (float(v) - sv)
        out[i] = sv
    return out


def read_radio_csv(radio_csv: Path) -> tuple[np.ndarray, np.ndarray]:
    """Return time_s, vbat arrays (skip # meta lines)."""
    times, vbats = [], []
    with radio_csv.open() as f:
        for line in f:
            if line.startswith("time_s,"):
                header = line.strip().split(",")
                break
        else:
            raise RuntimeError(f"No CSV header in {radio_csv}")
        vi = header.index("vbat")
        ti = header.index("time_s")
        for row in csv.reader(f):
            if not row or row[0].startswith("#"):
                continue
            try:
                times.append(float(row[ti]))
                vbats.append(float(row[vi]))
            except (ValueError, IndexError):
                continue
    return np.asarray(times, float), np.asarray(vbats, float)


def supply_voltage_on_usd_timeline(
    t_usd: np.ndarray,
    lag_s: float,
    radio_csv: Path,
    *,
    dt: float = 0.001,
) -> np.ndarray:
    """Map radio vbat → 1 kHz uSD timeline (t_usd seconds), then firmware LPF."""
    t_r, v_r = read_radio_csv(radio_csv)
    if len(t_r) < 2:
        return np.full(len(t_usd), 4.2)
    # uSD t_usd is absolute log time; radio time_s is session-relative from ~liftoff.
    t_log_rel = np.asarray(t_usd, float) - float(t_usd[0]) + float(lag_s)
    v_interp = np.interp(t_log_rel, t_r, v_r, left=v_r[0], right=v_r[-1])
    return lpf_supply_voltage(v_interp, dt=dt)


def flight_mean_vbat(radio_csv: Path, t_lo: float, t_hi: float) -> float:
    t_r, v_r = read_radio_csv(radio_csv)
    mask = (t_r >= t_lo) & (t_r <= t_hi)
    if not mask.any():
        return float(np.mean(v_r))
    return float(np.mean(v_r[mask]))


def radio_csv_for_meta(meta_path: Path, drone: str = "cf5") -> Path:
    """A8_2026-10-05_17-39-27.meta.json → A8_cf5_2026-10-05_17-39-27.csv"""
    stem = meta_path.name.replace(".meta.json", "")
    parts = stem.split("_", 1)
    scenario = parts[0]
    rest = parts[1] if len(parts) > 1 else stem
    return meta_path.parent / f"{scenario}_{drone}_{rest}.csv"
