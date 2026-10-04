#!/usr/bin/env python3
"""RPM measurement path model (delay, hold, noise, DShot slew) for SIL."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# traj_iface.c rpm_get_all — DShot-only (g_indi_rpm_source != 0)
DSHOT_RPM_ABS_MAX = 28000
DSHOT_SLEW_MAX_RPM = 10000


@dataclass
class RpmMeasurementModel:
    delay_s: float = 0.0
    f_rpm: float = 1000.0
    noise_std_rpm: float = 100.0
    rpm_source_dshot: bool = True
    rng: np.random.Generator = field(default_factory=lambda: np.random.default_rng(0))

    _hist_t: list = field(default_factory=list)
    _hist_rpm: list = field(default_factory=list)
    _held: np.ndarray | None = None
    _last_hold_t: float = -1e9
    _rpm_prev: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.uint16))

    def reset(self) -> None:
        self._hist_t.clear()
        self._hist_rpm.clear()
        self._held = None
        self._last_hold_t = -1e9
        self._rpm_prev[:] = 0

    def push_true(self, t: float, rpm: np.ndarray) -> None:
        rpm = np.asarray(rpm, dtype=float)
        self._hist_t.append(float(t))
        self._hist_rpm.append(rpm.copy())
        if len(self._hist_t) > 5000:
            self._hist_t.pop(0)
            self._hist_rpm.pop(0)

    def _true_at(self, t_query: float) -> np.ndarray:
        if not self._hist_t:
            return np.zeros(4)
        t_arr = np.asarray(self._hist_t)
        rpm_arr = np.stack(self._hist_rpm, axis=0)
        t_q = float(t_query) - float(self.delay_s)
        if t_q <= t_arr[0]:
            return rpm_arr[0]
        if t_q >= t_arr[-1]:
            return rpm_arr[-1]
        i = int(np.searchsorted(t_arr, t_q, side="right") - 1)
        i = max(0, min(i, len(t_arr) - 2))
        t0, t1 = t_arr[i], t_arr[i + 1]
        a = (t_q - t0) / (t1 - t0 + 1e-12)
        return (1 - a) * rpm_arr[i] + a * rpm_arr[i + 1]

    def _apply_dshot_slew(self, v: np.ndarray) -> np.ndarray:
        out = v.copy()
        for i in range(4):
            vi = int(round(out[i]))
            if vi <= 0:
                continue
            if vi > DSHOT_RPM_ABS_MAX:
                if self._rpm_prev[i] > 0:
                    out[i] = self._rpm_prev[i]
                continue
            prev = int(self._rpm_prev[i])
            if prev > 500:
                lo, hi = min(vi, prev), max(vi, prev)
                if hi - lo > DSHOT_SLEW_MAX_RPM and prev > 0:
                    out[i] = prev
                    continue
            self._rpm_prev[i] = np.uint16(vi)
        return out

    def read(self, t: float) -> np.ndarray:
        delayed = self._true_at(t)
        if self.f_rpm >= 999.0:
            sample = delayed
        else:
            period = 1.0 / float(self.f_rpm)
            if self._held is None or (t - self._last_hold_t) >= period - 1e-9:
                self._held = delayed.copy()
                self._last_hold_t = float(t)
            sample = self._held

        if self.noise_std_rpm > 0:
            sample = sample + self.rng.normal(0.0, self.noise_std_rpm, size=4)
        sample = np.maximum(sample, 0.0)

        if self.rpm_source_dshot:
            sample = self._apply_dshot_slew(sample)

        return np.asarray(np.round(sample), dtype=int)
