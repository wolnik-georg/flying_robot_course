#!/usr/bin/env python3
"""Metrics for NS2 closed-loop SIL (partner sanity, prediction stats, A8 crossing dips)."""

from __future__ import annotations

import numpy as np

TOP_GYRO_OK_DEG_S = 20.0
STEADY_T0 = 4.0


def find_crossing_times(t: np.ndarray, y: np.ndarray, n_expect: int = 4) -> list[float]:
    """Bottom drone A8: horizontal axis y; crossings ≈ |y| minima (same as ns2_2026_10_05_crossing_dip.py)."""
    y = np.asarray(y, float)
    t = np.asarray(t, float)
    if len(t) < 100:
        return []
    dy = np.abs(y)
    w = min(31, len(dy) // 10 * 2 + 1)
    if w >= 5:
        k = np.ones(w) / w
        dy_s = np.convolve(dy, k, mode="same")
    else:
        dy_s = dy
    order = max(1, len(t) // 200)
    mins: list[float] = []
    for i in range(order, len(t) - order):
        if dy_s[i] <= dy_s[i - order : i + order + 1].min() + 1e-6:
            if not mins or t[i] - mins[-1] > 2.5:
                mins.append(float(t[i]))
    if len(mins) > n_expect:
        mins.sort(key=lambda tc: np.min(dy[(t >= tc - 0.5) & (t <= tc + 0.5)]))
        mins = sorted(mins[:n_expect])
    return mins


def crossing_dip_stats(
    t: np.ndarray,
    pos_bot: np.ndarray,
    sp_bot: np.ndarray,
    *,
    n_crossings: int = 4,
    window_s: float = 1.0,
) -> dict:
    """Per-crossing minimum z tracking error [cm] (negative = dip below setpoint)."""
    t = np.asarray(t, float)
    pos = np.asarray(pos_bot, float)
    sp = np.asarray(sp_bot, float)
    tcross = find_crossing_times(t, pos[:, 1], n_expect=n_crossings)
    dips: list[float] = []
    for tc in tcross:
        m = (t >= tc - window_s) & (t <= tc + window_s)
        if not np.any(m):
            continue
        e_z_cm = (pos[m, 2] - sp[m, 2]) * 100.0
        dips.append(float(np.min(e_z_cm)))
    dips_arr = np.asarray(dips, float)
    out = {
        "n_crossings": len(dips),
        "dip_cm_each": dips,
        "dip_cm_mean": float(np.mean(dips_arr)) if len(dips) else float("nan"),
        "dip_cm_min": float(np.min(dips_arr)) if len(dips) else float("nan"),
        "dip_cm_max": float(np.max(dips_arr)) if len(dips) else float("nan"),
        "t_cross_s": tcross,
    }
    return out


def gyro_rms_deg(t: np.ndarray, gyro_deg: np.ndarray, t0: float = STEADY_T0) -> float:
    m = t >= t0
    g = gyro_deg[m]
    if g.ndim > 1:
        return float(np.sqrt(np.mean(np.sum(g**2, axis=-1))))
    return float(np.sqrt(np.mean(g**2)))


def pred_stats(pred_z: np.ndarray, pred_x: np.ndarray, pred_y: np.ndarray, clamp: np.ndarray) -> dict:
    pred_z = np.asarray(pred_z, float)
    pred_x = np.asarray(pred_x, float)
    pred_y = np.asarray(pred_y, float)
    clamp = np.asarray(clamp, float)
    n = len(pred_z)
    return {
        "n": n,
        "pred_z_mean": float(np.mean(pred_z)) if n else float("nan"),
        "pred_z_std": float(np.std(pred_z)) if n else float("nan"),
        "pred_z_min": float(np.min(pred_z)) if n else float("nan"),
        "pred_z_max": float(np.max(pred_z)) if n else float("nan"),
        "pred_x_max_abs": float(np.max(np.abs(pred_x))) if n else float("nan"),
        "pred_y_max_abs": float(np.max(np.abs(pred_y))) if n else float("nan"),
        "clamp_fraction": float(np.mean(clamp > 0)) if n else float("nan"),
    }


def corr_pred_a_res(pred_z: np.ndarray, a_res_z: np.ndarray) -> dict:
    n = min(len(pred_z), len(a_res_z))
    if n < 10:
        return {"corr": float("nan"), "rmse": float("nan"), "n": n}
    p = np.asarray(pred_z[:n], float)
    a = np.asarray(a_res_z[:n], float)
    if np.std(p) < 1e-9 or np.std(a) < 1e-9:
        corr = float("nan")
    else:
        corr = float(np.corrcoef(p, a)[0, 1])
    return {"corr": corr, "rmse": float(np.sqrt(np.mean((p - a) ** 2))), "n": n}


def steady_mask(
    t: np.ndarray,
    pos: np.ndarray,
    *,
    scenario: str,
    takeoff_s: float = STEADY_T0,
    n_crossings: int = 4,
    cross_window_s: float = 1.0,
) -> np.ndarray:
    t = np.asarray(t, float)
    m = t >= takeoff_s
    if scenario == "A8" and n_crossings > 0 and len(pos):
        tcross = find_crossing_times(t, np.asarray(pos, float)[:, 1], n_expect=n_crossings)
        for tc in tcross:
            m &= ~((t >= tc - cross_window_s) & (t <= tc + cross_window_s))
    return m


def tracking_outside_crossings(
    t: np.ndarray,
    pos: np.ndarray,
    sp: np.ndarray,
    scenario: str,
    *,
    takeoff_s: float = STEADY_T0,
    n_crossings: int = 4,
) -> dict:
    m = steady_mask(t, pos, scenario=scenario, takeoff_s=takeoff_s, n_crossings=n_crossings)
    if not np.any(m):
        return tracking_metrics(t, pos, sp, t0=takeoff_s)
    p = np.asarray(pos, float)[m]
    s = np.asarray(sp, float)[m]
    e = p - s
    return {
        "rms_z_cm": float(np.sqrt(np.mean(e[:, 2] ** 2)) * 100),
        "rms_lateral_cm": float(np.sqrt(np.mean(np.sum(e[:, :2] ** 2, axis=1))) * 100),
        "mean_z_err_cm": float(np.mean(e[:, 2]) * 100),
        "max_z_dip_cm": float(np.min(e[:, 2]) * 100),
        "max_tilt_proxy_deg": float("nan"),
        "n_samples": int(np.sum(m)),
    }


def tracking_metrics(t: np.ndarray, pos: np.ndarray, sp: np.ndarray, t0: float = STEADY_T0) -> dict:
    m = t >= t0
    p = pos[m]
    s = sp[m]
    e = p - s
    return {
        "rms_z_cm": float(np.sqrt(np.mean(e[:, 2] ** 2)) * 100),
        "rms_lateral_cm": float(np.sqrt(np.mean(np.sum(e[:, :2] ** 2, axis=1))) * 100),
        "mean_z_err_cm": float(np.mean(e[:, 2]) * 100),
        "max_z_dip_cm": float(np.min(e[:, 2]) * 100),
        "max_tilt_proxy_deg": float("nan"),
    }


def partner_ok(top_gyro_rms: float, *, pos_top: np.ndarray | None = None, t: np.ndarray | None = None) -> bool:
    """Top partner sane: gyro RMS in flight band, not a crashed-on-floor artifact."""
    if not (np.isfinite(top_gyro_rms) and top_gyro_rms < TOP_GYRO_OK_DEG_S):
        return False
    if pos_top is not None and t is not None and len(t) == len(pos_top):
        m = (t >= STEADY_T0) & (t <= t[-1])
        if np.any(m):
            z = pos_top[m, 2]
            if float(np.max(z) - np.min(z)) < 0.05 and float(np.mean(z)) < 0.15:
                return False
    return True


def tilt_from_quat_deg(q_wxyz: np.ndarray) -> float:
    """Angle between body +z and world +z (rowan quaternions are w,x,y,z)."""
    import rowan

    q = np.asarray(q_wxyz, float).reshape(-1)
    if q.shape[0] != 4:
        return float("nan")
    q = q / max(float(np.linalg.norm(q)), 1e-12)
    bz = rowan.rotate(q, np.array([0.0, 0.0, 1.0]))
    return float(np.degrees(np.arccos(np.clip(bz[2], -1.0, 1.0))))


def max_tilt_deg(quat_wxyz: np.ndarray) -> float:
    q = np.asarray(quat_wxyz, float)
    if q.ndim == 1:
        return tilt_from_quat_deg(q)
    if len(q) == 0:
        return float("nan")
    return float(np.max([tilt_from_quat_deg(qi) for qi in q]))


def _test_tilt_metric() -> None:
    import rowan

    q30 = rowan.from_euler(0.0, np.radians(30.0), 0.0)
    got = tilt_from_quat_deg(q30)
    assert abs(got - 30.0) < 0.05, got
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    assert tilt_from_quat_deg(q0) < 0.01


if __name__ == "__main__":
    _test_tilt_metric()
    print("tilt metric ok")
