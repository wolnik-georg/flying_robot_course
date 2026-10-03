"""Shared rules for meeting_simple_* figures (2026-10-03)."""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from meeting_hw_common import (
    CONTROLS_LOGS,
    LOGS,
    LIFTOFF_Z_M,
    STEADY_AFTER_LIFTOFF_S,
    TILT_EXCLUDE_DEG,
    TUMBLE_TILT_DEG,
    apply_mpl_style,
    controls_tilt_deg,
    liftoff_time,
    load_all_meta_lines,
    load_controls_csv,
    load_radio_csv,
    meta_json_for_csv,
    tilt_deg,
    trajectory_controller,
    trajectory_ki_z,
    z_command_for_vehicle,
)

OUT_ASSETS = Path(__file__).resolve().parents[2] / "docs/meetings/assets/2026-10-03"
OUT_TABLES = Path(__file__).resolve().parents[2] / "experiments/analysis/out/meeting_2026-10-03"

LANDING_BUFFER_S = 2.5
PLATEAU_DROP_M = 0.05
Z_ERR_YLIM_CM = 10.0
Z_BAND_CM = 2.0

SOLO_HOVER_KI16 = (
    "hover_mode1_kt0.008_2026-09-30_19-16-19.csv",
    "hover_mode1_kt0.008_2026-09-30_19-16-56.csv",
    "hover_mode1_kt0.008_2026-09-30_19-20-10.csv",
)
SOLO_F8_KI16 = (
    "figure8_mode1_kt0.05_2026-09-30_19-20-41.csv",
    "figure8_mode1_kt0.05_2026-09-30_19-21-14.csv",
)
Z_CMD_SOLO_M = 1.0
# Controls CSVs start on the ground (z≈−0.02 m); first z>0.05 m is not hover liftoff.
SOLO_HOVER_Z_START_M = 0.8

FORMATION_A8_1003 = (
    "A8_cf_second_2026-10-03_13-10-32.csv",
    "A8_cf_second_2026-10-03_13-15-00.csv",
)
CUT_1002 = {
    "A1": ("2026-10-02", "18-36-18"),
    "A8": ("2026-10-02", "18-23-23"),
}


def apply_meeting_style() -> None:
    apply_mpl_style()
    import matplotlib as mpl

    mpl.rcParams.update(
        {
            "font.size": 12,
            "axes.labelsize": 13,
            "axes.titlesize": 13,
            "legend.fontsize": 11,
            "xtick.labelsize": 11,
            "ytick.labelsize": 11,
            "figure.dpi": 150,
            "savefig.dpi": 150,
        }
    )


def stamp_from_name(path: Path) -> tuple[str, str]:
    m = re.search(r"(\d{4}-\d{2}-\d{2})_(\d{2}-\d{2}-\d{2})\.csv", path.name)
    if not m:
        return "?", "?"
    return m.group(1), m.group(2)


def stamp_ge(stamp_a: str, stamp_b: str) -> bool:
    return stamp_a >= stamp_b


@dataclass
class TrackRow:
    path: Path
    label: str
    scenario: str
    source: str  # hover | figure8 | formation
    vehicle: str
    included: bool
    exclude_reason: str
    t_lift: float
    t_steady_lo: float
    t_steady_hi: float
    t_plot_end: float
    z_cmd_m: float
    mean_err_cm: float
    mean_abs_err_cm: float
    max_abs_err_cm: float
    max_tilt_deg: float
    n_steady: int
    meta_note: str = ""


def hold_end_formation(
    scenario: str, radio_meta: dict[str, str], mj: dict[str, Any] | None, t_lift: float, t_last: float
) -> float:
    """A1: liftoff + param hold. A8: scenario duration minus landing buffer."""
    scen = scenario.upper()
    if scen == "A1":
        if mj and "params" in mj and "hold" in mj["params"]:
            hold = float(mj["params"]["hold"])
        else:
            hold = float(radio_meta.get("param_hold", 15.0))
        return t_lift + hold
    if scen == "A8":
        dur = float(radio_meta.get("duration_s", mj.get("duration", 26.0) if mj else 26.0))
        return min(t_lift + dur - LANDING_BUFFER_S, t_last - LANDING_BUFFER_S)
    return t_last - LANDING_BUFFER_S


def controls_hover_start(cols: dict[str, np.ndarray], z_min: float = SOLO_HOVER_Z_START_M) -> float | None:
    """First time altitude reaches the hover band (solo Controls logs)."""
    t, z = cols["time_s"], cols["z"]
    above = np.where(z >= z_min)[0]
    return float(t[above[0]]) if len(above) else None


def stitch_controls_time(t: np.ndarray) -> np.ndarray:
    """Make time strictly increasing when Controls logs reset each lap (figure-8)."""
    out = np.empty_like(t, dtype=float)
    out[0] = float(t[0])
    offset = 0.0
    for i in range(1, len(t)):
        if t[i] < t[i - 1] - 0.5:
            offset = out[i - 1] - float(t[i])
        out[i] = float(t[i]) + offset
    return out


def steady_end_before_descent(
    t: np.ndarray, z: np.ndarray, t_lo: float, z_cmd: float
) -> float:
    """Last sample still on plateau before landing descent."""
    lo_idx = int(np.where(t >= t_lo)[0][0]) if np.any(t >= t_lo) else 0
    plat_mask = (t >= t_lo) & (np.abs(z - z_cmd) < 0.12) & (z >= z_cmd - 0.2)
    if int(plat_mask.sum()) < 10:
        plat_mask = (t >= t_lo) & (z >= z_cmd - 0.15)
    plat_med = float(np.median(z[plat_mask])) if np.any(plat_mask) else float(z_cmd)
    drop_thr = plat_med - PLATEAU_DROP_M
    hold_thr = max(drop_thr, z_cmd - 0.02)
    last_plat = lo_idx
    for i in range(lo_idx, len(t) - 1):
        if z[i] >= hold_thr:
            last_plat = i
        if z[i] < drop_thr and z[i + 1] < z[i]:
            break
        if z[i] < hold_thr and z[i + 1] < z[i] and z[i + 1] < drop_thr:
            break
    return float(t[last_plat])


def controls_time_and_z(cols: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    t = stitch_controls_time(cols["time_s"])
    return t, cols["z"]


def formation_ki_z(mj: dict[str, Any] | None, vehicle: str, radio_meta: dict[str, str]) -> float:
    if mj and "per_drone" in mj and vehicle in mj["per_drone"]:
        kz = mj["per_drone"][vehicle].get("pos", {}).get("ki_z")
        if kz is not None:
            return float(kz)
    if "pos_ki_z" in radio_meta:
        return float(radio_meta["pos_ki_z"])
    return 0.0


def is_geometric_ki16(ctrl: int, mode: int, kz: float) -> bool:
    return ctrl == 6 and mode == 0 and abs(kz - 16.0) < 0.5


def solo_passes(path: Path) -> tuple[bool, str, dict[str, str]]:
    try:
        meta = load_all_meta_lines(path)
    except OSError as e:
        return False, str(e), {}
    c, m = trajectory_controller(meta)
    kz = trajectory_ki_z(meta)
    if c != 6 or m != 0:
        return False, f"trajectory controller {c} mode {m} (need 6/0)", meta
    if abs(kz - 16.0) > 0.5:
        return False, f"ki_z={kz} (need 16)", meta
    return True, "", meta


def analyze_controls_track(path: Path) -> TrackRow:
    label = path.name
    ok, reason, meta = solo_passes(path)
    traj = meta.get("run_trajectory", "?")
    scenario = "Hover" if traj == "hover" else "Figure-8"
    source = "hover" if traj == "hover" else "figure8"
    if not ok:
        return TrackRow(
            path, label, scenario, source, "cf5", False, reason,
            float("nan"), float("nan"), float("nan"), float("nan"), Z_CMD_SOLO_M,
            float("nan"), float("nan"), float("nan"), 0.0, 0,
        )
    try:
        _, cols = load_controls_csv(path)
    except ValueError as e:
        return TrackRow(
            path, label, scenario, source, "cf5", False, str(e),
            float("nan"), float("nan"), float("nan"), float("nan"), Z_CMD_SOLO_M,
            float("nan"), float("nan"), float("nan"), 0.0, 0,
        )
    t, z = controls_time_and_z(cols)
    tilt = controls_tilt_deg(cols)
    max_tilt = float(np.max(tilt)) if len(tilt) else 0.0
    if max_tilt > TUMBLE_TILT_DEG:
        return TrackRow(
            path, label, scenario, source, "cf5", False, f"max tilt {max_tilt:.1f}° > {TUMBLE_TILT_DEG:.0f}°",
            float("nan"), float("nan"), float("nan"), float("nan"), Z_CMD_SOLO_M,
            float("nan"), float("nan"), float("nan"), max_tilt, 0,
        )
    t_lift = controls_hover_start(cols)
    if t_lift is None:
        return TrackRow(
            path, label, scenario, source, "cf5", False, f"never reached z ≥ {SOLO_HOVER_Z_START_M} m",
            float("nan"), float("nan"), float("nan"), float("nan"), Z_CMD_SOLO_M,
            float("nan"), float("nan"), float("nan"), max_tilt, 0,
        )
    t_lo = t_lift + STEADY_AFTER_LIFTOFF_S
    t_hi = steady_end_before_descent(t, z, t_lo, Z_CMD_SOLO_M)
    mask = (t >= t_lo) & (t <= t_hi) & (tilt <= TILT_EXCLUDE_DEG)
    if not np.any(mask):
        return TrackRow(
            path, label, scenario, source, "cf5", False, "no steady samples (tilt>25° or empty window)",
            t_lift, t_lo, t_hi, float(t[-1]), Z_CMD_SOLO_M,
            float("nan"), float("nan"), float("nan"), max_tilt, 0,
        )
    err = (z[mask] - Z_CMD_SOLO_M) * 100.0
    return TrackRow(
        path, label, scenario, source, "cf5", True, "",
        t_lift, t_lo, t_hi, float(t[-1]), Z_CMD_SOLO_M,
        float(np.mean(err)), float(np.mean(np.abs(err))), float(np.max(np.abs(err))),
        max_tilt, int(mask.sum()),
        f"steady end = last sample before z >5 cm below plateau median & falling; stitched lap time",
    )


def analyze_formation_track(path: Path, vehicle: str, require_ki16: bool = True, assume_ki16: bool = False) -> TrackRow:
    scen = path.name.split("_")[0]
    label = path.name
    try:
        radio_meta, cols = load_radio_csv(path)
    except ValueError as e:
        return TrackRow(
            path, label, scen, "formation", vehicle, False, str(e),
            float("nan"), float("nan"), float("nan"), float("nan"), float("nan"),
            float("nan"), float("nan"), float("nan"), 0.0, 0,
        )
    mj = meta_json_for_csv(path)
    ctrl = int(float(radio_meta.get("controller", -1)))
    mode = int(float(radio_meta.get("ctrl_mode", -1)))
    kz = formation_ki_z(mj, vehicle, radio_meta)
    date, stamp = stamp_from_name(path)
    if require_ki16 and not is_geometric_ki16(ctrl, mode, kz):
        if assume_ki16 and ctrl == 6 and mode == 0 and kz == 0:
            meta_note = f"ki_z=16 from session yaml (not in {vehicle} radio # meta)"
        elif ctrl == 6 and mode == 0 and kz == 0 and vehicle == "cf_second":
            cut = CUT_1002.get(scen)
            explicit = path.name in FORMATION_A8_1003
            if explicit or (cut and date == cut[0] and stamp_ge(stamp, cut[1])):
                meta_note = "ki_z=16 from session yaml (not in cf_second radio # meta)"
            else:
                return TrackRow(
                    path, label, scen, "formation", vehicle, False,
                    f"controller {ctrl} mode {mode} ki_z={kz} (need geometric ki_z=16)",
                    float("nan"), float("nan"), float("nan"), float("nan"), float("nan"),
                    float("nan"), float("nan"), float("nan"), 0.0, 0,
                )
        else:
            return TrackRow(
                path, label, scen, "formation", vehicle, False,
                f"controller {ctrl} mode {mode} ki_z={kz} (need geometric ki_z=16)",
                float("nan"), float("nan"), float("nan"), float("nan"), float("nan"),
                float("nan"), float("nan"), float("nan"), 0.0, 0,
            )
    else:
        meta_note = f"c={ctrl} mode={mode} ki_z={kz}"

    t = cols["time_s"]
    z = cols["pos_z"]
    tilt = tilt_deg(cols)
    max_tilt = float(np.max(tilt)) if len(tilt) else 0.0
    if max_tilt > TUMBLE_TILT_DEG:
        return TrackRow(
            path, label, scen, "formation", vehicle, False, f"max tilt {max_tilt:.1f}° > {TUMBLE_TILT_DEG:.0f}°",
            float("nan"), float("nan"), float("nan"), float("nan"), float("nan"),
            float("nan"), float("nan"), float("nan"), max_tilt, 0, meta_note,
        )
    t_lift = liftoff_time(cols)
    if t_lift is None:
        return TrackRow(
            path, label, scen, "formation", vehicle, False, "no liftoff",
            float("nan"), float("nan"), float("nan"), float("nan"), float("nan"),
            float("nan"), float("nan"), float("nan"), max_tilt, 0, meta_note,
        )
    try:
        z_cmd = z_command_for_vehicle(radio_meta, mj, vehicle)
    except ValueError:
        z_cmd = float("nan")
    t_lo = t_lift + STEADY_AFTER_LIFTOFF_S
    t_desc = steady_end_before_descent(t, z, t_lo, z_cmd)
    t_cap = hold_end_formation(scen, radio_meta, mj, t_lift, float(t[-1]))
    if scen.upper() == "A8":
        # A8 `duration_s` is counted from the start of the trajectory (after takeoff), not from liftoff, so it
        # cuts off the last crossings. Use the real end of the hold = start of the landing descent instead.
        if vehicle == "cf5":
            # crossing dips (downwash, ~-8 cm) are not a descent: landing = first z > 10 cm below command, minus 1 s
            low = np.where((t >= t_lo) & (z < z_cmd - 0.10))[0]
            t_hi = float(t[low[0]]) - 1.0 if len(low) else float(t[-1]) - LANDING_BUFFER_S
        else:
            t_hi = t_desc
    else:
        t_hi = min(t_desc, t_cap)
    mask = (t >= t_lo) & (t <= t_hi) & (tilt <= TILT_EXCLUDE_DEG)
    if not np.any(mask):
        return TrackRow(
            path, label, scen, "formation", vehicle, False, "no steady samples",
            t_lift, t_lo, t_hi, float(t[-1]), z_cmd,
            float("nan"), float("nan"), float("nan"), max_tilt, 0, meta_note,
        )
    err = (z[mask] - z_cmd) * 100.0
    return TrackRow(
        path, label, scen, "formation", vehicle, True, "",
        t_lift, t_lo, t_hi, float(t[-1]), z_cmd,
        float(np.mean(err)), float(np.mean(np.abs(err))), float(np.max(np.abs(err))),
        max_tilt, int(mask.sum()), meta_note,
    )


def formation_candidate_paths() -> list[Path]:
    out: list[Path] = []
    for p in sorted(LOGS.glob("*_cf_second_*.csv")):
        scen = p.name.split("_")[0]
        if scen not in ("A1", "A8"):
            continue
        date, stamp = stamp_from_name(p)
        if p.name in FORMATION_A8_1003:
            out.append(p)
            continue
        cut = CUT_1002.get(scen)
        if cut and date == cut[0] and stamp_ge(stamp, cut[1]):
            out.append(p)
    for p in sorted(LOGS.glob("*_cf5_*.csv")):
        scen = p.name.split("_")[0]
        if scen not in ("A1", "A8"):
            continue
        mj = meta_json_for_csv(p)
        if mj and "per_drone" in mj and "cf5" in mj["per_drone"]:
            d = mj["per_drone"]["cf5"]
            kz = float(d.get("pos", {}).get("ki_z", 0) or 0)
            c, m = int(d["controller"]), int(d["ctrl_mode"])
            if is_geometric_ki16(c, m, kz):
                out.append(p)
    return out


def variant_name(controller: int, ctrl_mode: int) -> str | None:
    if controller == 9 and ctrl_mode == 0:
        return "Omar C"
    if controller == 10 and ctrl_mode == 0:
        return "Omar Rust"
    if controller == 6 and ctrl_mode == 3:
        return "Ours"
    return None
