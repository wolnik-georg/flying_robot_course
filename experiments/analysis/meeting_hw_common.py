"""Shared helpers for meeting figures (formation radio + Controls/simple_flight logs)."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

REPO = Path(__file__).resolve().parents[2]
LOGS = REPO / "experiments" / "logs"
CONTROLS_LOGS = REPO / "Controls" / "logs"

CF5_CSV_RE = re.compile(
    r"^(?P<scen>[A-Za-z0-9]+)_cf5_(?P<date>\d{4}-\d{2}-\d{2})_(?P<time>\d{2}-\d{2}-\d{2})\.csv$"
)
CF_SECOND_CSV_RE = re.compile(
    r"^(?P<scen>[A-Za-z0-9]+)_cf_second_(?P<date>\d{4}-\d{2}-\d{2})_(?P<time>\d{2}-\d{2}-\d{2})\.csv$"
)

TILT_EXCLUDE_DEG = 25.0
TUMBLE_TILT_DEG = 45.0
LIFTOFF_Z_M = 0.05
STEADY_AFTER_LIFTOFF_S = 6.0

# Fig. 1 Controls/logs plateau window
PLATEAU_Z_TOL_M = 0.05
PLATEAU_SETTLE_S = 3.0
DESCENT_VZ_M_S = -0.15
DESCENT_Z_DROP_M = 0.10

COL = {
    "green_dark": "#1F5C4D",
    "green_mid": "#4FB39A",
    "red": "#B4341C",
    "amber": "#9A6B12",
    "gray": "#6E7773",
    "blue": "#2E6DA4",
}

FIG1_YLIM_CM = 30.0


def apply_mpl_style() -> None:
    import matplotlib as mpl

    mpl.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 150,
            "font.size": 10,
            "axes.labelsize": 10,
            "axes.titlesize": 11,
            "legend.fontsize": 9,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "axes.grid": True,
            "grid.alpha": 0.35,
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )


def load_all_meta_lines(path: Path) -> dict[str, str]:
    meta: dict[str, str] = {}
    with open(path, newline="") as f:
        for line in f:
            if line.startswith("# meta:"):
                k, _, v = line[7:].partition("=")
                meta[k.strip()] = v.strip()
    return meta


def load_radio_csv(path: Path) -> tuple[dict[str, str], dict[str, np.ndarray]]:
    meta: dict[str, str] = {}
    header: list[str] | None = None
    rows: list[list[float]] = []
    with open(path, newline="") as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("# meta:"):
                k, _, v = line[7:].partition("=")
                meta[k.strip()] = v.strip()
            elif header is None and line and not line.startswith("#"):
                header = line.split(",")
            elif line and not line.startswith("#"):
                rows.append([float(x) for x in line.split(",")])
    if not header or not rows:
        raise ValueError(f"{path}: empty or missing header")
    cols = {n: np.array([r[i] for r in rows], dtype=float) for i, n in enumerate(header)}
    return meta, cols


def load_controls_csv(path: Path) -> tuple[dict[str, str], dict[str, np.ndarray]]:
    meta = load_all_meta_lines(path)
    header: list[str] | None = None
    rows: list[list[float]] = []
    with open(path, newline="") as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("# meta:"):
                continue
            if header is None and line.startswith("time_s,"):
                header = line.split(",")
            elif header and line and not line.startswith("#"):
                rows.append([float(x) for x in line.split(",")])
    if not header or not rows:
        raise ValueError(f"{path}: empty or missing header")
    cols = {n: np.array([r[i] for r in rows], dtype=float) for i, n in enumerate(header)}
    return meta, cols


def controls_tilt_deg(cols: dict[str, np.ndarray]) -> np.ndarray:
    r = cols.get("roll_deg", cols.get("roll", np.zeros_like(cols["time_s"])))
    p = cols.get("pitch_deg", cols.get("pitch", np.zeros_like(cols["time_s"])))
    return np.maximum(np.abs(r), np.abs(p))


def meta_json_for_csv(csv_path: Path) -> dict[str, Any] | None:
    m = CF5_CSV_RE.match(csv_path.name)
    if not m:
        m2 = CF_SECOND_CSV_RE.match(csv_path.name)
        if not m2:
            return None
        scen, date, time = m2.group("scen"), m2.group("date"), m2.group("time")
    else:
        scen, date, time = m.group("scen"), m.group("date"), m.group("time")
    for base in (csv_path.parent, LOGS):
        p = base / f"{scen}_{date}_{time}.meta.json"
        if p.is_file():
            return json.loads(p.read_text())
    p = LOGS / f"{scen}_{date}_{time}.meta.json"
    if p.is_file():
        return json.loads(p.read_text())
    return None


def tilt_deg(cols: dict[str, np.ndarray]) -> np.ndarray:
    r = cols.get("roll", np.zeros_like(cols["time_s"]))
    p = cols.get("pitch", np.zeros_like(cols["time_s"]))
    return np.maximum(np.abs(r), np.abs(p))


def liftoff_time(cols: dict[str, np.ndarray], z_thr: float = LIFTOFF_Z_M) -> float | None:
    t, z = cols["time_s"], cols["pos_z"]
    above = np.where(z > z_thr)[0]
    return float(t[above[0]]) if len(above) else None


def z_command_for_vehicle(
    radio_meta: dict[str, str], mj: dict[str, Any] | None, vehicle: str
) -> float:
    if mj is not None and "height" in mj:
        h = float(mj["height"])
        dz = float(mj.get("params", {}).get("dz", 0.0))
        names = mj.get("names", ["cf5", "cf_second"])
        if vehicle == names[0]:
            return h
        if vehicle in names[1:]:
            return h + dz
    if "height" in radio_meta:
        h = float(radio_meta["height"])
        dz = float(radio_meta.get("param_dz", 0.0) or 0.0)
        if vehicle == "cf5":
            return h
        return h + dz
    raise ValueError("no height in meta")


def hold_end_time(
    t_lift: float, radio_meta: dict[str, str], mj: dict[str, Any] | None
) -> float:
    if mj and "params" in mj and "hold" in mj["params"]:
        hold = float(mj["params"]["hold"])
        return t_lift + hold
    if "param_hold" in radio_meta:
        return t_lift + float(radio_meta["param_hold"])
    dur = float(mj["duration"]) if mj and "duration" in mj else float(radio_meta.get("duration_s", 0))
    return t_lift + dur


def cf5_config(radio_meta: dict[str, str], mj: dict[str, Any] | None) -> tuple[int, int, float]:
    if mj and "per_drone" in mj and "cf5" in mj["per_drone"]:
        d = mj["per_drone"]["cf5"]
        ctrl = int(d["controller"])
        mode = int(d["ctrl_mode"])
        kz = float(d.get("pos", {}).get("ki_z", 0.0) or 0.0)
    else:
        ctrl = int(float(radio_meta.get("controller", -1)))
        mode = int(float(radio_meta.get("ctrl_mode", -1)))
        kz = float(radio_meta.get("pos_ki_z", 0.0) or 0.0)
    if "pos_ki_z" in radio_meta:
        kz = float(radio_meta["pos_ki_z"])
    return ctrl, mode, kz


@dataclass
class SteadyMetrics:
    csv_path: Path
    scenario: str
    date: str
    controller: int
    ctrl_mode: int
    ki_z: float
    z_cmd_m: float
    t_lift: float
    t_steady_lo: float
    t_steady_hi: float
    n_steady: int
    n_excluded_tilt: int
    mean_err_m: float
    rmse_m: float
    max_tilt_deg: float
    tumbled: bool
    included: bool
    exclude_reason: str = ""
    extra: dict[str, Any] = field(default_factory=dict)

    @property
    def mean_err_cm(self) -> float:
        return self.mean_err_m * 100.0

    @property
    def rmse_cm(self) -> float:
        return self.rmse_m * 100.0


@dataclass
class PlateauMetrics:
    path: Path
    group: str
    z_cmd_m: float
    t_lo: float
    t_hi: float
    mean_err_m: float
    mean_err_first_half_m: float
    mean_err_second_half_m: float
    max_tilt_deg: float
    included: bool
    exclude_reason: str = ""
    meta: dict[str, str] = field(default_factory=dict)

    @property
    def mean_err_cm(self) -> float:
        return self.mean_err_m * 100.0


def plateau_window(
    t: np.ndarray,
    z: np.ndarray,
    vz: np.ndarray,
    z_cmd: float,
    tilt: np.ndarray,
) -> tuple[float, float, np.ndarray]:
    """Return (t_lo, t_hi, boolean mask) for steady plateau."""
    near = np.abs(z - z_cmd) < PLATEAU_Z_TOL_M
    if not np.any(near):
        return float("nan"), float("nan"), np.zeros(len(t), dtype=bool)

    i0 = int(np.where(near)[0][0])
    t_enter = float(t[i0])
    t_lo = t_enter + PLATEAU_SETTLE_S

    t_hi = float(t[-1])
    for i in range(i0, len(t)):
        if t[i] < t_lo:
            continue
        if vz[i] < DESCENT_VZ_M_S or z[i] < z_cmd - DESCENT_Z_DROP_M:
            t_hi = float(t[i])
            break

    mask = (t >= t_lo) & (t <= t_hi) & (tilt <= TILT_EXCLUDE_DEG)
    return t_lo, t_hi, mask


def analyze_controls_flight(path: Path, z_cmd: float = 1.0) -> PlateauMetrics:
    meta, cols = load_controls_csv(path)
    t = cols["time_s"]
    z = cols["z"]
    vz = cols["vz"]
    tilt = controls_tilt_deg(cols)
    max_tilt = float(np.max(tilt)) if len(tilt) else 0.0

    if meta.get("run_trajectory") == "figure8":
        # Hover-like samples along the path (commanded nominal height 1.0 m).
        mask = (
            (np.abs(z - z_cmd) < PLATEAU_Z_TOL_M)
            & (tilt <= TILT_EXCLUDE_DEG)
            & (np.abs(vz) < 0.12)
        )
        t_lo = float(t[mask][0]) if np.any(mask) else float("nan")
        t_hi = float(t[mask][-1]) if np.any(mask) else float("nan")
    else:
        t_lo, t_hi, mask = plateau_window(t, z, vz, z_cmd, tilt)
    if max_tilt > TUMBLE_TILT_DEG:
        return PlateauMetrics(
            path, "", z_cmd, t_lo, t_hi, float("nan"), float("nan"), float("nan"),
            max_tilt, False, f"max tilt {max_tilt:.1f}° > {TUMBLE_TILT_DEG:.0f}°",
            meta=meta,
        )

    if not np.any(mask):
        return PlateauMetrics(
            path, "", z_cmd, t_lo, t_hi, float("nan"), float("nan"), float("nan"),
            max_tilt, False, "no plateau samples (tilt>25° or never within 5 cm of target)",
            meta=meta,
        )

    err = z[mask] - z_cmd
    mid = len(err) // 2
    return PlateauMetrics(
        path,
        "",
        z_cmd,
        t_lo,
        t_hi,
        float(np.mean(err)),
        float(np.mean(err[:mid])) if mid else float("nan"),
        float(np.mean(err[mid:])) if mid else float("nan"),
        max_tilt,
        True,
        "",
        meta=meta,
    )


def trajectory_controller(meta: dict[str, str]) -> tuple[int, int]:
    c = int(float(meta.get("trajectory_stabilizer_controller", -1)))
    m = int(float(meta.get("trajectory_ctrl_mode", -1)))
    return c, m


def trajectory_ki_z(meta: dict[str, str]) -> float:
    return float(meta.get("full_pos_gains_ki_z", 0) or 0)


def steady_height_error(path: Path, vehicle: str = "cf5") -> SteadyMetrics:
    radio_meta, cols = load_radio_csv(path)
    mj = meta_json_for_csv(path)
    scen = path.name.split("_")[0]
    dm = re.search(r"(\d{4}-\d{2}-\d{2})", path.name)
    date = dm.group(1) if dm else "?"

    if vehicle == "cf5":
        ctrl, mode, kz = cf5_config(radio_meta, mj)
    elif mj and "per_drone" in mj and "cf_second" in mj["per_drone"]:
        d = mj["per_drone"]["cf_second"]
        ctrl = int(d["controller"])
        mode = int(d["ctrl_mode"])
        kz = float(d.get("pos", {}).get("ki_z", 0.0) or 0.0)
        if "pos_ki_z" in radio_meta:
            kz = float(radio_meta["pos_ki_z"])
    else:
        ctrl, mode, kz = 6, 0, 0.0

    t = cols["time_s"]
    z = cols["pos_z"]
    tilt = tilt_deg(cols)
    max_tilt = float(np.max(tilt)) if len(tilt) else 0.0
    tumbled = max_tilt > TUMBLE_TILT_DEG

    t_lift = liftoff_time(cols)
    if t_lift is None:
        return SteadyMetrics(
            path, scen, date, ctrl, mode, kz, float("nan"), float("nan"),
            float("nan"), float("nan"), 0, 0, float("nan"), float("nan"),
            max_tilt, tumbled, False, "no liftoff",
        )

    z_cmd = z_command_for_vehicle(radio_meta, mj, vehicle)
    t_hi = hold_end_time(t_lift, radio_meta, mj)
    t_lo = t_lift + STEADY_AFTER_LIFTOFF_S
    win = (t >= t_lo) & (t <= t_hi)
    calm = tilt <= TILT_EXCLUDE_DEG
    mask = win & calm
    n_excl = int(win.sum() - mask.sum())

    if not np.any(mask):
        reason = "no steady samples (tilt>25° or window empty)"
        return SteadyMetrics(
            path, scen, date, ctrl, mode, kz, z_cmd, t_lift, t_lo, t_hi,
            0, n_excl, float("nan"), float("nan"), max_tilt, tumbled, False, reason,
        )

    err = z[mask] - z_cmd
    return SteadyMetrics(
        path,
        scen,
        date,
        ctrl,
        mode,
        kz,
        z_cmd,
        t_lift,
        t_lo,
        t_hi,
        int(mask.sum()),
        n_excl,
        float(np.mean(err)),
        float(np.sqrt(np.mean(err**2))),
        max_tilt,
        tumbled,
        True,
        "",
    )


def att_std_steady_window(path: Path) -> tuple[float, float] | None:
    """Roll/pitch std (deg) in formation steady window for cf5."""
    sm = steady_height_error(path)
    if not sm.included:
        return None
    _, cols = load_radio_csv(path)
    t = cols["time_s"]
    mask = (t >= sm.t_steady_lo) & (t <= sm.t_steady_hi)
    tilt = tilt_deg(cols)
    mask &= tilt <= TILT_EXCLUDE_DEG
    if not np.any(mask):
        return None
    r = cols["roll"][mask]
    p = cols["pitch"][mask]
    return float(np.std(r)), float(np.std(p))


def paired_cf_second(cf5_path: Path) -> Path | None:
    name = cf5_path.name.replace("_cf5_", "_cf_second_")
    p = cf5_path.parent / name
    if p.is_file():
        return p
    p2 = LOGS / name
    return p2 if p2.is_file() else None


def variant_label(controller: int, ctrl_mode: int) -> str | None:
    if controller == 9:
        return "Omar C (c=9)"
    if controller == 10:
        return "Omar Rust (c=10)"
    if controller == 6 and ctrl_mode == 3:
        return "Ours full INDI (c=6, mode=3)"
    return None


def ours_a8_marker_class(path: Path) -> str:
    """filled_clean | hollow_crash | hollow_abort for A8 ours INDI flights."""
    name = path.name
    if "18-57-49" in name or "18-59-17" in name:
        return "hollow_crash"
    if "19-08-13" in name:
        return "hollow_abort"
    if "19-09-54" in name or "19-11-30" in name:
        return "filled_clean"
    return "hollow_other"
