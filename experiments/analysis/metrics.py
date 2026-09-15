"""Flight-metrics core for run_analysis.py.

Three log formats exist in this repo and they are NOT the same schema:

  ros     Per-vehicle CSV written by crazyflie_examples' DroneLogger
          (formation_flight.py / run_formation.py). Header:
          time_s,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,roll,pitch,yaw,thrust,vbat,
          gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,tau_x,tau_y,tau_z,alp_x,alp_y,alp_z,
          a_res_x,a_res_y,a_res_z
          No e_R yet (not wired into this path -- see README). Radio-only, so on
          hardware it is real; every example of it produced IN SIM by this repo so
          far has ZERO data rows, because the sim server never publishes the custom
          log topics DroneLogger subscribes to (confirmed 2026-09-08 while building
          this script -- every A1/A2/B2 sim CSV on disk has a header and no rows).

  merged  One CSV, one shared `t` column, then `{vehicle}.{field}` columns for every
          vehicle -- the format `tools/merge_usd_logs.py` produces from real uSD
          logs, and also what `analyse_residual_dryrun.py`'s sim dry run wrote
          directly. This is the only format with actual populated sim data on disk
          today (experiments/sim_validation/residual_collect.csv).

  usd     A single vehicle's decoded uSD binary log, via
          `flying_drone_stack/tools/decode_usd_log.py`'s `load()` -- a dict of
          numpy arrays keyed by the same short names (`x`, `a_res_x`, `e_r_x`, ...).
          Hardware only; carries `e_R` once a vehicle is flashed with the
          2026-09-08 logging addition.

There is no converter between `ros` and the other two because `ros` is missing
fields (`e_R`, `rnn_pred`, `ctrltarget`, `motor`) that were never wired into that
path -- see experiments/analysis/README.md.
"""
from __future__ import annotations

from dataclasses import dataclass
import re
import sys
from pathlib import Path

import numpy as np

# find_flight_window.py --extract's documented naming (README_usd_thesis_logging.md):
# <scenario>_<drone>_<date>_<time>_flight.csv. Recovering the plain drone name from it (rather
# than defaulting to the whole stem) is what lets a --sidecar's meta['names'] match these files
# without the caller having to rename anything.
_FLIGHT_CSV_NAME_RE = re.compile(
    r"^[A-Za-z]+\d+_(?P<drone>.+)_\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}_flight$")

ROS_HEADER = [
    "time_s", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z",
    "roll", "pitch", "yaw", "thrust", "vbat",
    "gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z",
    "tau_x", "tau_y", "tau_z", "alp_x", "alp_y", "alp_z",
    "a_res_x", "a_res_y", "a_res_z",
]

SIM_STATES_HEADER = ["timestamp", "x", "y", "z", "qw", "qx", "qy", "qz"]


@dataclass
class VehicleLog:
    name: str
    fmt: str                      # 'ros' | 'merged' | 'usd' | 'sim'
    t: np.ndarray                 # seconds
    pos: np.ndarray | None        # (N,3) measured position, world frame
    pos_des: np.ndarray | None    # (N,3) commanded/setpoint position, if directly logged
    a_res: np.ndarray | None      # (N,3)
    a_hat: np.ndarray | None      # (N,3) predicted residual (rnn_pred_*)
    e_r: np.ndarray | None        # (N,3)
    n_raw: int                    # rows in the source before any windowing
    source: str
    # 2026-09-15: what t=0 in THIS log actually means, for commanded_from_scenario callers.
    #   'scenario_start' -- t=0 IS the scenario's own t=0 (a --meta-aligned merged uSD file).
    #                        Reconstruct with t0=0, timescale=1 -- no meta.json needed at all.
    #   'sim_wall'        -- t is on the same absolute clock as a sidecar's t_start_sim
    #                        (ros-format radio logs). Reconstruct with t0=t_start_sim.
    #   'unknown'         -- neither is known to hold (an unaligned uSD merge, a raw usd log
    #                        with no sidecar context, sim). Do not attempt reconstruction.
    t_zero: str = "unknown"


def _read_text_header(path: Path) -> tuple[list[str], int]:
    """Return (header columns, line index of the header) skipping '# meta:' lines."""
    with open(path) as fh:
        for i, line in enumerate(fh):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            return line.split(","), i
    raise ValueError(f"{path}: no header line found")


def _read_meta_comments(path: Path) -> dict[str, str]:
    """Parse leading '# meta:key=value' lines into a dict. Stops at the first non-comment,
    non-blank line (the real header), same scan `_read_text_header` does."""
    out = {}
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            if line.startswith("# meta:"):
                k, _, v = line[len("# meta:"):].partition("=")
                out[k.strip()] = v.strip()
                continue
            if line.startswith("#"):
                continue
            break
    return out


def detect_format(path: Path) -> str:
    """ros / merged / usd_csv / sim by header; anything that fails to parse as text is 'usd'
    (the raw binary uSD log)."""
    try:
        header, _ = _read_text_header(path)
    except (UnicodeDecodeError, ValueError):
        return "usd"
    if header == ROS_HEADER:
        return "ros"
    if header == SIM_STATES_HEADER:
        return "sim"
    if header and header[0] == "t" and any("." in c for c in header[1:]):
        return "merged"
    # 2026-09-15: find_flight_window.py --extract writes a single-vehicle CSV in the same
    # short-name schema decode_usd_log.load() returns (t,x,y,z,...,a_res_x,...) but as text,
    # not the binary uSD format -- distinguished from `merged` by having no dotted (per-
    # vehicle-prefixed) column names. This used to fall through to the SystemExit below and
    # reject every extracted flight CSV outright.
    if header and header[0] == "t" and not any("." in c for c in header[1:]):
        return "usd_csv"
    raise SystemExit(
        f"{path}: header does not match any known format (ros/merged/usd_csv/sim), and it "
        f"is valid text so it is not a uSD binary either. Got: {header[:6]}...")


def load_ros_csv(path: Path, name: str | None = None) -> VehicleLog:
    header, skiprows = _read_text_header(path)
    assert header == ROS_HEADER
    data = np.loadtxt(path, delimiter=",", skiprows=skiprows + 1, ndmin=2)
    n_raw = data.shape[0]
    if n_raw == 0:
        # A ros-format file with zero data rows is a KNOWN, real state in this repo
        # (see module docstring) -- return empty arrays rather than crash, so the
        # caller can report it (n_used=0) instead of the script dying on `[0]`.
        t = np.zeros(0)
        pos = np.zeros((0, 3))
        a_res = np.zeros((0, 3))
    else:
        t = data[:, 0]
        pos = data[:, 1:4]
        a_res = data[:, 24:27]
    # Existing behaviour, unchanged: plot_flight.py already reconstructs the commanded
    # trajectory for this format using t0=meta['t_start_sim']. Recorded here explicitly so
    # that assumption is a stated fact this loader owns, not something the caller has to know.
    return VehicleLog(name or path.stem, "ros", t, pos, None, a_res, None, None,
                       n_raw, str(path), t_zero="sim_wall")


def load_sim_states_csv(path: Path, name: str | None = None) -> VehicleLog:
    header, skiprows = _read_text_header(path)
    assert header == SIM_STATES_HEADER
    data = np.loadtxt(path, delimiter=",", skiprows=skiprows + 1, ndmin=2)
    n_raw = data.shape[0]
    t = data[:, 0] if n_raw else np.zeros(0)
    pos = data[:, 1:4] if n_raw else np.zeros((0, 3))
    # record_states carries pose only (position + quaternion) -- no controller-internal
    # signal (a_res, e_R, rnn_pred) exists in this format at all; NaN, not 0.
    return VehicleLog(name or path.stem, "sim", t, pos, None, None, None, None,
                       n_raw, str(path))


def load_merged_csv(path: Path) -> dict[str, VehicleLog]:
    header, skiprows = _read_text_header(path)
    # 2026-09-15: merge_usd_logs.py --meta writes `# meta:t_zero=scenario_start` when it
    # re-zeroed every log at its own scenario start (each drone against ITS OWN commanded
    # trajectory) -- see that flag's own docstring in write_csv(). Without reading it back,
    # commanded_from_scenario() cannot tell this file's t=0 from an unaligned merge's t=0
    # (wherever the first log's raw samples happened to start), and silently reconstructing
    # against the wrong zero produced an all-NaN "Position tracking error" panel and
    # pos_rmse=nan on a real, good flight (A8, 2026-09-15 19:56) -- not because the data was
    # bad, but because t0 was off by ~1.7e9 seconds (a wall-clock t_start_sim applied to a
    # clock that was already zeroed).
    t_zero = _read_meta_comments(path).get("t_zero", "unknown")
    data = np.loadtxt(path, delimiter=",", skiprows=skiprows + 1, ndmin=2)
    n_raw = data.shape[0]
    cols = {c: i for i, c in enumerate(header)}
    names = sorted({c.split(".", 1)[0] for c in header[1:] if "." in c})
    t_all = data[:, cols["t"]] if n_raw else np.zeros(0)

    def col(name, field):
        key = f"{name}.{field}"
        return data[:, cols[key]] if key in cols and n_raw else None

    def vec3(name, prefix):
        parts = [col(name, f"{prefix}_{ax}") for ax in "xyz"]
        if any(p is None for p in parts):
            return None
        return np.stack(parts, axis=1)

    out = {}
    for name in names:
        pos = vec3(name, "") if all(f"{name}.{ax}" in cols for ax in "xyz") else None
        if pos is None:
            pos = np.stack([col(name, ax) for ax in "xyz"], axis=1) \
                if all(f"{name}.{ax}" in cols for ax in "xyz") else None
        pos_des = vec3(name, "cmd")
        a_res = vec3(name, "a_res")
        a_hat = vec3(name, "rnn_pred")
        e_r = vec3(name, "e_r")
        out[name] = VehicleLog(name, "merged", t_all, pos, pos_des, a_res, a_hat, e_r,
                                n_raw, str(path), t_zero=t_zero)
    return out


def load_usd_csv(path: Path, name: str | None = None) -> VehicleLog:
    """A single-vehicle CSV in decode_usd_log's short-name schema, as written by
    find_flight_window.py --extract. Text, not the binary uSD format -- see load_usd() for
    that. `t=0` in this file IS the scenario start (--extract sets `tt = t[mask] - lag`), so
    t_zero='scenario_start' exactly like a --meta-aligned merge; commanded_from_scenario()
    needs no meta.json wall-clock reconciliation for it. Unlike a merge, this format also
    carries `ctrltarget_*` -- the firmware's OWN logged setpoint -- so pos_des is read
    directly here rather than reconstructed at all, which is strictly more trustworthy.
    """
    header, skiprows = _read_text_header(path)
    data = np.loadtxt(path, delimiter=",", skiprows=skiprows + 1, ndmin=2)
    n_raw = data.shape[0]
    cols = {c: i for i, c in enumerate(header)}
    t = data[:, cols["t"]] if n_raw else np.zeros(0)

    def vec3(prefix):
        # 2026-09-15: `f"{prefix}_{ax}"` with prefix="" builds "_x", not "x" -- silently
        # returning None for `pos` on every call. Found via real data: this loader's own
        # extracted flight CSV has x/y/z columns and pos still came back None.
        keys = [f"{prefix}_{ax}" if prefix else ax for ax in "xyz"]
        if not all(k in cols for k in keys) or n_raw == 0:
            return None
        return np.stack([data[:, cols[k]] for k in keys], axis=1)

    pos = vec3("") if all(ax in cols for ax in "xyz") else None
    pos_des = vec3("ctrltarget")
    a_res = vec3("a_res")
    a_hat = vec3("rnn_pred")
    e_r = vec3("e_r")
    if name is None:
        m = _FLIGHT_CSV_NAME_RE.match(path.stem)
        name = m.group("drone") if m else path.stem
    return VehicleLog(name, "usd_csv", t, pos, pos_des, a_res, a_hat, e_r,
                       n_raw, str(path), t_zero="scenario_start")


def load_usd(path: Path, name: str | None = None) -> VehicleLog:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                           "flying_drone_stack" / "tools"))
    import decode_usd_log  # noqa: E402
    d = decode_usd_log.load(str(path))
    n_raw = len(d.get("t", []))

    def vec3(prefix):
        # 2026-09-15: same fix as load_usd_csv's vec3 -- prefix="" must give "x" not "_x", or
        # `pos` below silently comes back None despite x/y/z both being present in `d`. The
        # fallback branch that used to exist here (a second manual np.stack) was only ever
        # papering over this; removed now that the real function is correct.
        keys = [f"{prefix}_{ax}" if prefix else ax for ax in "xyz"]
        if not all(k in d for k in keys):
            return None
        return np.stack([d[k] for k in keys], axis=1)

    pos = vec3("")
    pos_des = vec3("ctrltarget")
    # NOTE: ctrltarget is the literal firmware setpoint, which for a height-compensated
    # drone (Z_OFFSET_COMPENSATION) INCLUDES that compensation -- pos_rmse against it is "did
    # you track what was commanded", not "did you achieve the experiment's intended geometry".
    # For the latter use the formation row's dz_mean/sag (measured relative state), or
    # reconstruct from scenarios.build() (which is deliberately uncompensated, see
    # find_flight_window.commanded_trajectory).
    a_res = vec3("a_res")
    a_hat = vec3("rnn_pred")
    e_r = vec3("e_r")
    return VehicleLog(name or path.stem, "usd", np.asarray(d.get("t", [])), pos,
                       pos_des, a_res, a_hat, e_r, n_raw, str(path))


def load_any(paths: list[str]) -> dict[str, VehicleLog]:
    """Accepts either one merged CSV (all vehicles inside) or N per-vehicle files."""
    paths = [Path(p) for p in paths]
    if len(paths) == 1 and detect_format(paths[0]) == "merged":
        return load_merged_csv(paths[0])
    out = {}
    for p in paths:
        fmt = detect_format(p)
        if fmt == "ros":
            v = load_ros_csv(p)
        elif fmt == "sim":
            v = load_sim_states_csv(p)
        elif fmt == "usd":
            v = load_usd(p)
        elif fmt == "usd_csv":
            v = load_usd_csv(p)
        else:
            raise SystemExit(f"{p}: merged format only supported alone, not mixed "
                              f"with other files -- pass it by itself")
        out[v.name] = v
    return out


def commanded_from_scenario(sc, robot_index: int, anchor, t0: float, timescale: float,
                             t: np.ndarray) -> np.ndarray:
    """Absolute commanded position for one robot, reconstructed from the scenario spec.

    Mirrors experiments/analysis/verify_formation_sim.py's own reconstruction (same
    anchor + slot + curve(t) rule) rather than inventing a second way to do this.
    """
    robot = sc.robots[robot_index]
    out = np.full((len(t), 3), np.nan)
    for i, ti in enumerate(t):
        tau = (ti - t0) / timescale
        if tau < 0 or tau > sc.duration:
            continue
        out[i] = anchor + robot.slot + np.asarray(robot.curve(tau))[:3]
    return out


def _rmse(err: np.ndarray) -> float:
    valid = ~np.any(np.isnan(err), axis=1)
    if not np.any(valid):
        return float("nan")
    return float(np.sqrt(np.mean(np.sum(err[valid] ** 2, axis=1))))


def _axis_rmse(err: np.ndarray, axis: int) -> float:
    col = err[:, axis]
    valid = ~np.isnan(col)
    if not np.any(valid):
        return float("nan")
    return float(np.sqrt(np.mean(col[valid] ** 2)))


def vehicle_metrics(v: VehicleLog, scenario: str, controller: str, n_robots: int,
                     pos_des: np.ndarray | None) -> dict:
    """One row of Chapter-5 metrics for a single vehicle.

    `pos_des`, if given, overrides v.pos_des (used when it had to be reconstructed
    from the scenario spec rather than read from the log).
    """
    des = pos_des if pos_des is not None else v.pos_des
    n_raw = v.n_raw
    n_used = 0 if v.pos is None else v.pos.shape[0]
    row = dict(
        scenario=scenario, controller=controller, n_robots=n_robots, vehicle_id=v.name,
        t_start=float(v.t[0]) if len(v.t) else float("nan"),
        t_end=float(v.t[-1]) if len(v.t) else float("nan"),
        n_samples=n_used, n_raw=n_raw,
        rate_hz_est=(float((n_used - 1) / (v.t[-1] - v.t[0]))
                     if n_used > 1 and v.t[-1] > v.t[0] else float("nan")),
    )
    if n_used == 0:
        row.update(nan_fraction=1.0, pos_rmse_m=float("nan"), pos_rmse_x=float("nan"),
                    pos_rmse_y=float("nan"), pos_rmse_z=float("nan"),
                    pos_peak_m=float("nan"), pos_peak_z=float("nan"),
                    a_res_rms=float("nan"), a_res_z_mean=float("nan"), a_res_z_rms=float("nan"),
                    a_hat_res_rms=float("nan"), a_hat_vs_a_res_rmse=float("nan"),
                    e_R_rmse=float("nan"), e_R_peak=float("nan"),
                    notes=f"NO DATA -- {n_raw} raw rows in source ({v.fmt} format)")
        return row

    if des is None:
        raise SystemExit(
            f"{v.name}: no commanded trajectory available (no setpoint in the log and "
            f"no scenario spec / sidecar given to reconstruct one) -- refusing to RMSE "
            f"against nothing. Pass --sidecar or check the log actually has a setpoint.")

    err = v.pos - des
    nan_frac = float(np.mean(np.any(np.isnan(err), axis=1)))
    row["nan_fraction"] = nan_frac
    row["pos_rmse_m"] = _rmse(err)
    row["pos_rmse_x"] = _axis_rmse(err, 0)
    row["pos_rmse_y"] = _axis_rmse(err, 1)
    row["pos_rmse_z"] = _axis_rmse(err, 2)
    dist = np.linalg.norm(err, axis=1)
    valid = ~np.isnan(dist)
    row["pos_peak_m"] = float(np.nanmax(dist)) if np.any(valid) else float("nan")
    row["pos_peak_z"] = (float(np.nanmax(np.abs(err[:, 2]))) if np.any(valid)
                          else float("nan"))

    if v.a_res is not None:
        mag = np.linalg.norm(v.a_res, axis=1)
        row["a_res_rms"] = float(np.sqrt(np.mean(mag ** 2)))
        row["a_res_z_mean"] = float(np.mean(v.a_res[:, 2]))
        row["a_res_z_rms"] = float(np.sqrt(np.mean(v.a_res[:, 2] ** 2)))
    else:
        row["a_res_rms"] = row["a_res_z_mean"] = row["a_res_z_rms"] = float("nan")

    if v.a_hat is not None:
        row["a_hat_res_rms"] = float(np.sqrt(np.mean(np.sum(v.a_hat ** 2, axis=1))))
        row["a_hat_vs_a_res_rmse"] = (_rmse(v.a_hat - v.a_res)
                                       if v.a_res is not None else float("nan"))
    else:
        row["a_hat_res_rms"] = row["a_hat_vs_a_res_rmse"] = float("nan")

    if v.e_r is not None:
        e_norm = np.linalg.norm(v.e_r, axis=1)
        row["e_R_rmse"] = float(np.sqrt(np.mean(e_norm ** 2)))
        row["e_R_peak"] = float(np.max(e_norm))
    else:
        row["e_R_rmse"] = row["e_R_peak"] = float("nan")

    row["notes"] = ""
    return row


def formation_row(scenario: str, controller: str, vehicles: list[VehicleLog],
                   dz_cmd: float | None) -> dict:
    """One aggregate row across all vehicle pairs.

    dz = z_upper - z_lower for a stacked pair. Positive sag = lower vehicle too low
    (the gap closed); this is a SIGN CONVENTION, documented here and in the README,
    not an assumption the reader has to infer from the numbers.
    """
    n = len(vehicles)
    row = dict(scenario=scenario, controller=controller, n_robots=n, vehicle_id="__formation__",
               dz_cmd_m=dz_cmd if dz_cmd is not None else float("nan"))
    usable = [v for v in vehicles if v.pos is not None and v.pos.shape[0] > 0]
    if len(usable) < 2 or dz_cmd is None:
        row.update(dz_mean_m=float("nan"), dz_err_mean_m=float("nan"),
                    dz_err_rms_m=float("nan"),
                    notes="not stacked / dz_cmd not given / fewer than 2 vehicles with data")
        return row
    a, b = usable[0], usable[1]
    t_lo = max(a.t[0], b.t[0])
    t_hi = min(a.t[-1], b.t[-1])
    if t_hi <= t_lo:
        row.update(dz_mean_m=float("nan"), dz_err_mean_m=float("nan"),
                    dz_err_rms_m=float("nan"), notes="no overlapping time window")
        return row
    # Interpolate onto whichever vehicle sampled slower -- coarser grid, no invented
    # oversampling of the finer one, matches the spirit of the merge_usd_logs.py sync.
    rate_a = (len(a.t) - 1) / (a.t[-1] - a.t[0]) if len(a.t) > 1 else 0
    rate_b = (len(b.t) - 1) / (b.t[-1] - b.t[0]) if len(b.t) > 1 else 0
    grid = a.t[(a.t >= t_lo) & (a.t <= t_hi)] if rate_a <= rate_b else \
        b.t[(b.t >= t_lo) & (b.t <= t_hi)]
    offset = float(a.t[0] - b.t[0])
    z_a = np.interp(grid, a.t, a.pos[:, 2])
    z_b = np.interp(grid, b.t, b.pos[:, 2])
    dz = z_a - z_b if a.pos[0, 2] >= b.pos[0, 2] else z_b - z_a
    sag = dz_cmd - dz  # positive = gap closed (lower vehicle too high / upper too low)
    row.update(dz_mean_m=float(np.mean(dz)),
                dz_err_mean_m=float(np.mean(sag)),
                dz_err_rms_m=float(np.sqrt(np.mean(sag ** 2))),
                notes=f"time offset between the two logs' own t[0]: {offset:+.4f} s")
    return row
