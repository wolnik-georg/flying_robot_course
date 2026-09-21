#!/usr/bin/env python3
"""Compute one row of docs/24_Downwash_Compensation_Comparison.md from a formation flight.

    # one run
    python3 compare_downwash.py experiments/logs/A8_2026-09-18_18-39-08.meta.json

    # several runs side by side (the usual case: geometric vs INDI vs stock INDI)
    python3 compare_downwash.py experiments/logs/A8_2026-09-18_18-31-15.meta.json \
                                experiments/logs/A8_2026-09-18_18-39-08.meta.json

    # emit a markdown table row ready to paste into docs/24
    python3 compare_downwash.py --markdown <meta.json> ...

Errors are computed against the RECONSTRUCTED COMMANDED TRAJECTORY, not against a
neighbouring drone or a nominal hover point: the scenario is rebuilt from
`crazyflie_examples.formations.scenarios` using the flight's own recorded params, each
robot's curve is evaluated and offset by `anchor + slot` from the run's own meta sidecar,
and the log is time-aligned to it by minimising RMS over the scenario window. That is the
only way rows stay comparable as controllers change.

⚠️ Z_OFFSET_COMPENSATION is deliberately NOT applied. It was removed 2026-09-18
(`crazyswarm2` d824fba). Runs BEFORE that carry a +0.40 m over-command on `cf_second` only,
which shows up as a large, spurious z error for that vehicle -- it is a config artefact, not
a controller effect, and `--z-comp` exists to model it when analysing those older runs.
Legacy runs on `cf231_active` never carried that compensation; the study drone is now **cf5**
(`--drone cf5`, default).

Written 2026-09-18, replacing the ad-hoc heredoc that produced the first two rows of docs/24,
so every later row is computed the same way rather than re-derived by hand.

**uSD path (2026-09-21):** pass `--merged-usd <merge.csv>` with the same `.meta.json`. Tracking
error uses **`ctrltarget.*`** from the merge when present (policy in docs/27); metrics are computed
over the scenario duration window with `t=0` at scenario start (`# meta:t_zero=scenario_start`).
Radio CSV + trajectory reconstruction remains for legacy runs.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

# The scenario definitions are ROS-free by design (see formations/scenarios.py), so this
# imports cleanly without a sourced ROS environment.
_CS2 = Path.home() / "Desktop" / "crazyswarm2" / "crazyflie_examples"
_REPO = Path(__file__).resolve().parents[2]  # flying_robot_course


def _load_scenarios():
    if str(_CS2) not in sys.path:
        sys.path.insert(0, str(_CS2))
    try:
        from crazyflie_examples.formations import scenarios  # noqa: E402
    except ImportError as exc:
        sys.exit(f"[compare] cannot import formation scenarios from {_CS2}: {exc}\n"
                 f"[compare] pass --cs2 <path to crazyflie_examples> if the repo lives elsewhere")
    return scenarios


def load_log(path: Path) -> list[dict]:
    """Rows of a run_formation.py per-drone CSV, skipping its '# meta:' preamble."""
    lines = path.read_text().splitlines(keepends=True)
    hdr = next(i for i, l in enumerate(lines) if l.startswith("time_s"))
    out = []
    for r in csv.DictReader(lines[hdr:]):
        try:
            out.append({k: float(v) for k, v in r.items() if k and v not in (None, "")})
        except (TypeError, ValueError):
            pass  # a torn final row, or a stray non-numeric field
    return out


def analyse_run(meta_path: Path, z_comp: dict[str, float], search: tuple[float, float, float]):
    scenarios = _load_scenarios()
    meta = json.loads(meta_path.read_text())
    sid = meta["scenario"]
    sc = scenarios.BUILDERS[sid](**meta["params"])
    anchor = np.array(meta["anchor"], dtype=float)
    # "A8_2026-09-18_18-39-08.meta.json" -> "2026-09-18_18-39-08"; .stem alone leaves ".meta"
    stamp = meta_path.name[:-len(".meta.json")].replace(f"{sid}_", "", 1)

    results = {}
    for idx, name in enumerate(meta["names"]):
        log = meta_path.parent / f"{sid}_{name}_{stamp}.csv"
        if not log.exists():
            print(f"[compare] WARN: {log.name} missing, skipping {name}")
            continue
        rows = load_log(log)
        slot = anchor + np.array(sc.robots[idx].slot, dtype=float)
        slot = slot + np.array([0.0, 0.0, z_comp.get(name, 0.0)])
        curve = sc.robots[idx].curve

        def errs_at(t0: float):
            e = []
            for r in rows:
                tt = r["time_s"] - t0
                if 0.0 <= tt <= sc.duration:
                    c = curve.at(tt)
                    cmd = slot + np.array([c[0], c[1], c[2]])
                    e.append((r, np.array([r["pos_x"], r["pos_y"], r["pos_z"]]) - cmd))
            return e

        # The log's t=0 is the logger's own start, not the trajectory's. Find the offset
        # that best explains the data rather than trusting a nominal value.
        lo, hi, step = search
        best_t0, best_rms = None, np.inf
        for t0 in np.arange(lo, hi, step):
            e = errs_at(t0)
            if len(e) < 200:
                continue
            rms = float(np.sqrt(np.mean([np.dot(d, d) for _, d in e])))
            if rms < best_rms:
                best_t0, best_rms = float(t0), rms
        if best_t0 is None:
            print(f"[compare] WARN: no usable alignment window for {name}, skipping")
            continue

        e = errs_at(best_t0)
        d = np.array([x[1] for x in e])
        g = lambda k: np.array([r.get(k, 0.0) for r, _ in e])  # noqa: E731
        rms = lambda v: float(np.sqrt(np.mean(np.square(v))))   # noqa: E731
        pk = lambda v: float(np.max(np.abs(v)))                 # noqa: E731

        results[name] = dict(
            t0=best_t0, n=len(e),
            ex_rms=rms(d[:, 0]), ey_rms=rms(d[:, 1]), ez_rms=rms(d[:, 2]),
            e3_rms=rms(np.linalg.norm(d, axis=1)), e3_max=float(np.max(np.linalg.norm(d, axis=1))),
            ex_pk=pk(d[:, 0]), ey_pk=pk(d[:, 1]), ez_pk=pk(d[:, 2]),
            roll_std=float(np.std(g("roll"))), roll_pk=pk(g("roll")),
            pitch_std=float(np.std(g("pitch"))), pitch_pk=pk(g("pitch")),
            yaw_std=float(np.std(g("yaw"))), yaw_pk=pk(g("yaw")),
            gx_std=float(np.std(g("gyro_x"))), gy_std=float(np.std(g("gyro_y"))),
            gz_std=float(np.std(g("gyro_z"))),
            arz_min=float(np.min(g("a_res_z"))),
        )

    cfg = meta.get("per_drone", {})
    return dict(sid=sid, stamp=stamp, params=meta["params"], per_drone=cfg, results=results)


def _load_merged_table(path: Path) -> tuple[list[str], np.ndarray, dict[str, int]]:
    lines = path.read_text().splitlines()
    hdr_i = next(i for i, l in enumerate(lines) if l.strip() and not l.startswith("#"))
    header = lines[hdr_i].split(",")
    cols = {c: i for i, c in enumerate(header)}
    skip = hdr_i + 1
    data = np.loadtxt(path, delimiter=",", skiprows=skip, ndmin=2)
    return header, data, cols


def analyse_run_merged(meta_path: Path, merged_path: Path):
    """docs/24-style metrics from a --meta-aligned merged uSD CSV."""
    metrics_dir = _REPO / "experiments" / "analysis"
    if str(metrics_dir) not in sys.path:
        sys.path.insert(0, str(metrics_dir))
    import metrics as M  # noqa: E402

    meta = json.loads(meta_path.read_text())
    sid = meta["scenario"]
    stamp = meta_path.name[:-len(".meta.json")].replace(f"{sid}_", "", 1)
    raw = M.load_merged_csv(merged_path)
    vehicles = {}
    for logical in meta["names"]:
        if logical in raw:
            vehicles[logical] = raw[logical]
            continue
        for k, v in raw.items():
            if k.startswith(f"{logical}_"):
                vehicles[logical] = v
                break
    scenarios = _load_scenarios()
    sc = scenarios.BUILDERS[sid](**meta["params"])
    anchor = np.array(meta["anchor"], dtype=float)
    t0 = 0.0 if any(v.t_zero == "scenario_start" for v in vehicles.values()) else float(
        meta.get("t_start_sim", 0.0))
    timescale = float(meta.get("timescale", 1.0))

    _, data, cols = _load_merged_table(merged_path)

    def series(name: str, field: str) -> np.ndarray | None:
        key = f"{name}.{field}"
        return data[:, cols[key]] if key in cols else None

    results = {}
    for idx, name in enumerate(meta["names"]):
        if name not in vehicles:
            print(f"[compare] WARN: {name} not in merge, skipping")
            continue
        v = vehicles[name]
        pos_des = v.pos_des
        if pos_des is None:
            pos_des = M.commanded_from_scenario(sc, idx, anchor, t0, timescale, v.t)
        mask = (v.t >= 0.0) & (v.t <= sc.duration)
        if mask.sum() < 200:
            print(f"[compare] WARN: too few samples in scenario window for {name}, skipping")
            continue
        pos = v.pos[mask]
        des = pos_des[mask]
        d = pos - des
        rms = lambda v_: float(np.sqrt(np.mean(np.square(v_))))  # noqa: E731
        pk = lambda v_: float(np.max(np.abs(v_)))                 # noqa: E731

        roll = series(name, "roll_deg")
        pitch = series(name, "pitch_deg")
        yaw = series(name, "yaw_deg")
        gx = series(name, "gyro_x")
        gy = series(name, "gyro_y")
        gz = series(name, "gyro_z")
        arz = series(name, "a_res_z")

        def stat(arr, m):
            if arr is None:
                return float("nan")
            x = arr[mask]
            return m(x)

        results[name] = dict(
            t0=0.0, n=int(mask.sum()),
            ex_rms=rms(d[:, 0]), ey_rms=rms(d[:, 1]), ez_rms=rms(d[:, 2]),
            e3_rms=rms(np.linalg.norm(d, axis=1)),
            e3_max=float(np.max(np.linalg.norm(d, axis=1))),
            ex_pk=pk(d[:, 0]), ey_pk=pk(d[:, 1]), ez_pk=pk(d[:, 2]),
            roll_std=stat(roll, np.std), roll_pk=stat(roll, lambda x: pk(x)),
            pitch_std=stat(pitch, np.std), pitch_pk=stat(pitch, lambda x: pk(x)),
            yaw_std=stat(yaw, np.std), yaw_pk=stat(yaw, lambda x: pk(x)),
            gx_std=stat(gx, np.std), gy_std=stat(gy, np.std), gz_std=stat(gz, np.std),
            arz_min=stat(arz, np.min),
        )

    cfg = meta.get("per_drone", {})
    return dict(sid=sid, stamp=stamp, params=meta["params"], per_drone=cfg, results=results,
                source="merged_usd", merged=str(merged_path))


def fmt_human(run):
    print("=" * 78)
    print(f"{run['sid']}  {run['stamp']}   params={run['params']}")
    for name, v in run["results"].items():
        c = run["per_drone"].get(name, {})
        tag = f"controller={c.get('controller')} ctrl_mode={c.get('ctrl_mode')}" if c else ""
        print(f"\n  {name}   {tag}")
        print(f"    aligned t0={v['t0']:.2f}s, n={v['n']}")
        print(f"    pos RMS   x {v['ex_rms']*1e3:6.1f}   y {v['ey_rms']*1e3:6.1f}   "
              f"z {v['ez_rms']*1e3:6.1f}   |e| {v['e3_rms']*1e3:6.1f} mm   max {v['e3_max']*1e3:6.1f} mm")
        print(f"    pos peak  x {v['ex_pk']*1e3:6.1f}   y {v['ey_pk']*1e3:6.1f}   z {v['ez_pk']*1e3:6.1f} mm")
        print(f"    att std   roll {v['roll_std']:5.2f}  pitch {v['pitch_std']:5.2f}  yaw {v['yaw_std']:5.2f} deg")
        print(f"    att peak  roll {v['roll_pk']:6.2f}  pitch {v['pitch_pk']:6.2f}  yaw {v['yaw_pk']:6.2f} deg")
        print(f"    gyro std  x {v['gx_std']:6.2f}  y {v['gy_std']:6.2f}  z {v['gz_std']:6.2f}")
        print(f"    a_res_z peak {v['arz_min']:.2f} m/s^2")


def fmt_markdown(runs, drone):
    """A docs/24 table row per run, for the named drone (the vehicle under study)."""
    print(f"\n<!-- docs/24 rows for {drone} -->")
    print("| Strategy / controller | pos RMS x | pos RMS y | pos RMS z | **pos RMS ‖e‖** | "
          "max ‖e‖ | roll std | pitch std | yaw std | gyro std x | `a_res_z` peak | run |")
    for run in runs:
        v = run["results"].get(drone)
        if not v:
            continue
        c = run["per_drone"].get(drone, {})
        label = f"`c={c.get('controller')} m={c.get('ctrl_mode')}`"
        print(f"| **FILL IN** {label} | {v['ex_rms']*1e3:.1f} mm | {v['ey_rms']*1e3:.1f} mm | "
              f"{v['ez_rms']*1e3:.1f} mm | **{v['e3_rms']*1e3:.1f} mm** | {v['e3_max']*1e3:.1f} mm | "
              f"{v['roll_std']:.2f}° | {v['pitch_std']:.2f}° | {v['yaw_std']:.2f}° | "
              f"{v['gx_std']:.2f} | {v['arz_min']:.2f} m/s² | {run['stamp']} |")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("meta", nargs="+", type=Path,
                    help="one or more <scenario>_<stamp>.meta.json sidecars")
    ap.add_argument("--merged-usd", type=Path, default=None,
                    help="single merged uSD CSV (same stamp as each meta); if set, all meta "
                         "files must share this merge or pass one meta only")
    ap.add_argument("--drone", default="cf5",
                    help="vehicle under study, used for --markdown (default cf5)")
    ap.add_argument("--markdown", action="store_true", help="also emit docs/24 table rows")
    ap.add_argument("--z-comp", default="",
                    help="pre-2026-09-18 runs only: model the removed height compensation, "
                         "e.g. 'cf_second=0.40'. Off by default -- see the module docstring.")
    ap.add_argument("--window", default="8,16,0.02",
                    help="trajectory-start search as lo,hi,step seconds (default 8,16,0.02)")
    ap.add_argument("--cs2", type=Path, default=None,
                    help="path to crazyflie_examples if not ~/Desktop/crazyswarm2/crazyflie_examples")
    args = ap.parse_args()

    if args.cs2:
        globals()["_CS2"] = args.cs2
    z_comp = {}
    for part in filter(None, args.z_comp.split(",")):
        k, _, v = part.partition("=")
        z_comp[k.strip()] = float(v)
    lo, hi, step = (float(x) for x in args.window.split(","))

    if args.merged_usd:
        if len(args.meta) != 1:
            ap.error("--merged-usd expects exactly one .meta.json (one flight per merge)")
        runs = [analyse_run_merged(args.meta[0], args.merged_usd)]
    else:
        runs = [analyse_run(m, z_comp, (lo, hi, step)) for m in args.meta]
    for run in runs:
        fmt_human(run)
    if args.markdown:
        fmt_markdown(runs, args.drone)


if __name__ == "__main__":
    main()
