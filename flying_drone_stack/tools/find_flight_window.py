#!/usr/bin/env python3
"""Find the real flight inside a uSD log by correlating against the commanded trajectory.

Why this exists
----------------
2026-09-15: `usddeck.c` does not start a new file per logging session -- toggling
`usd.logging` (including via the old check_usd_deck.py, see its docstring) appends into
whatever file is already open, and a card can carry a much longer recording than the actual
flight. The real ~18s flight had to be found inside a ~200s file by hand, by cross-correlating
measured position against the scenario's own commanded trajectory (recomputed from source,
not approximated) and searching over the WHOLE recording for the best-fit time lag.

That method worked and was unambiguous (best-fit lag beat the next-best candidate by 80-270x),
but doing it by hand in a one-off script took real effort. This packages it so any future uSD
log takes one command.

2026-09-15 usec.reset was ALSO added to run_formation.py (it never had it before -- that is
almost certainly why the file needed a search at all: without a shared broadcast reset right
before logging starts, there is no reason the drone's uSD clock lines up with anything). Flights
recorded after that fix should show the trajectory starting very close to the uSD file's own
t=0 for that session -- if this script finds a lag far from 0 on a NEW flight, suspect the
usec.reset broadcast failed (check the terminal output for its WARN message) rather than
assuming another long contaminated recording.

Usage
-----
    python3 find_flight_window.py <usd_log.bin> <role: bottom|top> <scenario.meta.json>

`role` must match one of the roles in the scenario's own definition (`formations/scenarios.py`)
-- for A8 that's "bottom" or "top". Prints the found window and, with --extract, writes a
trimmed CSV of just the flight (uSD sample rate, not resampled) for downstream analysis.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path("/home/georg/Desktop/crazyswarm2/crazyflie_examples/crazyflie_examples")))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from decode_usd_log import load  # noqa: E402
from formations import curves as C  # noqa: E402


def commanded_trajectory(meta: dict, role: str):
    """Recompute the exact commanded position(t) for one role of an A8 scenario.

    Only A8 is implemented (the Shuttle/Pause combination). Extending to other scenarios means
    reading the actual RobotPlan for that scenario from formations/scenarios.py -- do not guess
    a curve shape for a scenario this hasn't been verified against.
    """
    if meta["scenario"] != "A8":
        raise NotImplementedError(
            f"find_flight_window.py only knows A8's curve shape (Shuttle+Pause); "
            f"got scenario={meta['scenario']!r}. Read formations/scenarios.py's definition for "
            f"this scenario and add it here before trusting a result -- do not assume A8's "
            f"shape applies."
        )
    p = meta["params"]
    anchor = np.array(meta["anchor"])
    half = p["span"] / 2.0
    settle, duration, passes = p["settle"], p["duration"], p["passes"]
    total = meta["duration"]

    if role == "bottom":
        curve = C.Then(C.Pause(settle), C.Shuttle([0.0, p["span"], 0.0], duration, passes=passes))
        slot = anchor + np.array([0.0, -half, 0.0])
    elif role == "top":
        curve = C.Then(C.Pause(settle), C.Shuttle([0.0, -p["span"], 0.0], duration, passes=passes))
        slot = anchor + np.array([0.0, +half, p["dz"]])
    else:
        raise ValueError(f"role must be 'bottom' or 'top' for A8, got {role!r}")

    ts = np.linspace(0, total, 4000)
    cmd = np.array([slot + curve.at(t)[:3] for t in ts])
    return ts, cmd


def find_offset(t_meas, y_meas, t_cmd, y_cmd, search_lo, search_hi, dt=0.02):
    """Slide the commanded curve across the WHOLE recording; return the best-fit lag and MSE,
    plus the runner-up so the caller can judge how unambiguous the match is."""
    lags = np.arange(search_lo, search_hi, dt)
    errs = np.full(len(lags), np.nan)
    for i, lag in enumerate(lags):
        tt = t_meas - lag
        mask = (tt >= 0) & (tt <= t_cmd[-1])
        if mask.sum() < 100:
            continue
        cmd_interp = np.interp(tt[mask], t_cmd, y_cmd)
        errs[i] = np.mean((y_meas[mask] - cmd_interp) ** 2)
    order = np.argsort(errs)
    best = lags[order[0]]
    # runner-up: the next local minimum at least 3s away, so we're not just reporting two
    # samples either side of the same peak
    runner_up = None
    for idx in order[1:]:
        if abs(lags[idx] - best) > 3.0:
            runner_up = (lags[idx], errs[idx])
            break
    return best, errs[order[0]], runner_up


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("usd_log")
    ap.add_argument("role", choices=["bottom", "top"])
    ap.add_argument("meta_json")
    ap.add_argument("--extract", metavar="OUT_CSV", default=None,
                    help="write the trimmed flight window (uSD rate) to this CSV")
    ap.add_argument("--margin", type=float, default=3.0,
                    help="seconds of padding kept either side of the scenario window (default 3)")
    args = ap.parse_args()

    meta = json.load(open(args.meta_json))
    ts, cmd = commanded_trajectory(meta, args.role)
    total = meta["duration"]

    d = load(args.usd_log)
    t = np.array(d["t"])
    print(f"loaded {len(t)} samples, span [{t[0]:.2f}, {t[-1]:.2f}]s "
          f"({t[-1]-t[0]:.1f}s total -- scenario itself is only {total:.1f}s, so a large gap "
          f"between these two numbers is normal and expected, not a problem)")

    y = np.array(d["y"])
    lag, mse, runner_up = find_offset(t, y, ts, cmd[:, 1], 0.0, t[-1] - total - 1.0)

    print(f"\nbest-fit scenario start: uSD t={lag:.2f}s   mse={mse:.5f}")
    if runner_up is None:
        print("  no other candidate found more than 3s away -- unambiguous.")
    else:
        ratio = runner_up[1] / mse if mse > 0 else float("inf")
        note = "STRONG match" if ratio > 10 else "WEAK match -- verify by eye before trusting this"
        print(f"  next-best candidate: lag={runner_up[0]:.2f}s mse={runner_up[1]:.5f} "
              f"({ratio:.1f}x worse) -- {note}")
    if lag < 2.0:
        print("  lag is near zero: consistent with a flight recorded AFTER the 2026-09-15 "
              "usec.reset fix landed in run_formation.py. If this is an older flight, that's "
              "a coincidence, not the fix -- check which script/commit flew it.")

    lo, hi = lag - args.margin, lag + total + args.margin
    mask = (t >= lo) & (t <= hi)
    print(f"\nflight window: uSD t=[{t[mask][0]:.2f}, {t[mask][-1]:.2f}]s, {mask.sum()} samples")

    if args.extract:
        keys = [k for k in d if k != "t"]
        header = "t," + ",".join(keys)
        rows = []
        tt = t[mask] - lag
        for i, idx in enumerate(np.where(mask)[0]):
            rows.append(f"{tt[i]:.4f}," + ",".join(f"{d[k][idx]:.6f}" for k in keys))
        Path(args.extract).write_text(header + "\n" + "\n".join(rows) + "\n")
        print(f"wrote {args.extract} ({mask.sum()} rows, t=0 at scenario start)")


if __name__ == "__main__":
    main()
