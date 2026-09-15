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
    python3 find_flight_window.py <usd_log.bin> <role: bottom|top|...> <scenario.meta.json>

The scenario is read from the meta.json's own `scenario`/`params` fields -- whichever of the
16 formation-library scenarios (A1-A8, B1-B3, C1-C5) was actually flown is detected and rebuilt
automatically; `role` just says which of that scenario's robots this particular log is. Prints
the found window and, with --extract, writes a trimmed CSV of just the flight (uSD sample rate,
not resampled) for downstream analysis.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
# The OUTER directory, so `run_formation.py`'s own `from .formations import ...` resolves as
# part of the `crazyflie_examples` package -- importing the inner directory directly (as an
# earlier version of this file did) makes `run_formation` load as a top-level module, and its
# relative import then fails immediately.
sys.path.insert(0, str(Path("/home/georg/Desktop/crazyswarm2/crazyflie_examples")))

from decode_usd_log import load  # noqa: E402
from crazyflie_examples.formations import scenarios  # noqa: E402


def commanded_trajectory(meta: dict, role: str):
    """Recompute the exact commanded position(t) for one robot of WHATEVER scenario `meta`
    says was flown, by rebuilding the real `Scenario` from `formations/scenarios.py` --
    the same object `run_formation.py` compiled into the trajectory that actually flew --
    rather than re-deriving each scenario's curve shape by hand here.

    2026-09-15: this used to hardcode A8's own Shuttle/Pause construction and refuse every
    other scenario outright. `Scenario.params` is recorded specifically so a flight can be
    rebuilt exactly from its own sidecar (`scenarios.build`'s docstring: "a scenario has to
    be exactly reconstructable from what was recorded") -- using that directly means every
    scenario in the library is supported by construction, with no per-scenario curve code to
    keep in sync, and a scenario added to the library later needs nothing here at all.
    """
    sc = scenarios.build(meta["scenario"], **meta["params"])
    try:
        idx = next(i for i, r in enumerate(sc.robots) if r.role == role)
    except StopIteration:
        roles = [r.role for r in sc.robots]
        raise ValueError(f"scenario {meta['scenario']!r} has roles {roles}, got role={role!r}")
    robot = sc.robots[idx]

    anchor = np.array(meta["anchor"])
    slot = anchor + robot.slot
    # Deliberately NOT applying Z_OFFSET_COMPENSATION here, even though it IS what
    # run_formation.py adds to the command it sends to a compensated drone. That addition
    # exists to cancel a hardware shortfall (cf_second achieves ~0.40m less than commanded),
    # so its entire purpose is to make the ACHIEVED position equal the UNCOMPENSATED target --
    # this is confirmed by the compensated drone's real measured z sitting close to the
    # uncompensated slot, not the compensated one. Verification asks "did the drone reach what
    # was intended", so it must compare against that same uncompensated target. Adding the
    # compensation here as well double-counts it: an early version of this fix did exactly
    # that and turned a 1.8 cm RMS match into a 47 cm "failure" on cf_second/top.
    total = sc.duration
    ts = np.linspace(0, total, 4000)
    cmd = np.array([slot + np.asarray(robot.curve(t))[:3] for t in ts])
    return ts, cmd


def find_offset(t_meas, pos_meas, t_cmd, pos_cmd, search_lo, search_hi, dt=0.02,
                min_cover=0.8):
    """Slide the commanded curve across the WHOLE recording; return the best-fit lag and MSE,
    plus the runner-up so the caller can judge how unambiguous the match is.

    `pos_meas`/`pos_cmd` are (N, 3) position arrays, matched on full 3D Euclidean distance.
    2026-09-15: this used to take a single named channel (`y`), which was fine for A8 (motion
    is along y only, z constant) but wrong in general -- a circle moves in x AND y, A7 sweeps
    z continuously, and picking one axis per scenario is exactly the per-scenario special
    casing this tool is meant not to need. 3D distance works for every scenario's geometry
    without knowing which axis carries the motion.

    `min_cover` is the fraction of the scenario's duration that must actually be covered by
    measured samples for a lag to be scored at all. 2026-09-15: this guard used to be a flat
    `mask.sum() < 100`, which at 500 Hz is 0.2 s -- so a lag that overlapped only the last
    fraction of a second of a recording was scored on ~100 samples and could win outright
    with a near-zero MSE. That produced a confident "STRONG match" on a file that was not
    the flight at all. Requiring real coverage is what makes the result trustworthy.
    """
    lags = np.arange(search_lo, search_hi, dt)
    errs = np.full(len(lags), np.nan)
    if len(lags) == 0:
        return None, None, None
    # samples needed to call the scenario actually covered, from this log's own sample rate
    span = t_meas[-1] - t_meas[0]
    rate = len(t_meas) / span if span > 0 else 0.0
    need = int(min_cover * t_cmd[-1] * rate)
    for i, lag in enumerate(lags):
        tt = t_meas - lag
        mask = (tt >= 0) & (tt <= t_cmd[-1])
        if mask.sum() < need:
            continue
        cmd_interp = np.stack(
            [np.interp(tt[mask], t_cmd, pos_cmd[:, ax]) for ax in range(3)], axis=1)
        errs[i] = np.mean(np.sum((pos_meas[mask] - cmd_interp) ** 2, axis=1))
    if np.all(np.isnan(errs)):
        return None, None, None
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
    ap.add_argument("role", help="the role in THIS scenario the log belongs to, e.g. bottom/"
                                  "top/leader/follower/center/solo -- see the scenario's own "
                                  "definition in formations/scenarios.py")
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

    if t[-1] - t[0] < total:
        sys.exit(f"\n[find_flight_window] this recording is only {t[-1]-t[0]:.1f}s long but the "
                 f"scenario runs {total:.1f}s -- it cannot contain the whole flight, so there "
                 f"is nothing to locate. Either this is the wrong file for this scenario, or "
                 f"logging stopped early (a crash landing cuts usd.logging off mid-flight).")

    pos = np.stack([np.array(d["x"]), np.array(d["y"]), np.array(d["z"])], axis=1)
    # Search lags across the recording's OWN time axis. 2026-09-15: this used to start the
    # search at 0.0 regardless of t[0], which is wrong for any log whose clock does not start
    # near zero (i.e. every pre-usec.reset-fix recording) -- the real lag sat outside the
    # searched range entirely and the best "match" was whatever degenerate tail-overlap scored
    # lowest.
    lag, mse, runner_up = find_offset(t, pos, ts, cmd, t[0], t[-1] - total)
    if lag is None:
        sys.exit(f"\n[find_flight_window] no lag in [{t[0]:.1f}, {t[-1]-total:.1f}]s covers "
                 f"enough of the {total:.1f}s scenario to score. This recording does not "
                 f"contain a full run of this scenario.")

    print(f"\nbest-fit scenario start: uSD t={lag:.2f}s   mse={mse:.5f}  "
          f"(RMS {np.sqrt(mse)*100:.1f} cm)")
    if runner_up is None:
        print("  no other candidate found more than 3s away -- unambiguous.")
    else:
        ratio = runner_up[1] / mse if mse > 0 else float("inf")
        note = "STRONG match" if ratio > 10 else "WEAK match -- verify by eye before trusting this"
        print(f"  next-best candidate: lag={runner_up[0]:.2f}s mse={runner_up[1]:.5f} "
              f"({ratio:.1f}x worse) -- {note}")
    # NOTE (2026-09-15, second revision): do NOT expect lag ~= 0. usec.reset is broadcast
    # on the GROUND, before takeoff -- it cannot be sent mid-flight, because the high-level
    # commander shares that clock and zeroing it in the air crashes the vehicle (see the
    # call site in run_formation.py). So the clock reads roughly takeoff+climb+converge+
    # upload time (~10-12s) by the time the scenario actually starts. A lag in that range
    # is the EXPECTED, correct result for a post-fix flight; a lag of hundreds of seconds
    # means the reset never reached that drone (check the terminal for its WARN), and a lag
    # near zero would be surprising enough to investigate.
    if lag < 2.0:
        print("  NOTE: lag is near zero, which is NOT what a post-2026-09-15 flight should "
              "look like (usec.reset now fires pre-takeoff, so expect ~10-12s). Check which "
              "script/commit flew this.")
    elif lag > 60.0:
        print(f"  NOTE: lag of {lag:.0f}s is far larger than the ~10-12s a post-2026-09-15 "
              f"flight should show -- suspect the usec.reset broadcast never reached this "
              f"drone, or this is an older recording.")

    # 2026-09-15: what this tool proves, and what it does NOT.
    #
    # A match here proves the file contains A RUN OF THIS SCENARIO. It does NOT prove the file
    # is THAT PARTICULAR FLIGHT. Two back-to-back flights of the same scenario with the same
    # parameters have the same commanded trajectory, so they match every such meta.json equally
    # well -- on 2026-09-15 two A8 runs 8 minutes apart scored 1.7 and 1.8 cm RMS against BOTH
    # of their meta files, i.e. completely indistinguishable by correlation.
    #
    # The only ordering signal is the card's own file counter, which increments per logging
    # session: for two files on one card, the higher number is the later flight. That is an
    # assumption about session ordering, not a measurement, so it is worth stating out loud
    # whenever a card holds more than one run of the same scenario.
    print("\n  NOTE: this confirms the file contains a run of this scenario -- NOT that it is "
          "this specific\n        flight. Repeated runs of the same scenario are "
          "indistinguishable by correlation.\n        If the card held several, order them by "
          "the thesisNN counter (higher = later).")

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
