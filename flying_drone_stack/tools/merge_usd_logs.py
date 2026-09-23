#!/usr/bin/env python3
"""
Merge per-drone uSD logs from one formation flight into a single time-aligned dataset.

Why this is needed
------------------
Each Crazyflie stamps its uSD samples with `usecTimestamp()` — microseconds since ITS OWN
boot. Two drones therefore share no clock at all and their raw logs can be minutes apart.
Nothing in the firmware, crazyswarm2 or cflib merges them; `systemtests/SDplotting` is
single-drone only.

What makes it tractable is that `formation_flight.py` starts logging with ONE broadcast
(`allcfs.setParam("usd.logging", 1)`), so every drone begins within the broadcast jitter
plus one logging period. Subtracting each log's first timestamp therefore lines the drones
up to a few milliseconds.

Residual error, and why the tool measures rather than assumes:
  * broadcast propagation            ~1 ms
  * uSD task wake phase              <= 1 period (2 ms at 500 Hz)
  * crystal drift between drones     ~20-50 ppm -> 1-3 ms over a 60 s flight

The drift is a *rate* error, so a constant shift is not enough for long flights. If radio
CSVs from the same flight are available (they share a common clock), the tool cross-correlates
each drone's uSD trace against its own radio trace and reports the measured offset AND drift,
so the alignment is a number you can quote rather than an assumption.

Usage
-----
    python merge_usd_logs.py cf231.usd cf232.usd -o merged.csv
    python merge_usd_logs.py cf231.usd cf232.usd --radio experiments/logs/*_2026-08-25_*.csv
    python merge_usd_logs.py --self-test
"""

import re
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

CHECK_CHANNEL = "z"      # channel used for cross-correlation; present in radio and uSD alike
DEFAULT_RATE = 500.0     # Hz, matches usd_thesis_config.txt


# ── loading ─────────────────────────────────────────────────────────────────

def load_usd(path):
    """Decode one uSD log -> dict of arrays with 't' in seconds, zeroed at log start."""
    import decode_usd_log
    d = decode_usd_log.load(str(path))
    t = np.asarray(d["t"], dtype=float)
    out = {k: np.asarray(v, dtype=float) for k, v in d.items() if k != "t"}
    # Zero at the drone's first sample: with a broadcast start this is the common origin.
    out["t"] = t - t[0]
    return out


def load_radio_csv(path):
    """Load a formation_flight per-drone CSV (shared clock across drones)."""
    meta, header, rows = {}, None, []
    with open(path) as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("# meta:"):
                k, _, v = line[7:].partition("=")
                meta[k.strip()] = v.strip()
            elif header is None:
                header = line.split(",")
            elif line:
                rows.append([float(x) for x in line.split(",")])
    a = np.array(rows)
    cols = {n: a[:, i] for i, n in enumerate(header)}
    return meta, cols


# ── sync measurement ────────────────────────────────────────────────────────

def estimate_offset(t_a, y_a, t_b, y_b, max_shift=2.0, rate=500.0):
    """Cross-correlate two signals; return (offset_seconds, normalised peak).

    offset is what to ADD to t_a to line it up with t_b. The peak is a quality
    figure in [0, 1] — anything much below ~0.5 means the alignment is not trustworthy.

    The discrete correlation peak is quantised to 1/rate (2 ms at 500 Hz), coarser than
    the few-ms effects we care about, so it is only a starting point — see the refinement
    note below.

    Measured accuracy against synthetic signals with 1% noise: ~0.04 ms on a 60 s window,
    degrading to ~1 ms on a 15 s window. Longer windows average the noise down. Do not
    quote better than ~1 ms from a short segment.
    """
    lo = max(t_a[0], t_b[0])
    hi = min(t_a[-1], t_b[-1])
    if hi - lo < 1.0:
        return None, 0.0
    grid = np.arange(lo, hi, 1.0 / rate)
    A = np.interp(grid, t_a, y_a)
    B = np.interp(grid, t_b, y_b)
    A = A - A.mean()
    B = B - B.mean()
    if A.std() < 1e-9 or B.std() < 1e-9:
        return None, 0.0
    n = int(max_shift * rate)
    corr = np.correlate(B, A, mode="full")
    mid = len(A) - 1
    lo_i, hi_i = max(1, mid - n), min(len(corr) - 1, mid + n + 1)
    seg = corr[lo_i:hi_i]
    k = int(np.argmax(seg)) + lo_i
    peak = corr[k] / (len(A) * A.std() * B.std())
    lag = k - mid                                   # coarse, integer-sample

    # Refinement. Two techniques were tried and rejected before this one:
    #
    #  * Parabolic interpolation of the correlation peak — the textbook trick, but badly
    #    biased here. Flight motion is narrowband (sub-Hz), so the correlation peak is
    #    hundreds of samples wide and a parabola through three adjacent points barely
    #    curves: measured, it recovered only ~13% of the true sub-sample shift.
    #  * A single derivative correction on top of the coarse lag — accurate, but only
    #    while the coarse lag is right, and for such a broad peak the argmax is easily
    #    moved several samples by noise.
    #
    # So the coarse lag is used only as a starting point and the shift is then solved
    # iteratively with a first-order expansion, which is exact in the small-shift limit:
    #     y_b(t) = y_a(t - d) ≈ y_a(t) - d·y_a'(t)
    #  => d ≈ Σ (y_a - y_b)·y_a' / Σ y_a'²
    # Each pass shifts B by the running estimate and re-solves, so it converges from a
    # wrong starting lag and averages noise over the whole window.
    total = lag / rate
    da = np.gradient(A) * rate
    den = float(np.sum(da * da))
    if den <= 1e-12:
        return total, float(peak)
    edge = max(4, int(0.02 * len(A)))               # ignore ends (interp clamps there)
    sl = slice(edge, len(A) - edge)
    for _ in range(6):
        Bs = np.interp(grid, grid - total, B)       # undo the running estimate
        #  B(t)=A(t-total) -> want Bs(t)=B(t+total)=A(t); interp xp=grid-total gives that.
        #  (grid+total would shift the SAME way and the iteration compounds instead.)
        d = float(np.sum((A[sl] - Bs[sl]) * da[sl]) / float(np.sum(da[sl] * da[sl])))
        total += d
        if abs(d) < 1e-6:
            break
        if abs(total) > max_shift:                  # diverged
            return lag / rate, float(peak)
    return total, float(peak)


def estimate_drift(t_a, y_a, t_b, y_b, n_windows=4):
    """Offset measured in several windows -> linear fit gives offset + drift (ppm)."""
    lo, hi = max(t_a[0], t_b[0]), min(t_a[-1], t_b[-1])
    if hi - lo < 8.0:
        return None
    edges = np.linspace(lo, hi, n_windows + 1)
    cs, os_ = [], []
    for i in range(n_windows):
        ma = (t_a >= edges[i]) & (t_a < edges[i + 1])
        mb = (t_b >= edges[i]) & (t_b < edges[i + 1])
        if ma.sum() < 50 or mb.sum() < 50:
            continue
        off, pk = estimate_offset(t_a[ma], y_a[ma], t_b[mb], y_b[mb],
                                  max_shift=0.5, rate=500.0)
        if off is not None and pk > 0.3:
            cs.append(0.5 * (edges[i] + edges[i + 1]))
            os_.append(off)
    if len(cs) < 3:
        return None
    slope, intercept = np.polyfit(cs, os_, 1)
    return intercept, slope * 1e6      # seconds, ppm


# ── merge ───────────────────────────────────────────────────────────────────

def merge(logs, names, rate, mass):
    """Resample all drones onto one grid; add pairwise relative state and f_res."""
    t_hi = min(l["t"][-1] for l in logs)
    grid = np.arange(0.0, t_hi, 1.0 / rate)
    out = {"t": grid}
    for name, l in zip(names, logs):
        for k, v in l.items():
            if k == "t":
                continue
            out[f"{name}.{k}"] = np.interp(grid, l["t"], v)

    # Every drone must actually carry the channels the rest of this depends on. Without this
    # check the two blocks below are silently skipped by their `in out` guards and the merge
    # succeeds while producing no relative state and no f_res at all.
    required = ("x", "y", "z", "vx", "vy", "vz")
    for name in names:
        missing = [c for c in required if f"{name}.{c}" not in out]
        if missing:
            raise SystemExit(
                f"{name}: log is missing {missing}. Expected the names produced by "
                f"decode_usd_log.RENAME (x/y/z/vx/vy/vz, a_res_*). If the log has "
                f"'stateEstimate.x' instead, that channel is unmapped -- fix RENAME.")
        if f"{name}.a_res_x" not in out:
            print(f"WARNING: {name}: no a_res_* -- f_res will not be computed. "
                  f"On hardware this also means no RPM source was present.", file=sys.stderr)

    # relative state: the NN input. Sign convention: i relative to j.
    for i in range(len(names)):
        for j in range(len(names)):
            if i == j:
                continue
            for ax in ("x", "y", "z"):
                a, b = f"{names[i]}.{ax}", f"{names[j]}.{ax}"
                if a in out and b in out:
                    out[f"rel.{names[i]}_{names[j]}.{ax}"] = out[a] - out[b]
            va, vb = f"{names[i]}.vx", f"{names[j]}.vx"
            if va in out and vb in out:
                for ax in ("vx", "vy", "vz"):
                    out[f"rel.{names[i]}_{names[j]}.{ax}"] = (
                        out[f"{names[i]}.{ax}"] - out[f"{names[j]}.{ax}"])

    # f_res [N] from the logged residual acceleration
    for name in names:
        for ax in ("x", "y", "z"):
            k = f"{name}.a_res_{ax}"
            if k in out:
                out[f"{name}.f_res_{ax}"] = mass * out[k]
    return out


def write_csv(path, data, t_zero=None):
    """`t_zero`, if given, is written as a `# meta:` comment line (the convention the `ros`
    format already uses, and `metrics._read_text_header` already skips). Downstream tools use
    it to know what t=0 actually means in THIS file -- critical for `commanded_from_scenario`,
    which needs to know whether `t` is already scenario-relative or needs t_start_sim/timescale
    reconciliation. Without this, a --meta-aligned merge (t=0 IS scenario start) and an
    unaligned one (t=0 is just wherever the first log's samples happened to begin) are
    indistinguishable from the file alone, and guessing wrong silently reconstructs the wrong
    commanded trajectory rather than failing loudly.
    """
    keys = ["t"] + sorted(k for k in data if k != "t")
    n = len(data["t"])
    with open(path, "w") as f:
        if t_zero:
            f.write(f"# meta:t_zero={t_zero}\n")
        f.write(",".join(keys) + "\n")
        for i in range(n):
            f.write(",".join(f"{data[k][i]:.6f}" for k in keys) + "\n")


# ── self-test ───────────────────────────────────────────────────────────────

def self_test():
    """Prove the sync estimator recovers a known offset and a known drift."""
    print("self-test: recovering known offsets from synthetic logs\n")
    rate = 500.0
    t = np.arange(0, 60, 1 / rate)
    sig = np.sin(2 * np.pi * 0.35 * t) + 0.3 * np.sin(2 * np.pi * 1.1 * t)
    rng = np.random.default_rng(0)

    ok = True
    for true_off in (0.0, 0.004, -0.012, 0.25):
        y_a = sig + 0.01 * rng.standard_normal(len(t))
        # b is a shifted by true_off  ->  estimator should return +true_off
        y_b = np.interp(t, t + true_off, sig) + 0.01 * rng.standard_normal(len(t))
        est, pk = estimate_offset(t, y_a, t, y_b, max_shift=1.0, rate=rate)
        err = abs(est - true_off) if est is not None else float("inf")
        good = err < 1.5 / rate
        ok &= good
        print(f"  offset {true_off*1000:+7.1f} ms -> est {est*1000:+7.1f} ms  "
              f"err {err*1000:5.2f} ms  peak {pk:.3f}  {'OK' if good else 'FAIL'}")

    # short noisy window — the realistic worst case, ~1 ms is the honest bound
    t15 = np.arange(0, 15, 1 / rate)
    s15 = np.sin(2 * np.pi * 0.35 * t15) + 0.3 * np.sin(2 * np.pi * 1.1 * t15)
    print()
    for true_off in (0.002, 0.010):
        y_a = s15 + 0.01 * rng.standard_normal(len(t15))
        y_b = np.interp(t15, t15 + true_off, s15) + 0.01 * rng.standard_normal(len(t15))
        est, pk = estimate_offset(t15, y_a, t15, y_b, max_shift=0.5, rate=rate)
        err = abs(est - true_off)
        good = err < 1.5e-3          # 1.5 ms on a short noisy window
        ok &= good
        print(f"  15 s window, noisy, {true_off*1000:+5.1f} ms -> est {est*1000:+6.2f} ms  "
              f"err {err*1000:4.2f} ms  {'OK' if good else 'FAIL'}")

    # drift: b's clock runs 40 ppm fast
    ppm = 40.0
    y_a = sig + 0.01 * rng.standard_normal(len(t))
    t_b = t * (1 + ppm * 1e-6)
    y_b = np.interp(t, t_b, sig) + 0.01 * rng.standard_normal(len(t))
    res = estimate_drift(t, y_a, t, y_b)
    if res is None:
        print("\n  drift: FAILED to estimate")
        ok = False
    else:
        off, est_ppm = res
        good = abs(est_ppm - ppm) < 15    # ~35% — enough to correct most of the skew
        ok &= good
        print(f"\n  drift {ppm:+.0f} ppm -> est {est_ppm:+.1f} ppm  {'OK' if good else 'FAIL'}")
        print(f"    (over 60 s that is {ppm*60e-6*1000:.1f} ms of accumulated skew)")

    print("\n" + ("SELF-TEST PASSED" if ok else "SELF-TEST FAILED"))
    return 0 if ok else 1


# ── main ────────────────────────────────────────────────────────────────────

def drone_name_from(p: Path) -> str:
    """Drone name from a copy_usd_log.py filename, e.g.

        cf231_active_A8_thesis06_2026-09-15_19-59-43.bin -> cf231_active
        cf_second_A8_thesis08_2026-09-15_20-01-34.bin    -> cf_second
        cf_second_thesis01_2026-09-15_17-47-07.bin       -> cf_second

    2026-09-15: this used to be `p.stem.split("_")[0]`, which truncated every real drone
    name we have -- `cf231_active` became `cf231` and `cf_second` became `cf`. Drone names
    contain underscores, so splitting on the FIRST underscore can never work. The card-side
    file name (`thesisNN`) is the reliable delimiter: everything before it is the drone
    name plus any --tag, and the tag is stripped separately.
    """
    stem = p.stem
    parts = stem.split("_")
    for i, tok in enumerate(parts):
        if tok.startswith("thesis"):
            head = parts[:i]
            # drop a trailing --tag (a scenario id like A8/A1/B2), if one was passed
            if head and re.fullmatch(r"[A-Z]\d+", head[-1]):
                head = head[:-1]
            return "_".join(head) if head else stem
    # Not a copy_usd_log.py name -- fall back to the whole stem rather than a wrong guess.
    return stem


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="*", help="per-drone uSD log files")
    ap.add_argument("-o", "--out", default="merged.csv")
    ap.add_argument("--radio", nargs="*", default=[],
                    help="formation_flight per-drone CSVs (shared clock) to verify sync against")
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE)
    ap.add_argument("--mass", type=float, default=0.041)
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--meta", default=None,
                    help="scenario .meta.json. With --roles, aligns each drone by correlating "
                         "it against ITS OWN commanded trajectory instead of cross-correlating "
                         "the drones against each other. Strongly preferred -- see below.")
    ap.add_argument("--roles", nargs="*", default=[],
                    help="role per log, in the same order as the logs (e.g. bottom top)")
    ap.add_argument("--run-tag", type=int, default=None,
                    help="match logs by usd.runTag (post-2026-09-21 firmware); use with "
                         "--archive-t1 and --archive-t2")
    ap.add_argument("--archive-t1", default=None, help="THESIS1 (or cf5-card) archive dir")
    ap.add_argument("--archive-t2", default=None, help="THESIS2 (or top-card) archive dir")
    ap.add_argument("--tag-quality-only", action="store_true",
                    help="do not refuse merge when alignment RMS >= 15 cm (still printed). "
                         "Use with --run-tag auto-pairing, or human accept e.g. A1 close-stack "
                         "where bottom tracking error vs nominal command is expected downwash.")
    a = ap.parse_args()

    if a.self_test:
        sys.exit(self_test())

    if a.run_tag is not None:
        if not a.archive_t1 or not a.archive_t2:
            ap.error("--run-tag requires --archive-t1 and --archive-t2")
        import index_usd_archive
        i1 = index_usd_archive.index_directory(Path(a.archive_t1))["by_run_tag"]
        i2 = index_usd_archive.index_directory(Path(a.archive_t2))["by_run_tag"]
        key = str(a.run_tag)
        if key not in i1 or key not in i2:
            sys.exit(f"[merge] run_tag {a.run_tag} not found in both archives "
                     f"(t1={key in i1}, t2={key in i2})")
        p1, p2 = Path(i1[key]["path"]), Path(i2[key]["path"])
        a.logs = [str(p1), str(p2)]
        if not a.roles:
            a.roles = ["bottom", "top"]
        a.tag_quality_only = True
        print(f"[merge] run_tag {a.run_tag}: {p1.name} + {p2.name}")

    if not a.logs:
        ap.error("give at least one uSD log, or --self-test, or --run-tag")

    names, logs = [], []
    for p in a.logs:
        p = Path(p)
        try:
            logs.append(load_usd(p))
            names.append(drone_name_from(p))
        except Exception as e:
            sys.exit(f"[merge] cannot read {p}: {e}")

    print(f"[merge] {len(logs)} drone(s): {', '.join(names)}")
    for n, l in zip(names, logs):
        print(f"   {n}: {len(l['t'])} samples, {l['t'][-1]:.2f} s, "
              f"{int(round(len(l['t'])/max(l['t'][-1],1e-9)))} Hz")

    # ---- preferred alignment: each drone against its OWN commanded trajectory ----
    #
    # 2026-09-15: the cross-drone z-correlation below is unreliable for any scenario that
    # holds each vehicle at a near-constant altitude -- A8 is exactly that, and it produced
    # +282 ms at corr 0.39 on a flight whose true cross-drone offset was 30 ms. Correlating
    # each drone against its own commanded trajectory instead is unambiguous (<2 cm RMS on
    # that same flight) because the commanded curve is a known, strong, drone-specific
    # signal. When --meta/--roles are given, THAT is what sets the time base; the z
    # correlation is then only reported as a cross-check.
    if a.meta:
        if len(a.roles) != len(logs):
            ap.error(f"--roles needs one role per log ({len(logs)} given logs, "
                     f"{len(a.roles)} roles)")
        try:
            import json
            from find_flight_window import commanded_trajectory, find_offset as fw_offset
        except Exception as e:
            sys.exit(f"[merge] cannot load find_flight_window for --meta alignment: {e}")
        meta = json.load(open(a.meta))
        total = meta["duration"]
        print(f"\n[merge] aligning on commanded trajectory from {Path(a.meta).name} "
              f"(scenario {meta['scenario']}, {total:.1f}s)")
        lags = []
        for n, role, l in zip(names, a.roles, logs):
            ts, cmd = commanded_trajectory(meta, role)
            t = l["t"]
            if t[-1] - t[0] < total:
                sys.exit(f"[merge] {n}: recording is {t[-1]-t[0]:.1f}s but the scenario runs "
                         f"{total:.1f}s -- this log cannot contain the whole flight.")
            # 2026-09-15: find_offset now matches full 3D position, not a single named channel
            # -- needed once commanded_trajectory stopped being A8-only (some scenarios move in
            # x, some sweep z, not just y).
            pos = np.stack([l["x"], l["y"], l["z"]], axis=1)
            lag, mse, _ = fw_offset(t, pos, ts, cmd, t[0], t[-1] - total)
            if lag is None:
                sys.exit(f"[merge] {n}: no lag covers enough of the scenario to align on. "
                         f"Wrong file for this scenario, or wrong role?")
            rms = float(np.sqrt(mse)) * 100.0
            quality_only = getattr(a, "tag_quality_only", False)
            flag = "" if rms < 15.0 else "   <-- BAD FIT: wrong role, or wrong file for this flight"
            if quality_only and rms >= 15.0:
                flag = "   <-- quality flag (pair accepted by run_tag)"
            print(f"   {n:14s} role={role:7s} scenario starts at its t={lag:6.2f}s  "
                  f"RMS {rms:5.1f} cm{flag}")
            if rms >= 15.0 and not quality_only:
                sys.exit(f"[merge] refusing to merge on an alignment this poor -- fix the "
                         f"role/file pairing first.")
            lags.append(lag)
        # Re-zero every log at ITS OWN scenario start -> one shared, scenario-relative clock.
        for l, lag in zip(logs, lags):
            l["t"] = l["t"] - lag
        if len(lags) > 1:
            spread = (max(lags) - min(lags)) * 1000.0
            print(f"   -> cross-drone clock agreement: {spread:.0f} ms "
                  f"(independent clocks, shared usec.reset origin)")

    # ---- alignment quality ----
    if a.meta:
        print("\n[merge] cross-check: direct z-correlation between drones "
              "(unreliable when altitudes are flat -- the alignment above is authoritative)")
    else:
        print("\n[merge] alignment (logs zeroed at their own first sample = broadcast start)")
    if len(logs) > 1 and CHECK_CHANNEL in logs[0]:
        for i in range(1, len(logs)):
            off, pk = estimate_offset(logs[0]["t"], logs[0][CHECK_CHANNEL],
                                      logs[i]["t"], logs[i][CHECK_CHANNEL])
            if off is None:
                print(f"   {names[0]} vs {names[i]}: signals too flat to correlate")
            else:
                note = "" if abs(off) < 0.01 else "   <-- LARGER THAN EXPECTED"
                print(f"   {names[0]} vs {names[i]}: residual offset {off*1000:+.1f} ms "
                      f"(corr {pk:.2f}){note}")
            d = estimate_drift(logs[0]["t"], logs[0][CHECK_CHANNEL],
                               logs[i]["t"], logs[i][CHECK_CHANNEL])
            if d:
                print(f"      drift {d[1]:+.1f} ppm -> {abs(d[1])*logs[0]['t'][-1]*1e-6*1000:.1f} "
                      f"ms accumulated over this flight")

    # ---- optional check against the radio logs (independent common clock) ----
    for rp in a.radio:
        try:
            _, cols = load_radio_csv(Path(rp))
        except Exception:
            continue
        drone = Path(rp).stem.split("_")[-3] if len(Path(rp).stem.split("_")) >= 3 else ""
        for n, l in zip(names, logs):
            if n and drone and (n in drone or drone in n) and "pos_z" in cols:
                off, pk = estimate_offset(l["t"], l[CHECK_CHANNEL],
                                          cols["time_s"], cols["pos_z"])
                if off is not None:
                    print(f"   {n}: uSD vs radio offset {off*1000:+.1f} ms (corr {pk:.2f})")

    data = merge(logs, names, a.rate, a.mass)
    write_csv(a.out, data, t_zero="scenario_start" if a.meta else None)
    print(f"\n[merge] {len(data['t'])} rows x {len(data)} cols @ {a.rate:.0f} Hz -> {a.out}")
    rels = [k for k in data if k.startswith("rel.")]
    fres = [k for k in data if ".f_res_" in k]
    print(f"[merge] relative-state cols: {len(rels)}   f_res cols: {len(fres)}")
    if not fres:
        print("[merge] WARNING: no a_res in the logs — is indi.a_res_* in the uSD config,")
        print("        and does the drone have an RPM source? Without it there is no label.")


if __name__ == "__main__":
    main()
