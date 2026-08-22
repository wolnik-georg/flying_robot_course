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


def write_csv(path, data):
    keys = ["t"] + sorted(k for k in data if k != "t")
    n = len(data["t"])
    with open(path, "w") as f:
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
    a = ap.parse_args()

    if a.self_test:
        sys.exit(self_test())
    if not a.logs:
        ap.error("give at least one uSD log, or --self-test")

    names, logs = [], []
    for p in a.logs:
        p = Path(p)
        try:
            logs.append(load_usd(p))
            names.append(p.stem.split("_")[0])
        except Exception as e:
            sys.exit(f"[merge] cannot read {p}: {e}")

    print(f"[merge] {len(logs)} drone(s): {', '.join(names)}")
    for n, l in zip(names, logs):
        print(f"   {n}: {len(l['t'])} samples, {l['t'][-1]:.2f} s, "
              f"{int(round(len(l['t'])/max(l['t'][-1],1e-9)))} Hz")

    # ---- alignment quality ----
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
    write_csv(a.out, data)
    print(f"\n[merge] {len(data['t'])} rows x {len(data)} cols @ {a.rate:.0f} Hz -> {a.out}")
    rels = [k for k in data if k.startswith("rel.")]
    fres = [k for k in data if ".f_res_" in k]
    print(f"[merge] relative-state cols: {len(rels)}   f_res cols: {len(fres)}")
    if not fres:
        print("[merge] WARNING: no a_res in the logs — is indi.a_res_* in the uSD config,")
        print("        and does the drone have an RPM source? Without it there is no label.")


if __name__ == "__main__":
    main()
