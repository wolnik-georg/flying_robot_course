"""Robust A8 crossing detector (2026-10-09).

The bottom drone swaps sides in A8: y goes -0.5 -> +0.5 -> -0.5 ... so a crossing is a zero crossing of y (with hysteresis),
one per pass, 6 s apart. The older `ns2_2026_10_05_crossing_dip.find_crossing_times` takes minima of |y| and mislocated at least one
crossing in 8 of 29 A8 flights (2026-10-09 audit): use this function for any new dip/steady-window analysis.
"""
import numpy as np


def zero_crossings(t, y, n=4, hyst=0.15, spacing=6.0, tol=1.5):
    t = np.asarray(t, float); y = np.asarray(y, float)
    if len(t) < 20:
        return []
    k = 3 if np.median(np.diff(t)) > 0.02 else 25          # light smoothing: 3 samples at 20 Hz, 25 at 500 Hz
    ys = np.convolve(y, np.ones(k) / k, mode="same")
    times, state, j = [], None, None
    for i in range(len(ys)):
        s = 1 if ys[i] > hyst else (-1 if ys[i] < -hyst else 0)
        if s == 0:
            continue
        if state is not None and s != state:
            for m in range(j, i):
                if ys[m] * ys[m + 1] <= 0:
                    f = ys[m] / (ys[m] - ys[m + 1]) if ys[m] != ys[m + 1] else 0.0
                    times.append(float(t[m] + f * (t[m + 1] - t[m]))); break
        state, j = s, i
    # keep the longest run (<= n, >= 3) of consecutive crossings spaced ~spacing s (drops takeoff/landing artefacts;
    # a radio log that ends before the last pass legitimately yields 3)
    best = []
    for a in range(len(times)):
        run = [times[a]]
        for b in range(a + 1, len(times)):
            if abs((times[b] - run[-1]) - spacing) < tol and len(run) < n:
                run.append(times[b])
            else:
                break
        if len(run) > len(best):
            best = run
    return best if len(best) >= 3 else []
