#!/usr/bin/env python
"""Analyze a bench_actuator_id.py run: static curve, step asymmetry, chirp frequency
response at both amplitudes. Answers whether the brushless command->RPM path is
amplitude-dependent (slew limit) and what gain/phase it has at 6.3 Hz.

Usage: ~/.pyenv/versions/flying_robots/bin/python analyze_bench_actuator.py [csvfile]
       (defaults to newest Controls/logs/bench_actuator_*.csv)
"""
import glob
import sys

import numpy as np
from scipy import signal

HOVER = 32700


def load(path):
    markers = []          # (t, name)
    t, rpm, cmd = [], [], []
    with open(path) as f:
        next(f)
        for line in f:
            p = line.strip().split(",")
            if p[1] == "-1":
                markers.append((float(p[0]), p[6]))
                continue
            t.append(float(p[0]))
            rpm.append([float(x) for x in p[1:5]])
            cmd.append(float(p[5]))
    t = np.array(t); t -= t[0]
    for i, (mt, _) in enumerate(markers):
        markers[i] = (mt - (t[0] if len(t) else 0), markers[i][1])
    return t, np.array(rpm), np.array(cmd), markers


def seg(t, markers, name, t0_abs):
    starts = [mt - t0_abs for mt, n in markers if n == name]
    if not starts:
        return None
    s = starts[0]
    ends = [mt - t0_abs for mt, _ in markers if mt - t0_abs > s]
    e = min(ends) if ends else t[-1]
    return (t >= s + 0.5) & (t < e)


def chirp_response(t, x, y, fs, label):
    """x=cmd, y=rpm; welch TF in bands around target freqs"""
    x = x - x.mean(); y = y - y.mean()
    nseg = 1024
    f, pxx = signal.welch(x, fs=fs, nperseg=nseg)
    f, pxy = signal.csd(x, y, fs=fs, nperseg=nseg)
    f, coh = signal.coherence(x, y, fs=fs, nperseg=nseg)
    H = pxy / pxx
    # normalize |H| by its low-frequency value -> gain relative to static
    i_lo = np.argmin(np.abs(f - 1.2))
    H0 = np.abs(H[i_lo])
    print(f"\n  {label} (|H| normalized to {H0:.2f} RPM/count at 1.2 Hz):")
    print("    freq   |H|/|H0|   phase[deg]   coh")
    for ft in (1.2, 2, 3, 4, 5, 6.3, 8, 10, 12, 15):
        i = np.argmin(np.abs(f - ft))
        print(f"    {f[i]:5.2f}   {np.abs(H[i])/H0:6.3f}    {np.degrees(np.angle(H[i])):+7.1f}   {coh[i]:.2f}")


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(
        "/home/georg/Desktop/flying_robot_course/Controls/logs/bench_actuator_*.csv"))[-1]
    print(f"loading {path}")
    with open(path) as f:
        next(f)
        first_t = float(f.readline().split(",")[0])
    t, rpm, cmd, markers = load(path)
    fs = 1 / np.median(np.diff(t))
    print(f"{len(t)} rows, fs={fs:.1f} Hz")

    m = seg(t, markers, "staircase", first_t)
    if m is not None:
        print("\nSTATIC: cmd level -> mean RPM (check local slope vs sqrt curve)")
        lv = np.round(cmd[m] / 65535, 2)
        for u in sorted(set(lv)):
            sel = m.copy(); sel[m] = lv == u
            print(f"  ratio {u:.2f}: rpm {rpm[sel].mean(axis=0).round(0)}")

    for name in ("steps_10pct", "steps_30pct"):
        m = seg(t, markers, name, first_t)
        if m is None:
            continue
        c = cmd[m]; r = rpm[m, 0]; tt = t[m]
        # rise/fall 63% times on the first up and first down edge
        edges_up = np.where(np.diff(c) > 0.05 * HOVER)[0]
        edges_dn = np.where(np.diff(c) < -0.05 * HOVER)[0]
        def tau63(i0, up):
            r0 = r[i0]; r1 = r[min(i0 + int(0.8 * fs), len(r) - 1)]
            target = r0 + 0.63 * (r1 - r0)
            for j in range(i0, min(i0 + int(0.8 * fs), len(r) - 1)):
                if (r[j] >= target) == up:
                    return tt[j] - tt[i0]
            return np.nan
        if len(edges_up) and len(edges_dn):
            tu = [tau63(i, True) for i in edges_up[:4]]
            td = [tau63(i, False) for i in edges_dn[:4]]
            print(f"\n{name}: tau63 up {np.nanmean(tu)*1e3:.0f} ms, down {np.nanmean(td)*1e3:.0f} ms "
                  f"(asymmetry ratio {np.nanmean(td)/max(np.nanmean(tu),1e-9):.2f})")

    for name in ("chirp_5pct", "chirp_25pct", "diff_chirp_5pct", "diff_chirp_25pct"):
        m = seg(t, markers, name, first_t)
        if m is not None:
            # motor_m1 column is logged for both single and differential chirps (m1 is
            # always the "up" motor in the differential case, see bench_actuator_id.py).
            chirp_response(t[m], cmd[m], rpm[m, 0], fs, name)

    print("\nINTERPRETATION GUIDE:")
    print("  - chirp_5pct ~flat with small phase, chirp_25pct attenuated with -60..-90 deg at 6.3 Hz")
    print("      -> amplitude-dependent slew/accel limit (describing-function mechanism) CONFIRMED")
    print("  - both amplitudes identical first-order rolloff -> linear actuator pole; note its fc")
    print("  - tau63 down >> up -> spool-down asymmetry contributes")
    print("  - everything flat to 15 Hz at both amplitudes -> actuator NOT the cause; revisit")


if __name__ == "__main__":
    main()
