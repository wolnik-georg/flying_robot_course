#!/usr/bin/env python3
"""Measure a quadrotor inertia axis with a bifilar pendulum + the onboard gyro.

Hang the drone from TWO parallel vertical wires so that the axis you want to
measure points UP (aligned with the twist axis), give it a small torsional twist
(~5-10 deg), release, and this script logs the gyro, extracts the oscillation
period, and computes the inertia about that axis:

    J = m * g * D^2 * T^2 / (16 * pi^2 * L)

  m = drone mass [kg]      D = distance between the two wires [m]
  L = wire length [m]      T = torsional oscillation period [s]

Hang orientation → gyro axis:
  JZZ (yaw)  : drone flat, props up     → --axis z
  JXX (roll) : drone on its side        → --axis x
  JYY (pitch): drone nose up/down       → --axis y

Run 3x (one per axis). Repeat each a few times and average.

Usage:
  ~/.pyenv/versions/flying_robots/bin/python measure_inertia.py \
      --axis z --mass 0.0393 --sep 0.08 --length 0.60 --seconds 12
"""
import argparse, math, time
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = "radio://0/80/2M/E7E7E7E7E7"
G = 9.81


def collect_gyro(uri, axis, seconds):
    """Log gyro.{axis} at ~100 Hz for `seconds`; return (t[s], w[deg/s])."""
    var = f"gyro.{axis}"
    ts, ws = [], []
    lc = LogConfig(name="gyro", period_in_ms=10)  # 100 Hz
    lc.add_variable(var, "float")
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        scf.cf.log.add_config(lc)
        t0 = time.time()
        with lc as logger:
            print(f"[inertia] Logging {var} for {seconds:.0f}s — twist ~5-10 deg and release NOW.")
            for entry in logger:
                t = time.time() - t0
                ts.append(t); ws.append(entry[1][var])
                if t >= seconds:
                    break
    return np.asarray(ts), np.asarray(ws)


def period_from_signal(t, w):
    """Estimate oscillation period via zero-crossings and FFT (returns both)."""
    w = w - np.mean(w)
    # drop the first ~0.5 s (release transient)
    m = t > (t[0] + 0.5)
    t, w = t[m], w[m]
    # zero-crossing period (rising edges)
    sign = np.sign(w)
    rising = np.where((sign[:-1] < 0) & (sign[1:] >= 0))[0]
    tzc = None
    if len(rising) >= 2:
        # linear-interp the crossing times for sub-sample accuracy
        cross_t = []
        for i in rising:
            frac = -w[i] / (w[i + 1] - w[i])
            cross_t.append(t[i] + frac * (t[i + 1] - t[i]))
        cross_t = np.asarray(cross_t)
        tzc = np.mean(np.diff(cross_t))
    # FFT period
    dt = np.median(np.diff(t))
    freqs = np.fft.rfftfreq(len(w), dt)
    mag = np.abs(np.fft.rfft(w * np.hanning(len(w))))
    mag[0] = 0.0  # ignore DC
    fpk = freqs[np.argmax(mag)]
    tfft = 1.0 / fpk if fpk > 0 else None
    return tzc, tfft


def main():
    ap = argparse.ArgumentParser(description="Bifilar inertia measurement (gyro-based)")
    ap.add_argument("--axis", choices=["x", "y", "z"], required=True)
    ap.add_argument("--mass", type=float, required=True, help="drone mass m [kg]")
    ap.add_argument("--sep", type=float, required=True, help="wire separation D [m]")
    ap.add_argument("--length", type=float, required=True, help="wire length L [m]")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--uri", default=URI)
    args = ap.parse_args()

    cflib.crtp.init_drivers()
    t, w = collect_gyro(args.uri, args.axis, args.seconds)
    if len(t) < 20:
        print("[inertia] Not enough samples — check the link and retry.")
        return
    tzc, tfft = period_from_signal(t, w)
    print(f"[inertia] period: zero-cross = {tzc if tzc else float('nan'):.4f} s, "
          f"FFT = {tfft if tfft else float('nan'):.4f} s")
    T = tzc if tzc else tfft
    if not T:
        print("[inertia] Could not extract a period — twist harder / log longer.")
        return
    J = args.mass * G * args.sep**2 * T**2 / (16.0 * math.pi**2 * args.length)
    print(f"\n[inertia]  J_{args.axis}{args.axis} = {J:.4e} kg*m^2   "
          f"(m={args.mass} D={args.sep} L={args.length} T={T:.4f})")
    print("[inertia]  Repeat a few times; average. Paste JXX/JYY/JZZ into "
          "firmware_app/src/lib.rs (#[cfg(drone_bl)] block).")


if __name__ == "__main__":
    main()
