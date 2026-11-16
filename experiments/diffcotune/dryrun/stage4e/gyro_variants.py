"""Stage 4e: variants of the measured gyro in the J_sysId target.

The frsim state trace holds the TRUE body rate (episode.py: state_rows takes body.w), not the gyro the controller sees. The uSD logs of the lab hold the measured gyro, so
the "measured gyro" of a dry-run flight is rebuilt here: true body rate + white noise with the plant's gyro noise (sensors.py: degrees(w) + normal(gyro_noise_deg_s, 3);
P1 = 12.0755 deg/s), one fresh sample per trace row, fixed seed per (round or flight set, scenario). Variants: 'raw' = that measured gyro; 'sg' = the measured gyro smoothed by a
zero-phase Savitzky-Golay filter (scipy.signal.savgol_filter, polyorder 5, window given in ms and converted to an odd number of samples at the trace's own row rate: frsim
traces 100 Hz, uSD traces 500 Hz). The model's body rates are never filtered. Only the target columns 11:14 of the trace are changed."""
import sys
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter, savgol_coeffs
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent))
import reference  # noqa: E402,F401  (puts simulator/ on sys.path)
from frsim import config  # noqa: E402

POLY = 5


def sigma_rad_s(plant_overrides):
    return float(np.radians(config.plant_with(plant_overrides).gyro_noise_deg_s))


def win_samples(win_ms, dt):
    n = int(round(win_ms / 1000.0 / dt))
    n += 1 - n % 2                      # odd
    return max(n, POLY + 2)


def sg(w, win_ms, dt):
    return savgol_filter(w, win_samples(win_ms, dt), POLY, axis=0, mode='interp')


def response(win_ms, dt, n=4096):
    """(frequency Hz, |H|) of the SG filter (zero phase) and its -3 dB frequency, noise gain (sum of squared coefficients)."""
    c = savgol_coeffs(win_samples(win_ms, dt), POLY)
    H = np.abs(np.fft.rfft(c, n)); f = np.fft.rfftfreq(n, dt)
    i = int(np.argmax(H < 1 / np.sqrt(2)))
    return f, H, float(f[i]), float((c * c).sum())


def make_trace_fn(mode, plant_overrides, win_ms=110, seed_base=7000):
    """trace_fn(trace, l, i) for loop_lib.run_loop(trace_fn=...) / the T0 script. mode: None (trace as is) | 'truth' | 'raw' | 'sg'."""
    sig = sigma_rad_s(plant_overrides)

    def fn(tr, l, i):
        if mode is None:
            return tr
        tr = np.array(tr, float); dt = float(np.median(np.diff(tr[:, 0])))
        if mode == 'truth':
            return tr
        rng = np.random.default_rng(seed_base + 10 * l + i)
        w = tr[:, 11:14] + sig * rng.standard_normal((len(tr), 3))
        tr[:, 11:14] = w if mode == 'raw' else sg(w, win_ms, dt)
        return tr
    return fn


if __name__ == '__main__':      # self-test of the filter against a known case: a polynomial of order <= 5 is reproduced exactly, noise variance is reduced by the coefficient energy
    dt = 0.01; t = np.arange(400) * dt
    poly = 1 + 2 * t - 3 * t ** 2 + 0.5 * t ** 3 + 0.1 * t ** 5
    for ms in (70, 110, 210):
        err = np.abs(sg(np.stack([poly] * 3, 1), ms, dt)[:, 0] - poly).max()
        rng = np.random.default_rng(0); z = rng.standard_normal((200000, 3)); out = sg(z, ms, dt)
        f, H, fc, gain = response(ms, dt)
        print('window %3d ms = %2d samples: max error on a 5th-order polynomial %.2e; noise variance ratio %.3f (sum c^2 = %.3f); -3 dB at %.1f Hz (fs %.0f Hz)' % (ms, win_samples(ms, dt), err, out[1000:-1000].var() / z.var(), gain, fc, 1 / dt))
