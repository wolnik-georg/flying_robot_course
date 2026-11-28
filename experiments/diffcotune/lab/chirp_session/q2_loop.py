"""REPORT_48 Q2(b): chirp -> open-loop (loop) transfer of the attitude loop, gain / phase margin and the KR trend (the analysis for the day of the flight, tested here on frsim data with known answers).
INPUT (dict; real merged uSD columns / sim): t (s), u (n,) the chirp excitation (the setpoint acceleration of the chirp axis, m/s2: `ctrltarget_ay` for an y chirp), att_cmd (n, 2) the commanded roll / pitch in DEGREES (`omar_rpyd_x / y` = ctrlOmarIndi.rpydx / rpydy, the controller's rpy_des, which is logged in RADIANS: convert; `omar_rpy_*` is the actual rpy), att (n, 2) the measured roll / pitch the controller reads (`roll_deg`, `pitch_deg` = stabilizer.roll / pitch).
METHOD: error e = att_cmd - att; with the chirp as an INSTRUMENT (independent of the loop's noise): L(f) = S_{att,u} / S_{e,u} (the loop transfer from the error to the measured attitude; noise correlated with att but not with u does not bias it); windows of 750 rows (first 50 dropped, detrend, hann), bands 1.5-8 Hz, bootstrap over windows (5-95 %). From the complex L(f): the gain at 3.5 Hz (dB), the phase at 3.5 Hz, the sensitivity peak Ms = max_f 1 / |1 + L(f)| (dB, the robust distance to -1), the frequency of that peak, the phase margin at the |L| = 1 crossing and the gain margin at the -180 deg crossing when they exist in 1.5-8 Hz.
"""
import numpy as np
WIN = 750; SK = 50; N = WIN - SK; fr = np.fft.rfftfreq(N, 0.002); w_ = np.hanning(N); BANDS = [(1.5, 2.2), (2.2, 3.0), (3.0, 4.0), (4.0, 5.0), (5.0, 6.5), (6.5, 8.0)]
def spec(x): x = x - np.polyval(np.polyfit(np.arange(len(x)), x, 1), np.arange(len(x))); return np.fft.rfft(x * w_)
def windows(D, axis, tmin=5.0, tmax=65.0):
    t = D['t']; out = []
    for s in range(0, len(t) - WIN, WIN):
        if t[s] < tmin or t[s + WIN] > tmax: continue
        sl = slice(s + SK, s + WIN); y = D['att'][sl, axis]; e = D['att_cmd'][sl, axis] - y; out.append((spec(D['u'][sl]), spec(y), spec(e)))
    return out
def loop_curve(W, nboot=200, seed=1):
    rng = np.random.default_rng(seed); res = []
    for b in BANDS:
        sel = (fr >= b[0]) & (fr < b[1])
        def est(idx):
            syu = seu = 0
            for i in idx: U, Y, E = W[i]; syu += (np.conj(U[sel]) * Y[sel]).sum(); seu += (np.conj(U[sel]) * E[sel]).sum()
            return syu / seu
        L = est(range(len(W))); bs = [est(rng.integers(0, len(W), len(W))) for _ in range(nboot)]; res.append((fr[sel].mean(), L, [abs(x) for x in bs], [np.angle(x) for x in bs]))
    return res
def metrics(res):
    f = np.array([r[0] for r in res]); L = np.array([r[1] for r in res]); ms = 1 / abs(1 + L); i = int(np.argmax(ms)); k35 = int(np.argmin(abs(f - 3.5)))
    out = dict(f=f, L=L, gain35_db=20 * np.log10(abs(L[k35])), phase35=np.degrees(np.angle(L[k35])), Ms_db=20 * np.log10(ms[i]), Ms_f=f[i])
    ph = np.unwrap(np.angle(L)); g = abs(L); pm = gm = None
    for j in range(len(f) - 1):
        if (g[j] - 1) * (g[j + 1] - 1) < 0: a = (1 - g[j]) / (g[j + 1] - g[j]); pm = float(np.degrees(np.interp(a, [0, 1], [ph[j], ph[j + 1]])) + 180)
        if (np.degrees(ph[j]) + 180) * (np.degrees(ph[j + 1]) + 180) < 0: a = (-np.pi - ph[j]) / (ph[j + 1] - ph[j]); gm = float(-20 * np.log10(np.interp(a, [0, 1], [g[j], g[j + 1]])))
    out['pm'] = pm; out['gm_db'] = gm; return out
def analyse(D, axis=1, **kw): return metrics(loop_curve(windows(D, axis, **kw)))
