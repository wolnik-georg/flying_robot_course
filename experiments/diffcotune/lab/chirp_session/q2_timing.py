"""REPORT_48 Q2(a): gyro timing against the mocap quaternion (the analysis for the day of the flight, tested here on frsim data with known answers).
INPUT (a dict, the same fields for the real merged uSD file and for the sim): t (s, rows 2.000 ms), gyro (n, 3) deg/s as logged, mq (n, 4) mocap quaternion x y z w as logged (held between packets), mtick (n) the packet tick of the last packet (ms; changes when a new packet arrives).
METHOD: packets = rows where mtick (or the quaternion) changes; roll / pitch of the mocap quaternion (scipy 'xyz' Euler, as the firmware / frsim state); linear interpolation of the packet series onto the 500 Hz rows; windows of 750 rows (first 50 dropped, detrend, hann); the gyro integrated in the frequency domain (gyro / (j 2 pi f)); per band the cross-spectrum S_{gyro angle, mocap angle}: tau = lag of the mocap series behind the integrated gyro = -phase / (2 pi f) (+ = mocap later), gain |H| (a mocap filter shows as a gain roll-off), coherence; bootstrap over windows (5-95 %).
With d the mocap latency, tau_f the group delay of a mocap filter and g_gyro the gyro channel delay:  tau = d + tau_f - g_gyro.  A first-order filter of time constant tau_m is separated from the delay by the gain: |H| = 1 / sqrt(1 + (2 pi f tau_m)^2), phase -atan(2 pi f tau_m) - 2 pi f (d - g_gyro): fit of both over the bands (least squares on the complex H).
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot
WIN = 750; SK = 50; N = WIN - SK; fr = np.fft.rfftfreq(N, 0.002); w_ = np.hanning(N); BANDS = [(1.5, 2.5), (2.5, 3.5), (3.5, 4.5), (4.5, 6), (6, 8)]
def spec(x): x = x - np.polyval(np.polyfit(np.arange(len(x)), x, 1), np.arange(len(x))); return np.fft.rfft(x * w_)
def mocap_angles(D):
    q = D['mq']; tk = D['mtick']; ch = np.flatnonzero(np.r_[True, np.diff(tk) != 0]) if tk is not None else np.flatnonzero(np.r_[True, np.any(np.diff(q, axis=0) != 0, axis=1)])
    e = Rot.from_quat(q[ch] / np.linalg.norm(q[ch], axis=1)[:, None]).as_euler('xyz'); t = D['t'][ch]; D['_pkt_t'] = t   # packet times kept for the gap test of windows()
    return np.stack([np.interp(D['t'], t, np.unwrap(e[:, 0])), np.interp(D['t'], t, np.unwrap(e[:, 1]))], 1)   # rad, roll and pitch (yaw removed by the Euler split)
def windows(D, axis, tmin=0.0, tmax=1e9, max_gap_ms=25.0):
    """windows whose packet stream has a gap longer than max_gap_ms are DROPPED (REPORT_49 R4: with the real dropouts the linear interpolation across a gap biases the phase)"""
    ang = mocap_angles(D)[:, axis]; pk = D['_pkt_t']; g = np.radians(D['gyro'][:, axis]); t = D['t']; out = []; om = 2 * np.pi * fr; jw = np.where(om > 0, 1j * om, 1.0)
    for s in range(0, len(t) - WIN, WIN):
        if t[s] < tmin or t[s + WIN] > tmax: continue
        sl = slice(s + SK, s + WIN); inwin = pk[(pk >= t[s] - 0.02) & (pk <= t[min(s + WIN, len(t) - 1)] + 0.02)]
        if len(inwin) < 3 or np.max(np.diff(inwin)) * 1e3 > max_gap_ms: continue
        out.append((spec(g[sl]) / jw, spec(ang[sl])))
    return out
def band(W, b, nboot=300, seed=1):
    sel = (fr >= b[0]) & (fr < b[1]); f0 = fr[sel].mean(); rng = np.random.default_rng(seed)
    def est(idx):
        a = c = d = 0
        for i in idx: G, M = W[i]; a += (abs(G[sel]) ** 2).sum(); c += (np.conj(G[sel]) * M[sel]).sum(); d += (abs(M[sel]) ** 2).sum()
        h = c / a; return h, abs(c) ** 2 / (a * d)
    h, coh = est(range(len(W))); bs = [est(rng.integers(0, len(W), len(W)))[0] for _ in range(nboot)]; tau = lambda x: -np.angle(x) / (2 * np.pi * f0) * 1e3
    lo, hi = np.percentile([tau(x) for x in bs], [5, 95]); return dict(f=f0, H=h, gain=abs(h), tau=tau(h), tau_ci=(lo, hi), coh=coh)
def fit_filter(res):
    """least squares on the complex H over the bands: H = exp(-j 2 pi f D) / (1 + j 2 pi f tau_m); returns D (ms, = d - g_gyro) and tau_m (ms)"""
    f = np.array([r['f'] for r in res]); H = np.array([r['H'] for r in res]); best = None
    for tm in np.arange(0, 60.1, 1.0):
        for D in np.arange(-30, 30.1, 0.5):
            m = np.exp(-2j * np.pi * f * D * 1e-3) / (1 + 2j * np.pi * f * tm * 1e-3); c = float(np.sum(abs(H - m) ** 2))
            if best is None or c < best[0]: best = (c, D, tm)
    return best[1], best[2]
def analyse(D, axis=1, tmin=0.0, tmax=1e9):
    W = windows(D, axis, tmin, tmax); res = [band(W, b) for b in BANDS]; Dd, tm = fit_filter(res); return res, Dd, tm
def mocap_stream(D, d_ms, tau_m_ms, rng, rate_hz=100, noise_q=0.0, real_stats=None):
    """synthetic mocap quaternion stream from the true quaternion of a sim run: 100 Hz packets, latency d, first-order filter tau_m on the quaternion components, held onto the 500 Hz rows, packet ticks in ms"""
    t = D['t']; q = D['q_true']; step = int(round(500 / rate_hz)); idx = np.arange(0, len(t), step); tp = t[idx]; qp = q[idx].copy()
    if tau_m_ms > 0:
        a = 1 - np.exp(-(1 / rate_hz) / (tau_m_ms * 1e-3)); y = qp[0].copy()
        for k in range(len(qp)): y = y + a * (qp[k] - y); qp[k] = y
    if noise_q > 0: qp = qp + rng.normal(0, noise_q, qp.shape)   # per-component quaternion noise (0.0009 = 0.1 deg of rotation)
    if real_stats is not None:   # REPORT_49 R4: capture grid with the REAL dropouts (capture indices k_j of the packets that arrived) and the REAL arrival jitter delta_j (ms) of a raw uSD log, tiled to the run length
        kk, dd = np.asarray(real_stats['k'], float), np.asarray(real_stats['delta'], float); n_rep = int(np.ceil((t[-1] - t[0]) * rate_hz / max(kk.max(), 1))) + 1
        K = np.concatenate([kk + r * (kk.max() + 1) for r in range(n_rep)]); DJ = np.tile(dd, n_rep); sel_ = (K / rate_hz < t[-1] - t[0]); K = K[sel_]; DJ = DJ[sel_]
        tc = t[0] + K / rate_hz; qi = np.stack([np.interp(tc, t, q[:, c]) for c in range(4)], 1)
        if tau_m_ms > 0:
            a_ = 1 - np.exp(-(1 / rate_hz) / (tau_m_ms * 1e-3)); y = qi[0].copy()
            for k_ in range(len(qi)): y = y + a_ * (qi[k_] - y); qi[k_] = y
        if noise_q > 0: qi = qi + rng.normal(0, noise_q, qi.shape)
        qp = qi / np.linalg.norm(qi, axis=1)[:, None]; ta = tc + d_ms * 1e-3 + DJ * 1e-3; ta = np.maximum.accumulate(ta)
    else:
        qp = qp / np.linalg.norm(qp, axis=1)[:, None]; ta = tp + d_ms * 1e-3   # arrival time of each packet = capture + latency
    mq = np.zeros_like(q); tick = np.zeros(len(t)); j = -1; cur = qp[0]; ct = 0.0
    for i in range(len(t)):
        while j + 1 < len(ta) and ta[j + 1] <= t[i]: j += 1; cur = qp[j]; ct = ta[j] * 1e3
        mq[i] = cur; tick[i] = ct
    return dict(t=t, gyro=D['gyro'], mq=mq, mtick=tick)
