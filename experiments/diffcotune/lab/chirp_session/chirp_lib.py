"""chirp_lib (REPORT_59 L3 / L4): analysis library of the chirp session. Everything works on a v3 uSD file decoded by decode_usd_log_v3 (real decoder) and resampled (zero-order hold, true time stamps) to a uniform 2.000 ms grid.
Pieces (all verified on frsim known answers, REPORT_48-58): E_full loop estimate (command break, chirp acceleration as instrument), gyro-vs-mocap timing, INDI split A / B / R, the 25-32 Hz line.
Axis convention: a y chirp excites ROLL (axis 0, instrument ctrltarget_ay), an x chirp PITCH (axis 1, instrument ctrltarget_ax)."""
import os, sys, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import decode_usd_log_v3 as DEC, rebuild as RB, q2_loop as Q, q2_timing as QT
from scipy.signal import welch, csd
DT = 0.002; VARS = json.load(open(os.path.join(HERE, 'v3_variables.json')))
AX = {'y': (0, 'ctrltarget_ay', 'x'), 'x': (1, 'ctrltarget_ax', 'y')}   # chirp axis -> (attitude axis index, instrument column, suffix of the omar columns)
def load(path, quiet=True):
    import io, contextlib
    with contextlib.redirect_stderr(io.StringIO()) if quiet else contextlib.nullcontext(): d = DEC.load(path)
    t = np.asarray(d['t'], float); tg = np.arange(t[0], t[-1], DT); idx = np.clip(np.searchsorted(t, tg, 'right') - 1, 0, len(t) - 1)
    D = {k: np.asarray(v)[idx] for k, v in d.items() if k != 't'}; D['t'] = tg - tg[0]; D['_raw_t'] = t; D['_raw_n'] = len(t); D['_names'] = [k for k in d if k != 't']; return D
def chirp_window(D, ax, thr_frac=0.05):
    ins = D[AX[ax][1]]; peak = np.nanmax(np.abs(ins)); idx = np.flatnonzero(np.abs(ins) > thr_frac * max(peak, 1e-9))
    return (D['t'][idx[0]], D['t'][idx[-1]], peak) if len(idx) else (0.0, 0.0, 0.0)
def signals(D, ax):
    k = AX[ax][2]; a = AX[ax][0]
    KR = D['omar_KR_' + k]; KW = D['omar_Kw_' + k]; y = D['omar_rpy_' + k]; w = np.radians(D['gyro_' + k]); ind = D['omar_tau_rpm_f' + k] - D['omar_tau_gyro_f' + k]; U = D['omar_tau_' + k]
    return dict(fb=KR * y + KW * w - ind, U=U, ins=D[AX[ax][1]], trf=D['omar_tau_rpm_f' + k], tgf=D['omar_tau_gyro_f' + k], KR=KR, KW=KW)
def _windows(D, ax, t0, t1, fields):
    S = signals(D, ax); out = []; n = len(D['t'])
    for s in range(0, n - Q.WIN, Q.WIN):
        if D['t'][s] < t0 + 1.0 or D['t'][min(s + Q.WIN, n - 1)] > t1: continue
        sl = slice(s + Q.SK, s + Q.WIN); out.append(tuple(Q.spec(S[f][sl]) for f in fields))
    return out
def _band_ratio(W, num, den, ins, bands, idx=None):
    idx = range(len(W)) if idx is None else idx; res = []
    for b in bands:
        sel = (Q.fr >= b[0]) & (Q.fr < b[1]); n = d = 0
        for i in idx: w = W[i]; n += (np.conj(w[ins][sel]) * w[num][sel]).sum(); d += (np.conj(w[ins][sel]) * w[den][sel]).sum()
        res.append((Q.fr[sel].mean(), n / d))
    return res
def efull(D, ax, nboot=200, seed=1):
    """E_full: L = (KR y + Kw w - indi) / U with the chirp acceleration as instrument. Returns bands (f, gain dB, phase deg), Ms / PM / GM and 5-95 % bootstrap intervals over windows"""
    t0, t1, peak = chirp_window(D, ax); W = _windows(D, ax, t0, t1, ('fb', 'U', 'ins'))
    if len(W) < 5: return dict(ok=False, reason='only %d chirp windows' % len(W))
    L = _band_ratio(W, 0, 1, 2, Q.BANDS); M = Q.metrics([(f, l, 0, 0) for f, l in L]); rng = np.random.default_rng(seed); bs = []
    for _ in range(nboot):
        idx = rng.integers(0, len(W), len(W)); Lb = _band_ratio(W, 0, 1, 2, Q.BANDS, idx); Mb = Q.metrics([(f, l, 0, 0) for f, l in Lb]); bs.append([Mb['Ms_db'], np.nan if Mb['pm'] is None else Mb['pm'], np.nan if Mb['gm_db'] is None else Mb['gm_db'], Mb['gain35_db'], Mb['phase35']])
    bs = np.array(bs); ci = {k: [float(np.nanpercentile(bs[:, i], 5)), float(np.nanpercentile(bs[:, i], 95))] for i, k in enumerate(('Ms_db', 'pm', 'gm_db', 'gain35_db', 'phase35'))}
    return dict(ok=True, n_windows=len(W), peak_acc=float(peak), f=[float(x[0]) for x in L], L_re=[float(x[1].real) for x in L], L_im=[float(x[1].imag) for x in L], gain_db=[float(20 * np.log10(abs(x[1]))) for x in L], phase_deg=[float(np.degrees(np.angle(x[1]))) for x in L],
                Ms_db=float(M['Ms_db']), Ms_f=float(M['Ms_f']), pm=None if M['pm'] is None else float(M['pm']), gm_db=None if M['gm_db'] is None else float(M['gm_db']), gain35_db=float(M['gain35_db']), phase35=float(M['phase35']), ci=ci)
def timing(D, ax):
    """gyro delay minus mocap latency (ms) and the mocap filter time constant (ms) from the gyro vs the mocap quaternion (REPORT_48 / 49); windows with packet gaps > 25 ms are dropped"""
    a = AX[ax][0]; t0, t1, _ = chirp_window(D, ax); need = ('locsrv_qx', 'locsrv_qy', 'locsrv_qz', 'locsrv_qw')
    if not all(k in D for k in need): return dict(ok=False, reason='quaternion not logged')
    E = dict(t=D['t'], gyro=np.stack([D['gyro_x'], D['gyro_y'], D['gyro_z']], 1), mq=np.stack([D[k] for k in need], 1), mtick=D.get('locsrv_tick'))
    try: res, Dd, tm = QT.analyse(E, axis=a, tmin=t0 + 1.0, tmax=t1)
    except Exception as e: return dict(ok=False, reason='timing failed: %s' % e)
    return dict(ok=True, D_ms=float(Dd), tau_m_ms=float(tm), coh=[float(r['coh']) for r in res], tau_band_ms=[float(r['tau']) for r in res], f=[float(r['f']) for r in res], gain=[float(r['gain']) for r in res])
def split(D, ax, bands=((1, 2), (2, 3), (3, 4), (4, 5))):
    """INDI split (REPORT_56): A = tau_rpm_f/u, B = tau_gyro_f/u (instrument-free cross-spectra, biased up by about 0.1 at 1-3 Hz), R = tau_rpm_f / tau_gyro_f (u-free, unbiased)"""
    t0, t1, _ = chirp_window(D, ax); S = signals(D, ax); m = (D['t'] >= t0 + 1.0) & (D['t'] <= t1); u, tr, tg = S['U'][m], S['trf'][m], S['tgf'][m]; nper = 1024
    cs = lambda x, y: csd(x - x.mean(), y - y.mean(), fs=500.0, nperseg=nper, noverlap=nper // 2)[1]; f, Suu = welch(u - u.mean(), fs=500.0, nperseg=nper, noverlap=nper // 2); Sru, Sgu, Srg = cs(u, tr), cs(u, tg), cs(tg, tr); Sgg = welch(tg - tg.mean(), fs=500.0, nperseg=nper, noverlap=nper // 2)[1]; out = []
    for a, b in bands:
        s = (f >= a) & (f < b); out.append(dict(band=[a, b], A=complex(Sru[s].sum() / Suu[s].sum()), B=complex(Sgu[s].sum() / Suu[s].sum()), R=complex(Srg[s].sum() / Sgg[s].sum())))
    return [dict(band=o['band'], A_mag=abs(o['A']), A_ph=float(np.degrees(np.angle(o['A']))), B_mag=abs(o['B']), B_ph=float(np.degrees(np.angle(o['B']))), R_mag=abs(o['R']), R_ph=float(np.degrees(np.angle(o['R']))), C_mag=abs(o['A'] - o['B'])) for o in out]
def line(D, ax='y'):
    """25-32 Hz line of the INDI correction loop: rpm roll differential (raw DShot after the firmware spike rules), peak 15-60 Hz, strength = peak / median PSD 100-200 Hz, half-power width and Q"""
    rp = np.stack([RB.rpm_filter(D['motor_m%d_rpm' % i]) for i in (1, 2, 3, 4)], 1)
    x = (-rp[:, 0] - rp[:, 1] + rp[:, 2] + rp[:, 3]) if ax == 'y' else (-rp[:, 0] + rp[:, 1] + rp[:, 2] - rp[:, 3]); t0, t1, _ = chirp_window(D, ax); m = (D['t'] >= t0) & (D['t'] <= t1); x = x[m]
    f, P = welch(x - x.mean(), fs=500.0, nperseg=8192, noverlap=4096); s = (f >= 15) & (f <= 60); i = int(np.argmax(np.where(s, P, 0))); pk = P[i]; fl = np.median(P[(f >= 100) & (f <= 200)]); j = i
    while j > 0 and P[j] > pk / 2: j -= 1
    k = i
    while k < len(P) - 1 and P[k] > pk / 2: k += 1
    return dict(f0=float(f[i]), strength=float(pk / fl), width=float(f[k] - f[j]), Q=float(f[i] / max(f[k] - f[j], 1e-9)))
