#!/usr/bin/env python3
"""chirp_flight.py (REPORT_61: matches the lab run sheet chirp_session/RUN_SHEET_DRAFT.md v3 sections 0, 2, 3 and 5). Streams a lateral acceleration chirp as a full-state setpoint (cf.cmdFullState, 100 Hz) in a SOLO hover of cf5, logs through the verified uSD gate, aborts safely.

LOCATION / LAUNCH (run sheet section 5 row 9): this file lives in  experiments/diffcotune/lab/chirp_session/chirp_flight.py  and is started with  python3 chirp_flight.py ...  from a FOREGROUND terminal (the abort keys need a TTY; the script refuses to fly without one). No change to crazyswarm2.
ENVIRONMENT it needs: the lab PC workspace sourced (ROS 2 + the built crazyswarm2 packages: crazyflie_py, crazyflie_interfaces, crazyflie_examples with usd_gate.py of commit 42fc430) ; the CS2 server running SOLO (lab_run.sh solo S1 <l>: cf_second disabled, so allcfs.crazyflies == [cf5]) with the server binary that carries crazyswarm2 71219f3 / fd1167f (runtime params post-set fix, marker CS2_RUNTIME_PARAMS_POST_SET_V1); numpy; the gains of the flight already applied and read back (lab_run.sh prep); cf5's card with config v3 and the flashed patched image; battery fresh, rest >= 4.1 V, swap if < 4.0 V (S0_LAB_CARD row 6; NO in-flight voltage abort exists in the protocol and none is added).
USAGE:  python3 chirp_flight.py --height 0.6 --amp 0.3 --flight F1          (F1 .. F6; amplitude ramp 0.1 -> 0.3 m/s2 over the first 20 s of each chirp, then 0.3)
        python3 chirp_flight.py --ground --T 20 --flight G2                 (G2: stream on the ground, clamped, props off; no takeoff)
        python3 chirp_flight.py --g7 --flight G7                            (G7: tethered, hover 0.35 m, 0.1 m/s2 flat, y chirp 20 s only)
ORDER (same as run_formation): TTY check -> uSD taps -> ground precheck (motors idle; refuses takeoff) -> usec.reset ON THE GROUND -> takeoff + hover -> z floor check -> uSD start_and_verify IN HOVER BEFORE THE CHIRP -> hold -> y chirp -> rest -> x chirp -> hold -> land -> uSD stop_and_verify -> finally usd.logging 0. NEVER usec.reset after takeoff.
KEYS (Enter after the key): q = land now, e = emergency stop (motors off, the drone falls; last resort).
ABORT triggers: key q / e; telemetry tilt > 15 deg or |rate| > 300 deg/s for > 0.3 s; telemetry silent > 0.5 s; position excursion > 0.3 m from the hover point (session safety limit, user-approved 2026-11-28); z < 0.25 m after the takeoff wait (protocol floor rule); uSD start not verified. Sequence: hold burst at the CURRENT position (0.3 s, 100 Hz) -> notifySetpointsStop -> HLC land; last resort cf.emergency().
Path (verified in the code): cf.cmdFullState -> /cfX/cmd_full_state -> crazyflie_server -> cflib send_full_state_setpoint (int16 mm, mm/s, mm/s2) -> firmware fullStateDecoder (position / velocity / acceleration; jerk = snap = 0) -> Omar INDI controller."""
import os, sys, time, json, argparse, threading
import numpy as np
try:
    from crazyflie_py import Crazyswarm
except ImportError:   # mock tests / analysis PCs
    Crazyswarm = None
try:
    from crazyflie_interfaces.msg import LogDataGeneric   # <cf>/attitude = [stabilizer.roll, pitch, yaw, thrust, pm.vbat] (deg), <cf>/gyro_acc = [gyro.x, y, z (deg/s), acc.x, y, z]
except ImportError:
    LogDataGeneric = None
RATE_HZ = 100.0
LIM_TILT_DEG = 15.0; LIM_RATE_DPS = 300.0; LIM_WINDOW_S = 0.3; STALE_S = 1e9;   # LAB 2026-11-28 (user order): telemetry-silent abort disabled (radio CSVs are not the dataset; usual flights have none; F1 tripped on it). Was 0.5 s.
LIM_EXCURSION_M = 0.3; Z_FLOOR_M = 0.25
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')

def chirp_triple(tau, T, f0, f1, amp0, amp, ramp, taper):
    """position, velocity, acceleration of the chirp on the time grid tau (s): log sweep f0 -> f1 over T, smooth envelope, acceleration amplitude amp0 -> amp linearly over the first `ramp` s (ramp None / 0 = flat amp); position = -env A(t) / w(t)^2 sin(phase), v and a its numerical derivatives (the consistent triple used in frsim)"""
    r = f1 / f0; ph = 2 * np.pi * f0 * T / np.log(r) * (r ** (tau / T) - 1); env = np.clip(np.minimum(tau, T - tau) / taper, 0, 1); env = 0.5 - 0.5 * np.cos(np.pi * env)
    A = np.full_like(tau, amp) if not ramp else amp0 + (amp - amp0) * np.clip(tau / ramp, 0.0, 1.0)
    fi = f0 * r ** (tau / T); p = -env * A / (2 * np.pi * fi) ** 2 * np.sin(ph); v = np.gradient(p, tau); a = np.gradient(v, tau); return p, v, a

class Monitor:
    """Abort sources: keyboard, radio telemetry (tilt / rate over the limit for > LIM_WINDOW_S; silent > STALE_S), pose (excursion, z floor). Callbacks run inside th.sleep / sleepForRate (crazyflie_py spins the node there), i.e. in the main thread."""
    def __init__(self, node, name, clock=time.time):
        self.clock = clock; self.abort = False; self.emergency = False; self.reason = ''; self.roll = self.pitch = 0.0; self.gyro = np.zeros(3); self.over = None; self.bad_now = False
        self.t_abort = None; self.last_msg = clock(); self.armed = False; self.pos0 = None; self.floor_armed = False
        if node is not None and LogDataGeneric is not None:
            node.create_subscription(LogDataGeneric, f'{name}/attitude', self._att, 10); node.create_subscription(LogDataGeneric, f'{name}/gyro_acc', self._gyro, 10)
        threading.Thread(target=self._keys, daemon=True).start()
    def _trip(self, reason, emergency=False):
        if not self.abort: self.abort = True; self.reason = reason; self.t_abort = self.clock()
        if emergency: self.emergency = True
    def _keys(self):
        for line in sys.stdin:
            k = line.strip().lower()
            if k == 'q': self._trip('operator key (land)')
            if k == 'e': self._trip('operator key (EMERGENCY STOP)', True); return
    def _att(self, msg): self.roll, self.pitch = msg.values[0], msg.values[1]; self.last_msg = self.clock(); self._check()
    def _gyro(self, msg): self.gyro = np.array(msg.values[:3]); self.last_msg = self.clock(); self._check()
    def _check(self):
        self.bad_now = max(abs(self.roll), abs(self.pitch)) > LIM_TILT_DEG or np.max(np.abs(self.gyro)) > LIM_RATE_DPS; now = self.clock()
        if self.bad_now:
            self.over = now if self.over is None else self.over
            if now - self.over > LIM_WINDOW_S: self._trip('tilt %.1f / %.1f deg, rate %.0f deg/s over the limit for > %.1f s' % (self.roll, self.pitch, np.max(np.abs(self.gyro)), LIM_WINDOW_S))
        else: self.over = None
    def poll(self, pos=None):
        """time-based and pose checks (called from every loop)"""
        if self.armed and self.clock() - self.last_msg > STALE_S: self._trip('telemetry silent for > %.1f s' % STALE_S)
        if pos is not None and self.floor_armed and pos[2] < Z_FLOOR_M: self._trip('z %.2f m below the floor %.2f m after takeoff' % (pos[2], Z_FLOOR_M))
        if pos is not None and self.pos0 is not None and np.linalg.norm(np.asarray(pos, float) - self.pos0) > LIM_EXCURSION_M: self._trip('position excursion %.2f m > %.2f m from the hover point' % (np.linalg.norm(np.asarray(pos, float) - self.pos0), LIM_EXCURSION_M))

# ---- uSD gate (usd_start_snippet.py of the lab agent; source crazyswarm2 usd_gate.py 42fc430, run_formation.py fd1167f) ----
def usd_setup(allcfs, th):
    """right after Crazyswarm(): taps, ground precheck (motors idle, nobody writing, topic alive), usec.reset ON THE GROUND. Exits (no takeoff) if the precheck fails."""
    from crazyflie_examples import usd_gate
    cfs = allcfs.crazyflies; names = [c.prefix.lstrip('/') for c in cfs]
    taps = usd_gate.make_taps(allcfs, names, LogDataGeneric); th.sleep(2.0)
    pre = usd_gate.precheck(allcfs, names, taps, th.sleep, th.time); print(f'[chirp] uSD precheck (ground): passed={pre["passed"]}')
    if not pre['passed']: raise SystemExit(f'[chirp] uSD precheck failed on the ground: NOT taking off ({pre.get("reason")})')
    allcfs.setParam('usec.reset', 1); th.sleep(0.2)                                  # GROUND ONLY, before takeoff
    return dict(allcfs=allcfs, cfs=cfs, names=names, taps=taps, th=th, started=False, result=None, usd_gate=usd_gate)
def usd_start(ctx):
    """in hover, before the chirp; True if the deck is provably writing, False -> the caller lands"""
    th = ctx['th']; run_tag = int(time.time()); res = ctx['usd_gate'].start_and_verify(ctx['allcfs'], ctx['cfs'], ctx['names'], ctx['taps'], run_tag, th.sleep, th.time, time.monotonic)
    ctx['result'] = res; ctx['started'] = True                                       # started=True even on failure: the finally block sends usd.logging 0
    print(f'[chirp] uSD start passed={res["passed"]} run_tag={run_tag} {json.dumps(res, sort_keys=True)[:300]}'); return bool(res['passed'])
def usd_stop(ctx):
    """after landing (report only, never raises)"""
    try: r = ctx['usd_gate'].stop_and_verify(ctx['allcfs'], ctx['cfs'], ctx['names'], ctx['taps'], ctx['th'].sleep, ctx['th'].time); print(f'[chirp] uSD stop verified: {r.get("passed")}'); return r
    except Exception as e: print(f'[chirp] uSD stop check skipped ({e})')
def usd_finally(ctx):
    """in the script's finally: every exit path closes the file (idempotent)"""
    if ctx and ctx.get('started'):
        for _ in range(3):   # LAB 2026-11-28: resend the close 3x (an unclosed file = 0 B on the card); then tell the operator when power-off is safe
            try: ctx['allcfs'].setParam('usd.logging', 0); ctx['th'].sleep(0.5)
            except Exception: pass
    print('[chirp] DONE - uSD file closed. Wait 3 s, then power cf5 off.')

def asleep(th, mon, dur, cf=None):
    """abortable sleep"""
    t0 = th.time()
    while th.time() - t0 < dur and not mon.abort and not th.isShutdown(): th.sleep(0.02); mon.poll(None if cf is None else cf.position)

def stream(cf, th, mon, pos0, axis, T, f0, f1, amp0, amp, ramp, rest, log, flight):
    """stream one chirp (T s; amp 0 = pure hold) then hold for `rest` s; the triple is precomputed on a 1 kHz grid and indexed by time (as in frsim)"""
    grid = np.arange(0.0, max(T, 0.001), 0.001); p, v, a = chirp_triple(grid, T, f0, f1, amp0, amp, ramp, 2.0) if amp > 0 and T > 0 else (np.zeros_like(grid),) * 3
    t_start = th.time(); n = 0
    while not th.isShutdown() and not mon.abort:
        mon.poll(cf.position); t = th.time() - t_start
        if t > T + rest: break
        k = min(int(t * 1000), len(grid) - 1) if t <= T else None
        pos = pos0.copy(); vel = np.zeros(3); acc = np.zeros(3)
        if k is not None: pos[axis] += p[k]; vel[axis] = v[k]; acc[axis] = a[k]
        cf.cmdFullState(pos, vel, acc, 0.0, np.zeros(3)); log.append((th.time(), axis, *pos, *vel, *acc)); n += 1
        th.sleepForRate(RATE_HZ)
    return n

def stream_land(cf, th, xy_pos, z_start):
    """LAB 2026-11-28: streamed landing, copied from crazyswarm2 flight.py _onboard_stream_land (fd1167f). After cmdFullState the firmware
    stays in low-level mode, so HLC land() is ignored and the setpoint watchdog then cuts the motors in the air (documented 2026-09-09; F1 today).
    Keep streaming, chase the measured height down at 0.20 m/s, disarm via z only on the ground, then hand back with notifySetpointsStop."""
    zero3 = np.zeros(3); xy = np.array(xy_pos[:2], float); z_meas = float(z_start); t0 = th.time(); max_s = max(z_meas - 0.12, 0.0) / 0.20 + 8.0
    print(f'[chirp] streamed landing from z={z_meas:.2f} m')
    while not th.isShutdown() and th.time() - t0 < max_s:
        p = np.asarray(cf.position, float); z_meas = float(p[2]) if np.any(p) else z_meas
        if z_meas <= 0.12: break
        cf.cmdFullState(np.array([xy[0], xy[1], max(z_meas - 0.04, 0.07)]), np.array([0.0, 0.0, -0.20]), zero3, 0.0, zero3); th.sleepForRate(20.0)
    t1 = th.time()
    while not th.isShutdown() and th.time() - t1 < 0.5: cf.cmdFullState(np.array([xy[0], xy[1], 0.02]), zero3, zero3, 0.0, zero3); th.sleepForRate(20.0)
    th.sleep(0.2); cf.notifySetpointsStop(); th.sleep(0.3); cf.arm(False); print('[chirp] landed, disarmed')

def abort_sequence(cf, th, mon, height, log, ground=False):
    """emergency key -> cf.emergency() FIRST; otherwise (1) hold burst at the CURRENT position (0.3 s, 100 Hz; one packet already stops the chirp by the zero-order hold), (2) notifySetpointsStop, (3) HLC land, (4) last resort: 'e' key or limits still exceeded 1.5 s after the land command -> cf.emergency()"""
    log.append((th.time(), -1, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    if mon.emergency: print('[chirp] EMERGENCY STOP'); cf.emergency(); return
    cur = np.array(cf.position, float) if np.any(np.asarray(cf.position)) else mon.pos0
    if cur is not None and not ground:
        for _ in range(30): cf.cmdFullState(cur, np.zeros(3), np.zeros(3), 0.0, np.zeros(3)); th.sleepForRate(RATE_HZ)
    if ground: return
    if cur is None: cur = np.array([0.0, 0.0, height])
    stream_land(cf, th, cur, cur[2] if cur[2] > 0 else height)

def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0]); ap.add_argument('--flight', default='FX'); ap.add_argument('--height', type=float, default=0.6); ap.add_argument('--amp', type=float, default=0.3); ap.add_argument('--amp0', type=float, default=0.1); ap.add_argument('--ramp', type=float, default=20.0)
    ap.add_argument('--T', type=float, default=60.0); ap.add_argument('--f0', type=float, default=1.0); ap.add_argument('--f1', type=float, default=8.0); ap.add_argument('--axes', default='yx'); ap.add_argument('--hold', type=float, default=10.0); ap.add_argument('--rest', type=float, default=5.0)
    ap.add_argument('--takeoff-wait', type=float, default=4.0); ap.add_argument('--out', default=OUT_DIR); ap.add_argument('--ground', action='store_true', help='G2: stream on the ground (clamped, props off), no takeoff, no land'); ap.add_argument('--g7', action='store_true', help='G7: tethered, 0.35 m, 0.1 m/s2 flat, y chirp 20 s only')
    ap.add_argument('--no-usd', action='store_true', help='bench / mock tests only: no uSD gate (no file will exist)'); a = ap.parse_args(argv)
    if a.g7: a.height = 0.35 if a.height == 0.6 else a.height; a.amp = a.amp0 = 0.1; a.ramp = 0.0; a.T = 20.0; a.axes = 'y'
    if a.ground: a.axes = 'y'
    return a

def main(argv=None):
    a = parse(argv)
    if not sys.stdin.isatty() and os.environ.get('CHIRP_ALLOW_NO_TTY') != '1_MOCK_TEST_ONLY':
        sys.exit('[chirp] refusing to fly: stdin is not a terminal (the abort keys q / e need a foreground terminal)')
    print('[chirp] battery: fresh pack, rest >= 4.1 V, swap if < 4.0 V (checked on the pad); no in-flight voltage abort. flight %s, height %.2f m, amplitude %.2f -> %.2f m/s2 (ramp %.0f s), axes %s' % (a.flight, a.height, a.amp0, a.amp, a.ramp, a.axes))
    swarm = Crazyswarm(); th = swarm.timeHelper; cf = swarm.allcfs.crazyflies[0]; mon = Monitor(swarm.allcfs, cf.prefix); log = []; ctx = None; os.makedirs(a.out, exist_ok=True); pos0 = None
    try:
        if not a.no_usd: ctx = usd_setup(swarm.allcfs, th)
        if a.ground: pos0 = np.array(cf.position, float); mon.armed = False
        else:
            # LAB FIX 2026-11-28 (lab agent, user order): same as run_formation.py (fd1167f): EKF reset on the ground (estimator 1 -> 2, resetEstimation) and cf.arm(True) BEFORE takeoff. Without arm(True) the brushless cf5 silently never takes off (G7 attempt: z 0.05 m -> floor abort).
            cf.setParam('stabilizer.estimator', 1); th.sleep(1.0); cf.setParam('stabilizer.estimator', 2); th.sleep(0.2)
            cf.setParam('kalman.resetEstimation', 1); th.sleep(0.2); cf.setParam('kalman.resetEstimation', 0); th.sleep(3.0)
            cf.arm(True); th.sleep(0.5)
            cf.takeoff(targetHeight=a.height, duration=3.0); asleep(th, mon, a.takeoff_wait, cf); mon.armed = True; mon.last_msg = mon.clock()
            pos0 = np.array(cf.position, float)
            if not np.any(pos0): pos0 = np.array(cf.initialPosition, float) + np.array([0.0, 0.0, a.height])
            mon.pos0 = pos0.copy(); mon.floor_armed = True; mon.poll(pos0)                # z floor check right after the takeoff wait
        if not mon.abort and ctx is not None and not usd_start(ctx): mon._trip('uSD start not verified')
        if not mon.abort:
            stream(cf, th, mon, pos0, 1, 0.0, a.f0, a.f1, 0.0, 0.0, 0.0, a.hold, log, a.flight)          # hold by streaming (keeps the low-level mode)
            for ax in a.axes:
                if mon.abort: break
                stream(cf, th, mon, pos0, 1 if ax == 'y' else 0, a.T, a.f0, a.f1, a.amp0, a.amp, a.ramp or None, 5.0 if ax != a.axes[-1] else a.hold, log, a.flight)
        if not mon.abort and not a.ground:                                                   # normal end: hold burst not needed, land
            p_now = np.asarray(cf.position, float); stream_land(cf, th, pos0, float(p_now[2]) if np.any(p_now) else a.height)
    finally:
        if mon.abort: print('[chirp] ABORT:', mon.reason)
        try:
            if mon.abort: abort_sequence(cf, th, mon, a.height, log, a.ground)
        finally:
            if ctx is not None and not mon.emergency: usd_stop(ctx)
            usd_finally(ctx)
            if log: np.savetxt(os.path.join(a.out, '%s_setpoints_%d.csv' % (a.flight, int(time.time()))), np.array(log), delimiter=',', header='t,axis,px,py,pz,vx,vy,vz,ax,ay,az')
if __name__ == '__main__':
    main()
