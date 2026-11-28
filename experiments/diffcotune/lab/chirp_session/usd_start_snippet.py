"""uSD start / stop snippet for chirp_flight.py (lab agent, 2026-11-28). NOT RUN, NOT FOR HARDWARE UNTIL THE USER DECIDES.

SOURCE (verified path used in every DiffCoTune flight, validated on ground + flight 2026-11-20):
  crazyswarm2 repo (laptop ~/Desktop/crazyswarm2, lab PC main), crazyflie_examples/crazyflie_examples/
    usd_gate.py         introduced in commit 42fc430 ("uSD start gate ..."), functions make_taps, precheck, start_and_verify, stop_and_verify
    run_formation.py    usage at HEAD fd1167f (post-set fix: runtime params forwarded from the post-set callback, validated 11-20):
                        taps l.556-558, precheck l.790-799, usec.reset on the ground l.804-821, start l.1019-1048,
                        stop l.1105-1108, final idempotent usd.logging 0 l.1112-1116
  The runtime sets themselves (allcfs.setParam) only work with the server binary that carries crazyswarm2 71219f3 / fd1167f
  (lab_run.sh stack_guard checks the marker CS2_RUNTIME_PARAMS_POST_SET_V1).
Order in a flight (same as run_formation): taps -> ground precheck (motors idle, refuses takeoff) -> usec.reset ON THE GROUND
-> takeoff + hover -> start_and_verify (hover, BEFORE the chirp) -> chirp -> land -> stop_and_verify -> finally usd.logging 0.
NEVER send usec.reset after takeoff.
Solo: cf_second is disabled in the yaml (lab_run.sh solo), so allcfs.crazyflies == [cf5]; names = ['cf5'].
Before the flight (outside this snippet): check_usd_deck.py <cf5 uri> (canLog 1), usd_card_count.py < 85 files.
"""
import json, time
from crazyflie_examples import usd_gate


def usd_setup(allcfs, th):
    """call right after Crazyswarm(); returns the context. Sends usec.reset on the GROUND; exits (no takeoff) if the precheck fails."""
    from crazyflie_interfaces.msg import LogDataGeneric
    cfs = allcfs.crazyflies
    names = [c.prefix.lstrip('/') for c in cfs]
    taps = usd_gate.make_taps(allcfs, names, LogDataGeneric)          # /<cf>/usd_status (yaml custom topic)
    th.sleep(2.0)
    pre = usd_gate.precheck(allcfs, names, taps, th.sleep, th.time)   # motors idle, nobody writing, topic alive
    print(f'[chirp] uSD precheck (ground): passed={pre["passed"]}')
    if not pre['passed']:
        raise SystemExit(f'[chirp] uSD precheck failed on the ground: NOT taking off ({pre.get("reason")})')
    allcfs.setParam('usec.reset', 1)                                  # GROUND ONLY, before takeoff
    th.sleep(0.2)
    return dict(allcfs=allcfs, cfs=cfs, names=names, taps=taps, th=th, started=False, result=None)


def usd_start(ctx):
    """in hover, before the chirp. Returns True if the deck is provably writing; False -> the caller must land (abort_sequence)."""
    th = ctx['th']
    run_tag = int(time.time())
    res = usd_gate.start_and_verify(ctx['allcfs'], ctx['cfs'], ctx['names'], ctx['taps'], run_tag, th.sleep, th.time, time.monotonic)
    ctx['result'] = res; ctx['started'] = True                        # started=True even on failure: the finally block sends usd.logging 0
    print(f'[chirp] uSD start passed={res["passed"]} run_tag={run_tag} {json.dumps(res, sort_keys=True)[:300]}')
    return bool(res['passed'])


def usd_stop(ctx):
    """after landing (report only, never raises)."""
    try:
        r = usd_gate.stop_and_verify(ctx['allcfs'], ctx['cfs'], ctx['names'], ctx['taps'], ctx['th'].sleep, ctx['th'].time)
        print(f'[chirp] uSD stop verified: {r.get("passed")}')
        return r
    except Exception as e:
        print(f'[chirp] uSD stop check skipped ({e})')


def usd_finally(ctx):
    """in the script's finally: every exit path closes the file (idempotent)."""
    if ctx and ctx.get('started'):
        try:
            ctx['allcfs'].setParam('usd.logging', 0)
        except Exception:
            pass
