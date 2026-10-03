#!/usr/bin/env python3
"""Read onboard RNN evaluation timing via radio log (rnn.us_last / us_max / us_avg).

Stop CS2 (or any node publishing log blocks) before connecting — same rule as param writes.

Usage:
  python3 flying_drone_stack/tools/read_rnn_timing.py radio://0/80/2M/E7E7E7BB02
  python3 flying_drone_stack/tools/read_rnn_timing.py --mock-self-test
  python3 flying_drone_stack/tools/read_rnn_timing.py --mock-self-test-capture

Requires flash-RNN firmware with rnn.us_* in the log TOC (bench yaml firmware_logging).
`rnn.us_max` / `rnn.us_avg` are log variables only — not params.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from dataclasses import dataclass, field
from typing import Callable

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig, LogTocElement
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.toc import Toc

PASS_PEAK_US = 300


@dataclass
class TimingSamples:
    us_last: list[int] = field(default_factory=list)
    us_max_logged: list[int] = field(default_factory=list)
    us_avg_logged: list[float] = field(default_factory=list)
    rnn_div: str | None = None


def log_has_var(cf, complete_name: str) -> bool:
    return cf.log.toc.get_element_by_complete_name(complete_name) is not None


def p99(values: list[int]) -> float:
    if not values:
        return float("nan")
    s = sorted(values)
    idx = min(len(s) - 1, int(round(0.99 * (len(s) - 1))))
    return float(s[idx])


def evaluate(
    samples: TimingSamples,
    *,
    fail_above_us: int = 1000,
) -> tuple[int, list[str]]:
    """Return (exit_code, lines to print). 0 = PASS timing gate."""
    lines: list[str] = []
    if samples.rnn_div is not None:
        lines.append(f"rnn.div (param)      = {samples.rnn_div}")

    if not samples.us_last:
        lines.append(
            "FAIL: no rnn.us_last samples — CS2 still running, missing firmware_logging, "
            "or not the 100 Hz+timing build."
        )
        return 1, lines

    peak_last = max(samples.us_last)
    mean_last = statistics.mean(samples.us_last)
    p99_last = p99(samples.us_last)
    peak_max = max(samples.us_max_logged) if samples.us_max_logged else peak_last
    peak = max(peak_last, peak_max)
    last_avg = samples.us_avg_logged[-1] if samples.us_avg_logged else float("nan")

    lines.append(f"rnn.us_last samples  = {len(samples.us_last)}")
    lines.append(
        f"  mean = {mean_last:.1f} µs  max = {peak_last} µs  p99 = {p99_last:.1f} µs"
    )
    if samples.us_max_logged:
        lines.append(f"logged us_max peak  = {peak_max} µs")
    if samples.us_avg_logged:
        lines.append(f"last logged us_avg  = {last_avg:.1f} µs")

    if all(x == 0 for x in samples.us_last):
        lines.append(
            "FAIL: all rnn.us_last samples are 0 — network not running (never treat as pass)."
        )
        return 1, lines

    if peak > fail_above_us:
        lines.append(
            f"FAIL: peak timing {peak} µs exceeds --fail-above-us {fail_above_us} µs."
        )
        return 1, lines

    if peak < PASS_PEAK_US:
        lines.append(f"PASS: peak {peak} µs < {PASS_PEAK_US} µs (bench target).")
    else:
        lines.append(
            f"PASS: peak {peak} µs within limit (< {fail_above_us} µs; "
            f"ideal < {PASS_PEAK_US} µs)."
        )
    return 0, lines


def _build_log_config(period_ms: int, on_log) -> LogConfig:
    lg = LogConfig(name="rnn_timing", period_in_ms=period_ms)
    lg.add_variable("rnn.us_last", "uint16_t")
    lg.add_variable("rnn.us_max", "uint16_t")
    lg.add_variable("rnn.us_avg", "float")
    lg.data_received_cb.add_callback(on_log)
    return lg


def capture_from_connected_cf(
    cf,
    seconds: float,
    period_ms: int,
    *,
    idle_fn: Callable[[LogConfig], None] | None = None,
) -> TimingSamples:
    """Collect timing samples from an already-connected Crazyflie (or test double)."""
    out = TimingSamples()

    def on_log(_ts, data, _log):
        if "rnn.us_last" in data:
            out.us_last.append(int(data["rnn.us_last"]))
        if "rnn.us_max" in data:
            out.us_max_logged.append(int(data["rnn.us_max"]))
        if "rnn.us_avg" in data:
            out.us_avg_logged.append(float(data["rnn.us_avg"]))

    if not log_has_var(cf, "rnn.us_last"):
        raise RuntimeError(
            "rnn.us_last not in log TOC — this is not the 100 Hz+timing build. "
            "Flash build_artifacts/cf21bl_rnn_100hz.bin (laptop), power-cycle, "
            "and add rnn.us_* to crazyflies.yaml firmware_logging for bench."
        )

    lg = _build_log_config(period_ms, on_log)
    cf.log.add_config(lg)
    if getattr(cf.log, "_is_fake", False):
        lg.start = lambda: None  # type: ignore[method-assign, assignment]
        lg.stop = lambda: None  # type: ignore[method-assign, assignment]

    lg.start()
    try:
        if idle_fn is not None:
            idle_fn(lg)
        else:
            time.sleep(seconds)
    finally:
        lg.stop()

    try:
        out.rnn_div = cf.param.get_value("rnn.div")
    except Exception:
        out.rnn_div = "(param read failed)"
    return out


def capture(uri: str, seconds: float, period_ms: int) -> TimingSamples:
    cflib.crtp.init_drivers()
    print(f"Connecting to {uri} (CS2 should be stopped)...")
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        return capture_from_connected_cf(scf.cf, seconds, period_ms)


def _add_toc_var(toc: Toc, group: str, name: str, ident: int, ctype: str) -> None:
    el = LogTocElement(ident=ident)
    el.group = group
    el.name = name
    el.ctype = ctype
    toc.add_element(el)


def _make_fake_cf(*, with_rnn_toc: bool):
    """Minimal Crazyflie stand-in for capture-path tests."""

    class FakeLog:
        _is_fake = True

        def __init__(self):
            self.toc = Toc()
            if with_rnn_toc:
                _add_toc_var(self.toc, "rnn", "us_last", 1, "uint16_t")
                _add_toc_var(self.toc, "rnn", "us_max", 2, "uint16_t")
                _add_toc_var(self.toc, "rnn", "us_avg", 3, "float")
            self.log_blocks = []

        def add_config(self, logconf):
            logconf.cf = self._cf
            logconf.valid = True
            logconf.id = 1
            self.log_blocks.append(logconf)

    class FakeParam:
        def get_value(self, _name):
            return "10"

    cf = type("FakeCF", (), {})()
    cf.log = FakeLog()
    cf.log._cf = cf
    cf.param = FakeParam()
    return cf


def run_mock_self_test() -> int:
    """Exercise evaluate() exit paths without hardware."""
    cases: list[tuple[str, TimingSamples, int, int]] = [
        (
            "normal samples → exit 0",
            TimingSamples(
                us_last=[120, 130, 125, 140],
                us_max_logged=[150, 160],
                us_avg_logged=[125.0, 128.0],
                rnn_div="10",
            ),
            1000,
            0,
        ),
        (
            "all us_last zero → exit 1",
            TimingSamples(us_last=[0, 0, 0], us_max_logged=[0], rnn_div="10"),
            1000,
            1,
        ),
        (
            "no samples → exit 1",
            TimingSamples(),
            1000,
            1,
        ),
        (
            "peak > fail-above-us → exit 1",
            TimingSamples(us_last=[500, 1200, 800], us_max_logged=[1500], rnn_div="10"),
            1000,
            1,
        ),
    ]
    worst = 0
    for label, samples, fail_above, expect in cases:
        code, lines = evaluate(samples, fail_above_us=fail_above)
        print(f"=== mock evaluate: {label} (expect exit {expect}) ===")
        for ln in lines:
            print(ln)
        print(f"exit_code={code}")
        print()
        if code != expect:
            print(f"UNEXPECTED exit {code} != {expect}", file=sys.stderr)
            worst = 1
    return worst


def _fire_log(lg: LogConfig, payload: dict) -> None:
    lg.data_received_cb.call(0, payload, lg)


def run_mock_self_test_capture() -> int:
    """Exercise capture_from_connected_cf + main-style exit codes (no radio)."""
    worst = 0

    # (a) TOC present + normal samples → exit 0
    cf = _make_fake_cf(with_rnn_toc=True)

    def idle_ok(lg):
        _fire_log(lg, {"rnn.us_last": 140, "rnn.us_max": 160, "rnn.us_avg": 128.0})
        _fire_log(lg, {"rnn.us_last": 125, "rnn.us_max": 150, "rnn.us_avg": 126.0})

    samples = capture_from_connected_cf(cf, 0.0, 10, idle_fn=idle_ok)
    code, lines = evaluate(samples, fail_above_us=1000)
    print("=== mock capture: TOC ok + normal samples (expect exit 0) ===")
    for ln in lines:
        print(ln)
    print(f"exit_code={code}")
    print()
    if code != 0:
        worst = 1

    # (b) TOC missing → exit 2
    cf_bad = _make_fake_cf(with_rnn_toc=False)
    print("=== mock capture: TOC missing (expect exit 2) ===")
    try:
        capture_from_connected_cf(cf_bad, 0.0, 10, idle_fn=lambda _lg: None)
        print("UNEXPECTED: no exception", file=sys.stderr)
        worst = 1
    except RuntimeError as e:
        print(f"FAIL: {e}", file=sys.stderr)
        print("exit_code=2")
    print()

    # (c) TOC present + all-zero samples → exit 1
    cf0 = _make_fake_cf(with_rnn_toc=True)

    def idle_zero(lg):
        _fire_log(lg, {"rnn.us_last": 0, "rnn.us_max": 0, "rnn.us_avg": 0.0})

    samples0 = capture_from_connected_cf(cf0, 0.0, 10, idle_fn=idle_zero)
    code0, lines0 = evaluate(samples0, fail_above_us=1000)
    print("=== mock capture: TOC ok + all-zero samples (expect exit 1) ===")
    for ln in lines0:
        print(ln)
    print(f"exit_code={code0}")
    print()
    if code0 != 1:
        worst = 1

    return worst


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("uri", nargs="?", help="Crazyradio URI")
    ap.add_argument("--seconds", type=float, default=5.0, help="Log duration")
    ap.add_argument("--period-ms", type=int, default=10, help="Log period in ms")
    ap.add_argument(
        "--fail-above-us",
        type=int,
        default=1000,
        help="Non-zero exit if peak us_last/us_max exceeds this (default 1000 µs)",
    )
    ap.add_argument(
        "--mock-self-test",
        action="store_true",
        help="Run offline evaluate() checks",
    )
    ap.add_argument(
        "--mock-self-test-capture",
        action="store_true",
        help="Run offline capture-path checks (fake TOC + log callbacks)",
    )
    args = ap.parse_args()

    if args.mock_self_test:
        return run_mock_self_test()

    if args.mock_self_test_capture:
        return run_mock_self_test_capture()

    if not args.uri:
        ap.error("uri required unless --mock-self-test or --mock-self-test-capture")

    try:
        samples = capture(args.uri, args.seconds, args.period_ms)
    except RuntimeError as e:
        print(f"FAIL: {e}", file=sys.stderr)
        return 2

    code, lines = evaluate(samples, fail_above_us=args.fail_above_us)
    for ln in lines:
        print(ln)
    return code


if __name__ == "__main__":
    sys.exit(main())
