#!/usr/bin/env python3
"""Replay old vs new rpm_filter_step on 2026-10-05 A8 uSD DShot RPM channels."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "flying_drone_stack" / "tools"))
from decode_usd_log import load  # noqa: E402

SENTINEL = 0xFFFF
ABS_MAX = 28000
SLEW_MAX = 10000
KT = 4.1e-10


def filter_step(raw: int, rpm_source: int, prev: int, *, sentinel_hold: bool) -> tuple[int, int]:
    v = int(raw) & 0xFFFF
    if v == SENTINEL:
        if sentinel_hold and rpm_source != 0 and prev > 0:
            return prev, prev
        v = 0
    if rpm_source != 0 and v > 0:
        reject = False
        if v > ABS_MAX:
            reject = True
        elif prev > 500:
            lo, hi = min(v, prev), max(v, prev)
            if hi - lo > SLEW_MAX:
                reject = True
        if reject and prev > 0:
            v = prev
        elif not reject:
            prev = v
    elif rpm_source == 0 and v > 0:
        prev = v
    return v, prev


def replay_motor(raw: np.ndarray, *, sentinel_hold: bool) -> np.ndarray:
    prev = 0
    out = np.zeros(len(raw), dtype=np.uint16)
    for i, r in enumerate(raw.astype(np.uint16)):
        o, prev = filter_step(int(r), 1, prev, sentinel_hold=sentinel_hold)
        out[i] = o
    return out


def thrust_proxy_n(rpm: np.ndarray) -> float:
    return float(KT * np.sum(rpm.astype(np.float64) ** 2))


def run_lengths(mask: np.ndarray) -> list[int]:
    if not np.any(mask):
        return []
    m = mask.astype(np.int8)
    edges = np.diff(np.concatenate([[0], m, [0]]))
    starts = np.where(edges == 1)[0]
    ends = np.where(edges == -1)[0]
    return [int(e - s) for s, e in zip(starts, ends)]


def analyze(path: Path) -> dict:
    d = load(str(path))
    motors = [f"motor_m{i}_rpm" for i in range(1, 5)]
    per_motor = {}
    total_sent = 0
    max_abs_diff = 0
    max_thrust_delta_n = 0.0
    n_samples = len(d["t"])

    for name in motors:
        raw = d[name].astype(np.uint16)
        old = replay_motor(raw, sentinel_hold=False)
        new = replay_motor(raw, sentinel_hold=True)
        sent = raw == SENTINEL
        n_sent = int(np.sum(sent))
        total_sent += n_sent
        diff = np.abs(old.astype(np.int32) - new.astype(np.int32))
        max_d = int(np.max(diff)) if len(diff) else 0
        max_abs_diff = max(max_abs_diff, max_d)
        rl = run_lengths(sent)
        thrust_d = []
        for idx in np.where(sent)[0]:
            td = abs(thrust_proxy_n(old[idx]) - thrust_proxy_n(new[idx]))
            thrust_d.append(td)
            max_thrust_delta_n = max(max_thrust_delta_n, td)
        per_motor[name] = {
            "n_sentinel": n_sent,
            "pct_sentinel": 100.0 * n_sent / max(len(raw), 1),
            "run_lengths": rl,
            "max_run_length": max(rl) if rl else 0,
            "max_abs_rpm_diff": max_d,
            "max_thrust_delta_n_at_sentinel": max(thrust_d) if thrust_d else 0.0,
        }

    return {
        "file": path.name,
        "n_samples": n_samples,
        "total_sentinel_motor_samples": total_sent,
        "max_abs_rpm_diff_any_motor": max_abs_diff,
        "max_thrust_delta_n_at_sentinel": max_thrust_delta_n,
        "motors": per_motor,
    }


def main() -> None:
    log_dir = REPO / "experiments" / "logs" / "usd_raw"
    patterns = [
        "cf5_A8*_2026-10-05_19-*.bin",
        "cf5*17-44-01.bin",
    ]
    paths: list[Path] = []
    for pat in patterns:
        paths.extend(sorted(log_dir.glob(pat)))
    paths = sorted(set(paths))
    if not paths:
        print("No log files found", file=sys.stderr)
        sys.exit(1)

    results = [analyze(p) for p in paths]
    out_path = REPO / "experiments" / "analysis" / "out" / "rpm_sentinel" / "usd_quantify.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps({"flights": results}, indent=2))

    print(f"Wrote {out_path}")
    for r in results:
        print(
            f"{r['file']}: samples={r['n_samples']} "
            f"sentinel_motor_ticks={r['total_sentinel_motor_samples']} "
            f"max|ΔRPM|={r['max_abs_rpm_diff_any_motor']} "
            f"max|ΔΣkt·rpm²|@sentinel={r['max_thrust_delta_n_at_sentinel']:.6f} N"
        )


if __name__ == "__main__":
    main()
