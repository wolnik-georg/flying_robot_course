#!/usr/bin/env python3
"""Invoke Rust bisect test and copy JSON to out/."""

from __future__ import annotations

import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
STACK = REPO / "flying_drone_stack"
OUT = Path(__file__).resolve().parent / "out" / "indi_harness_bisect"


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    env = {**dict(__import__("os").environ), "INDI_HARNESS_BISECT": "1"}
    subprocess.run(
        ["cargo", "test", "--test", "indi_harness_bisect", "bisect_matrix", "--", "--nocapture"],
        cwd=str(STACK),
        env=env,
        check=True,
    )
    src = OUT / "bisect.json"
    print("OK", src if src.is_file() else "missing bisect.json")


if __name__ == "__main__":
    main()
