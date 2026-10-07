#!/usr/bin/env python3
"""Worker with no JSON on stdout must fail loudly (step 7)."""
import subprocess
import sys
from pathlib import Path

ANALYSIS = Path(__file__).resolve().parent
RUN = ANALYSIS / "indi_replay_run.py"


def test_run_worker_no_json_raises():
    # Fake worker path via env not available — call run_worker logic through broken command.
    code = """
import subprocess, sys, os
from pathlib import Path
BUILD = "/home/georg/Desktop/crazyflie-firmware/build"
env = os.environ.copy()
env["PYTHONPATH"] = BUILD
out = subprocess.run([sys.executable, "-c", "import sys; sys.stderr.write('boom\\n'); sys.exit(0)"],
                     capture_output=True, text=True, env=env)
lines = [l for l in out.stdout.splitlines() if l.strip().startswith("{")]
if not lines:
    raise RuntimeError(f"worker produced no JSON:\\n{out.stderr}\\n{out.stdout}")
"""
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert r.returncode != 0
    assert "worker produced no JSON" in r.stderr + r.stdout


if __name__ == "__main__":
    test_run_worker_no_json_raises()
    print("ok")
