#!/usr/bin/env python3
"""Phase 3 gate (docs/plan_omar_tree_ns2_2026-10-14.md): with rnn.en = 0 the Omar C controller of the NS2-enabled
build must produce BIT-IDENTICAL thrust and torque to the build without the NS2 hook, on logged flight inputs.

Both builds are exercised through the same replay worker (experiments/analysis/indi_replay_equal_worker.py, side
omar_c) in separate processes (the two same-named `cffirmware` extension modules must not share a process); the
worker output CSVs are compared byte for byte.

  python3 flying_drone_stack/firmware_app/host/test_omar_ns2_off_identical.py --old-dir <dir with the pre-NS2 cffirmware.py + .so>
The pre-NS2 build is obtained by building the bindings from a tree without ns2_residual_module.patch and the NS2 lines of
controller_omar_indi.{c,h}.patch (or from the backup taken before the hook was added)."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WORKER = ROOT / "experiments/analysis/indi_replay_equal_worker.py"
CASES = {
    "A8_2026-10-02 (A8, indi 3)": ("out/indi_replay_ladder/rung0/A8_17-39-27", 3),
    "A8_2026-10-02 (A8, indi 0)": ("out/indi_replay_ladder/rung0/A8_17-39-27", 0),
    "A1 (indi 3)": ("out/indi_replay_ladder/rung2/A1_19-15-47", 3),
    "A1 (indi 0)": ("out/indi_replay_ladder/rung2/A1_19-15-47", 0),
}


def run(build_dir: str, flight_dir: Path, indi: int, out_csv: Path) -> None:
    ov = out_csv.with_suffix(".overrides.json")
    ov.write_text(json.dumps({"omar_indi": indi}))
    env = dict(os.environ, CFFIRMWARE_DIR=build_dir)
    r = subprocess.run([sys.executable, str(WORKER), "omar_c", str(flight_dir / "inputs.npz"), str(out_csv), str(ov),
                        str(flight_dir / "flight_config.json")], capture_output=True, text=True, env=env)
    if r.returncode != 0:
        raise SystemExit(f"worker failed ({build_dir}): {r.stderr[-500:]}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--old-dir", required=True)
    ap.add_argument("--new-dir", default="/home/georg/Desktop/crazyflie-firmware/build")
    a = ap.parse_args()
    from concurrent.futures import ThreadPoolExecutor
    fails = 0
    with tempfile.TemporaryDirectory() as td:
        jobs = []
        for i, (name, (rel, indi)) in enumerate(CASES.items()):
            fd = ROOT / "experiments/analysis" / rel
            if not (fd / "inputs.npz").is_file():
                print(f"  SKIP  {name}: {fd} missing")
                continue
            for side, bd in (("old", a.old_dir), ("new", a.new_dir)):
                jobs.append((name, side, bd, fd, indi, Path(td) / f"{i}_{side}.csv"))
        with ThreadPoolExecutor(8) as ex:
            list(ex.map(lambda j: run(j[2], j[3], j[4], j[5]), jobs))
        for i, (name, (rel, indi)) in enumerate(CASES.items()):
            o, n = Path(td) / f"{i}_old.csv", Path(td) / f"{i}_new.csv"
            if not o.exists():
                continue
            bo, bn = o.read_bytes(), n.read_bytes()
            ok = bo == bn and len(bo) > 1000
            rows = bo.count(b"\n")
            print(f"  {'PASS' if ok else 'FAIL'}  {name}: {rows} rows, sha256 old {hashlib.sha256(bo).hexdigest()[:12]} new {hashlib.sha256(bn).hexdigest()[:12]}")
            fails += 0 if ok else 1
    print("RESULT:", "ALL IDENTICAL" if fails == 0 else f"{fails} MISMATCH(ES)")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
