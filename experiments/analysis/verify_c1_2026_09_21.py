#!/usr/bin/env python3
"""Independent re-verification of 2026-09-21 C.1 manifest merges (Task 5)."""
from __future__ import annotations

import json
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOGS = ROOT / "experiments" / "logs"
MERGED = LOGS / "c1_2026-09-21_merged"
MANIFEST = MERGED / "manifest_2026-09-21_c1.json"
MERGE = ROOT / "flying_drone_stack/tools/merge_usd_logs.py"
PY = "/home/georg/.pyenv/versions/flying_robots/bin/python"
ENV = {"PYTHONPATH": "/home/georg/Desktop/crazyswarm2/crazyflie_examples"}
RMS_RE = re.compile(r"role=(\S+)\s+scenario starts at its t=\s*([\d.]+)s\s+RMS\s+([\d.]+)\s*cm")

STALE_EMPTY = [
    ("A1", "13-27-27"),
    ("A3", "13-05-58"),
]


def find_usd_symlinks(folder: Path):
    cf5 = list(folder.glob("cf5*.usd")) + list(folder.glob("*cf5*.usd"))
    top = list(folder.glob("cf_second*.usd")) + list(folder.glob("*cf_second*.usd"))
    # prefer symlinks
    cf5 = [p for p in cf5 if p.is_symlink() or p.suffix == ".usd"]
    top = [p for p in top if p.is_symlink() or p.suffix == ".usd"]
    return cf5[:1], top[:1]


def main():
    manifest = json.loads(MANIFEST.read_text())
    claimed = defaultdict(list)
    problems = []

    print("=== Manifest entries ===\n")
    for entry in manifest:
        sc, stamp = entry["scenario"], entry["stamp"]
        folder = MERGED / f"{sc}_2026-09-21_{stamp}"
        meta = folder / f"{sc}_2026-09-21_{stamp}.meta.json"
        merged_csv = folder / f"{sc}_2026-09-21_{stamp}_merged_usd.csv"
        cf5_links, top_links = find_usd_symlinks(folder)

        print(f"{sc} {stamp}")
        if not merged_csv.is_file() or merged_csv.stat().st_size < 100:
            problems.append(f"{sc} {stamp}: missing or empty merged CSV")
            print("  FAIL: no merged CSV\n")
            continue
        if not meta.is_file():
            problems.append(f"{sc} {stamp}: missing meta.json")
        if len(cf5_links) != 1 or len(top_links) != 1:
            problems.append(f"{sc} {stamp}: expected one cf5 and one cf_second symlink")
            print(f"  WARN: symlinks cf5={cf5_links} top={top_links}")

        tp = entry["thesis_pair"]
        pair_key = (tp["card_bottom"], tp["bottom_idx"], tp["card_top"], tp["top_idx"])
        claimed[pair_key].append(f"{sc}_{stamp}")

        proc = subprocess.run(
            [PY, str(MERGE), str(cf5_links[0]), str(top_links[0]),
             "-o", "/tmp/verify_merge_out.csv", "--meta", str(meta),
             "--roles", "bottom", "top"],
            capture_output=True, text=True, cwd=str(ROOT), env={**dict(**ENV), **dict(**{})},
        )
        out = proc.stdout + proc.stderr
        if proc.returncode != 0:
            problems.append(f"{sc} {stamp}: merge re-run failed")
            print(f"  FAIL merge exit {proc.returncode}\n{out[-800:]}\n")
            continue
        rms = {}
        for m in RMS_RE.finditer(out):
            rms[m.group(1)] = float(m.group(3))
        exp = entry["rms_cm"]
        for role in ("bottom", "top"):
            got = rms.get(role)
            if got is None:
                problems.append(f"{sc} {stamp}: no RMS for role {role}")
            elif got >= 15.0:
                problems.append(f"{sc} {stamp}: {role} RMS {got} >= 15")
            elif abs(got - exp[role]) > 1.5:
                problems.append(f"{sc} {stamp}: {role} RMS {got} vs manifest {exp[role]}")

        meta_j = json.loads(meta.read_text())
        dur = meta_j.get("duration", 0)
        n_lines = sum(1 for _ in open(merged_csv)) - 1
        expect_rows = int(dur * 500 * 0.85)
        if n_lines < expect_rows * 0.5:
            problems.append(f"{sc} {stamp}: merged rows {n_lines} low vs duration {dur}s")

        print(f"  OK  RMS bottom={rms.get('bottom')} top={rms.get('top')} "
              f"rows={n_lines} pair={tp}\n")

    dup = {k: v for k, v in claimed.items() if len(v) > 1}
    if dup:
        for k, v in dup.items():
            problems.append(f"duplicate thesis pair {k} used by {v}")

    print("=== Stale folders (must have NO merged CSV) ===\n")
    for sc, stamp in STALE_EMPTY:
        folder = MERGED / f"{sc}_2026-09-21_{stamp}"
        csvs = list(folder.glob("*_merged_usd.csv"))
        if csvs:
            problems.append(f"{sc} {stamp}: stale merge CSV should be absent: {csvs}")
            print(f"{sc} {stamp} FAIL still has merged CSV")
        else:
            print(f"{sc} {stamp} OK (no merged CSV)")

    print("\n=== Re-run grid search for unmergeable stamps ===\n")
    proc = subprocess.run(
        [PY, str(ROOT / "experiments/analysis/merge_c1_2026_09_21.py")],
        capture_output=True, text=True, cwd=str(ROOT),
        env={**dict(**ENV), "PYTHONPATH": ENV["PYTHONPATH"]},
    )
    print(proc.stdout[-4000:] if len(proc.stdout) > 4000 else proc.stdout)
    if proc.returncode != 0:
        print(proc.stderr[-2000:])

    print("\n=== Summary ===")
    if problems:
        print(f"PROBLEMS ({len(problems)}):")
        for p in problems:
            print(f"  - {p}")
        return 1
    print(f"All {len(manifest)} manifest entries verified; stale folders clean.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
