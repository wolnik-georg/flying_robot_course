#!/usr/bin/env python3
"""Build an updated C.1 training manifest: base bank + new lab-day merges.

Scans ``experiments/logs/c1_<date>_merged/<Scenario>_<date>_<stamp>/`` for
``*_merged_usd.csv`` and appends entries not already in the base manifest.

Example (after A5/A6/C4 flights on 2026-09-30):

  python3 experiments/analysis/build_c1_training_manifest.py \\
    --merge-dir experiments/logs/c1_2026-09-30_merged \\
    --scenarios A5 A6 C4 \\
    -o experiments/analysis/out/c2_e2e_2026-09-30/training_manifest_lab.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BASE = (
    ROOT / "experiments/analysis/out/c2_e2e_2026-09-28/training_manifest_2026-09-28.json"
)


def record_from_csv(csv_path: Path) -> dict:
    folder = csv_path.parent.name
    parts = folder.split("_")
    scenario = parts[0]
    date = parts[1] if len(parts) > 1 else ""
    stamp = "_".join(parts[2:]) if len(parts) > 2 else ""
    rel = csv_path.relative_to(ROOT)
    return {
        "folder": folder,
        "scenario": scenario,
        "date": date,
        "stamp": stamp,
        "path": str(rel),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--base", type=Path, default=DEFAULT_BASE, help="existing manifest JSON")
    ap.add_argument(
        "--merge-dir",
        type=Path,
        required=True,
        help="lab merge root, e.g. experiments/logs/c1_2026-09-30_merged",
    )
    ap.add_argument(
        "--scenarios",
        nargs="+",
        default=["A5", "A6", "C4"],
        help="only add flights whose folder name starts with these scenario ids",
    )
    ap.add_argument("-o", "--out", type=Path, required=True)
    args = ap.parse_args()

    base = json.loads(args.base.read_text())
    known_paths = {f["path"] for f in base.get("flights", [])}
    flights = list(base.get("flights", []))

    merge_root = args.merge_dir if args.merge_dir.is_absolute() else ROOT / args.merge_dir
    if not merge_root.is_dir():
        raise SystemExit(f"merge-dir not found: {merge_root}")

    added = []
    for csv_path in sorted(merge_root.glob("*/*_merged_usd.csv")):
        rec = record_from_csv(csv_path)
        if rec["scenario"] not in args.scenarios:
            continue
        if rec["path"] in known_paths:
            continue
        flights.append(rec)
        known_paths.add(rec["path"])
        added.append(rec["folder"])

    out_rec = {
        "date": base.get("date", "") + "+lab",
        "note": (
            f"Base: {args.base.name}; added {len(added)} flight(s) from {merge_root.name}: "
            + (", ".join(added) if added else "(none found — check merge paths)")
        ),
        "base_manifest": str(args.base.relative_to(ROOT) if args.base.is_relative_to(ROOT) else args.base),
        "n_paths": len(flights),
        "flights": flights,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out_rec, indent=2) + "\n")
    print(f"wrote {args.out}  n_paths={len(flights)}  added={len(added)}")
    for name in added:
        print(f"  + {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
