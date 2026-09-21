#!/usr/bin/env python3
"""Build a run_tag -> file index for a uSD raw archive directory.

Pre-upgrade firmware logs have no usd.runTag channel; this tool reports them loudly instead
of silently skipping. See docs/39_USD_Radio_Logging_Robustness_Plan.md.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import decode_usd_log  # noqa: E402


def run_tag_from_file(path: Path):
    """Return (tag, n_rows, duration_s, unique_tags_in_file) or raise on decode failure."""
    d = decode_usd_log.load(str(path))
    t = d["t"]
    duration_s = float(t[-1] - t[0]) if len(t) > 1 else 0.0
    n_rows = len(t)
    if "run_tag" not in d:
        return None, n_rows, duration_s, []
    rt = d["run_tag"]
    uniq = sorted({int(x) for x in rt if x == x and x > 0})
    if not uniq:
        return 0, n_rows, duration_s, [0]
    if len(uniq) > 1:
        return uniq[0], n_rows, duration_s, uniq
    return uniq[0], n_rows, duration_s, uniq


def index_directory(archive_dir: Path, card_label: str | None = None):
    archive_dir = Path(archive_dir)
    if not archive_dir.is_dir():
        sys.exit(f"[index_usd_archive] not a directory: {archive_dir}")

    index = {}
    no_tag = []
    inconsistent = []
    card = card_label or archive_dir.name

    for path in sorted(archive_dir.glob("*.bin")):
        try:
            tag, n_rows, duration_s, uniq = run_tag_from_file(path)
        except Exception as e:
            print(f"[index_usd_archive] ERROR decoding {path.name}: {e}", file=sys.stderr)
            continue
        if tag is None:
            no_tag.append(path.name)
            continue
        if len(uniq) > 1:
            inconsistent.append({"file": path.name, "tags": uniq})
            continue
        if tag == 0:
            print(f"[index_usd_archive] WARNING {path.name}: run_tag column present but all zero",
                  file=sys.stderr)
            no_tag.append(path.name)
            continue
        key = str(tag)
        if key in index:
            print(f"[index_usd_archive] WARNING duplicate run_tag {tag} in {card}: "
                  f"{index[key]['path']} and {path.name}", file=sys.stderr)
        index[key] = {
            "path": str(path.resolve()),
            "card": card,
            "n_rows": n_rows,
            "duration_s": round(duration_s, 3),
        }

    if no_tag:
        print(f"[index_usd_archive] {len(no_tag)} file(s) in {archive_dir.name} have NO run_tag "
              f"(pre-upgrade firmware or empty tag) — RMS-search pairing still required:",
              file=sys.stderr)
        for name in no_tag[:20]:
            print(f"    {name}", file=sys.stderr)
        if len(no_tag) > 20:
            print(f"    ... and {len(no_tag) - 20} more", file=sys.stderr)

    if inconsistent:
        print(f"[index_usd_archive] {len(inconsistent)} file(s) with INCONSISTENT run_tag values:",
              file=sys.stderr)
        for item in inconsistent:
            print(f"    {item['file']}: {item['tags']}", file=sys.stderr)

    return {"card": card, "directory": str(archive_dir.resolve()), "by_run_tag": index,
            "no_run_tag_files": no_tag, "inconsistent_run_tag": inconsistent}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("archive_dir", help="e.g. experiments/logs/usd_raw/2026-09-21_THESIS1")
    ap.add_argument("--card", default=None, help="label for JSON (default: directory name)")
    ap.add_argument("-o", "--out", default=None, help="write JSON index to this path")
    args = ap.parse_args()

    report = index_directory(Path(args.archive_dir), args.card)
    text = json.dumps(report, indent=2)
    if args.out:
        Path(args.out).write_text(text + "\n")
        print(f"[index_usd_archive] wrote {args.out} ({len(report['by_run_tag'])} tagged files)")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
