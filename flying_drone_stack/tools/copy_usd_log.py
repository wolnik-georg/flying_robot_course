#!/usr/bin/env python3
"""Copy the latest uSD log off a mounted card, named unambiguously, verified byte-for-byte.

Why this exists
----------------
2026-09-15: a uSD log was manually `cp`'d and renamed by hand, then later a radio CSV that
happened to arrive via `git pull` around the same time was assumed to be its match -- it
wasn't; the radio CSV was from the previous night. The uSD log's own timestamp
(`usecTimestamp()`) is µs since THAT DRONE'S power-on, not a calendar time, so it cannot name
the file and cannot be used to line files up across drones or across sessions. The only
trustworthy real-world timestamp is the copying machine's wall clock at the moment of copy --
which is what this script uses, so the filename is never a guess.

This script also enforces the separation the mistake came from: it copies uSD logs into their
own directory (`experiments/logs/usd_raw/`) and touches nothing under the plain
`experiments/logs/*.csv` radio-log namespace. Matching a uSD file to a radio CSV from the same
flight is a separate, explicit step (by scenario name and approximate time), never an
assumption from both files appearing in a `git pull` at once.

Usage
-----
    python3 copy_usd_log.py /media/georg/THESIS1 cf_second
    python3 copy_usd_log.py /media/georg/THESIS2 cf231_active --dest ../../experiments/logs/usd_raw

Finds the largest non-empty `thesis*` (or `config.txt`-adjacent) log file on the card -- not
just the highest-numbered one, since the firmware increments the counter on every logging
session including ones that produced nothing (see 2026-09-15's empty `thesis00` on both cards).
"""

import argparse
import hashlib
import shutil
import sys
import time
from pathlib import Path

LOG_STEM = "thesis"  # matches usd_thesis_config.txt line 3 (file name prefix)


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def find_latest_log(mount: Path) -> Path:
    candidates = sorted(mount.glob(f"{LOG_STEM}*"))
    if not candidates:
        sys.exit(f"[copy_usd_log] no '{LOG_STEM}*' files found on {mount} -- wrong mount point, "
                  f"or config.txt's file-name prefix (line 3) doesn't match '{LOG_STEM}'?")
    sized = [(p, p.stat().st_size) for p in candidates]
    for p, sz in sized:
        print(f"  found {p.name}: {sz:,} bytes" + ("  <- empty, skipping" if sz == 0 else ""))
    nonempty = [(p, sz) for p, sz in sized if sz > 0]
    if not nonempty:
        sys.exit(f"[copy_usd_log] every '{LOG_STEM}*' file on {mount} is 0 bytes -- this card "
                  f"recorded nothing. Check usd.logging actually reached 1 during the flight "
                  f"(check_usd_deck.py can toggle it manually to confirm the deck works at all).")
    # Largest file, not highest-numbered: the counter increments on every session, including
    # ones that produced an empty file, so "latest number" and "the real flight" can differ.
    best, best_sz = max(nonempty, key=lambda t: t[1])
    if len(nonempty) > 1:
        print(f"  -> multiple non-empty files; picking the largest ({best.name}, {best_sz:,} "
              f"bytes). If that's wrong, copy the others by hand -- this script only takes one.")
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mount", help="uSD card mount point, e.g. /media/georg/THESIS1")
    ap.add_argument("drone", help="drone name, e.g. cf_second, cf231_active")
    ap.add_argument("--dest", default=None,
                    help="destination directory (default: experiments/logs/usd_raw next to "
                         "this repo checkout)")
    args = ap.parse_args()

    mount = Path(args.mount)
    if not mount.is_dir():
        sys.exit(f"[copy_usd_log] {mount} is not a mounted directory")

    if args.dest:
        dest_dir = Path(args.dest)
    else:
        # this file lives at flying_drone_stack/tools/copy_usd_log.py
        dest_dir = Path(__file__).resolve().parents[2] / "experiments" / "logs" / "usd_raw"
    dest_dir.mkdir(parents=True, exist_ok=True)

    src = find_latest_log(mount)
    stamp = time.strftime("%Y-%m-%d_%H-%M-%S")  # THIS machine's wall clock -- the only real one
    dest = dest_dir / f"{args.drone}_{stamp}.bin"
    if dest.exists():
        sys.exit(f"[copy_usd_log] {dest} already exists -- refusing to overwrite. Wait a second "
                 f"and retry, or pass a more specific --dest.")

    print(f"[copy_usd_log] {src} -> {dest}")
    shutil.copy2(src, dest)

    src_hash, dst_hash = sha256(src), sha256(dest)
    if src_hash != dst_hash:
        dest.unlink(missing_ok=True)
        sys.exit(f"[copy_usd_log] COPY VERIFICATION FAILED (sha256 mismatch) -- deleted the "
                 f"partial copy. Card may be failing; do not trust this card's data blindly.")

    print(f"[copy_usd_log] verified byte-identical (sha256 {dst_hash[:16]}...)")
    print(f"[copy_usd_log] wrote {dest}  ({dest.stat().st_size:,} bytes)")
    print(f"[copy_usd_log] NOTE: to match this against a radio CSV from the same flight, do it "
          f"explicitly by scenario name and approximate flight time -- never assume a CSV that "
          f"happens to arrive in the same git pull is the match (this is exactly the mistake "
          f"this tool exists to prevent).")


if __name__ == "__main__":
    main()
