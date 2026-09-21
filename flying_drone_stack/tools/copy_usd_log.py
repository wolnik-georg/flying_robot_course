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
    # Highest-numbered non-empty file = the most recent session that actually recorded.
    #
    # 2026-09-15: this used to pick the LARGEST file instead, on the reasoning that the
    # counter increments even for sessions that produced nothing. That is true, but picking
    # by size is worse: a card accumulates sessions across a whole evening, and the largest
    # is whichever ran longest -- not the newest. On this date the largest file on the card
    # was a 14.9 MB corrupted recording (altitudes to 40 m) from hours earlier, while the
    # flight just landed was a 1.18 MB file with a higher number. Empty sessions are already
    # filtered out above, so "highest-numbered non-empty" is both correct and what a person
    # means by "the log from the flight I just did".
    #
    # The counter is still not a timestamp: it does not reset on format and says nothing
    # about WHICH DRONE produced the file (cards get moved between vehicles). Always confirm
    # identity from the data itself -- find_flight_window.py against the scenario's meta.json
    # is the check that actually proves which flight a file holds.
    best, best_sz = max(nonempty, key=lambda t: t[0].name)
    if len(nonempty) > 1:
        others = ", ".join(f"{p.name} ({sz:,}B)" for p, sz in nonempty if p != best)
        print(f"  -> multiple non-empty files; picking the highest-numbered ({best.name}, "
              f"{best_sz:,} bytes) as the most recent session.")
        print(f"     others present: {others}")
        print(f"     if you need one of those instead, pass --file <name>.")
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mount", help="uSD card mount point, e.g. /media/georg/THESIS1")
    ap.add_argument("drone", help="drone name, e.g. cf_second, cf231_active")
    ap.add_argument("--dest", default=None,
                    help="destination directory (default: experiments/logs/usd_raw next to "
                         "this repo checkout)")
    ap.add_argument("--file", default=None, metavar="NAME",
                    help="copy this exact file from the card (e.g. thesis06) instead of "
                         "auto-picking the most recent non-empty one")
    ap.add_argument("--tag", default=None,
                    help="extra label for the filename, e.g. the scenario ('A8'). Recommended: "
                         "it is the only scenario hint the filename will ever carry.")
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

    if args.file:
        src = mount / args.file
        if not src.is_file():
            sys.exit(f"[copy_usd_log] {src} does not exist on the card")
        if src.stat().st_size == 0:
            sys.exit(f"[copy_usd_log] {src} is 0 bytes -- that session recorded nothing")
        print(f"  using explicitly requested file {src.name} ({src.stat().st_size:,} bytes)")
    else:
        src = find_latest_log(mount)
    stamp = time.strftime("%Y-%m-%d_%H-%M-%S")  # THIS machine's wall clock -- the only real one
    tag = f"_{args.tag}" if args.tag else ""
    dest = dest_dir / f"{args.drone}{tag}_{src.name}_{stamp}.bin"
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

    tools_dir = Path(__file__).resolve().parent
    sys.path.insert(0, str(tools_dir))
    try:
        import decode_usd_log
        d = decode_usd_log.load(str(dest))
        duration_s = float(d["t"][-1] - d["t"][0]) if len(d["t"]) > 1 else 0.0
        n_rows = len(d["t"])
        print(f"[copy_usd_log] decoded: {n_rows} rows, duration {duration_s:.1f} s")
        if duration_s < 5.0:
            print("[copy_usd_log] WARNING: recording shorter than 5 s — likely aborted or "
                  "failed; do not assume this is a full flight.", file=sys.stderr)
        if "run_tag" in d:
            tags = {int(x) for x in d["run_tag"] if x == x and x > 0}
            if len(tags) == 1:
                print(f"[copy_usd_log] usd.runTag = {tags.pop()} (match meta.json usd_run_tag)")
            elif tags:
                print(f"[copy_usd_log] WARNING: inconsistent run_tag values in file: {tags}",
                      file=sys.stderr)
            else:
                print("[copy_usd_log] WARNING: run_tag column all zero", file=sys.stderr)
        else:
            print("[copy_usd_log] no usd.runTag channel (pre-tag firmware archive)")
    except Exception as e:
        print(f"[copy_usd_log] could not decode for run_tag/duration check: {e}", file=sys.stderr)

    print(f"[copy_usd_log] NOTE: to match this against a radio CSV from the same flight, do it "
          f"explicitly by scenario name and approximate flight time -- never assume a CSV that "
          f"happens to arrive in the same git pull is the match (this is exactly the mistake "
          f"this tool exists to prevent).")


if __name__ == "__main__":
    main()
