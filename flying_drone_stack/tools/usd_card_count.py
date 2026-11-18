#!/usr/bin/env python3
"""Count the thesis* log files on mounted uSD cards (read-only).

    python3 flying_drone_stack/tools/usd_card_count.py /media/georg/THESIS1 /media/georg/THESIS2 [--expect N]

Prints per card: number of thesis* files (top level only, archive folders ignored), the newest file (highest number)
and its size, and the empty (0-byte) files. Exit code non-zero if --expect N is given and any card differs, if a card
has no readable path, or if any card has >= 85 files (the card must never reach thesis99 = 100 files).
"""
import argparse
import re
import sys
from pathlib import Path

LIMIT = 85


def scan(path):
    files = []
    for f in Path(path).iterdir():
        m = re.fullmatch(r'thesis(\d+)', f.name)
        if f.is_file() and m:
            files.append((int(m.group(1)), f.name, f.stat().st_size))
    return sorted(files)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cards', nargs='+')
    ap.add_argument('--expect', type=int, default=None)
    a = ap.parse_args()
    rc = 0
    for c in a.cards:
        try:
            fs = scan(c)
        except OSError as e:
            print(f'{c}: NOT READABLE ({e})')
            rc = 1
            continue
        newest = f'{fs[-1][1]} {fs[-1][2]} B' if fs else '-'
        empty = [n for _, n, sz in fs if sz == 0]
        line = f'{c}: {len(fs)} thesis* files, newest {newest}, empty {empty or "none"}'
        if a.expect is not None and len(fs) != a.expect:
            line += f'   <-- EXPECTED {a.expect}'
            rc = 1
        if len(fs) >= LIMIT:
            line += f'   <-- >= {LIMIT} FILES: archive now (card must never reach thesis99)'
            rc = 1
        print(line)
    return rc


if __name__ == '__main__':
    sys.exit(main())
