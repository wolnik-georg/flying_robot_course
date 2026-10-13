#!/usr/bin/env python3
"""Put the 96-variable STUDY config (usd_study_config.txt) on a mounted uSD card, keeping whatever config.txt was there as a dated backup, and verify by hash.
  python3 flying_drone_stack/tools/restore_study_sd_config.py /media/georg/THESIS2            # study config
  python3 flying_drone_stack/tools/restore_study_sd_config.py /media/georg/THESIS2 --debug    # Tal & Karaman debug config (usd_tk_debug_config.txt)
Nothing else on the card is touched; log files are never read or moved. Added 2026-10-13 (the cf5 card carried the debug config after the Tal & Karaman tests)."""
import argparse, datetime, hashlib, os, shutil, sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent


def sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


ap = argparse.ArgumentParser(); ap.add_argument("mount", type=Path); ap.add_argument("--debug", action="store_true"); a = ap.parse_args()
src = TOOLS / ("usd_tk_debug_config.txt" if a.debug else "usd_study_config.txt")
dst = a.mount / "config.txt"
if not a.mount.is_dir():
    sys.exit(f"{a.mount} is not a mounted directory")
if dst.is_file() and sha(dst) == sha(src):
    print(f"already in place: {dst} == {src.name} (sha256 {sha(src)[:16]}…)"); sys.exit(0)
if dst.is_file():
    bak = a.mount / f"config_before_{datetime.datetime.now():%Y-%m-%d_%H%M%S}.bak.txt"
    shutil.copy2(dst, bak); print("backed up the old config as", bak.name)
shutil.copy(src, dst); os.sync()
ok = sha(dst) == sha(src)
print(f"{'OK' if ok else 'MISMATCH'}: {dst} <- {src.name}, sha256 {sha(dst)[:16]}…, variables: {len([l for l in dst.read_text().splitlines()[7:] if l.strip()])}")
sys.exit(0 if ok else 1)
