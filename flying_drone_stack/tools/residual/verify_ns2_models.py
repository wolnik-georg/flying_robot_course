#!/usr/bin/env python3
"""Verify that every registered NS2 weight file (weights/registry.json) is still byte-identical, tracked in git and read-only.

  python3 flying_drone_stack/tools/residual/verify_ns2_models.py [--list]
Exit 1 if a file is missing or its sha256 changed. A retrain must write to a NEW out-dir and be added to the registry as a NEW id;
registered files are never edited, moved or overwritten."""
import argparse, hashlib, json, os, subprocess, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[3]; REG = Path(__file__).parent / "weights" / "registry.json"

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--list", action="store_true"); a = ap.parse_args()
    reg = json.load(open(REG)); bad = 0
    for m in reg["models"]:
        p = ROOT / m["file"]
        if not p.exists(): print(f"MISSING  {m['id']}  {m['file']}"); bad += 1; continue
        h = hashlib.sha256(p.read_bytes()).hexdigest()
        tr = subprocess.run(["git", "ls-files", "--error-unmatch", m["file"]], cwd=ROOT, capture_output=True).returncode == 0
        ro = not os.access(p, os.W_OK)
        ok = h == m["sha256"]; bad += (not ok)
        print(f"{'OK     ' if ok else 'CHANGED'}  {m['id']:<36} sha {h[:12]}  tracked={tr}  read-only={ro}  n_flights={m['trained_on'].get('n_flights')}  [{m['status']}]")
        if a.list: print(f"           trained on: {m['trained_on'].get('manifest') or m['trained_on'].get('description')}\n           labels: {m['labels']}\n           result: {m['result']}")
    return 1 if bad else 0
if __name__ == "__main__": sys.exit(main())
