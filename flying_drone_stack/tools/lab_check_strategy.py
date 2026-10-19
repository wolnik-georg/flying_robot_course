#!/usr/bin/env python3
"""Read-only check on the lab PC: which strategy (S0..S3) does the crazyswarm2 yaml that CS2 will read describe?
  python3 flying_drone_stack/tools/lab_check_strategy.py [path/to/crazyswarm2]
Prints: the repo and its HEAD commit, the cf5 / cf_second values of ctrlOmarIndi.indi and rnn.en in the source yaml (with line numbers), the strategy they mean,
and every installed copy of crazyflies.yaml under ~/georg/ros2_ws/install (symlink or copy, identical to the source or STALE).
Nothing is modified. Run it after `git pull` and BEFORE `ros2 launch`; after the launch also read the `Update parameter "rnn.en"` lines of cf5 in the launch output."""
import hashlib, os, re, subprocess, sys
from pathlib import Path

def find_repo():
    cands = [sys.argv[1]] if len(sys.argv) > 1 else []
    cands += [os.environ.get("CS2", ""), "~/georg/ros2_ws/src/crazyswarm2", "~/Desktop/crazyswarm2", "~/ros2_ws/src/crazyswarm2"]
    for c in cands:
        p = Path(c).expanduser() if c else None
        if p and (p / "crazyflie/config/crazyflies.yaml").is_file():
            return p
    sys.exit("crazyswarm2 not found; pass its path as the first argument")

def robot_block(text, name):
    m = re.search(rf"^  {name}:\s*$", text, re.M)
    if not m: return None, 0
    n = re.search(r"^  \S", text[m.end():], re.M)
    end = m.end() + (n.start() if n else len(text) - m.end())
    return text[m.start():end], text[:m.start()].count("\n")

def find_val(block, offset, group, key):
    """first `key: value` under a line `group:` inside the block (comments ignored); returns (value, absolute line number)"""
    lines = block.split("\n"); in_g = False; g_indent = None
    for i, l in enumerate(lines):
        s = l.split("#")[0].rstrip()
        if re.match(rf"^\s*{group}:\s*$", s):
            in_g, g_indent = True, len(l) - len(l.lstrip()); continue
        if in_g:
            if s.strip() and (len(l) - len(l.lstrip())) <= g_indent: in_g = False; continue
            m = re.match(rf"^\s*{key}:\s*(\S+)", s)
            if m: return m.group(1), offset + i + 1
    return None, None

repo = find_repo(); yml = repo / "crazyflie/config/crazyflies.yaml"; text = yml.read_text()
head = subprocess.run(["git", "-C", str(repo), "log", "-1", "--format=%h %ad %s", "--date=format:%H:%M"], capture_output=True, text=True).stdout.strip()
dirty = subprocess.run(["git", "-C", str(repo), "status", "--short", "crazyflie/config/crazyflies.yaml"], capture_output=True, text=True).stdout.strip()
print(f"repo   : {repo}\nHEAD   : {head}\nyaml   : {yml}  {'(LOCALLY MODIFIED: ' + dirty + ')' if dirty else '(clean)'}")
res = {}
for name in ("cf5", "cf_second"):
    blk, off = robot_block(text, name)
    if blk is None: print(f"{name}: block not found"); continue
    indi, li = find_val(blk, off, "ctrlOmarIndi", "indi"); en, le = find_val(blk, off, "rnn", "en"); ctl, lc = find_val(blk, off, "stabilizer", "controller")
    enabled = re.search(r"^\s*enabled:\s*(\S+)", blk, re.M)
    res[name] = (indi, en)
    print(f"{name:9s}: enabled {enabled.group(1) if enabled else '?'} | controller {ctl} (line {lc}) | ctrlOmarIndi.indi {indi} (line {li}) | rnn.en {en} (line {le})")
S = {("0", "0"): "S0 geometric", ("3", "0"): "S1 INDI", ("0", "1"): "S2 geometric + NS2", ("3", "1"): "S3 INDI + NS2"}
print("bottom drone cf5 in the yaml  =>", S.get(res.get("cf5"), f"NOT one of S0..S3 {res.get('cf5')}"))
if res.get("cf_second") != ("0", "0"): print("WARNING: the top drone cf_second should be indi 0 / rnn.en 0 (always geometric, no NS2)")
src_hash = hashlib.sha256(yml.read_bytes()).hexdigest(); print("\ninstalled copies of crazyflies.yaml (what `ros2 launch` may actually read):")
found = False
for root in (Path("~/georg/ros2_ws/install").expanduser(), Path("~/ros2_ws/install").expanduser()):
    if not root.is_dir(): continue
    for p in root.rglob("crazyflies.yaml"):
        found = True; kind = "symlink" if p.is_symlink() else "COPY"; same = hashlib.sha256(p.read_bytes()).hexdigest() == src_hash
        print(f"  {kind:7s} {'identical to source' if same else '!!! DIFFERENT FROM SOURCE (stale: rebuild or use --symlink-install) !!!'}  {p}")
if not found: print("  none found under ~/georg/ros2_ws/install (set the path if your workspace is elsewhere)")
