#!/usr/bin/env python3
"""Set the study tree strategy (S0..S3) in crazyflies.yaml: cf5 `ctrlOmarIndi.indi` (0 geometric / 3 INDI) and cf5 `rnn.en`
(0 / 1). Works on the yaml that already carries a tree_S*.patch (anchors are the comments the patches wrote).
  python3 flying_drone_stack/tools/set_tree_strategy.py S2 [--yaml <path>] [--solo]   (--solo: cf_second enabled: false)
S0: indi 0, rnn.en 0 | S1: indi 3, rnn.en 0 | S2: indi 0, rnn.en 1 | S3: indi 3, rnn.en 1. cf_second is always indi 0, rnn.en 0."""
import argparse, re, sys
from pathlib import Path
ap = argparse.ArgumentParser(); ap.add_argument("strategy", choices=("S0", "S1", "S2", "S3"))
ap.add_argument("--yaml", default=str(Path.home() / "Desktop/crazyswarm2/crazyflie/config/crazyflies.yaml"))
ap.add_argument("--solo", action="store_true"); a = ap.parse_args()
indi, en = {"S0": (0, 0), "S1": (3, 0), "S2": (0, 1), "S3": (3, 1)}[a.strategy]
p = Path(a.yaml); s = p.read_text()
def sub(pat, new):
    global s
    s, n = re.subn(pat, new, s, count=1)
    if n != 1: sys.exit(f"anchor not found: {pat[:60]} (is the tree_S*.patch applied to this yaml?)")
sub(r"(indi: )\d(\s+# 2026-10-14 tree strategy: 0 = Omar geometric)", rf"\g<1>{indi}\2")
# 2026-11-18: the cf5 rnn.en comment on crazyswarm2 main / study_S* is "# S<k>: NS2 ..." (not the 2026-10-14 anchor); accept both
label = {"S0": "S0: NS2 off (geometric)", "S1": "S1: NS2 off, INDI on", "S2": "S2: NS2 on (C, flash weights) on top of the geometric Omar", "S3": "S3: NS2 on (C, flash weights) together with full INDI (two plain switches)"}[a.strategy]
if re.search(r"en: \d\s+# 2026-10-14 tree strategy: 1 = NS2", s):
    sub(r"(en: )\d(\s+# 2026-10-14 tree strategy: 1 = NS2)", rf"\g<1>{en}\2")
else:
    i5 = s.index("\n  cf5:\n"); j5 = s.find("\n  cf", i5 + 8)          # restrict to the cf5 block
    blk, n = re.subn(r"(\n\s+rnn:\n\s+en: )\d(\s+# )S\d: NS2[^\n]*", rf"\g<1>{en}\g<2>{label}", s[i5:j5], count=1)
    if n != 1: sys.exit("anchor not found: cf5 rnn.en with a '# S<k>: NS2' comment")
    s = s[:i5] + blk + s[j5:]
lines = s.split("\n"); inblk = False
for i, l in enumerate(lines):
    if re.match(r"^  cf_second:", l): inblk = True
    elif re.match(r"^  cf5:", l): inblk = False
    if inblk and re.match(r"^\s*enabled:\s*(true|false)", l):
        lines[i] = re.sub(r"enabled:\s*(true|false)", "enabled: " + ("false" if a.solo else "true"), l, count=1); break
p.write_text("\n".join(lines))
print(f"{p}: cf5 indi {indi}, rnn.en {en}, cf_second {'disabled (solo)' if a.solo else 'enabled'} -> strategy {a.strategy}")
