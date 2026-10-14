#!/usr/bin/env python3
"""Generate the four study yaml patches of the Omar C controller tree (S0..S3) against the committed crazyflies.yaml.

Both drones are PINNED to controller 9 (per-robot firmware_params), so takeoff and landing also run Omar's controller
(the formation framework only switches to its ramp controller 6 for drones without a per-robot pin). cf_second always runs
Omar geometric (indi 0, NS2 off). cf5 per strategy:  S0 indi 0 / NS2 off,  S1 indi 3 / off,  S2 indi 0 / on,  S3 indi 3 / on.
Kpos_Iz 1.5 in all of them (shared parameters stay Omar's defaults: Kpos_P 7, Kpos_D 4, KR, Kw, mass 0.0427).

  python3 experiments/analysis/study_yaml_tree_variants.py            # writes crazyswarm2/crazyflie/config/study/tree_S{0..3}.patch
Each patch applies on the committed (clean) yaml:  git checkout crazyflie/config/crazyflies.yaml && git apply crazyflie/config/study/tree_S1.patch
"""
from __future__ import annotations

import difflib
import subprocess
import sys
from pathlib import Path

CS2 = Path("/home/georg/Desktop/crazyswarm2")
YAML = "crazyflie/config/crazyflies.yaml"
OUT = CS2 / "crazyflie/config/study"
STRAT = {"S0": (0, 0), "S1": (3, 0), "S2": (0, 1), "S3": (3, 1)}   # (ctrlOmarIndi.indi, rnn.en) on cf5


def head_yaml() -> str:
    return subprocess.check_output(["git", "-C", str(CS2), "show", f"HEAD:{YAML}"], text=True)


def once(s: str, old: str, new: str) -> str:
    assert s.count(old) == 1, f"anchor not unique/found ({s.count(old)}x): {old[:70]!r}"
    return s.replace(old, new, 1)


def make(base: str, indi: int, en: int) -> str:
    s = base
    # --- cf_second: pinned to Omar geometric ---
    s = once(s, "        controller: 6   # 2026-10-02: switched from permanent stock-Lee pin (controller=5) to",
             "        controller: 9   # 2026-10-14 Omar C tree (study S0-S3): top drone = Omar geometric (indi 0). Was: 2026-10-02: switched from permanent stock-Lee pin (controller=5) to")
    s = once(s, "      indi_gains:\n        ctrl_mode: 0    # plain geometric (not full INDI) -- now live since controller=6.\n",
             "      ctrlOmarIndi:\n        indi: 0         # 2026-10-14 Omar tree: top drone is Omar geometric in every strategy\n        Kpos_Iz: 1.5\n"
             "      rnn:\n        en: 0           # NS2 only on the bottom drone (cf5)\n"
             "      indi_gains:\n        ctrl_mode: 0    # plain geometric (not full INDI) -- now live since controller=6.\n")
    # --- cf5 ---
    s = once(s, "        controller: 6   # DEFAULT = geometric (+NS2). Omar INDI of the study",
             "        controller: 9   # 2026-10-14 Omar C tree (study S0-S3), pinned so takeoff/landing also run Omar. Was: DEFAULT = geometric (+NS2). Omar INDI of the study")
    s = once(s, "        indi: 3\n        Kpos_Iz: 1.5    # study value (docs/study_parameter_decisions.md); 0 = Omar exact\n",
             f"        indi: {indi}         # 2026-10-14 tree strategy: 0 = Omar geometric, 3 = Omar full INDI\n        Kpos_Iz: 1.5    # study value (docs/study_parameter_decisions.md); 0 = Omar exact\n")
    s = once(s, "      rnn:\n        en: 0           # TEMP: network off for non-NS2 blocks\n",
             f"      rnn:\n        en: {en}           # 2026-10-14 tree strategy: 1 = NS2 (C, flash weights ns2_v1) on top of Omar\n")
    return s


def main() -> int:
    base = head_yaml()
    OUT.mkdir(parents=True, exist_ok=True)
    for sid, (indi, en) in STRAT.items():
        new = make(base, indi, en)
        diff = "".join(difflib.unified_diff(base.splitlines(True), new.splitlines(True),
                                            fromfile=f"a/{YAML}", tofile=f"b/{YAML}", n=3))
        p = OUT / f"tree_{sid}.patch"
        p.write_text(diff)
        r = subprocess.run(["git", "-C", str(CS2), "apply", "--check", str(p)], capture_output=True, text=True)
        print(f"{p.name}: {'applies' if r.returncode == 0 else 'FAILS ' + r.stderr[:200]}  (cf5 indi {indi}, rnn.en {en})")
        if r.returncode:
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
