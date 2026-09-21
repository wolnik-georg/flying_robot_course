#!/usr/bin/env python3
"""Merge all pairable 2026-09-21 C.1 flights into experiments/logs/c1_2026-09-21_merged/.

Pairing is NOT same-index: THESIS1/THESIS2 card counters desync (aborts, empty files,
card swaps mid-day -- see experiments/logs/usd_raw/2026-09-21_PAIRING.md). For each radio
stamp we search THESIS1 x THESIS2 files within an index window around the expected
counter, score every candidate with merge_usd_logs.py --meta --roles bottom top, and keep
the first pair where BOTH sides report RMS < 15 cm. An (i, j) pair already accepted for one
A1 stamp is rejected for a different A1 stamp with a different params.dz -- the static-hold
trajectory is dz-invariant on the bottom (cf5) side, so a bottom-only fit cannot disambiguate
and reusing a file across two dz values is almost always a false positive (see 2026-09-21
desk notes: thesis15 spuriously "fits" 3 different dz values on RMS alone).
"""
from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOGS = ROOT / "experiments" / "logs"
T1 = LOGS / "usd_raw/2026-09-21_THESIS1"
T2 = LOGS / "usd_raw/2026-09-21_THESIS2"
OUT = LOGS / "c1_2026-09-21_merged"
MERGE = ROOT / "flying_drone_stack/tools/merge_usd_logs.py"
PY = "/home/georg/.pyenv/versions/flying_robots/bin/python"
PYTHONPATH_EXTRA = "/home/georg/Desktop/crazyswarm2/crazyflie_examples"

RMS_RE = re.compile(r"RMS\s+([\d.]+)\s*cm")
# Most archive filenames repeat the counter twice (thesisNN_thesisNN); the very first
# morning-A8 batch (thesis00/01/02) was copied before that convention and has it once.
# Match on the first occurrence either way.
IDX_RE = re.compile(r"thesis(\d+)")

# (scenario, stamp, expected_t1_idx or None, expected_t2_idx or None, search_radius)
# expected index seeds the search window; None means "search the whole card".
STAMPS = [
    ("A8", "12-11-05", 1, 1, 0),   # morning: T1=top card, T2=bottom card (see roles below)
    ("A8", "12-11-46", 2, 2, 0),
    ("A8", "13-14-36", 14, 14, 1),
    ("A1", "12-40-41", 4, 4, 5),
    ("A1", "12-43-34", 6, 6, 5),
    ("A1", "12-49-36", 7, 7, 5),
    ("A1", "12-51-16", 8, 8, 5),
    ("A3", "13-00-57", 9, 9, 1),
    ("A3", "13-02-56", 10, 10, 1),
    ("A3", "13-04-34", 11, 11, 1),
    ("A3", "13-05-58", 12, 12, 1),   # known: cf5 (T1) side missing entirely -> REFLY
    ("A1", "13-25-10", 15, 15, 5),
    ("A1", "13-27-27", 16, 16, 5),
    ("A1", "13-28-49", 17, 17, 5),
    ("A1", "13-30-30", 18, 18, 5),
]

# Morning A8 (~12:08-12:11): card roles are swapped relative to every other block --
# THESIS2 was in cf5 (bottom), THESIS1 was in cf_second (top). Everywhere else THESIS1=bottom.
MORNING_A8_STAMPS = {"12-11-05", "12-11-46"}

# Manually verified pairs where the correct (i, j) is NOT the same index on both cards.
# (scenario, stamp) -> (bottom_idx, top_idx)
KNOWN_PAIRS = {
    ("A1", "12-51-16"): (6, 8),
    ("A1", "13-25-10"): (15, 13),
}

# dz-ambiguity guard: an (i, j) validated for one A1 stamp must not be reused for another
# A1 stamp whose params.dz differs -- see module docstring.
CLAIMED_FOR_DZ = {}  # (i, j) -> (stamp, dz) that already claimed it


def idx_of(p: Path):
    m = IDX_RE.search(p.name)
    return int(m.group(1)) if m else None


def index_card(d: Path):
    out = {}
    for p in d.glob("*.bin"):
        i = idx_of(p)
        if i is None:
            continue
        out.setdefault(i, []).append(p)
    for i in out:
        # prefer the "_pm_" desk-copy name when duplicated; content is byte-identical
        out[i] = sorted(out[i], key=lambda x: ("_pm_" not in x.name, str(x)))
    return {i: v[0] for i, v in out.items()}


def run_merge(meta_path: Path, bottom: Path, top: Path, out_csv: Path | None):
    env = {"PYTHONPATH": PYTHONPATH_EXTRA, "PATH": "/usr/bin:/bin"}
    dest = str(out_csv) if out_csv else "/dev/null"
    cmd = [PY, str(MERGE), str(bottom), str(top), "--meta", str(meta_path),
           "--roles", "bottom", "top", "-o", dest]
    p = subprocess.run(cmd, capture_output=True, text=True, env=env, cwd=str(ROOT))
    text = p.stdout + p.stderr
    rms = [float(x) for x in RMS_RE.findall(text)]
    return p.returncode == 0, rms, text


def search_pair(scenario, stamp, meta_path, t1_idx, t2_idx, radius, t1_files, t2_files, dz,
                 used_bottom, used_top, bottom_tag, top_tag):
    """Try the known/expected pair first, then grid-search a window around it.

    Every physical file (identified by its own archive card + index) can back exactly one
    accepted stamp for the whole day -- `used_bottom`/`used_top` enforce that globally, not
    just within one scenario, since a uSD recording session is a single real flight.
    """
    key = (scenario, stamp)
    if key in KNOWN_PAIRS:
        bi, ti = KNOWN_PAIRS[key]
        if (bi in t1_files and ti in t2_files
                and (bottom_tag, bi) not in used_bottom and (top_tag, ti) not in used_top):
            ok, rms, text = run_merge(meta_path, t1_files[bi], t2_files[ti], None)
            if ok:
                return bi, ti, rms, text

    if t1_idx is None:
        candidates_i = sorted(t1_files)
    else:
        candidates_i = sorted((i for i in t1_files if abs(i - t1_idx) <= radius),
                               key=lambda i: abs(i - t1_idx))
    if t2_idx is None:
        candidates_j = sorted(t2_files)
    else:
        candidates_j = sorted((j for j in t2_files if abs(j - t2_idx) <= radius),
                               key=lambda j: abs(j - t2_idx))

    for i in candidates_i:
        if (bottom_tag, i) in used_bottom:
            continue
        for j in candidates_j:
            if (top_tag, j) in used_top:
                continue
            pair_key = (i, j)
            if scenario == "A1" and pair_key in CLAIMED_FOR_DZ:
                claimed_stamp, claimed_dz = CLAIMED_FOR_DZ[pair_key]
                if claimed_dz is not None and dz is not None and claimed_dz != dz:
                    continue  # ambiguous match already spent on a different dz -- reject
            ok, rms, text = run_merge(meta_path, t1_files[i], t2_files[j], None)
            if ok:
                if scenario == "A1":
                    CLAIMED_FOR_DZ[pair_key] = (stamp, dz)
                return i, j, rms, text
    return None, None, [], ""


def symlink_usd(raw: Path, dest_dir: Path, drone: str, stamp: str, tag: str) -> Path:
    dest_dir.mkdir(parents=True, exist_ok=True)
    link = dest_dir / f"{drone}_{tag}_thesis_{stamp}.usd"
    if link.exists() or link.is_symlink():
        link.unlink()
    link.symlink_to(raw.resolve())
    return link


def main() -> int:
    t1_files = index_card(T1)
    t2_files = index_card(T2)

    results = []
    manifest = []
    used_bottom = set()   # (card_tag, idx) already spent on an accepted stamp
    used_top = set()

    for scenario, stamp, t1_idx, t2_idx, radius in STAMPS:
        meta = LOGS / f"{scenario}_2026-09-21_{stamp}.meta.json"
        if not meta.is_file():
            results.append((scenario, stamp, "SKIP", "no meta.json"))
            continue
        meta_obj = json.loads(meta.read_text())
        dz = meta_obj.get("params", {}).get("dz")

        # morning A8: THESIS2 = bottom(cf5), THESIS1 = top(cf_second) -- swap the search
        if stamp in MORNING_A8_STAMPS:
            bottom_card, top_card = t2_files, t1_files
            bottom_tag, top_tag = "THESIS2", "THESIS1"
        else:
            bottom_card, top_card = t1_files, t2_files
            bottom_tag, top_tag = "THESIS1", "THESIS2"

        bi, ti, rms, text = search_pair(scenario, stamp, meta, t1_idx, t2_idx, radius,
                                         bottom_card, top_card, dz,
                                         used_bottom, used_top, bottom_tag, top_tag)
        if bi is None:
            results.append((scenario, stamp, "NO-PAIR", "no unused (i,j) in search window scored <15cm both sides"))
            continue
        used_bottom.add((bottom_tag, bi))
        used_top.add((top_tag, ti))

        pkg = OUT / f"{scenario}_2026-09-21_{stamp}"
        tag = f"{scenario}_{stamp}"
        bottom_path, top_path = bottom_card[bi], top_card[ti]
        lb = symlink_usd(bottom_path, pkg, "cf5", stamp, tag)
        lt = symlink_usd(top_path, pkg, "cf_second", stamp, tag)
        meta_dest = pkg / meta.name
        if not meta_dest.exists():
            meta_dest.write_bytes(meta.read_bytes())
        out_csv = pkg / f"{scenario}_2026-09-21_{stamp}_merged_usd.csv"
        ok, rms, text = run_merge(meta, lb, lt, out_csv)
        if not ok or not out_csv.is_file() or out_csv.stat().st_size < 1000:
            results.append((scenario, stamp, "FAIL", "merge failed after pairing"))
            continue
        results.append((scenario, stamp, "PASS", f"bottom_idx={bi} top_idx={ti} rms={rms}"))
        manifest.append({
            "scenario": scenario,
            "stamp": stamp,
            "path": str(out_csv.relative_to(ROOT)),
            "params": meta_obj.get("params", {}),
            "thesis_pair": {"card_bottom": bottom_tag, "card_top": top_tag,
                             "bottom_idx": bi, "top_idx": ti},
            "rms_cm": {"bottom": rms[0] if rms else None, "top": rms[1] if len(rms) > 1 else None},
        })
        print(f"\n=== {scenario} {stamp}: PASS (bottom={bi} top={ti} rms={rms}) ===")

    print("\n\n--- SUMMARY ---")
    for sc, st, status, msg in results:
        print(f"{sc} {st}: {status} — {msg}")

    manifest_path = OUT / "manifest_2026-09-21_c1.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"\nWrote {manifest_path} ({len(manifest)} training-eligible merges)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
