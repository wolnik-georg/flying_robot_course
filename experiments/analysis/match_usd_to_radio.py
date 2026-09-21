#!/usr/bin/env python3
"""Match uSD thesisNN files to formation radio logs for one lab day.

Uses merge alignment RMS plus radio↔uSD z cross-correlation (per meta sidecar).
Raw uSD archives are never modified — read-only inputs.

    python3 match_usd_to_radio.py \\
        --usd-card experiments/logs/usd_raw/2026-09-19_THESIS1 \\
        --usd-card experiments/logs/usd_raw/2026-09-19_THESIS2 \\
        --meta-glob 'experiments/logs/A8_2026-09-19_*.meta.json' \\
        --bottom-card 0 --top-card 1
"""
from __future__ import annotations

import argparse
import csv
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parents[2] / "flying_drone_stack" / "tools"
sys.path.insert(0, str(TOOLS))
from decode_usd_log import load  # noqa: E402
from find_flight_window import commanded_trajectory, find_offset  # noqa: E402


def thesis_files(card: Path) -> list[Path]:
    out = []
    for p in sorted(card.glob("thesis*")):
        if p.stat().st_size == 0:
            continue
        m = re.fullmatch(r"thesis(\d+)", p.name)
        if m:
            out.append((int(m.group(1)), p))
    return [p for _, p in sorted(out)]


def role_scores(usd: Path, meta: dict) -> dict[str, float]:
    total = meta["duration"]
    d = load(str(usd))
    t = np.asarray(d["t"], dtype=float)
    if t[-1] - t[0] < total:
        return {}
    pos = np.stack([d["x"], d["y"], d["z"]], axis=1)
    scores = {}
    for role in meta.get("roles", ("bottom", "top")):
        ts, cmd = commanded_trajectory(meta, role)
        lag, mse, _ = find_offset(t, pos, ts, cmd, t[0], t[-1] - total)
        if lag is not None:
            scores[role] = float(np.sqrt(mse) * 100.0)
    return scores


def load_radio_z(stem: Path) -> tuple[np.ndarray, np.ndarray]:
    """stem like experiments/logs/A8_2026-09-19_14-58-02 (no .meta.json)."""
    cols_t, cols_z = [], []
    for drone in ("cf5", "cf_second"):
        p = stem.parent / f"A8_{drone}_{stem.name.replace('A8_', '')}.csv"
        if not p.exists():
            # stem is A8_2026-09-19_14-58-02.meta.json parent + name
            pass
        p = list(stem.parent.glob(f"A8_{drone}_*.csv"))
        p = [x for x in p if stem.name.replace(".meta.json", "") in x.name]
        if len(p) != 1:
            raise FileNotFoundError(f"radio csv for {drone} @ {stem}")
        p = p[0]
        t, z = [], []
        with open(p) as fh:
            hdr = None
            for line in fh:
                if line.startswith("#"):
                    continue
                if hdr is None:
                    hdr = line.strip().split(",")
                    continue
                if not line.strip():
                    continue
                row = line.strip().split(",")
                t.append(float(row[0]))
                z.append(float(row[hdr.index("pos_z")]))
        cols_t.append(np.array(t))
        cols_z.append(np.array(z))
    return cols_t[0], cols_z[0], cols_t[1], cols_z[1]


def radio_for_meta(meta_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    stamp = meta_path.stem.replace("A8_", "").replace(".meta", "")
    logs = meta_path.parent
    out = {}
    for drone in ("cf5", "cf_second"):
        p = logs / f"A8_{drone}_{stamp}.csv"
        t, z = [], []
        with open(p) as fh:
            hdr = None
            for line in fh:
                if line.startswith("#"):
                    continue
                if hdr is None:
                    hdr = line.strip().split(",")
                    continue
                if line.strip():
                    row = line.strip().split(",")
                    t.append(float(row[0]))
                    z.append(float(row[hdr.index("pos_z")]))
        out[drone] = (np.array(t), np.array(z))
    return (*out["cf5"], *out["cf_second"])


def merge_and_score(
    bottom: Path,
    top: Path,
    meta_path: Path,
    t_cf5: np.ndarray,
    z_cf5: np.ndarray,
    t_cs: np.ndarray,
    z_cs: np.ndarray,
) -> dict | None:
    with tempfile.NamedTemporaryFile(suffix=".csv", delete=False) as tmp:
        out = Path(tmp.name)
    cmd = [
        sys.executable,
        str(TOOLS / "merge_usd_logs.py"),
        str(bottom),
        str(top),
        "--meta",
        str(meta_path),
        "--roles",
        "bottom",
        "top",
        "-o",
        str(out),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        out.unlink(missing_ok=True)
        return None
    text = proc.stdout + proc.stderr
    rms_b = rms_t = None
    for line in text.splitlines():
        if "role=bottom" in line and "RMS" in line:
            rms_b = float(line.split("RMS")[1].split("cm")[0].strip())
        if "role=top" in line and "RMS" in line:
            rms_t = float(line.split("RMS")[1].split("cm")[0].strip())
    if rms_b is None or rms_t is None or rms_b > 15 or rms_t > 15:
        out.unlink(missing_ok=True)
        return None

    # Parse merged CSV (thesis names as columns)
    lines = out.read_text().splitlines()
    hdr = None
    rows = []
    for line in lines:
        if line.startswith("#"):
            continue
        if hdr is None:
            hdr = line.split(",")
            continue
        if line.strip():
            rows.append(line.split(","))
    out.unlink(missing_ok=True)
    if not rows:
        return None
    data = {h: np.array([float(r[i]) for r in rows]) for i, h in enumerate(hdr)}
    t = data["t"]
    bname, tname = bottom.name, top.name
    zb = data[f"{bname}.z"]
    zt = data[f"{tname}.z"]

    def corr_radio(t_r, z_r, t_u, z_u):
        lo, hi = max(t_r[0], t_u[0]), min(t_r[-1], t_u[-1])
        if hi - lo < 2.0:
            return 0.0
        grid = np.arange(lo, hi, 0.05)
        R = np.interp(grid, t_r, z_r)
        U = np.interp(grid, t_u, z_u)
        R = R - R.mean()
        U = U - U.min()
        if R.std() < 1e-6 or U.std() < 1e-6:
            return 0.0
        return float(np.corrcoef(R, U)[0, 1])

    # Radio clock ≠ scenario clock — align radio by removing its t0, uSD merge is scenario-relative
    t_cf5 = t_cf5 - t_cf5[0]
    t_cs = t_cs - t_cs[0]
    # Scenario starts ~7s into radio log (takeoff/ramp); search lag
    best = -1.0
    for lag in np.arange(0, 15.0, 0.25):
        c1 = corr_radio(t_cf5 + lag, z_cf5, t, zb)
        c2 = corr_radio(t_cs + lag, z_cs, t, zt)
        best = max(best, c1 + c2)
    return dict(
        rms_b=rms_b,
        rms_t=rms_t,
        radio_score=best,
        n_rows=len(rows),
        merge_log=text,
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--usd-card", action="append", required=True, type=Path)
    ap.add_argument("--meta-glob", required=True)
    ap.add_argument("--bottom-card", type=int, default=0,
                    help="index into --usd-card for cf5 / bottom drone")
    ap.add_argument("--top-card", type=int, default=1)
    ap.add_argument("--out", type=Path, default=None, help="write JSON manifest")
    args = ap.parse_args()

    repo = Path(__file__).resolve().parents[2]
    metas = sorted(repo.glob(args.meta_glob))
    if not metas:
        sys.exit(f"no meta files for {args.meta_glob}")

    ref_meta = json.load(open(metas[0]))
    cards = [Path(c) for c in args.usd_card]
    def candidates(card: Path, role: str) -> list[Path]:
        out = []
        for p in thesis_files(card):
            sc = role_scores(p, ref_meta)
            if sc.get(role, 999.0) < 25:
                out.append(p)
        return out

    bottoms = candidates(cards[args.bottom_card], "bottom")
    tops = candidates(cards[args.top_card], "top")
    print(f"[match] {len(bottoms)} bottom candidates, {len(tops)} top candidates, "
          f"{len(metas)} radio metas")

    ref_meta_path = metas[0]
    radio_cache = {mp.name: radio_for_meta(mp) for mp in metas}

    def radio_score_from_merged(t, zb, zt, t5, z5, ts, zs):
        t5 = t5 - t5[0]
        ts = ts - ts[0]

        def corr_radio(t_r, z_r, t_u, z_u):
            lo, hi = max(t_r[0], t_u[0]), min(t_r[-1], t_u[-1])
            if hi - lo < 2.0:
                return 0.0
            grid = np.arange(lo, hi, 0.05)
            R = np.interp(grid, t_r, z_r)
            U = np.interp(grid, t_u, z_u)
            R = R - R.mean()
            U = U - U.min()
            if R.std() < 1e-6 or U.std() < 1e-6:
                return 0.0
            return float(np.corrcoef(R, U)[0, 1])

        best = -1.0
        for lag in np.arange(0, 15.0, 0.25):
            c1 = corr_radio(t5 + lag, z5, t, zb)
            c2 = corr_radio(ts + lag, zs, t, zt)
            best = max(best, c1 + c2)
        return best

    pair_cache = {}
    scores = {}
    for b in bottoms:
        for t in tops:
            pk = (b.name, t.name)
            if pk not in pair_cache:
                s = merge_and_score(
                    b, t, ref_meta_path,
                    *radio_cache[ref_meta_path.name],
                )
                if not s:
                    continue
                pair_cache[pk] = s
                print(f"  merge ok {b.name}+{t.name} rms={s['rms_b']:.1f}/{s['rms_t']:.1f} cm "
                      f"n={s['n_rows']}")
            base = pair_cache[pk]
            # Re-read merged z from a fresh merge is expensive; stash arrays in cache
            if "t" not in base:
                cmd = [
                    sys.executable, str(TOOLS / "merge_usd_logs.py"),
                    str(b), str(t), "--meta", str(ref_meta_path),
                    "--roles", "bottom", "top", "-o", "/tmp/_match_pair.csv",
                ]
                subprocess.run(cmd, capture_output=True, check=True)
                lines = Path("/tmp/_match_pair.csv").read_text().splitlines()
                hdr = None
                rows = []
                for line in lines:
                    if line.startswith("#"):
                        continue
                    if hdr is None:
                        hdr = line.split(",")
                        continue
                    if line.strip():
                        rows.append(line.split(","))
                data = {h: np.array([float(r[i]) for r in rows]) for i, h in enumerate(hdr)}
                base["t"] = data["t"]
                base["zb"] = data[f"{b.name}.z"]
                base["zt"] = data[f"{t.name}.z"]

            for mp in metas:
                t5, z5, ts, zs = radio_cache[mp.name]
                rs = radio_score_from_merged(base["t"], base["zb"], base["zt"], t5, z5, ts, zs)
                scores[(mp.name, b.name, t.name)] = dict(
                    radio_score=rs,
                    rms_b=base["rms_b"],
                    rms_t=base["rms_t"],
                    n_rows=base["n_rows"],
                )

    # One-to-one: walk metas in time order, best unused pair per meta
    used_b, used_t = set(), set()
    assignment = []
    for mp in metas:
        mn = mp.name
        best = None
        for b in bottoms:
            if b.name in used_b:
                continue
            for t in tops:
                if t.name in used_t:
                    continue
                key = (mn, b.name, t.name)
                if key not in scores:
                    continue
                sc = scores[key]
                if best is None or sc["radio_score"] > best[0]:
                    best = (sc["radio_score"], b.name, t.name, sc)
        if best is None:
            print(f"  WARN: no pair left for {mn}")
            continue
        _, bn, tn, sc = best
        used_b.add(bn)
        used_t.add(tn)
        assignment.append(dict(meta=mn, bottom_usd=bn, top_usd=tn, **sc))

    print(f"\n[match] assigned {len(assignment)}/{len(metas)} flights")
    for a in assignment:
        print(f"  {a['meta']}: {a['bottom_usd']} + {a['top_usd']} "
              f"(radio={a['radio_score']:.3f}, rms {a['rms_b']:.1f}/{a['rms_t']:.1f} cm)")

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        payload = dict(
            bottom_card=str(cards[args.bottom_card]),
            top_card=str(cards[args.top_card]),
            assignments=assignment,
        )
        args.out.write_text(json.dumps(payload, indent=2) + "\n")
        print(f"[match] wrote {args.out}")


if __name__ == "__main__":
    main()
