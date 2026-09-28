#!/usr/bin/env python3
"""Generate plot_flight + plot_interaction PNGs for every merged 2-drone uSD log.

Discovers flights from C.1 manifests (2026-09-21 / 2026-09-23) and the 2026-09-19 A8
success archive. Skips solo merges (e.g. C5). Writes stable filenames under:

    experiments/analysis/out/two_drone_plot_gallery/

    {scenario}_{date}_{stamp}_dashboard.png
    {scenario}_{date}_{stamp}_interaction.png

Uses pyenv `flying_robots` for matplotlib (same as README).

    ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/batch_plot_two_drone_merged.py
"""
from __future__ import annotations

import json
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PYENV = Path.home() / ".pyenv/versions/flying_robots/bin/python"
GALLERY = ROOT / "experiments/analysis/out/two_drone_plot_gallery"
PLOT_FLIGHT = Path(__file__).parent / "plot_flight.py"
PLOT_IX = Path(__file__).parent / "plot_interaction.py"

sys.path.insert(0, str(Path(__file__).resolve().parent))
import metrics as M  # noqa: E402

# plot_flight.py only accepts geometric | indi (metrics label, not study_controller verbatim)
A8_STUDY_TO_PLOT_CTRL = {
    "geometric": "geometric",
    "full_indi": "indi",
    "stock_lee": "geometric",
}


@dataclass
class Flight:
    scenario: str
    date: str
    stamp: str
    merged: Path
    meta: Path
    ctrl: str
    study_controller: str | None
    bottom: str
    top: str
    source: str = "c1_manifest"

    @property
    def flight_id(self) -> str:
        return f"{self.scenario}_{self.date}_{self.stamp}"


def resolve_meta(merged: Path, entry: dict) -> Path | None:
    fdir = merged.parent
    stem = merged.stem.replace("_merged_usd", "")
    for cand in (
        fdir / f"{stem}.meta.json",
        ROOT / "experiments/logs" / f"{stem}.meta.json",
    ):
        if cand.is_file():
            return cand
    sc = entry.get("scenario")
    stamp = entry.get("stamp")
    if sc and stamp:
        for date in ("2026-09-21", "2026-09-23", "2026-09-19"):
            flat = ROOT / "experiments/logs" / f"{sc}_{date}_{stamp}.meta.json"
            if flat.is_file():
                return flat
    return None


def meta_is_two_drone(meta_path: Path) -> tuple[bool, list[str]]:
    meta = json.loads(meta_path.read_text())
    names = meta.get("names") or []
    return len(names) >= 2, names


def merged_vehicle_keys(merged: Path, logical_names: list[str]) -> tuple[str, str]:
    """Map meta logical names (cf5, cf_second) to merge_usd column prefixes."""
    raw = M.load_merged_csv(merged)
    keys = list(raw.keys())

    def one(logical: str) -> str:
        if logical in keys:
            return logical
        for k in keys:
            if k.startswith(f"{logical}_"):
                return k
        raise KeyError(f"{logical!r} not in merged keys {keys}")

    return one(logical_names[0]), one(logical_names[1])


def dz_from_meta(meta_path: Path) -> float | None:
    meta = json.loads(meta_path.read_text())
    p = meta.get("params") or {}
    if "dz" in p:
        return float(p["dz"])
    return None


def iter_c1_manifest(manifest: Path, date: str) -> list[Flight]:
    out: list[Flight] = []
    for entry in json.loads(manifest.read_text()):
        if entry.get("merge_status") not in (None, "merged"):
            continue
        merged = ROOT / entry["path"]
        if not merged.is_file():
            continue
        meta = resolve_meta(merged, entry)
        if meta is None:
            print(f"[skip] no meta for {merged.name}", file=sys.stderr)
            continue
        ok2, names = meta_is_two_drone(meta)
        if not ok2:
            continue
        bottom, top = names[0], names[1]
        out.append(
            Flight(
                scenario=entry["scenario"],
                date=date,
                stamp=entry["stamp"],
                merged=merged,
                meta=meta,
                ctrl="geometric",
                study_controller="geometric",
                bottom=bottom,
                top=top,
            )
        )
    return out


def iter_a8_2026_09_19() -> list[Flight]:
    session = ROOT / "experiments/logs/a8_2026-09-19_successful/session_manifest.json"
    if not session.is_file():
        return []
    out: list[Flight] = []
    for pkg in json.loads(session.read_text()).get("flights") or []:
        if pkg.get("status") != "success":
            continue
        stamp_full = pkg["stamp"]  # 2026-09-19_14-58-02
        fdir = ROOT / "experiments/logs/a8_2026-09-19_successful" / stamp_full
        merged = fdir / f"A8_{stamp_full}_merged_usd.csv"
        meta = fdir / f"A8_{stamp_full}.meta.json"
        if not merged.is_file() or not meta.is_file():
            continue
        ok2, names = meta_is_two_drone(meta)
        if not ok2:
            continue
        study = pkg.get("study_controller", "geometric")
        ctrl = A8_STUDY_TO_PLOT_CTRL.get(study, "geometric")
        short = stamp_full.split("_", 1)[1]
        out.append(
            Flight(
                scenario="A8",
                date="2026-09-19",
                stamp=short,
                merged=merged,
                meta=meta,
                ctrl=ctrl,
                study_controller=study,
                bottom=names[0],
                top=names[1],
                source="a8_2026-09-19",
            )
        )
    return out


def collect_flights() -> list[Flight]:
    flights: list[Flight] = []
    flights.extend(
        iter_c1_manifest(
            ROOT / "experiments/logs/c1_2026-09-21_merged/manifest_2026-09-21_c1.json",
            "2026-09-21",
        )
    )
    flights.extend(
        iter_c1_manifest(
            ROOT / "experiments/logs/c1_2026-09-23_merged/manifest_2026-09-23_c1.json",
            "2026-09-23",
        )
    )
    flights.extend(iter_a8_2026_09_19())
    # de-dupe by merged path
    seen: set[str] = set()
    uniq: list[Flight] = []
    for f in flights:
        key = str(f.merged.resolve())
        if key in seen:
            continue
        seen.add(key)
        uniq.append(f)
    return sorted(uniq, key=lambda f: (f.scenario, f.date, f.stamp))


def run_one(f: Flight, tmp: Path) -> dict:
    rep = {
        "flight_id": f.flight_id,
        "merged": str(f.merged.relative_to(ROOT)),
        "ctrl": f.ctrl,
        "study_controller": f.study_controller,
        "dashboard": None,
        "interaction": None,
        "dashboard_ok": False,
        "interaction_ok": False,
    }
    if not PYENV.is_file():
        rep["error"] = f"missing pyenv python: {PYENV}"
        return rep

    tmp.mkdir(parents=True, exist_ok=True)
    pf_args = [
        str(PYENV),
        str(PLOT_FLIGHT),
        "--scenario",
        f.scenario,
        "--ctrl",
        f.ctrl,
        "--logs",
        str(f.merged),
        "--sidecar",
        str(f.meta),
        "--source",
        "hardware",
        "--out",
        str(tmp),
    ]
    dz = dz_from_meta(f.meta)
    if dz is not None:
        pf_args.extend(["--dz-cmd", str(dz)])
    pr = subprocess.run(pf_args, capture_output=True, text=True)
    rep["dashboard_ok"] = pr.returncode == 0
    if not rep["dashboard_ok"]:
        rep["dashboard_stderr"] = (pr.stdout + pr.stderr)[-800:]
        return rep

    dashboards = sorted(tmp.glob(f"{f.scenario}_{f.ctrl}_*_dashboard.png"))
    if not dashboards:
        rep["dashboard_ok"] = False
        rep["dashboard_stderr"] = "plot_flight succeeded but no PNG found"
        return rep
    dest_dash = GALLERY / f"{f.flight_id}_dashboard.png"
    shutil.copy2(dashboards[-1], dest_dash)
    rep["dashboard"] = str(dest_dash.relative_to(ROOT))

    try:
        bottom_k, top_k = merged_vehicle_keys(f.merged, [f.bottom, f.top])
    except KeyError as e:
        rep["interaction_ok"] = False
        rep["interaction_stderr"] = str(e)
        return rep

    dest_ix = GALLERY / f"{f.flight_id}_interaction.png"
    pi = subprocess.run(
        [
            str(PYENV),
            str(PLOT_IX),
            str(f.merged),
            "--bottom",
            bottom_k,
            "--top",
            top_k,
            "--out",
            str(dest_ix),
        ],
        capture_output=True,
        text=True,
    )
    rep["interaction_ok"] = pi.returncode == 0
    if rep["interaction_ok"]:
        rep["interaction"] = str(dest_ix.relative_to(ROOT))
    else:
        rep["interaction_stderr"] = (pi.stdout + pi.stderr)[-800:]
    return rep


def main() -> int:
    flights = collect_flights()
    GALLERY.mkdir(parents=True, exist_ok=True)
    tmp = GALLERY / "_tmp_plot_flight"
    if tmp.exists():
        shutil.rmtree(tmp)

    print(f"[batch] {len(flights)} two-drone merged flights -> {GALLERY}")
    manifest_rows = []
    n_fail = 0
    for i, f in enumerate(flights, 1):
        print(f"[{i}/{len(flights)}] {f.flight_id} ({f.source}) ctrl={f.ctrl}")
        row = run_one(f, tmp)
        manifest_rows.append(row)
        if not row.get("dashboard_ok"):
            n_fail += 1

    if tmp.exists():
        shutil.rmtree(tmp)

    index = {
        "gallery": str(GALLERY.relative_to(ROOT)),
        "n_flights": len(flights),
        "n_dashboard_fail": sum(1 for r in manifest_rows if not r.get("dashboard_ok")),
        "n_interaction_fail": sum(1 for r in manifest_rows if not r.get("interaction_ok")),
        "flights": manifest_rows,
    }
    index_path = GALLERY / "index.json"
    index_path.write_text(json.dumps(index, indent=2) + "\n")
    print(f"wrote {index_path}")
    readme = GALLERY / "README.md"
    readme.write_text(
        "# Two-drone merged uSD plot gallery\n\n"
        "Auto-generated by `experiments/analysis/batch_plot_two_drone_merged.py`.\n\n"
        f"- **Flights:** {len(flights)}\n"
        "- **Dashboard PNGs:** `{scenario}_{date}_{stamp}_dashboard.png`\n"
        "- **Interaction PNGs:** `{scenario}_{date}_{stamp}_interaction.png`\n"
        "- **Index:** `index.json`\n\n"
        "Regenerate:\n\n"
        "```bash\n"
        "~/.pyenv/versions/flying_robots/bin/python "
        "experiments/analysis/batch_plot_two_drone_merged.py\n"
        "```\n",
    )
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
