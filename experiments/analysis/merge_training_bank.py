#!/usr/bin/env python3
"""Merge a block of two-vehicle flights into a retrain bank in one command (pairing by usd.runTag, Omar-label reconstruction).

  python3 experiments/analysis/merge_training_bank.py --date 2026-10-15 --out experiments/logs/bank_2026-10-15 \\
      [--after 17-30] [--before 19-00] [--scenarios A1 A8] [--only-tag-contains tree_] [--rpm deck]

For every `experiments/logs/<SCEN>_<date>_<HH-MM-SS>.meta.json` in the window it takes `usd_run_tag`, finds the cf5 and the cf_second file
in `experiments/logs/usd_raw` carrying that tag (decoded from the log, not from the name), merges them with `merge_usd_logs.py --meta --roles
bottom top --omar-labels <rpm> --tag-quality-only` and writes `<out>/<SCEN>_<date>_<stamp>/..._merged_usd.csv` plus a manifest
`<out>/training_manifest.json` in the format `run_c2_lab_retrain.py` reads. Flights without a tag match or with an empty/short log
are listed and skipped, never merged silently. Needs the system python3 (3.10 host bindings for the label step)."""
import argparse, glob, json, re, subprocess, sys
sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]; TOOLS = ROOT / "flying_drone_stack/tools"
sys.path.insert(0, str(TOOLS)); import decode_usd_log  # noqa: E402

def tag_of(p):
    try:
        d = decode_usd_log.load(str(p))
        if len(d["t"]) < 1000 or "run_tag" not in d: return None, len(d["t"])
        u = sorted({int(x) for x in d["run_tag"] if x == x and x > 0})
        return (u[0] if len(u) == 1 else None), len(d["t"])
    except Exception as e:
        return None, 0

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--date", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--after", default="00-00"); ap.add_argument("--before", default="23-59")
    ap.add_argument("--scenarios", nargs="*"); ap.add_argument("--only-tag-contains", default="")
    ap.add_argument("--rpm", default="deck", choices=["deck", "dshot", "rpmf"])
    ap.add_argument("--raw", default=str(ROOT / "experiments/logs/usd_raw"))
    ap.add_argument("--allow-short", type=float, default=0.0, help="pass-through to merge_usd_logs.py (seconds a recording may be shorter than the scenario)")
    a = ap.parse_args(); out = Path(a.out).resolve(); out.mkdir(parents=True, exist_ok=True)
    idx = {"cf5": {}, "cf_second": {}}
    for drone in idx:
        for p in sorted(Path(a.raw).glob(f"{drone}_*.bin")):
            if a.only_tag_contains and a.only_tag_contains not in p.name: continue
            t, n = tag_of(p)
            if t: idx[drone].setdefault(t, []).append(p)
    flights, skipped = [], []
    for mp in sorted((ROOT / "experiments/logs").glob(f"*_{a.date}_*.meta.json")):
        m = re.match(r"([A-Z]\d)_(\d{4}-\d{2}-\d{2})_(\d{2}-\d{2})-(\d{2})\.meta\.json", mp.name)
        if not m: continue
        scen, date, hm, ss = m.groups(); stamp = f"{hm}-{ss}"
        if a.scenarios and scen not in a.scenarios: continue
        if not (a.after <= hm <= a.before): continue
        tag = json.load(open(mp)).get("usd_run_tag")
        c5, c2 = idx["cf5"].get(tag, []), idx["cf_second"].get(tag, [])
        if not tag or len(c5) != 1 or len(c2) != 1:
            skipped.append((mp.name, f"tag {tag}: cf5 {len(c5)} / cf_second {len(c2)} matching files")); continue
        folder = f"{scen}_{date}_{stamp}"; d = out / folder; d.mkdir(exist_ok=True); dest = d / f"{folder}_merged_usd.csv"
        cmd = [sys.executable, str(TOOLS / "merge_usd_logs.py"), str(c5[0]), str(c2[0]), "--meta", str(mp), "--roles", "bottom", "top",
               "--omar-labels", a.rpm, "--tag-quality-only", "-o", str(dest)] + (["--allow-short", str(a.allow_short)] if a.allow_short else [])
        r = subprocess.run(cmd, capture_output=True, text=True)
        rms = [l.strip() for l in (r.stdout + r.stderr).split("\n") if "RMS" in l][:2]
        if r.returncode != 0 or not dest.exists():
            skipped.append((mp.name, f"merge rc {r.returncode}: {(r.stderr or r.stdout).strip().splitlines()[-1:]}")); continue
        from bank_labels import zero_top_labels; zero_top_labels(dest)
        flights.append({"folder": folder, "scenario": scen, "date": date, "stamp": stamp, "path": str(dest.relative_to(ROOT)),
                        "run_tag": tag, "cf5": c5[0].name, "cf_second": c2[0].name})
        print(f"{folder}  tag {tag}  {c5[0].name} + {c2[0].name}  {rms}")
    man = {"date": a.date, "note": f"merged by merge_training_bank.py, omar labels {a.rpm}", "n_paths": len(flights), "flights": flights}
    (out / "training_manifest.json").write_text(json.dumps(man, indent=1) + "\n")
    print(f"\n{len(flights)} flights merged -> {out}/training_manifest.json; skipped {len(skipped)}")
    for n, why in skipped: print("  skipped", n, "-", why)

if __name__ == "__main__":
    main()
