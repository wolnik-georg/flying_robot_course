#!/usr/bin/env python3
"""Relabel the 26-flight v1 training bank with the tree residual definition (Omar's filter chain, deck RPM).

  python3 experiments/analysis/relabel_v1_bank.py [--manifest experiments/analysis/out/c2_e2e_2026-10-01/training_manifest_lab.json]
         [--out experiments/logs/bank_v1_relabelled] [--rpm deck]

Per flight: scenario meta -> usd_run_tag -> raw cf5 / cf_second logs in experiments/logs/usd_raw/<date>_THESIS1 / _THESIS2 ->
merge_usd_logs.py --run-tag ... --meta ... --omar-labels <rpm>. QA per flight: correlation of the new a_res_z with the label logged
by the old controller (best lag within +-3 s, the two merges use different time bases). Writes <out>/training_manifest.json and qa.json.
Needs the system python3 (3.10 host bindings)."""
import argparse, json, subprocess, sys
sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[2]; TOOLS = ROOT / "flying_drone_stack/tools"

def load(p):
    L = [l for l in open(p) if not l.startswith("#")]; h = L[0].strip().split(",")
    a = np.loadtxt(L[1:], delimiter=",", ndmin=2); return {k: a[:, i] for i, k in enumerate(h)}

# flights from before the run-tag firmware (or whose tag is not in the date archive): explicit pairs from the pairing notes
# (usd_raw/2026-09-21_PAIRING.md, 2026-09-23_PAIRING.md); bottom file first. The merge RMS printed per flight is the check.
U = "experiments/logs/usd_raw/"
MANUAL = {
    "A1_2026-09-21_12-51-16": ("2026-09-21_THESIS1/cf5__thesis06_thesis06_2026-09-21_12-56-14.bin", "2026-09-21_THESIS2/cf_second__A1_thesis08_thesis08_2026-09-21_12-54-58.bin"),
    "A3_2026-09-21_13-00-57": ("2026-09-21_THESIS1/cf5_pm_thesis09_thesis09_2026-09-21_13-18-45.bin", "2026-09-21_THESIS2/cf_second_pm_thesis09_thesis09_2026-09-21_13-17-55.bin"),
    "A3_2026-09-21_13-02-56": ("2026-09-21_THESIS1/cf5_pm_thesis10_thesis10_2026-09-21_13-18-45.bin", "2026-09-21_THESIS2/cf_second_pm_thesis10_thesis10_2026-09-21_13-17-56.bin"),
    "A3_2026-09-21_13-04-34": ("2026-09-21_THESIS1/cf5_pm_thesis11_thesis11_2026-09-21_13-18-45.bin", "2026-09-21_THESIS2/cf_second_pm_thesis11_thesis11_2026-09-21_13-17-56.bin"),
    # the two 09-23 A1 repeats: the exact files the v1 merge used are kept next to its merged CSV (.usd copies)
    "A1_2026-09-23_17-17-26": ("../c1_2026-09-23_merged/A1_2026-09-23_17-17-26/cf5_thesis19.usd", "../c1_2026-09-23_merged/A1_2026-09-23_17-17-26/cf_second_thesis19.usd"),
    "A1_2026-09-23_17-18-48": ("../c1_2026-09-23_merged/A1_2026-09-23_17-18-48/cf5_thesis20.usd", "../c1_2026-09-23_merged/A1_2026-09-23_17-18-48/cf_second_thesis20.usd"),
}

def qa(old_csv, new_csv):
    o, n = load(old_csv), load(new_csv); res = {}
    for who in ("cf5", "cf_second"):
        ko = [k for k in o if k.endswith(".a_res_z") and k.startswith(who)]; kn = [k for k in n if k.endswith(".a_res_z") and k.startswith(who)]
        if not ko or not kn: continue
        tg = np.arange(max(o["t"][0], n["t"][0]) + 4, min(o["t"][-1], n["t"][-1]) - 4, 0.01)
        if len(tg) < 200: continue
        y1 = np.interp(tg, o["t"], o[ko[0]]); best = (-2, 0)
        for lag in np.arange(-3, 3.001, 0.02):
            c = np.corrcoef(y1, np.interp(tg + lag, n["t"], n[kn[0]]))[0, 1]
            if c > best[0]: best = (c, lag)
        y2 = np.interp(tg + best[1], n["t"], n[kn[0]])
        res[who] = dict(r=round(float(best[0]), 3), lag_s=round(float(best[1]), 2), std_old=round(float(y1.std()), 3),
                        std_new=round(float(y2.std()), 3), rms_diff=round(float(np.sqrt(np.mean((y1 - y2) ** 2))), 3))
    return res

_IDX = {}
def IDX(d):
    if d not in _IDX:
        sys.path.insert(0, str(TOOLS)); import index_usd_archive
        _IDX[d] = index_usd_archive.index_directory(Path(d))["by_run_tag"]
    return _IDX[d]

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--manifest", default="experiments/analysis/out/c2_e2e_2026-10-01/training_manifest_lab.json")
    ap.add_argument("--out", default="experiments/logs/bank_v1_relabelled"); ap.add_argument("--rpm", default="deck", choices=["deck", "dshot", "rpmf"])
    a = ap.parse_args(); out = (ROOT / a.out).resolve(); out.mkdir(parents=True, exist_ok=True)
    man = json.load(open(ROOT / a.manifest)); keep, skipped, QA = [], [], {}
    for f in man["flights"]:
        scen, date, stamp = f["scenario"], f["date"], f["stamp"]
        mp = ROOT / f"experiments/logs/{scen}_{date}_{stamp}.meta.json"
        t1, t2 = ROOT / f"experiments/logs/usd_raw/{date}_THESIS1", ROOT / f"experiments/logs/usd_raw/{date}_THESIS2"
        if not mp.exists() or (f["folder"] not in MANUAL and (not t1.is_dir() or not t2.is_dir())):
            skipped.append((f["folder"], f"meta {mp.exists()} archives {t1.is_dir()}/{t2.is_dir()}")); continue
        tag = json.load(open(mp)).get("usd_run_tag")
        d = out / f["folder"]; d.mkdir(exist_ok=True); dest = d / f"{f['folder']}_merged_usd.csv"
        if f["folder"] in MANUAL:
            bot, top = (ROOT / (U + x) for x in MANUAL[f["folder"]])
            cmd = [sys.executable, str(TOOLS / "merge_usd_logs.py"), str(bot), str(top), "--meta", str(mp), "--roles", "bottom", "top",
                   "--omar-labels", a.rpm, "--tag-quality-only", "-o", str(dest)]
        elif tag:
            # the cards swap between the drones from flight to flight: resolve both files by tag and assign the roles by the file name
            # (cf5_* = bottom, cf_second_* = top), not by card, otherwise each drone is aligned against the other's trajectory
            hit = [IDX(t1).get(str(tag)), IDX(t2).get(str(tag))]
            if not all(hit):
                skipped.append((f["folder"], f"run_tag {tag} not in both archives")); continue
            paths = sorted((Path(h["path"]) for h in hit), key=lambda q: 0 if q.name.startswith("cf5") else 1)
            if not (paths[0].name.startswith("cf5") and paths[1].name.startswith("cf_second")):
                skipped.append((f["folder"], f"cannot assign roles from names: {[q.name for q in paths]}")); continue
            cmd = [sys.executable, str(TOOLS / "merge_usd_logs.py"), str(paths[0]), str(paths[1]), "--meta", str(mp), "--roles", "bottom", "top",
                   "--omar-labels", a.rpm, "--tag-quality-only", "-o", str(dest)]
        else:
            skipped.append((f["folder"], "no usd_run_tag in meta and no manual pair")); continue
        r = subprocess.run(cmd, capture_output=True, text=True)
        rms = [l.strip()[:90] for l in (r.stdout + r.stderr).split("\n") if "RMS" in l][:2]
        if r.returncode != 0 or not dest.exists():
            skipped.append((f["folder"], f"merge rc {r.returncode}: {(r.stderr or r.stdout).strip().splitlines()[-1:]}")); continue
        from bank_labels import zero_top_labels; zero_top_labels(dest)
        q = qa(ROOT / f["path"], dest); QA[f["folder"]] = {**q, "merge_rms": rms}
        keep.append({**{k: f[k] for k in ("folder", "scenario", "date", "stamp")}, "path": str(dest.relative_to(ROOT)), "run_tag": tag})
        print(f"{f['folder']:28s} " + "  ".join(f"{w}: r={v['r']:.3f} lag {v['lag_s']:+.2f}s rms {v['rms_diff']:.2f}/std {v['std_old']:.2f}" for w, v in q.items() if w == "cf5") + "  | " + " / ".join(x.split("RMS")[-1].strip() for x in rms), flush=True)
    (out / "training_manifest.json").write_text(json.dumps({"date": "v1 bank relabelled", "note": f"omar labels {a.rpm}", "n_paths": len(keep), "flights": keep}, indent=1) + "\n")
    (out / "qa.json").write_text(json.dumps(QA, indent=1) + "\n")
    print(f"\n{len(keep)} of {len(man['flights'])} relabelled -> {out}; skipped {len(skipped)}")
    for n, why in skipped: print("  skipped", n, "-", why)

if __name__ == "__main__":
    main()
