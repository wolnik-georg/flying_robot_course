#!/usr/bin/env python3
"""Coverage of the NS2 training bank in the NS2 input space, and what the not-yet-flown card scenarios would add.

Bank flights (measured, outliers excluded) vs. planned scenarios (commanded, via the scenario generator): seconds inside the NS2 gate
(|dx|,|dy| < 0.2 m, |dvx| < 1.5 m/s, top above) split by lateral offset, vertical gap and bottom-vehicle speed. Also writes a per-flight metrics CSV.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/ns2_bank_coverage.py \\
      --bank experiments/logs/bank_ns2train_2026-10-16 --out experiments/results/ns2_collection_2026-10-16
Needs the crazyswarm2 checkout at ~/Desktop/crazyswarm2 (scenario generator). Bank speed = commanded bottom speed (ctrltarget_v*); the merged
files are 500 Hz. Planned scenarios use commanded gaps; the real gap is larger in a held stack (the bottom vehicle sags 15-35 cm)."""
import argparse, json, re, sys
from pathlib import Path
import numpy as np, pandas as pd

ROOT = Path(__file__).resolve().parents[2]
CS2 = Path.home() / "Desktop/crazyswarm2/crazyflie_examples"; sys.path.insert(0, str(CS2))
from crazyflie_examples.formations import safety, scenarios  # noqa: E402
from crazyflie_examples.run_formation import centred_anchor  # noqa: E402
lim = safety.Limits()
RB = [0, .05, .10, .15, .20]; DB = [0, .35, .45, .6, .8, 2]; SB = [0, .1, .25, .4, 2]

def bins(r, dz, sp, dt, gate):
    return {"r": np.histogram(r[gate], RB)[0] * dt, "dz": np.histogram(dz[gate], DB)[0] * dt,
            "sp": np.histogram(sp[gate], SB)[0] * dt, "joint": np.histogram2d(r[gate], dz[gate], [RB, DB])[0] * dt}

def planned(name, **kw):
    sc = scenarios.build(name, **kw); base = centred_anchor(sc, lim, 0.5)
    ts, pos = safety.sample_positions(sc, n=int(sc.duration * 50), base=base); pos = np.asarray(pos); dt = ts[1] - ts[0]
    b, t = pos[:, 0], pos[:, 1]; d = t - b; v = np.gradient(b, dt, axis=0); vt = np.gradient(t, dt, axis=0)
    gate = (abs(d[:, 0]) < .2) & (abs(d[:, 1]) < .2) & (abs(vt[:, 0] - v[:, 0]) < 1.5) & (d[:, 2] > 0)
    return bins(np.hypot(d[:, 0], d[:, 1]), d[:, 2], np.hypot(v[:, 0], v[:, 1]), dt, gate), gate.sum() * dt

def bank_flight(f, excl):
    m = json.load(open(ROOT / f"experiments/logs/{f['scenario']}_{f['date']}_{f['stamp']}.meta.json")); du = m["duration"]
    hdr = pd.read_csv(ROOT / f["path"], comment="#", nrows=0).columns
    B = [c for c in hdr if c.startswith("cf5_")][0].split(".")[0] + "."; T = B.replace("cf5_", "cf_second_")
    d = pd.read_csv(ROOT / f["path"], comment="#"); w = d[(d.t >= 0) & (d.t <= du)]; t = w.t.values; dt = float(np.median(np.diff(t)))
    g = lambda p, c: w[p + c].values
    dx, dy, dz = g(T, "locsrv_x") - g(B, "locsrv_x"), g(T, "locsrv_y") - g(B, "locsrv_y"), g(T, "locsrv_z") - g(B, "locsrv_z")
    k = max(1, int(0.1 / dt)); sm = lambda a: np.convolve(a, np.ones(k) / k, "same")
    vbx, vtx = np.gradient(sm(g(B, "locsrv_x")), t), np.gradient(sm(g(T, "locsrv_x")), t)
    sp = np.hypot(g(B, "ctrltarget_vx"), g(B, "ctrltarget_vy"))
    gate = (abs(dx) < .2) & (abs(dy) < .2) & (abs(vtx - vbx) < 1.5) & (dz > 0)
    ez, a = 100 * g(B, "omar_err_pos_z"), g(B, "a_res_z"); bz = g(B, "locsrv_z")
    row = dict(scenario=f["scenario"], stamp=f["stamp"], dz_nominal=m.get("params", {}).get("dz"),
               duration_s=round(du, 1), bottom_z_median=round(float(np.median(bz)), 3), bottom_z_min=round(float(bz.min()), 3), z_err_rms_cm=round(float(np.sqrt((ez ** 2).mean())), 2),
               gate_s=round(float(gate.sum() * dt), 2), a_res_z_in_gate=round(float(a[gate].mean()), 3) if gate.any() else np.nan, a_res_z_out=round(float(a[~gate].mean()), 3) if (~gate).any() else np.nan,
               a_res_z_min=round(float(a.min()), 2), z_err_peak_in_gate_cm=round(float(ez[gate].max()), 1) if gate.any() else np.nan,
               gap_in_gate_median=round(float(np.median(dz[gate])), 3) if gate.any() else np.nan, gap_in_gate_min=round(float(dz[gate].min()), 3) if gate.any() else np.nan,
               excluded=f["stamp"] in excl)
    return row, bins(np.hypot(dx, dy), dz, sp, dt, gate), gate.sum() * dt

def total(L):
    out = {}
    for H, _ in L:
        for k, v in H.items(): out[k] = out.get(k, 0) + v
    return out, sum(g for _, g in L)

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bank", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--exclude", nargs="*", default=["18-02-54", "18-26-13"], help="stamps of outlier flights")
    a = ap.parse_args(); out = ROOT / a.out if not Path(a.out).is_absolute() else Path(a.out); out.mkdir(parents=True, exist_ok=True)
    man = json.load(open(Path(a.bank) / "training_manifest.json" if Path(a.bank).is_absolute() else ROOT / a.bank / "training_manifest.json"))
    rows, BL = [], []
    for f in man["flights"]:
        row, H, g = bank_flight(f, a.exclude); rows.append(row)
        if not row["excluded"]: BL.append((H, g))
    pd.DataFrame(rows).to_csv(out / "flight_metrics.csv", index=False)
    json.dump({"flights": man["flights"], "excluded": a.exclude}, open(out / "training_manifest_snapshot.json", "w"), indent=1)
    plans = {
        "A4 offset steps #91-99": [planned("A4", dz=z, speed=.3, offset=o, motion="line") for z in (.5, .25, .75) for o in (0, .1, .15)],
        "A4 offset-0.05 sweeps #76-90 (skipped)": [planned("A4", dz=z, speed=s, offset=.05, motion="line") for z in (.5, .25, .75) for s in (.1, .2, .3, .4, .5)],
        "R1 seeds 101-120": [planned("R1", seed=s, total=40.0, bias=0.7) for s in range(101, 121)],
        "A7 #61-65": [planned("A7", speed=s) for s in (.1, .2, .3, .4, .5)],
        "A1 #66-75": [planned("A1", dz=z, hold=15.0) for z in (.5, .5, .25, .25, .75, .75, .375, .375, .625, .625)],
        "A2+A3 repeats dz .25/.75 (skipped)": [planned("A2", dz=z, path="circle", speed=s) for z in (.25, .75) for s in (.1, .2, .3, .4, .5)] + [planned("A3", dz=z, speed=s) for z in (.25, .75) for s in (.1, .2, .3, .4, .5)],
        "A5 (skipped)": [planned("A5", dz=z, speed=s) for z in (.5, .25, .75) for s in (.1, .2, .3, .4, .5)]}
    L = ["# NS2 bank coverage (generated by `experiments/analysis/ns2_bank_coverage.py`)", "",
         f"Bank: {len(BL)} usable flights ({', '.join(a.exclude)} excluded). Seconds inside the NS2 gate. Bank = measured (speed = commanded), planned = commanded.", "",
         "| source | gate s | lateral 0-.05 / .05-.1 / .1-.15 / .15-.2 m | real gap <.35 / .35-.45 / .45-.6 / .6-.8 / >.8 m | bottom speed <.1 / .1-.25 / .25-.4 / >.4 m/s |", "|---|---|---|---|---|"]
    def line(name, Lx):
        H, g = total(Lx); fmt = lambda x: " / ".join(f"{v:.0f}" for v in x)
        return f"| {name} | {g:.0f} | {fmt(H['r'])} | {fmt(H['dz'])} | {fmt(H['sp'])} |"
    L.append(line("**bank (measured)**", BL))
    for k, v in plans.items(): L.append(line(k, v))
    (out / "coverage.md").write_text("\n".join(L) + "\n"); print("\n".join(L))
if __name__ == "__main__": main()
