#!/usr/bin/env python3
"""P4 (joint version: S0-S3 pilot cells + S4 FBL 10-26 cells): calibrate a few plant parameters of the S0-S3 plant on the PILOT cells (2026-10-20, default gains), then run the gate on the UNSEEN-BLOCK cells (tree_gate.py).
Parameters: plume torque amplitude scale pa and width scale ps, downwash force scale dws, thrust available at full command fmax [N], motor lag [ms], gyro vibration gn [deg/s], accelerometer noise accn [g], rpm telemetry delay rd [ms].
  CFFIRMWARE_DIR=~/Desktop/crazyflie-firmware/build python3 experiments/tuning/plant_calibration.py --budget 120 --pop 12 --procs 10"""
import argparse, json, sys, time
from multiprocessing import Pool
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import plant_core as P
from tree_gate import STRAT, metric

PILOT = {   # unified_overview_2026-10-20: dz 0.5, 0.5 m/s, default gains (S0, S1, S2, S3)
    "A1": ("A1", dict(dz=0.5, hold=15.0), {"S0": -17.7, "S1": -1.8, "S2": -8.9, "S3": 2.7}),
    "A8": ("A8", None, {"S0": -12.5, "S1": -8.4, "S2": -5.6, "S3": -3.8}),
    "A4 off0.05": ("A4", dict(dz=0.5, offset=0.05, axis='y', motion='line', speed=0.5), {"S0": 17.2, "S1": 5.0, "S2": 11.7, "S3": 3.8}),
    "A4 off0.10": ("A4", dict(dz=0.5, offset=0.10, axis='y', motion='line', speed=0.5), {"S0": 10.5, "S1": 2.8, "S2": 5.0, "S3": 3.2}),
    "A4 off0.15": ("A4", dict(dz=0.5, offset=0.15, axis='y', motion='line', speed=0.5), {"S0": 6.2, "S1": 2.4, "S2": 2.3, "S3": 2.7})}
FBL = {   # S4 FBL (preset 1, inner 1, cd 0.55, network on), uSD flights 2026-10-26 (fbl_flights.csv, variant final): z metric as above, xy = horizontal RMS
    "A1 dz0.5": ("A1", dict(dz=0.5, hold=15.0), dict(z=-0.0, xy=3.4)), "A8 dz0.4": ("A8", None, dict(z=-1.2, xy=5.6)),
    "A4 off0.075": ("A4", dict(dz=0.4, offset=0.075, axis='y', motion='line', speed=0.35), dict(z=1.5, xy=8.2)), "A4 off0.125": ("A4", dict(dz=0.4, offset=0.125, axis='y', motion='line', speed=0.35), dict(z=1.1, xy=10.2))}
NAMES = ["dws", "fmax", "lag", "gn", "accn", "rd", "pa", "ps"]; LO = np.array([0.6, 0.16, 20.0, 1.0, 0.005, 0.0, 0.5, 0.6]); HI = np.array([2.0, 0.30, 60.0, 20.0, 0.15, 20.0, 5.0, 2.0]); X0 = np.array([1.0, 0.25, 30.0, 8.0, 0.02, 0.0, 1.0, 1.0])


FIX = {}      # parameters pinned to a measured value (--fix lag=30); the search runs over the free ones only
def decode(u):
    free = [i for i, n in enumerate(NAMES) if n not in FIX]; full = (X0 - LO) / (HI - LO); full = np.array(full, float); full[free] = np.clip(u, 0, 1)
    d = dict(zip(NAMES, [float(v) for v in LO + full * (HI - LO)])); d.update(FIX); return d
def cell_cfgs(pp, seed=0, cells=PILOT, tag="cal"):
    out = []; meta = []
    for name, (scen, kw, hw) in cells.items():
        for st, so in STRAT.items():
            c = P.plant(dict(J="omar", dws=pp["dws"], fmax=pp["fmax"], lag=pp["lag"], gn=pp["gn"], accn=pp["accn"], rd=pp["rd"])); A0, s0, n0 = c["plume"]; c["plume"] = (A0 * pp["pa"], s0 * pp["ps"], n0); c.update(scen=scen, sc=kw, gains={}, seed=seed, tree=True, nm=f"{tag}_{name.replace(' ', '')}_{st}_{seed}_{int(abs(hash(str(pp))) % 99999)}", tag=tag, **so); out.append(c); meta.append((name, st, scen, hw[st]))
    return out, meta


def fbl_cfgs(pp, seed=0, tag="calf"):
    out = []; meta = []
    for name, (scen, kw, hw) in FBL.items():
        c = P.plant(dict(J="fbl", dws=pp["dws"], fmax=pp["fmax"], lag=pp["lag"], gn=pp["gn"], accn=pp["accn"], rd=pp["rd"])); A0, s0, n0 = c["plume"]; c["plume"] = (A0 * pp["pa"], s0 * pp["ps"], n0)
        c.update(scen=scen, sc=kw, gains={}, seed=seed, nm=f"{tag}_{name.replace(' ', '')}_{seed}_{int(abs(hash(str(pp))) % 99999)}", tag=tag); out.append(c); meta.append((name, "S4", scen, hw))
    return out, meta


def objective_fbl(results, meta):
    sq = []; per = []
    for r, (name, st, scen, hw) in zip(results, meta):
        mz = metric(scen, r, dur=6.25); mxy = float(np.hypot(r["x_rms_cm"], r["y_rms_cm"]))
        for m, h in ((mz, hw["z"]), (mxy, hw["xy"])):
            sq.append(25.0 if (np.isnan(m) or not r["stable"]) else min(((m - h) / max(1.5, 0.3 * abs(h))) ** 2, 25.0))
        per.append((mz, mxy))
    return float(np.sum(sq)), len(sq), per


def objective(results, meta):
    sq = []; per = []
    for r, (name, st, scen, hw) in zip(results, meta):
        m = metric(scen, r, dur=4.375) if scen == "A8" else metric(scen, r)
        d = 25.0 if (np.isnan(m) or not r["stable"]) else min(((m - hw) / max(1.5, 0.3 * abs(hw))) ** 2, 25.0); sq.append(d); per.append(m)
    return float(np.mean(sq)), per


if __name__ == "__main__":
    import cma
    ap = argparse.ArgumentParser(); ap.add_argument("--budget", type=int, default=120); ap.add_argument("--pop", type=int, default=12); ap.add_argument("--procs", type=int, default=10); ap.add_argument("--seed", type=int, default=0); ap.add_argument("--fix", nargs="*", default=[]); ap.add_argument("--tag", default=""); a = ap.parse_args()
    FIX.update({k: float(v) for k, v in (x.split("=") for x in a.fix)})
    out = Path(__file__).parent / "out"; lp = out / f"plant_calibration_joint{a.tag}_s{a.seed}.jsonl"
    if lp.exists(): lp.unlink()
    x0 = [((X0 - LO) / (HI - LO))[i] for i, n in enumerate(NAMES) if n not in FIX]; es = cma.CMAEvolutionStrategy(x0, 0.25, dict(bounds=[0, 1], popsize=a.pop, seed=a.seed + 1, verbose=-9)); n = 0; best = (1e9, None); t0 = time.time()
    with Pool(a.procs) as pool:
        while n < a.budget:
            us = es.ask(); allcfg = []; metas = []
            for u in us: c, m = cell_cfgs(decode(np.asarray(u))); cf, mf = fbl_cfgs(decode(np.asarray(u))); allcfg += c + cf; metas.append((m, mf))
            res = pool.map(P.run, allcfg, chunksize=1); k = len(metas[0][0]) + len(metas[0][1]); fs = []
            for j, u in enumerate(us):
                rr = res[j * k:(j + 1) * k]; nt = len(metas[j][0]); f1, per = objective(rr[:nt], metas[j][0]); f2, n2, perf = objective_fbl(rr[nt:], metas[j][1]); f = (f1 * nt + f2) / (nt + n2); per = per + [list(x) for x in perf]; fs.append(f)
                with open(lp, "a") as fh: fh.write(json.dumps(dict(params=decode(np.asarray(u)), f=f, per=per, index=n, t=time.time() - t0)) + "\n")
                n += 1
                if f < best[0]: best = (f, decode(np.asarray(u)))
            es.tell(us, fs); print("gen %d n %d best %.3f gen-best %.3f %s" % (es.countiter, n, best[0], min(fs), {k_: round(v, 3) for k_, v in best[1].items()}), flush=True)
    json.dump(dict(best=best[1], f=best[0]), open(out / f"plant_calibration_joint{a.tag}_best_s{a.seed}.json", "w"), indent=1)
