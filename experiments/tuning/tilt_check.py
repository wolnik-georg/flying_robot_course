"""Tilt RMS (deg) over 2-15 s of A1: plant (joint calibration) vs hardware unseen-block flights, per strategy."""
import glob, json, sys
from multiprocessing import Pool
from pathlib import Path
import numpy as np, pandas as pd
sys.path.insert(0, str(Path(__file__).parent)); import plant_core as P
from tree_gate import STRAT
best = json.load(open(Path(__file__).parent / "out/plant_calibration_joint_best_s0.json"))["best"]; pa, ps = best.pop("pa"), best.pop("ps")
def cfg(st, so, sd):
    c = P.plant(dict(best, J="omar")); c["plume"] = (c["plume"][0] * pa, c["plume"][1] * ps, c["plume"][2]); c.update(scen="A1", sc=dict(dz=0.45, hold=15.0), gains={}, seed=sd, tree=True, nm=f"tilt_{st}_{sd}", tag="tilt", **so); return c
if __name__ == "__main__":
    cfgs = [cfg(st, so, sd) for st, so in STRAT.items() for sd in range(2)]
    with Pool(8) as pool: res = pool.map(P.run, cfgs, chunksize=1)
    for k, st in enumerate(STRAT):
        pl = np.mean([r["tilt_rms_deg"] for r in res[2 * k:2 * k + 2]]); print("plant %s tilt rms %.1f deg" % (st, pl))
    # tilt_rms_deg in the plant is over the scenario window; hardware: roll/pitch of the unseen-block A1 flights, 2-15 s
    banks = {"S0": "bank_unseen_S0_2026-10-23", "S1": "bank_unseen_S1_2026-10-23", "S3": "bank_unseen_S3_2026-10-23", "S2": "bank_unseen_S3S2_2026-10-23"}
    REPO = Path(__file__).resolve().parents[2]
    for st, b in banks.items():
        v = []
        for f in sorted(glob.glob(str(REPO / "experiments/logs" / b / "A1_*/*merged_usd.csv"))):
            df = pd.read_csv(f, comment="#"); pb = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf5")})[0] + "."
            if pb + "roll_deg" not in df: continue
            t = df.t.values; w = (t >= 2) & (t <= 15) & (df[pb + "z"].values > 0.2); v.append(np.sqrt(np.mean(df[pb + "roll_deg"].values[w] ** 2 + df[pb + "pitch_deg"].values[w] ** 2)))
        print("hardware %s tilt rms %s deg (n=%d)" % (st, np.round(v, 1).tolist(), len(v)))
