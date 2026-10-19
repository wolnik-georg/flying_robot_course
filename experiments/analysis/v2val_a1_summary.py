#!/usr/bin/env python3
"""A1 (stacked hold, dz 0.5, 15 s) summary of the merged uSD bank, bottom vehicle; uSD logs only.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/v2val_a1_summary.py experiments/logs/bank_a1clean_2026-10-19 [--strategy S3 S3 S3]"""
import argparse, glob, json, sys
from pathlib import Path
from unittest.mock import MagicMock
import numpy as np, pandas as pd
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "flying_drone_stack/firmware_app/host")); sys.modules.setdefault("cffirmware", MagicMock())
from test_residual_nn import reference  # noqa: E402
W = {k: np.asarray(np.load(REPO / f"flying_drone_stack/tools/residual/weights/ns2_{k}.npz")["weights"], np.float32) for k in ("v1", "v2")}
ap = argparse.ArgumentParser(); ap.add_argument("bank"); ap.add_argument("--strategy", nargs="*", default=None); a = ap.parse_args()
res = []
for k, d in enumerate(sorted(glob.glob(a.bank + "/A1_*"))):
    df = pd.read_csv(glob.glob(d + "/*_merged_usd.csv")[0], comment="#")
    pb = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf5_")})[0]; pt = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf_second_")})[0]
    b = df[[c for c in df.columns if c.startswith(pb + ".")]].rename(columns=lambda c: c.split(".", 1)[1]); t = df[[c for c in df.columns if c.startswith(pt + ".")]].rename(columns=lambda c: c.split(".", 1)[1])
    tt = df.t.values; w = (tt >= 2.0) & (tt <= 15.0) & (b.z.values > 0.2)
    ez = (b.z - b.ctrltarget_z).values; ex = (b.x - b.ctrltarget_x).values; ey = (b.y - b.ctrltarget_y).values; et = (t.z - t.ctrltarget_z).values
    gap = (t.z - b.z).values
    def ref(wk):
        o = []
        for i in np.where(w)[0][::5]:
            p = np.array([b.x.iat[i], b.y.iat[i], b.z.iat[i]]); v = np.array([b.vx.iat[i], b.vy.iat[i], b.vz.iat[i]]); pt_ = np.array([t.x.iat[i], t.y.iat[i], t.z.iat[i]]); vt = np.array([t.vx.iat[i], t.vy.iat[i], t.vz.iat[i]])
            o.append((b.rnn_pred_z.iat[i], reference(W[wk], [(pt_ - p, vt - v)] if pt_[2] > p[2] else [], float(p[2]), v, 0.0427)[0][2]))
        return np.array(o)
    r1, r2 = ref("v1"), ref("v2")
    res.append(dict(flight=k + 1, file=Path(d).name, strategy=(a.strategy[k] if a.strategy and k < len(a.strategy) else "?"), z_mean_cm=100 * float(ez[w].mean()), z_std_cm=100 * float(ez[w].std()),
                    z_rms_cm=100 * float(np.sqrt((ez[w] ** 2).mean())), z_drift_cm=100 * float(ez[(tt > 12) & (tt < 15)].mean() - ez[(tt > 2) & (tt < 5)].mean()), xy_rms_cm=100 * float(np.sqrt((ex[w] ** 2 + ey[w] ** 2).mean())),
                    gap_mean_m=float(gap[w].mean()), top_z_rms_cm=100 * float(np.sqrt((et[w] ** 2).mean())), roll_pitch_max_deg=float(max(np.abs(b.roll_deg[w]).max(), np.abs(b.pitch_deg[w]).max())),
                    indi_active_pct=100 * float((np.abs(b.omar_a_rpm_fz[w]) > 1e-6).mean()), rnn_pred_z_mean=float(b.rnn_pred_z[w].mean()), v1_rms_err=float(np.sqrt(((r1[:, 0] - r1[:, 1]) ** 2).mean())),
                    v2_rms_err=float(np.sqrt(((r2[:, 0] - r2[:, 1]) ** 2).mean())), vbat_start=float(b.vbat[w].iloc[0]), vbat_mean=float(b.vbat[w].mean()), vbat_min=float(b.vbat[w].min()),
                    motor_max_pct=float(100 * max(b[f"motor_m{i}"][w].max() for i in range(1, 5)) / 65535), file_end_s=float(tt[-1])))
out = Path(a.bank); json.dump(res, open(out / "a1_summary.json", "w"), indent=1)
for r in res: print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()}))
