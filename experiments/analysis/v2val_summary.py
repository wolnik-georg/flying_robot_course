#!/usr/bin/env python3
"""Summary of the 2026-10-19 v2 tree validation (A8, one flight per strategy S0..S3, merged bank).
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/v2val_summary.py experiments/logs/bank_v2val_2026-10-19 [--out DIR]
Per flight (bottom vehicle): strategy evidence from the log (INDI active fraction, NS2 prediction in the gate, sign of its use is checked by tree_log_check.py),
z tracking, crossing dips (A8 crossings = zero crossings of y), battery, motor headroom, and NS2 v1-vs-v2 identification:
the logged C prediction rnn_pred_z is compared with the reference forward pass of ns2_v1 and ns2_v2 evaluated on the logged relative state."""
import argparse, glob, json, sys
from pathlib import Path
from unittest.mock import MagicMock
import numpy as np, pandas as pd
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "experiments/analysis")); sys.path.insert(0, str(REPO / "flying_drone_stack/firmware_app/host"))
sys.modules.setdefault("cffirmware", MagicMock())
from a8_crossings import zero_crossings  # noqa: E402
from test_residual_nn import reference  # noqa: E402

W = {k: np.asarray(np.load(REPO / f"flying_drone_stack/tools/residual/weights/ns2_{k}.npz")["weights"], np.float32) for k in ("v1", "v2")}
NAMES = ["S0", "S1", "S2", "S3"]


def ref_pred(w, b, t, step=5):
    out = np.full(len(b), np.nan)
    for i in range(0, len(b), step):
        p = np.array([b.x.iat[i], b.y.iat[i], b.z.iat[i]]); v = np.array([b.vx.iat[i], b.vy.iat[i], b.vz.iat[i]])
        pt = np.array([t.x.iat[i], t.y.iat[i], t.z.iat[i]]); vt = np.array([t.vx.iat[i], t.vy.iat[i], t.vz.iat[i]])
        rel = [(pt - p, vt - v)] if pt[2] > p[2] else []
        out[i] = reference(w, rel, float(p[2]), v, 0.0427)[0][2]
    return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("bank"); ap.add_argument("--out", default=None); a = ap.parse_args()
    res = []
    for k, d in enumerate(sorted(glob.glob(a.bank + "/A8_*"))):
        f = glob.glob(d + "/*_merged_usd.csv")[0]; df = pd.read_csv(f, comment="#")
        pre = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf5_")})[0]; pt = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf_second_")})[0]
        b = df[[c for c in df.columns if c.startswith(pre + ".")]].rename(columns=lambda c: c.split(".", 1)[1]); b["t"] = df.t
        t = df[[c for c in df.columns if c.startswith(pt + ".")]].rename(columns=lambda c: c.split(".", 1)[1]); t["t"] = df.t
        air = (b.z > 0.3).values; tt = b.t.values
        ez = (b.z - b.ctrltarget_z).values; et = (t.z - t.ctrltarget_z).values
        cr = zero_crossings(tt, b.y.values, n=4, hyst=0.15, spacing=4.375, tol=1.0)
        win = np.zeros(len(tt), bool)
        for c in cr: win |= np.abs(tt - c) < 0.75
        steady = air & ~win & (tt > 3)
        base = float(np.median(ez[steady])) if steady.any() else 0.0
        dips = [float(ez[np.abs(tt - c) < 0.75].min() - base) for c in cr]; ups = [float(ez[np.abs(tt - c) < 0.75].max() - base) for c in cr]
        rel_gate = (np.abs(t.x - b.x) < 0.2) & (np.abs(t.y - b.y) < 0.2) & (t.z > b.z)
        pz = b.rnn_pred_z.values
        r1 = ref_pred(W["v1"], b, t); r2 = ref_pred(W["v2"], b, t); m = ~np.isnan(r2) & air
        e1 = float(np.sqrt(np.nanmean((pz[m] - r1[m]) ** 2))); e2 = float(np.sqrt(np.nanmean((pz[m] - r2[m]) ** 2)))
        g = m & rel_gate.values
        res.append(dict(flight=k + 1, name=NAMES[k], file=Path(f).parent.name, duration_s=float(tt[-1]), crossings=[round(c, 2) for c in cr],
                        z_rms_cm=100 * float(np.sqrt(np.mean(ez[steady] ** 2))), z_offset_cm=100 * base, dip_cm=[round(100 * x, 1) for x in dips], dip_mean_cm=100 * float(np.mean(dips)) if dips else None,
                        up_peak_cm=[round(100 * x, 1) for x in ups], top_z_rms_cm=100 * float(np.sqrt(np.mean(et[air & (tt > 3)] ** 2))),
                        gate_s=float(rel_gate[air].sum() / 500), indi_a_rpm_active_pct=100 * float((np.abs(b.omar_a_rpm_fz[air]) > 1e-6).mean()),
                        rnn_pred_z_min=float(np.nanmin(pz[air])), rnn_pred_z_in_gate_median=float(np.nanmedian(pz[g])) if g.any() else None, rnn_pred_nonzero_pct=100 * float((np.abs(pz[air]) > 1e-4).mean()),
                        rms_err_logged_vs_ref_v1=e1, rms_err_logged_vs_ref_v2=e2, corr_logged_ref_v1=float(np.corrcoef(pz[m], r1[m])[0, 1]), corr_logged_ref_v2=float(np.corrcoef(pz[m], r2[m])[0, 1]),
                        vbat_min=float(b.vbat[air].min()), vbat_start=float(b.vbat[air].iloc[0]), motor_max_pct=float(100 * max(b[f"motor_m{i}"][air].max() for i in range(1, 5)) / 65535),
                        tilt_max_deg=float(max(np.abs(b.roll_deg[air]).max(), np.abs(b.pitch_deg[air]).max()))))
    out = Path(a.out or a.bank); out.mkdir(parents=True, exist_ok=True); json.dump(res, open(out / "v2val_summary.json", "w"), indent=1)
    for r in res: print(json.dumps(r))
if __name__ == "__main__":
    main()
