#!/usr/bin/env python3
"""A1 (stacked hover) cf5 uSD summary, 2026-10-08: network off vs on (res_sign=-1)."""
import sys, json, numpy as np
from pathlib import Path
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "flying_drone_stack/tools"))
import decode_usd_log as d
FL = [("17-45-15","thesis00","off"),("17-46-51","thesis01","off"),("17-51-55","thesis02","on"),("17-53-48","thesis03","on"),("17-55-30","thesis04","on")]
out = []
print("stamp net | steady: roll_sd pitch_sd |roll|p99 gyro_x_sd | xy_rms cm  z_err cm z_sd cm | f_osc Hz | rnn_pred_z mean sd | a_res_z mean")
for st, th, net in FL:
    r = d.load(str(REPO / f"experiments/logs/usd_raw/cf5_A1_{th}_2026-10-08_{st}.bin"))
    t = r["t"] - r["t"][0]; z = r["z"]; sp = r["ctrltarget_z"]
    up = z > 0.3; t0 = t[up][0]; t1 = t[up][-1]
    m = up & (t > t0 + 6) & (t < t1 - 4)
    roll, pitch = r["roll_deg"][m], r["pitch_deg"][m]
    x0, y0 = np.nanmedian(r["x"][m]), np.nanmedian(r["y"][m])
    xy = np.sqrt((r["x"][m]-r["ctrltarget_x"][m])**2 + (r["y"][m]-r["ctrltarget_y"][m])**2)
    dt = np.median(np.diff(t)); f = np.fft.rfftfreq(m.sum(), dt); P = np.abs(np.fft.rfft(roll - roll.mean()))
    fo = float(f[1:][np.argmax(P[1:])])
    row = dict(stamp=st, net=net, roll_sd=float(roll.std()), pitch_sd=float(pitch.std()),
               roll_p99=float(np.percentile(np.abs(roll), 99)), gyro_x_sd=float(r["gyro_x"][m].std()),
               xy_rms_cm=float(np.sqrt(np.mean(xy**2))*100), z_err_cm=float(np.mean(z[m]-sp[m])*100), z_sd_cm=float(np.std(z[m])*100),
               f_osc=fo, pred_z_mean=float(np.nanmean(r["rnn_pred_z"][m])), pred_z_sd=float(np.nanstd(r["rnn_pred_z"][m])),
               a_res_z_mean=float(np.nanmean(r["a_res_z"][m])), clamped=float(np.nanmean(r["rnn_clamped"][m])))
    out.append(row)
    print(st, net, "| %.1f %.1f %.0f %.0f | %.1f %+.2f %.2f | %.2f | %+.2f %.2f | %+.2f | clamped %.2f" % (
        row["roll_sd"],row["pitch_sd"],row["roll_p99"],row["gyro_x_sd"],row["xy_rms_cm"],row["z_err_cm"],row["z_sd_cm"],row["f_osc"],row["pred_z_mean"],row["pred_z_sd"],row["a_res_z_mean"],row["clamped"]))
json.dump(out, open(REPO / "experiments/analysis/out/ns2_signtest_2026-10-08/a1_usd_summary.json","w"), indent=1)
