#!/usr/bin/env python3
"""Fast pass/fail verdict on a single-drone hover/circle/figure8 log.

    python3 experiments/analysis/check_flight.py            # newest log in Controls/logs
    python3 experiments/analysis/check_flight.py <path.csv>

Answers one question: did the attitude loop stay bounded? That is what the 2026-09-09
crash session turned on, and it is what gates every later step (retuning, multi-drone).
System python3 -- stdlib only, no numpy, so it runs anywhere including the lab PC.

Reads the config the flight ACTUALLY used out of the CSV's own meta block, preferring
`full_selected_pos_*` (what simple_flight pushed) over `full_pos_gains_*` (what the yaml
said) when they disagree -- see audit finding N2.
"""
import csv
import glob
import math
import os
import sys

LOGS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Controls", "logs")


def read(path):
    meta, data = {}, []
    with open(path) as fh:
        lines = fh.readlines()
    for line in lines:
        if line.startswith("# meta:") and "=" in line:
            k, v = line[len("# meta:"):].strip().split("=", 1)
            meta[k.strip()] = v.strip()
    body = [line for line in lines if not line.startswith("#")]
    for row in csv.DictReader(body):
        data.append(row)
    return meta, data


def col(rows, key):
    out = []
    for r in rows:
        try:
            out.append(float(r[key]))
        except (KeyError, TypeError, ValueError):
            out.append(float("nan"))
    return out


def stats(vals):
    w = [v for v in vals if v == v]
    if not w:
        return None
    m = sum(w) / len(w)
    sd = math.sqrt(sum((x - m) ** 2 for x in w) / len(w))
    h = len(w) // 2
    s1 = math.sqrt(sum((x - m) ** 2 for x in w[:h]) / h) if h else float("nan")
    s2 = math.sqrt(sum((x - m) ** 2 for x in w[h:]) / (len(w) - h)) if len(w) > h else float("nan")
    return sd, max(abs(min(w)), abs(max(w))), s1, s2


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        cands = glob.glob(os.path.join(LOGS, "*.csv"))
        if not cands:
            sys.exit(f"no logs in {LOGS}")
        path = max(cands, key=os.path.getmtime)

    meta, rows = read(path)
    print(f"\n{os.path.basename(path)}   ({len(rows)} rows)")

    ctrl = meta.get("full_yaml_stabilizer_controller", meta.get("yaml_stabilizer_controller", "?"))
    mode = meta.get("full_indi_gains_ctrl_mode", meta.get("yaml_ctrl_mode", "?"))
    name = {"0": "geometric", "1": "pos INDI", "2": "att INDI", "3": "full INDI"}.get(mode, "?")
    # N2: the pushed gains win over the yaml dump when they disagree.
    kp = meta.get("full_selected_pos_kp_xy", meta.get("pos_kp_xy", "?"))
    kv = meta.get("full_selected_pos_kv_xy", meta.get("pos_kv_xy", "?"))
    print(f"  controller={ctrl}  ctrl_mode={mode} ({name})  kp_xy={kp} kv_xy={kv}"
          f"  clamp_en={meta.get('full_indi_gains_clamp_en', '?')}"
          f"  res_sign={meta.get('full_indi_gains_res_sign', 'default +1')}")

    z = col(rows, "z")
    hov = [i for i, v in enumerate(z) if v > 0.5]
    if not hov:
        zz = [v for v in z if v == v]
        print(f"  ** never reached hover (z max {max(zz) if zz else float('nan'):.3f}) **")
        return 1
    a, b = hov[0], hov[-1]

    verdict_bad = False
    for axis in ("roll_deg", "pitch_deg"):
        s = stats(col(rows, axis)[a:b])
        if not s:
            continue
        sd, peak, s1, s2 = s
        grow = s2 > 1.3 * s1
        flag = "GROWING" if grow else "bounded"
        print(f"  {axis[:5]:5s} std {sd:6.2f}  peak {peak:6.1f}   {s1:5.2f} -> {s2:5.2f}  {flag}")
        if grow or sd > 10.0:
            verdict_bad = True

    zz = [v for v in z[a:b] if v == v]
    print(f"  z     mean {sum(zz)/len(zz):6.3f}  min {min(zz):.3f}  max {max(zz):.3f}"
          f"   final {z[-1]:.3f}")
    if z[-1] > 0.20:
        print("  ** log ends above 0.20 m -- check the landing completed **")

    rp = [col(rows, f"rpm_m{i}") for i in (1, 2, 3, 4)]
    zeros = sum(1 for v in rp[0][a:b] if v == 0)
    if zeros:
        print(f"  ** rpm_m1 reads 0 in {zeros}/{b-a} in-flight samples **")

    print(f"\n  VERDICT: {'FAIL -- do not continue' if verdict_bad else 'PASS'}\n")
    return 1 if verdict_bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
