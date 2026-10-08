#!/usr/bin/env python3
"""Formation (A8/A1, NS2 plant) runs for the Omar z integral; fresh process per run; offset-relative crossing dips."""
import json, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
OUT = HERE / "out" / "omar_z_integral"; OUT.mkdir(parents=True, exist_ok=True)
SNIP = ("import sys,json; sys.path.insert(0,%r); import omar_z_integral_sil as S; "
        "c=json.loads(sys.argv[1]); r=S.run_formation_sil(c['sc'],c['ctrl'],cmd_gain=c['cg'],kpos_iz=c['kiz'],ns2_plant=True,return_trace=True); "
        "sys.stdout.write('RESULT '+json.dumps(r)+'\\n')") % str(HERE)
def one(cfg):
    p = subprocess.run([sys.executable, "-c", SNIP, json.dumps(cfg)], capture_output=True, text=True, cwd=str(HERE))
    for ln in p.stdout.splitlines():
        if ln.startswith("RESULT "):
            r = json.loads(ln[7:]); r["cfg"] = cfg; return r
    return {"cfg": cfg, "error": (p.stderr or p.stdout)[-300:]}
def rel_dips(r):
    """crossing dips relative to the steady offset (median e_z outside +-1.5 s of crossings)."""
    from ns2_closed_loop_sil_metrics import find_crossing_times
    tr = r["trace"]; t = np.array(tr["t"]); pos = np.array(tr["pos"]); sp = np.array(tr["sp"])
    e = (pos[:, 2] - sp[:, 2]) * 100
    tc = find_crossing_times(t, pos[:, 1], n_expect=4)
    near = np.zeros(len(t), bool)
    for x in tc: near |= (np.abs(t - x) < 1.5)
    steady = (t > 4.0 + 6.0) & ~near
    off = float(np.median(e[steady])) if steady.any() else float("nan")
    dips = [float(np.min(e[(t >= x - 1) & (t <= x + 1)]) - off) for x in tc]
    return off, dips
if __name__ == "__main__":
    jobs = []
    for sc in ("A8", "A1"):
        for ctrl in ("oot4", "oot5"):
            for k in (0.0, 0.5, 1.0, 1.5, 2.0): jobs.append(dict(sc=sc, ctrl=ctrl, cg=1.14, kiz=k))
            for cg in (1.0, 1.10, 1.17):
                for k in ((0.0, 1.5) if cg == 1.0 else (0.0, 1.0, 1.5)): jobs.append(dict(sc=sc, ctrl=ctrl, cg=cg, kiz=k))
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    with ThreadPoolExecutor(n) as ex: res = list(ex.map(one, jobs))
    rows = []
    for r in res:
        c = r["cfg"]
        if "error" in r: print(c, "ERR", r["error"]); continue
        row = dict(c, mean_z=r["mean_z_err_cm"], steady_z=r["mean_z_steady_cm"], max_z=r["max_z_err_cm"], gyro=r["gyro_rms_deg_s"])
        if c["sc"] == "A8":
            off, d = rel_dips(r); row.update(offset=off, dips_rel=d)
        rows.append(row)
    (OUT / "validate_formation.json").write_text(json.dumps(rows, indent=1))
    print("sc  ctrl cg   Iz  | mean_z steady maxz  gyro | offset  dips_rel(4)")
    for w in rows:
        print("%-3s %-4s %.2f %.1f | %6.2f %6.2f %5.1f %5.1f | %s" % (w["sc"], w["ctrl"], w["cg"], w["kiz"], w["mean_z"], w["steady_z"], w["max_z"], w["gyro"], ("%6.2f %s" % (w["offset"], np.round(w["dips_rel"], 1).tolist())) if "offset" in w else ""))
