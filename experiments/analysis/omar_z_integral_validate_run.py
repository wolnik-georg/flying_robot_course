#!/usr/bin/env python3
import json, subprocess, sys, itertools
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
OUT = Path(__file__).resolve().parent / "out" / "omar_z_integral"; OUT.mkdir(parents=True, exist_ok=True)
def one(cfg):
    p = subprocess.run([sys.executable, str(Path(__file__).resolve().parent / "omar_z_integral_validate.py"), json.dumps(cfg)], capture_output=True, text=True)
    for ln in p.stdout.splitlines():
        if ln.startswith("RESULT "): return json.loads(ln[7:])
    return {"cfg": cfg, "error": (p.stderr or p.stdout)[-300:]}
if __name__ == "__main__":
    jobs = [dict(controller=c, cmd_gain=g, kiz=k, profile=pr) for pr in ("hover", "fig8") for c in ("oot4", "oot5") for g in (1.0, 1.14) for k in (0.0, 0.5, 1.0, 1.5, 2.0)]
    with ThreadPoolExecutor(6) as ex: res = list(ex.map(one, jobs))
    (OUT / "validate_solo.json").write_text(json.dumps(res, indent=1))
    print("profile ctrl  cg   Iz | zmean zrms  xyrms  climb_ovs tilt  gyro | land_min  touchdown_vz")
    for r in res:
        c = r["cfg"]
        if "error" in r: print(c, "ERR", r["error"]); continue
        print("%-6s %-4s %.2f %.1f | %6.2f %5.2f %6.2f %8.1f %5.1f %5.1f | %6.1f  %s" % (c["profile"], c["controller"], c["cmd_gain"], c["kiz"], r["z_err_mean_cm"], r["z_err_rms_cm"], r["xy_err_rms_cm"], r["climb_overshoot_cm"], r["max_tilt_deg"], r["gyro_rms_deg_s"], r["land_min_z_cm"], None if r["touchdown_vz_mps"] is None else round(r["touchdown_vz_mps"], 2)))
