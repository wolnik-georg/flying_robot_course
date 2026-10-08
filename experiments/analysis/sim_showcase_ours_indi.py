#!/usr/bin/env python3
"""Ours INDI (bottom drone) in the NS2-plant A8 SIL — runs seeds in fresh subprocesses (system python3), caches to
out/sim_showcase_2026-10-09/results/ours_indi_seed{n}.json. usage: python3 sim_showcase_ours_indi.py [n_seeds]"""
import json, os, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
HERE = Path(__file__).resolve().parent
RES = HERE / "out" / "sim_showcase_2026-10-09" / "results"; RES.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(HERE))
from ns2_closed_loop_sil_calibrate import a8_cfg  # noqa: E402
ENV = {**os.environ, "PYTHONPATH": os.path.expanduser("~/Desktop/crazyflie-firmware/build"), "BOTH": "1"}  # both drones ours INDI (host gains are global)

def one(seed):
    out = RES / f"ours_indi_seed{seed}.json"
    if out.is_file():
        return json.loads(out.read_text())
    cfg = {**a8_cfg(1.0, seed=seed, meas_noise=True, rnn_en=False, downwash_plant="bank"),
           "torque_c": 0.0032, "plant_symmetrize": True, "res_sign": 1, "label": f"ours_indi_seed{seed}", "record_gain_audit": True}
    p = subprocess.run([sys.executable, str(HERE / "sim_showcase_ours_indi_child.py"), json.dumps(cfg)],
                       capture_output=True, text=True, env=ENV, cwd=str(HERE))
    r = None
    for ln in p.stdout.splitlines():
        if ln.strip().startswith("{"):
            r = json.loads(ln); break
    if r is None:
        r = {"error": (p.stderr or p.stdout)[-1500:]}
    out.write_text(json.dumps(r))
    return r

if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    with ThreadPoolExecutor(min(n, 3)) as ex:
        rs = list(ex.map(one, range(n)))
    for i, r in enumerate(rs):
        print(i, "ERROR " + str(r["error"])[-300:] if "error" in r else "ok keys=" + ",".join(list(r)[:8]))
