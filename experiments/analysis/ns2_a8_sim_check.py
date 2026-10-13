#!/usr/bin/env python3
"""Run the closed-loop A8 simulation yourself: geometric (network off) vs NS2 (res_sign -1, 100 Hz), same plant as docs/62 and docs/72.

  python3 experiments/analysis/ns2_a8_sim_check.py [--seeds 3] [--workers 6]      # system python3 (torch + SIL bindings)

Prints the crossing dip per seed (zero-crossing detector, min z error within +-1 s of each of the 4 crossings) and the mean, next to the
hardware reference (uSD, A8, 10-05..10-09: geometric -5.9 cm, NS2 res_sign -1 -3.2 cm). Without --csv-dir nothing is written; with --csv-dir <folder> one CSV per run is written (100 Hz: time, both drones' positions, bottom setpoint, z error).
One episode takes ~1.5 min; with 3 seeds x 2 cohorts and 6 workers about 4 min."""
import argparse, json, subprocess, sys, tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
WEIGHTS = str(HERE / "out/c2_e2e_2026-10-01/full_bank_c1_complete.npz")
HW = {"network off (geometric)": -5.9, "NS2 res_sign -1": -3.2}


SCALE = 1.0


def cfg(network: bool, seed: int, logs: bool = False) -> dict:
    return {"scenario": "A8", "anchor": [0.0, 0.0, 0.5], "dz": 0.5, "peer_hz": 100.0, "div": 10, "ki_z": 16.0, "res_sign": -1 if network else 1,
            "takeoff_s": 4.0, "land_s": 3.0, "log_hz": 100, "passes": 4, "settle_s": 2.0, "shuttle_duration": 6.0, "duration_s": 33.0, "n_crossings": 4,
            "setpoint_mode": "hlc", "downwash": True, "downwash_scale": SCALE, "downwash_plant": "bank", "skip_rnn_upload": not network, "rnn_en": network,
            "meas_noise": True, "seed": seed, "return_logs": logs, "plant_weights": WEIGHTS, "weights": WEIGHTS, "torque_c": 0.0032,
            "plant_symmetrize": True, "label": f"a8_{'ns2' if network else 'off'}_seed{seed}"}


CSV_DIR = None


def run(job):
    network, seed = job
    r = subprocess.run([sys.executable, str(HERE / "ns2_closed_loop_sil_sim.py"), json.dumps(cfg(network, seed, CSV_DIR is not None))], capture_output=True, text=True, timeout=1800)
    line = [l for l in r.stdout.splitlines() if l.startswith("{")]
    res = json.loads(line[-1].replace("NaN", "null")) if line else {"error": r.stderr[-300:]}
    if CSV_DIR is not None and res.get("logs"):
        import csv
        lg = res["logs"]
        with open(CSV_DIR / f"a8_{'ns2' if network else 'off'}_seed{seed}.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s", "bottom_x", "bottom_y", "bottom_z", "top_x", "top_y", "top_z", "bottom_z_setpoint", "bottom_z_error_cm"])
            for i, t in enumerate(lg["t"]):
                pb, pt, sp = lg["pos_bot"][i], lg["pos_top"][i], lg["sp_bot"][i]
                w.writerow([t, *pb, *pt, sp[2], (pb[2] - sp[2]) * 100.0])
    return res


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--seeds", type=int, default=3); ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--scale", type=float, default=1.0, help="scale of the simulated downwash force (plant), default 1.0 = the trained network")
    ap.add_argument("--csv-dir", type=Path, default=None, help="also write one CSV per run (time, both drones' positions, setpoint, z error)")
    a = ap.parse_args()
    global CSV_DIR, SCALE
    SCALE = a.scale
    if a.csv_dir:
        a.csv_dir.mkdir(parents=True, exist_ok=True); CSV_DIR = a.csv_dir
    jobs = [(n, s) for n in (False, True) for s in range(a.seeds)]
    with ThreadPoolExecutor(max_workers=a.workers) as ex:
        res = list(ex.map(run, jobs))
    out = {}
    for (n, s), r in zip(jobs, res):
        name = "NS2 res_sign -1" if n else "network off (geometric)"
        if "error" in r and r["error"]:
            print(f"{name} seed {s}: ERROR {r['error']}"); continue
        d = (r.get("crossing_dips") or {}).get("dip_cm_each", [])
        out.setdefault(name, []).append(sum(d) / len(d) if d else float("nan"))
        print(f"{name:26s} seed {s}: dips per crossing {[round(x, 1) for x in d]}  mean {out[name][-1]:+.1f} cm")
    print("\ncohort                      simulation mean (cm)   hardware (cm, uSD)")
    for name, v in out.items():
        print(f"{name:26s}  {sum(v) / len(v):+8.2f}              {HW[name]:+.1f}")


if __name__ == "__main__":
    main()
