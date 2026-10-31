"""Auto-tuning pipeline core (plan: docs/auto_tuning_plan_rev3_2026-10-28.md).
A job = controller + parameter space (log factors around the default) + tuning cells + plant variants. An evaluation = mean over (cells x variants) of the 3-D RMS error [cm]
of the bottom drone, +100 for an unstable/crashed episode. Every evaluation is appended to a JSONL log. System python3 + CFFIRMWARE_DIR (host bindings); `cma` for CMA-ES.
"""
import json, os, sys, time
from multiprocessing import Pool
from pathlib import Path
import numpy as np
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).parent))
import plant_core as P  # noqa: E402   (plant of record: host firmware + quadratic thrust law + plume torque + noise; run(), TUNING_CELLS, ROBUST_VARIANTS ...)
CAL = json.load(open(Path(__file__).parent / os.environ.get("PLANT_CAL", "out/plant_calibration_joint_lag30_best_s0.json")))["best"]      # plant of record since 2026-10-30: motor lag fixed at 30 ms (docs/tuning_actuator_id_2026-10-29.md); the lag-22 plant: PLANT_CAL=out/plant_calibration_joint_best_s0.json      # joint calibration of 2026-10-29 (docs/tuning_plant_p4_2026-10-29.md)


def base_plant(extra=None, J="fbl"):
    """The calibrated plant for one episode; `extra` overrides (motor lag, gyro noise, delay ...) for the robustness variants."""
    c = P.plant(dict(J=J, dws=CAL["dws"], fmax=CAL["fmax"], lag=CAL["lag"], gn=CAL["gn"], accn=CAL["accn"], rd=CAL["rd"]))
    A0, s0, n0 = c["plume"]; c["plume"] = (A0 * CAL["pa"], s0 * CAL["ps"], n0); c.update(extra or {}); return c


SEARCH_VARIANTS = [dict(), dict(gn=CAL['gn'] * 1.7), dict(md=40), dict(ffs=0.4)]       # nominal calibrated plant, more vibration, state delay 40 ms, feedforward x0.4 (used during the search)
FULL_VARIANTS = SEARCH_VARIANTS + [dict(lag=60), dict(md=60), dict(ffs=0.0), dict(kt=0.5), dict(ffs=0.4, md=40)]                         # used to rank the final candidates


class Job:
    def __init__(self, name, params, cells, variants=SEARCH_VARIANTS, extra=None, tree_map=None):
        """params: {gain name: (default, lo_factor, hi_factor)}; tied names are listed once (kp_xy applies to x and y)."""
        self.name, self.params, self.cells, self.variants, self.extra, self.tree_map = name, params, cells, variants, extra or {}, tree_map      # tree_map: gains -> {C struct field: value} for the Omar tree jobs (S0-S3)
        self.names = list(params); self.d = len(self.names)
        self.lo = np.log([params[n][1] for n in self.names]); self.hi = np.log([params[n][2] for n in self.names])

    def gains(self, u):                 # u in [0, 1]^d (unit cube, log scale between the bounds)
        x = self.lo + np.clip(u, 0, 1) * (self.hi - self.lo)
        return {n: float(self.params[n][0] * np.exp(v)) for n, v in zip(self.names, x)}

    def default_u(self):
        return (0.0 - self.lo) / (self.hi - self.lo)


def episodes(job, gains, variants=None, cells=None):
    cfgs = []
    for v in (variants or job.variants):
        for sc, kw in (cells or job.cells):
            c = base_plant(dict(v, **job.extra)); c.update(scen=sc, sc=kw, gains={} if job.tree_map else gains, seed=0)
            if job.tree_map: c['tree_gains'] = job.tree_map(gains)
            cfgs.append(c)
    return cfgs


def score_episodes(rs, n_cells):
    s = [100.0 if not r['stable'] else float(np.sqrt(r['x_rms_cm'] ** 2 + r['y_rms_cm'] ** 2 + r['z_rms_cm'] ** 2)) for r in rs]
    return float(np.mean(s)), s


def evaluate_batch(job, pool, gains_list, variants=None, cells=None):
    """Evaluate several candidates in parallel; returns [{score, per_episode, gains}]."""
    variants = variants or job.variants; cells = cells or job.cells
    cfgs = [c for g in gains_list for c in episodes(job, g, variants, cells)]
    res = pool.map(P.run, cfgs, chunksize=1); n = len(variants) * len(cells); out = []
    for k, g in enumerate(gains_list):
        sc, per = score_episodes(res[k * n:(k + 1) * n], len(cells)); out.append(dict(gains=g, score=sc, per_episode=per, tilt_max=max(r['tilt_max_deg'] for r in res[k * n:(k + 1) * n])))
    return out


def log_eval(path, rec):
    with open(path, 'a') as f: f.write(json.dumps(rec) + "\n")
