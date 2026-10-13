#!/usr/bin/env python3
"""Closed-loop safety / equivalence check before the lab: one drone, hover and the A8 bottom path (+ A2 circle), measurement noise on, the real vehicle as the plant
(0.041 kg, flown k_t, 44 ms motor lag), controllers = the host-compiled firmware.

Variants: ours frozen (docs/77), ours with Omar's values (yaml patches ours_omar_values*.patch), Omar C (controller 9, with / without the z integral 1.5),
Tal & Karaman native and with Omar's values. One process per run (host gains are process-global).

  python3 experiments/analysis/indi_variants_sil_check.py [--jobs 10] [--scenarios hover A8 A2]
Output: experiments/analysis/out/indi_variants_sil_check/<variant>__<scenario>.json (+ trace npz), summary.md.  Single drone: no downwash, no estimator delay, no ground contact."""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
import os
AIR = os.environ.get("SIL_AIRBORNE") == "1"
OUT = HERE / "out" / ("indi_variants_sil_check_airborne" if AIR else "indi_variants_sil_check")
FROZEN = HERE.parents[1] / "docs" / "frozen" / "ours_indi_2026-10-13" / "resolved_cf5_parameters.json"
OM_KT = 4.2899e-10
J_TK = (23.951e-6, 23.951e-6, 32.347e-6)
SCEN = {"hover": dict(scenario="hover", seconds=14.0, kwargs={}),
        "A8": dict(scenario="A8", seconds=0.0, kwargs=dict(dz=0.5, span=1.0, duration=6.0, passes=4)),
        "A2": dict(scenario="A2", seconds=0.0, kwargs=dict(dz=0.5, path="circle", speed=0.4))}


def ours_sets(over: dict | None = None, pos_over: dict | None = None) -> dict:
    r = json.loads(FROZEN.read_text())
    ig = dict(r["indi_gains"]); pg = dict(r["pos_gains"])
    ig.update(over or {}); pg.update(pos_over or {})
    sets = {"g_controller_mode": int(ig.pop("ctrl_mode"))}
    sets["g_indi_res_sign"] = int(ig.pop("res_sign"))
    ig.pop("rpm_source", None)
    for k in ("kr_geo", "kw_geo", "kr_z_geo", "kw_z_geo"):   # geometric-mode gains, not exposed on the host library, unused in ctrl_mode 3
        ig.pop(k, None)
    for k, v in ig.items():
        sets[f"g_indi_{k}"] = int(v) if k in ("filt_order", "filt_tau", "filt_prewarp", "filt_dt_us", "clamp_en", "notch_en", "ff_free", "dt_usec") else float(v)
    for k, v in pg.items():
        sets[f"g_{k}"] = float(v)
    return sets


OMAR_IG = dict(mass=0.0427, kt1=OM_KT, kt2=OM_KT, kt3=OM_KT, kt4=OM_KT, kr=421.3, kw=69.2, kr_z=273.4, kw_z=68.3, j_scale=0.693, fc_bw=30.0, res_fc=30.0, filt_order=0, res_sign=-1)
OMAR_IG_KEEP = {k: v for k, v in OMAR_IG.items() if k not in ("fc_bw", "res_fc", "filt_order")}   # Omar's values but ours' own filters (206 Hz as flown)
OMAR_PG = dict(kp_xy=7.0, kp_z=7.0, kv_xy=4.0, kv_z=4.0)
TK_OMAR = {"g_indi_mass": 0.0427, **{f"g_indi_kt{i}": OM_KT for i in range(1, 5)}, "g_tk_kx_xy": 7.0, "g_tk_kx_z": 7.0, "g_tk_kv_xy": 4.0, "g_tk_kv_z": 4.0,
           "g_tk_ka_xy": 0.0, "g_tk_ka_z": 0.0, "g_tk_kxi_xy": 0.007 / J_TK[0], "g_tk_kxi_z": 0.008 / J_TK[2], "g_tk_kom_xy": 0.00115 / J_TK[0], "g_tk_kom_z": 0.002 / J_TK[2],
           "g_tk_fc": 30.0, "g_tk_jrz": 0.0}
VARIANTS = {
    "ours_frozen": dict(controller="oot", sets=ours_sets()),
    "ours_omar_values_keepfilters": dict(controller="oot", sets=ours_sets(OMAR_IG_KEEP, OMAR_PG | {"ki_z": 0.0})),
    "ours_omar_values_keepfilters_iz": dict(controller="oot", sets=ours_sets(OMAR_IG_KEEP, OMAR_PG | {"ki_z": 1.5, "ki_z_limit": 2.0})),
    "ours_omar_values_30hz": dict(controller="oot", sets=ours_sets(OMAR_IG, OMAR_PG | {"ki_z": 0.0})),
    "omar_c": dict(controller="oot4", sets={}, omar={}),
    "omar_c_iz": dict(controller="oot4", sets={}, omar={"Kpos_Iz": 1.5}),
    "tk_native": dict(controller="oot6", sets={}),
    "tk_omar_values": dict(controller="oot6", sets=TK_OMAR),
}


def child(variant: str, scen: str) -> None:
    import numpy as np
    import tk_indi_sil_run as T
    v = VARIANTS[variant]; sc = SCEN[scen]

    def post(cf):
        if v.get("omar") and hasattr(cf, "omar_indi_control"):
            o = cf.omar_indi_control
            if "Kpos_Iz" in v["omar"]:
                vec = o.Kpos_I; vec.z = float(v["omar"]["Kpos_Iz"]); o.Kpos_I = vec

    res, (t, p, s, q) = T.run(sc["scenario"], sc["seconds"], sets=dict(v["sets"]), controller=v["controller"], noise=True, seed=1,
                              kwargs=sc["kwargs"], verbose=False, post_setup=post, airborne=AIR)
    OUT.mkdir(parents=True, exist_ok=True)
    res["variant"], res["scenario_name"] = variant, scen
    (OUT / f"{variant}__{scen}.json").write_text(json.dumps(res))
    np.savez_compressed(OUT / f"{variant}__{scen}.npz", t=t, p=p, s=s)
    print(json.dumps(res))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--jobs", type=int, default=10)
    ap.add_argument("--scenarios", nargs="+", default=["hover", "A8", "A2"])
    ap.add_argument("--child", nargs=2)
    a = ap.parse_args()
    if a.child:
        child(*a.child); return
    OUT.mkdir(parents=True, exist_ok=True)
    jobs = [(v, s) for s in a.scenarios for v in VARIANTS if not (OUT / f"{v}__{s}.json").is_file()]

    def one(j):
        r = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--child", *j], cwd=str(HERE), capture_output=True, text=True)
        return f"{j}: {'ok' if r.returncode == 0 else 'FAILED ' + r.stderr.strip()[-300:]}"

    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        for m in ex.map(one, jobs):
            print(m, flush=True)
    rows = ["| variant | scenario | crashed | xy RMS [cm] | z mean [cm] | z RMS [cm] | max tilt [°] |", "|---|---|---|---|---|---|---|"]
    for s in a.scenarios:
        for v in VARIANTS:
            f = OUT / f"{v}__{s}.json"
            if f.is_file():
                r = json.loads(f.read_text())
                rows.append(f"| {v} | {s} | {r['crashed'] or 'no'} | {r['rms_xy_cm']:.1f} | {r['mean_dz_cm']:+.1f} | {r['rms_dz_cm']:.1f} | {r['max_tilt_deg']:.0f} |")
    (OUT / "summary.md").write_text("\n".join(rows) + "\n"); print("wrote", OUT / "summary.md")


if __name__ == "__main__":
    main()
