#!/usr/bin/env python3
"""Study readiness matrix (rewritten 2026-10-11 after the first version was found unusable).

Evidence, in this order (every number in the output carries its source):
  1. curated flight labels: `experiments/logs/flight_decisions_history_a8.csv` (A8, docs/71) and
     `out/ns2_signtest_2026-10-08/a1_usd_summary.json` (A1 on 2026-10-08, network off/on);
  2. meta.json `per_drone.cf5` (controller, ctrl_mode, pos gains, ki_z) -- the metas do NOT record rnn.en / Kpos_Iz / the network
     state, and `rnn_pred_*` is logged (shadow mode) even with the network off, so the network state cannot be inferred from them.
Writes `docs/study_readiness_matrix.md` and `out/study_readiness/inventory.json`.
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
LOGS = REPO / "experiments" / "logs"
USD = LOGS / "usd_raw"
OUT_JSON = REPO / "experiments" / "analysis" / "out" / "study_readiness" / "inventory.json"
OUT_MD = REPO / "docs" / "study_readiness_matrix.md"
HISTORY = LOGS / "flight_decisions_history_a8.csv"
A1_SUMMARY = REPO / "experiments" / "analysis" / "out" / "ns2_signtest_2026-10-08" / "a1_usd_summary.json"
BANK_MANIFEST = REPO / "experiments" / "analysis" / "out" / "c2_e2e_2026-10-01" / "training_manifest_lab.json"

STUDY_SCENARIOS = ("A1", "A2", "A3", "A4", "A5", "A7", "A8")
STRATEGIES = ("S0", "S1", "S2", "ours")
S0_GAINS = {"kp_xy": 40.0, "kp_z": 30.0, "kv_xy": 8.0, "kv_z": 10.0, "ki_z": 16.0}   # protocol section 2

CS2 = Path("/home/georg/Desktop/crazyswarm2/crazyflie_examples")
sys.path.insert(0, str(CS2))
from crazyflie_examples.formations import safety, scenarios  # noqa: E402
from crazyflie_examples.run_formation import centred_anchor  # noqa: E402

sys.path.insert(0, str(REPO / "flying_drone_stack" / "tools" / "residual"))
sys.path.insert(0, str(REPO / "experiments" / "analysis"))
import dataset as rnn_dataset  # noqa: E402


# ------------------------------------------------------------------ 1. flight inventory
_A8_USD = None


def _a8_usd_set() -> set:
    """A8 uSD files are named by COPY time, not flight stamp: take the matched paths from the validated A8 comparison."""
    global _A8_USD
    if _A8_USD is None:
        f = REPO / "experiments" / "analysis" / "out" / "a8_compare_2026-10-09" / "flights_used.csv"
        _A8_USD = {(r["date"], r["stamp"]) for r in csv.DictReader(open(f)) if r.get("usd_path")} if f.is_file() else set()
    return _A8_USD


def has_usd(date: str, stamp: str, scen: str) -> bool:
    if scen == "A8" and (date, stamp) in _a8_usd_set():
        return True
    return any(p.stat().st_size > 0 for p in USD.glob(f"cf5_*{date}_{stamp}.bin")) or \
        any(p.stat().st_size > 0 for p in USD.glob(f"cf5_{scen}_*{date}_{stamp}*.bin"))


def curated_labels() -> dict[tuple[str, str], dict]:
    lab: dict[tuple[str, str], dict] = {}
    if HISTORY.is_file():
        for r in csv.DictReader(open(HISTORY)):
            lab[(r["date"], r["stamp"])] = {"strategy": r["strategy"], "decision": r["decision"], "source": "flight_decisions_history_a8.csv"}
    if A1_SUMMARY.is_file():
        for r in json.load(open(A1_SUMMARY)):
            lab[("2026-10-08", r["stamp"])] = {"strategy": "S2" if r["net"] == "on" else "S0", "decision": "clean (uSD summary)",
                                               "source": "a1_usd_summary.json (network " + r["net"] + ", res_sign -1)"}
    return lab


def classify(meta: dict, date: str, stamp: str, labels: dict) -> tuple[str, str]:
    """-> (class, evidence). class in S0 S1 S2 ours | ambiguous | other."""
    lab = labels.get((date, stamp))
    if lab:
        return lab["strategy"], lab["source"]
    c = (meta.get("per_drone") or {}).get("cf5") or {}
    if (meta.get("names") or [""])[0] != "cf5" or not c:
        return "other", "cf5 is not the bottom drone / no per_drone.cf5"
    ctrl, mode = c.get("controller"), c.get("ctrl_mode")
    pos = c.get("pos") or {}
    ki = pos.get("ki_z")
    if ctrl == 9:
        return "ambiguous", "controller 9 (Omar C); Kpos_Iz not recorded in the meta (S1 only if listed in the curated history)"
    if ctrl == 6 and mode == 3:
        return ("ours", "meta: ctrl_mode 3, ki_z 0") if ki == 0.0 else ("other", "ctrl_mode 3 with ki_z != 0 (pre-fix)")
    if ctrl == 6 and mode == 0:
        if all(pos.get(k) == v for k, v in S0_GAINS.items()):
            return "ambiguous", "geometric with the study gains (40/30/8/10, ki_z 16) but the network state (rnn.en) is not recorded: S0 or S2"
        return "other", f"geometric with older gains / no z integral (ki_z={ki}, kp_xy={pos.get('kp_xy')})"
    return "other", f"controller {ctrl} ctrl_mode {mode}"


def inventory() -> dict:
    labels = curated_labels()
    flights = []
    for p in sorted(LOGS.glob("A[1-8]_20*.meta.json")):
        meta = json.loads(p.read_text())
        scen = meta.get("scenario")
        if scen not in STUDY_SCENARIOS:
            continue
        stem = p.name[:-len(".meta.json")]
        date, stamp = stem.split("_", 1)[1][:10], stem.split("_", 1)[1][11:]
        cls, ev = classify(meta, date, stamp, labels)
        lab = labels.get((date, stamp), {})
        flights.append({"scenario": scen, "date": date, "stamp": stamp, "class": cls, "evidence": ev,
                        "decision": lab.get("decision", "unknown"), "usd_cf5": has_usd(date, stamp, scen)})
    counts = {}
    for sc in STUDY_SCENARIOS:
        row = {}
        for st in STRATEGIES:
            f = [x for x in flights if x["scenario"] == sc and x["class"] == st]
            row[st] = {"flights": len(f), "clean": sum(1 for x in f if x["decision"].startswith("clean")), "usd": sum(1 for x in f if x["usd_cf5"]),
                       "dates": sorted({x["date"] for x in f})}
        row["ambiguous"] = sum(1 for x in flights if x["scenario"] == sc and x["class"] == "ambiguous")
        counts[sc] = row
    return {"flights": flights, "counts": counts}


# ------------------------------------------------------------------ 2. NS2 gate coverage
GATE_XY = 0.2   # model.build_mask: |dx| < 0.2 and |dy| < 0.2 (and |dv| < 1.5) -- dataset.py docstring


def bank_dz_samples() -> np.ndarray:
    m = json.loads(BANK_MANIFEST.read_text())
    dz = []
    n_fl = 0
    for f in m["flights"]:
        p = REPO / f["path"]
        if not p.is_file():
            continue
        try:
            rel, mask, _, _, _ = rnn_dataset.build([str(p)], verbose=False)
        except Exception:
            continue
        present = mask > 0.5
        if np.any(present):
            dz.extend(np.abs(rel[present][:, 2]).tolist()); n_fl += 1
    return np.asarray(dz), n_fl, [f["folder"] for f in m["flights"]]


def planned_settings():
    a2_period = {0.2: 12.6, 0.3: 8.4, 0.4: 6.3, 0.5: 5.0}
    a8_dur = {0.2: 10.9, 0.3: 7.3, 0.4: 5.5, 0.5: 4.4}
    out = []
    for dz in (0.25, 0.5, 0.75):
        out.append(("A1", f"dz={dz}", dict(dz=dz, hold=15.0)))
    for sp in (0.2, 0.3, 0.4, 0.5):
        out += [("A8", f"speed={sp}", dict(dz=0.5, duration=a8_dur[sp], passes=4, span=1.0)),
                ("A2", f"speed={sp}", dict(dz=0.5, speed=sp, path="circle")),
                ("A5", f"speed={sp}", dict(dz=0.5, period=a2_period[sp])),
                ("A3", f"speed={sp}", dict(dz=0.5, speed=sp)),
                ("A4", f"speed={sp}", dict(dz=0.5, speed=sp, offset=0.10, motion="line")),
                ("A7", f"speed={sp}", dict(speed=sp))]
    return out


def gate_coverage(bank_dz: np.ndarray) -> list[dict]:
    lo, hi = float(np.percentile(bank_dz, 1)), float(np.percentile(bank_dz, 99))
    rows = []
    for scen, setting, kw in planned_settings():
        sc = scenarios.build(scen, **kw)
        ts = np.arange(0.0, sc.duration, 0.05)
        rel = np.array([sc.relative(0, 1, t) for t in ts])
        dxy = np.abs(rel[:, :2])
        gated = (dxy[:, 0] < GATE_XY) & (dxy[:, 1] < GATE_XY)
        dz = np.abs(rel[:, 2])
        frac_gate = float(gated.mean())
        if gated.any():
            dzg = dz[gated]
            inside = float(((dzg >= lo) & (dzg <= hi)).mean())
            dz_med = float(np.median(dzg))
            density = float(((bank_dz > dz_med - 0.05) & (bank_dz < dz_med + 0.05)).mean())
            rng = (float(np.percentile(dzg, 5)), float(np.percentile(dzg, 95)))
        else:
            inside, dz_med, density, rng = float("nan"), float("nan"), float("nan"), (float("nan"),) * 2
        rows.append({"scenario": scen, "setting": setting, "time_in_gate": frac_gate, "dz_in_bank_range": inside,
                     "dz_p5_p95_in_gate": rng, "bank_density_near_median_dz": density, "median_dz_in_gate": dz_med})
    return rows


# ------------------------------------------------------------------ 3. feasibility (with --auto-center, as flown)
def feasibility() -> list[dict]:
    rows = []
    for scen, setting, kw in planned_settings():
        sc = scenarios.build(scen, **kw)
        lim, lim_x = safety.Limits(allow_extreme=False), safety.Limits(allow_extreme=True)
        base = centred_anchor(sc, lim, 0.5)
        prob, prob_x = safety.check(sc, lim, base=base), safety.check(sc, lim_x, base=base)
        _, pos = safety.sample_positions(sc, base=base)
        rows.append({"scenario": scen, "setting": setting, "peak_speed": sc.realised.get("peak_speed"), "peak_accel": sc.realised.get("peak_accel"),
                     "x_extent": [float(pos[:, :, 0].min()), float(pos[:, :, 0].max())], "y_extent": [float(pos[:, :, 1].min()), float(pos[:, :, 1].max())],
                     "extreme_tag": bool(sc.extreme), "needs_allow_extreme": bool(prob) and not prob_x, "fails_even_with_extreme": bool(prob_x),
                     "violations": prob if prob else [], "violations_with_extreme": prob_x})
    return rows


# ------------------------------------------------------------------ markdown
def f3(x, nd=2):
    return "—" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{nd}f}"


def write_markdown(inv, cov, feas, bank_info):
    c = inv["counts"]
    L = ["# Study readiness matrix (generated)", "",
         "Generated by `experiments/analysis/build_study_readiness.py` (2026-10-11); numbers traceable in `experiments/analysis/out/study_readiness/inventory.json`.", "",
         "## 1. Flights per scenario × strategy (cf5 = bottom drone, two-drone scenarios)",
         "**Evidence.** (a) curated labels: A8 from `flight_decisions_history_a8.csv` (docs/71), A1 of 2026-10-08 from `a1_usd_summary.json` (network off/on); "
         "(b) the meta `per_drone.cf5` (controller, ctrl_mode, pos gains, ki_z). The metas do **not** record `rnn.en` or `Kpos_Iz`, and `rnn_pred_*` is logged in shadow mode even "
         "with the network off, so the network state cannot be read from any log — flights without a curated label stay *ambiguous*.", "",
         "Strict definitions: **S0** geometric (`controller 6, ctrl_mode 0`, gains 40/30/8/10, `ki_z 16`), network off · **S1** Omar C + `Kpos_Iz 1.5` · **S2** as S0 with network on "
         "(`res_sign −1`, 100 Hz) · **ours** `ctrl_mode 3`, `ki_z 0` (supplementary). Flights with older gains or no z integral (all C.1 flights, `ki_z` null) are not study data.", "",
         "| Scenario | S0 clean (uSD) | S1 clean (uSD) | S2 clean (uSD) | ours (uSD) | ambiguous (study gains, network state unknown / Omar C) |", "|---|---|---|---|---|---|"]
    for sc in STUDY_SCENARIOS:
        r = c[sc]
        cell = lambda st: f"{r[st]['clean']} ({r[st]['usd']})" if r[st]["flights"] else "0"
        L.append(f"| {sc} | {cell('S0')} | {cell('S1')} | {cell('S2')} | {r['ours']['flights']} ({r['ours']['usd']}) | {r['ambiguous']} |")
    L += ["", "*clean* = labelled clean in the curated source; (n) = flights with a non-empty cf5 uSD file. Clean/excluded for the other flights is decided at the card pull "
          "(`flight_decisions.csv`), not known here. A2, A3, A4, A5, A7 have no flight with the final study configuration (their only flights are C.1 flights with older gains).", "",
          "## 2. NS2 coverage (does the network act, and does the training bank cover it?)",
          f"Training bank: {bank_info['n_flights']} of the {bank_info['n_manifest']} manifest flights loaded (gated samples: {bank_info['n_samples']}; unsigned |dz| 1st–99th percentile "
          f"{bank_info['dz_lo']:.2f}–{bank_info['dz_hi']:.2f} m; the bank holds only samples inside the gate |dx|<0.2, |dy|<0.2). "
          "The network acts only while the neighbour is inside that gate, so per planned setting (commanded geometry sampled every 0.05 s):", "",
          "- **time in gate** = share of the flight in which the network can act; **|dz| in bank range** = share of those samples whose |dz| lies within the bank's 1–99 percentile; "
          "**bank density** = share of all bank samples within ±0.05 m of the median gated |dz|.", "",
          "| Scenario | setting | time in gate | gated |dz| p5–p95 (m) | |dz| in bank range | bank density near median dz |", "|---|---|---:|---|---:|---:|"]
    for r in cov:
        L.append(f"| {r['scenario']} | {r['setting']} | {f3(r['time_in_gate'])} | {f3(r['dz_p5_p95_in_gate'][0])}–{f3(r['dz_p5_p95_in_gate'][1])} | {f3(r['dz_in_bank_range'])} | {f3(r['bank_density_near_median_dz'],3)} |")
    L += ["", "## 3. Feasibility per setting (geofence check as flown: `--auto-center`, height 0.5 m)", "",
          "| Scenario | setting | peak v (m/s) | peak a (m/s²) | x extent (m) | y extent (m) | tagged extreme | needs `--allow-extreme` | fails even with it |", "|---|---|---:|---:|---|---|---|---|---|"]
    for r in feas:
        L.append(f"| {r['scenario']} | {r['setting']} | {f3(r['peak_speed'])} | {f3(r['peak_accel'])} | {f3(r['x_extent'][0])}…{f3(r['x_extent'][1])} | {f3(r['y_extent'][0])}…{f3(r['y_extent'][1])} | "
                 f"{r['extreme_tag']} | {r['needs_allow_extreme']} | {r['fails_even_with_extreme']} |")
    bad = [r for r in feas if r["fails_even_with_extreme"]]
    L += ["", "Violations that remain even with `--allow-extreme`: " + ("; ".join(sorted({f"{r['scenario']} {r['setting']}: {r['violations_with_extreme'][0]}" for r in bad})) if bad else "none") + ".", "",
          "## 4. Known risks (with source)",
          "- **A1 under the geometric controller oscillates.** cf5 swings ±35–40° (roll/pitch sd 19–22°, ~1.5 Hz, xy rms 7–8 cm) with the network off *and* on, n = 2 vs 3 flights on 2026-10-08 "
          "(`docs/68`, `a1_usd_summary.json`); attributed to the geometric controller under the stacked downwash, cf5 solo hover is calm. The protocol abort rule (|roll| or |pitch| > 25° for 0.5 s) "
          "would trigger on these flights — an A1 abort rule is needed before the shakedown.",
          "- **Our INDI saturates / oscillates on A1** (96 % of the steady samples with a motor at the PWM ceiling on 2026-10-02, `docs/64`); Omar C + Iz 1.5 has **never flown A1**.",
          "- **Omar C reads the unfiltered optical-deck RPM**: no dropout in 12 flights (A8 only); never tried in the moving 2-drone scenarios.",
          "- **Tracker pose swaps and battery** caused the 2026-10-05 crashes, not the network (`docs/lab_sessions/2026-10-05.md`); fresh batteries and the pose bag per flight.",
          f"- **NS2 coverage (table 2):** the network can act during {min(r['time_in_gate'] for r in cov if r['scenario']=='A8'):.0%}–{max(r['time_in_gate'] for r in cov if r['scenario']=='A8'):.0%} of an A8 flight "
          f"(A8 is not in the training bank); the bank is thin near dz 0.75 m ({[r for r in cov if r['setting']=='dz=0.75'][0]['bank_density_near_median_dz']:.1%} of its samples within ±0.05 m, "
          f"against {[r for r in cov if r['setting']=='dz=0.5' and r['scenario']=='A1'][0]['bank_density_near_median_dz']:.1%} at 0.5 m and {[r for r in cov if r['setting']=='dz=0.25'][0]['bank_density_near_median_dz']:.1%} at 0.25 m).", "",
          "## 5. Shakedown test list (prioritised, from the facts above)",
          "1. **A1 with S0, S2, S1, ours** — the abort rule problem and untested Omar C; fly each once, record roll p99.",
          "2. **S1 (Omar C + Iz 1.5) on the moving scenarios A2, A3, A4, A5, A7** — never flown; check deck RPM dropouts and the z offset (one flight each, speed 0.4).",
          "3. **S0 and S2 on A2, A3, A4, A5, A7 with the study gains and `ki_z 16`** — only older-gain C.1 flights exist.",
          "4. **Speed 0.5 and dz 0.25 / 0.75** — settings tagged or flagged in tables 2–3 (A7 is extreme-tagged).",
          "5. **S0 anchor on A8** bookends the session.", "",
          "Abort criteria: |roll| or |pitch| > 25° for 0.5 s (revisit for A1, see risks), z < 0.25 m, pose jump, 2 crashes stop the block; fresh batteries (rest ≥ 4.1 V).", "",
          "## 6. Could not be determined",
          "- Clean/excluded per flight outside the curated sources (decided at the card pull, `flight_decisions.csv`).",
          "- Network state of the flights with the study gains but no curated label (`rnn.en` is not in the meta).",
          "- `Kpos_Iz` of Omar C flights outside the curated A8 history.",
          "- 100 Hz network rate (`rnn.div`) per flight; the planned-setting coverage uses the commanded geometry, not flown trajectories."]
    OUT_MD.write_text("\n".join(L) + "\n")


def main():
    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    inv = inventory()
    bank_dz, n_fl, manifest = bank_dz_samples()
    bank_info = {"n_flights": n_fl, "n_manifest": len(manifest), "n_samples": int(len(bank_dz)),
                 "dz_lo": float(np.percentile(bank_dz, 1)), "dz_hi": float(np.percentile(bank_dz, 99)), "manifest_flights": manifest}
    cov, feas = gate_coverage(bank_dz), feasibility()
    OUT_JSON.write_text(json.dumps({"inventory": inv, "bank": bank_info, "gate_coverage": cov, "feasibility": feas}, indent=1, default=str))
    write_markdown(inv, cov, feas, bank_info)
    print(f"wrote {OUT_JSON}\nwrote {OUT_MD}")


if __name__ == "__main__":
    main()
