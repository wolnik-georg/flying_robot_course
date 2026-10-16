#!/usr/bin/env python3
"""Study flight plan generator (2026-10-11): setting table, tier cells, shakedown card, session order.

Every command is validated before it is printed: the scenario is built with exactly the arguments of the command, the geofence
check runs as flown (--auto-center, height 0.5), and --allow-extreme is added only where the scenario needs it.
Outputs: docs/study_shakedown_card.md, out/study_plan/{settings.csv,tier_cells.csv,tier1_sessions.md}.
  python3 experiments/analysis/study_flight_plan.py            # regenerate everything
Settings mapping (speed -> scenario parameter) is the first-pass table of the protocol; it is fixed at the shakedown."""
import csv, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
CS2 = Path.home() / "Desktop" / "crazyswarm2" / "crazyflie_examples"
sys.path.insert(0, str(CS2))
from crazyflie_examples.formations import safety, scenarios  # noqa: E402
from crazyflie_examples.run_formation import centred_anchor  # noqa: E402

OUT = HERE / "out" / "study_plan"; OUT.mkdir(parents=True, exist_ok=True)
SPEEDS = (0.2, 0.3, 0.4, 0.5)
A8_DURATION = {0.2: 10.9, 0.3: 7.3, 0.4: 5.5, 0.5: 4.4}     # peak speeds 0.20 / 0.30 / 0.40 / 0.50 m/s (checked below)
A2_PERIOD = {0.2: 12.6, 0.3: 8.4, 0.4: 6.3, 0.5: 5.0}        # r 0.4 m
# 2026-10-14: Omar C controller tree, S0 geometric / S1 full INDI / S2 geometric + NS2 / S3 full INDI + NS2 (docs/plan_topics_2026-10-14.md)
STRATEGIES = ("S0", "S1", "S2", "S3")


def setting(scen: str, key: str, val: float) -> dict:
    """-> {scenario, setting, kwargs (scenarios.build), cli (list of strings)}."""
    if scen == "A1":
        kw = dict(dz=val, hold=15.0)
    elif scen == "A8":
        kw = dict(dz=0.5, span=1.0, passes=4, duration=A8_DURATION[val])
    elif scen == "A2":
        kw = dict(dz=0.5, path="circle", speed=val)
    elif scen == "A5":
        kw = dict(dz=0.5, period=A2_PERIOD[val])
    elif scen == "A3":
        kw = dict(dz=0.5, speed=val)
    elif scen == "A4":
        kw = dict(dz=0.5, speed=val, offset=0.10, motion="line")
    elif scen == "A7":
        kw = dict(speed=val)
    else:
        raise ValueError(scen)
    return {"scenario": scen, "setting": f"{key}={val}", "kwargs": kw}


def validate(s: dict) -> dict:
    sc = scenarios.build(s["scenario"], **s["kwargs"])
    lim, lim_x = safety.Limits(allow_extreme=False), safety.Limits(allow_extreme=True)
    base = centred_anchor(sc, lim, 0.5)
    prob, prob_x = safety.check(sc, lim, base=base), safety.check(sc, lim_x, base=base)
    s = dict(s)
    s["allow_extreme"] = bool(prob) and not prob_x
    s["valid"] = not prob_x
    s["problems"] = prob_x
    s["peak_speed"] = sc.realised.get("peak_speed")
    s["duration_s"] = sc.duration
    return s


def cli(s: dict, strategy: str, session: str) -> str:
    parts = ["ros2 run crazyflie_examples run_formation", f"--scenario {s['scenario']}"]
    for k, v in s["kwargs"].items():
        parts.append(f"--{k} {v}")
    # A7 sets its own low anchor (apply_a7_lab_defaults): an explicit --height 0.5 is REFUSED there (0.5 + dz_start 1.1 > A7_TOP_MAX 1.2)
    parts += ([] if s["scenario"] == "A7" else ["--height 0.5"]) + ["--auto-center", "--yes"]
    if s["allow_extreme"]:
        parts.append("--allow-extreme")
    # 2026-10-15: the --study readback gate is not used (readback returned NaN; strategy is proven from the log by
    # tree_log_check.py and from the bench readback). strategy / session stay in the signature for the plan tables.
    return " ".join(parts)


def all_settings():
    out = []
    for dz in (0.25, 0.5, 0.75):
        out.append(validate(setting("A1", "dz", dz)))
    for sp in SPEEDS:
        for scen in ("A8", "A2", "A5", "A3", "A4", "A7"):
            out.append(validate(setting(scen, "speed", sp)))
    return out


TIERS = {  # protocol section 3: tier 1 = A1 x3 dz, A8 x4 speeds, A2/A3/A4/A5/A7 at 0.4; tier 2 adds A2 and A3 at 0.2/0.3/0.5; tier 3 adds A4, A5, A7 at 0.2/0.3/0.5
    1: lambda s: (s["scenario"] == "A1") or (s["scenario"] == "A8") or (s["setting"] == "speed=0.4"),
    2: lambda s: s["scenario"] in ("A2", "A3") and s["setting"] != "speed=0.4",
    3: lambda s: s["scenario"] in ("A4", "A5", "A7") and s["setting"] != "speed=0.4",
}


def shakedown_rows(S):
    """Tree validation card (lab plan docs/lab_sessions/next_session_plan.md): A1 stack hold and A8 per strategy, order S0, S2, S1, S3
    (S1 and S3 last: INDI takeoff from the ground with Omar's controller has never been flown). Solo hovers (simple_flight) are in the lab plan."""
    by = {(s["scenario"], s["setting"]): s for s in S}
    g = lambda sc, st: by[(sc, st)]
    a8, a1 = g("A8", "speed=0.4"), g("A1", "dz=0.5")
    rows = [("S0", "scenario", a8), ("S0", "scenario", a1)]
    rows += [("S2", "scenario", a1), ("S2", "scenario", a8)]
    rows += [("S1", "scenario", a8), ("S1", "scenario", a1)]
    rows += [("S3", "scenario", a1), ("S3", "scenario", a8)]
    rows += [("S0", "anchor", a8)]
    return rows


def main():
    S = all_settings()
    bad = [s for s in S if not s["valid"]]
    with open(OUT / "settings.csv", "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["scenario", "setting", "kwargs", "peak_speed", "duration_s", "needs_allow_extreme", "valid"])
        for s in S:
            w.writerow([s["scenario"], s["setting"], s["kwargs"], f"{s['peak_speed']:.3f}", f"{s['duration_s']:.1f}", s["allow_extreme"], s["valid"]])
    cells = []
    for tier, pred in TIERS.items():
        for s in S:
            if pred(s):
                for st in STRATEGIES:
                    cells.append({"tier": tier, "scenario": s["scenario"], "setting": s["setting"], "strategy": st, "clean_flights_needed": 5})
    with open(OUT / "tier_cells.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(cells[0].keys())); w.writeheader(); w.writerows(cells)
    n = {t: sum(1 for c in cells if c["tier"] <= t) for t in (1, 2, 3)}

    # tier-1 session order (controller-grouped, rotating order, S0 anchors at start and end) -- 8 sessions of ~3 h is the protocol estimate
    t1 = [s for s in S if TIERS[1](s)]
    orders = [("S0", "S1", "S3", "S2"), ("S1", "S2", "S0", "S3"), ("S2", "S3", "S1", "S0"), ("S3", "S0", "S2", "S1")]   # balanced Latin square (each strategy once per position, each ordered pair once)
    lines = ["# Tier 1 session order (generated)", "", f"{len(t1)} settings × {len(STRATEGIES)} strategies = {len(t1) * len(STRATEGIES)} cells; 5 clean flights per cell = {len(t1) * len(STRATEGIES) * 5} clean flights. "
             "Per session: 2 × S0 A8 anchor at the start and 2 at the end (they also count for the S0 A8 cell), controller blocks in the rotating order below, settings in the fixed order below.",
             "", "Setting order: " + ", ".join(f"{s['scenario']} {s['setting']}" for s in t1), "", "Rotation by session: " + " | ".join("→".join(o) for o in orders) + " (repeat)", ""]
    OUT_TXT = OUT / "tier1_sessions.md"; OUT_TXT.write_text("\n".join(lines) + "\n")

    # shakedown card
    rows = shakedown_rows(S)
    L = ["# Tree validation flight card (generated by `experiments/analysis/study_flight_plan.py`; file name kept)", "",
         f"{len(rows)} flights, controller-grouped (S0 → S2 → S1 → S3, S0 anchor at the end; Omar C tree, both drones controller 9). Every command was validated with the scenario generator and the geofence check as flown (`--auto-center`, height 0.5).",
         "Before each block: select the strategy's yaml (`git checkout crazyflie/config/crazyflies.yaml && git apply crazyflie/config/study/tree_S<k>.patch`; tags `study-S<k>` once they exist), relaunch the server, check `--study` prints the config block.",
         "Abort rule (protocol §4): |roll| or |pitch| > 25° for 0.5 s, or z < 0.25 m; stop the block after 2 crashes. Fresh batteries (rest ≥ 4.1 V). One line per flight in the decisions file (`flight_decision.py`).", "",
         "| # | block | kind | scenario | setting | peak v (m/s) | duration (s) | command | done | clean/excl | vbat min | notes |", "|---|---|---|---|---|---:|---:|---|---|---|---|---|"]
    for i, (st, kind, s) in enumerate(rows, 1):
        L.append(f"| {i} | {st} | {kind} | {s['scenario']} | {s['setting']} | {s['peak_speed']:.2f} | {s['duration_s']:.0f} | `{cli(s, st, 'shakedown')}` | ☐ | | | |")
    L += ["", f"Estimated time at ~12 flights/hour: {len(rows) / 12:.1f} h (estimate from 2026-10-09, to be replaced by the measured rate).", "",
          f"Study size from the settings table: tier 1 = {n[1]} cells ({n[1] * 5} clean flights), tier 1+2 = {n[2]} cells ({n[2] * 5}), all = {n[3]} cells ({n[3] * 5}); cells in `out/study_plan/tier_cells.csv`.", "",
          "A8 speed is set with `--duration` (added to `run_formation` 2026-10-11; before, every A8 flight used the default 6.0 s = 0.365 m/s peak, so the nominal 'speed=0.4' of the old A8 flights is 0.365)."]
    (REPO / "docs" / "study_shakedown_card.md").write_text("\n".join(L) + "\n")
    print(f"settings {len(S)}, invalid {len(bad)}; shakedown {len(rows)} flights; cells tier1/2/3 = {n[1]}/{n[2]}/{n[3]}")
    for s in bad:
        print("INVALID", s["scenario"], s["setting"], s["problems"])
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
