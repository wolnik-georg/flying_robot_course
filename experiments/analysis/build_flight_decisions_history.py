#!/usr/bin/env python3
"""Seed `experiments/logs/flight_decisions_history_a8.csv` from the validated A8 comparison (docs/71):
flights_used.csv -> decision clean (note carries the variant and flags), flights_excluded.csv -> excluded.
Strategy mapping: Geometric baseline -> S0; NS2 res_sign -1 -> S2; Omar C + Iz 1.5 -> S1; Ours INDI* -> ours;
everything else (Omar Rust, Omar C exact / Iz 1.0 / 2.0, NS2 res_sign +1) -> other:<variant> (history, not study strategies).
Historical (2026-10-02..10-09), reconstructed from the lab docs / a8_variant_comparison.py, not decisions taken at a card pull."""
import csv
from pathlib import Path

HERE = Path(__file__).resolve().parent
CMP = HERE / "out" / "a8_compare_2026-10-09"
OUT = HERE.parents[0] / "logs" / "flight_decisions_history_a8.csv"


def strategy(variant: str) -> str:
    if variant == "Geometric baseline":
        return "S0"
    if variant == "NS2 res_sign −1":
        return "S2"
    if variant.startswith("Omar C + Iz 1.5"):
        return "S1"
    if variant.startswith("Ours INDI"):
        return "ours"
    return "other:" + variant


def main():
    rows = []
    for r in csv.DictReader(open(CMP / "flights_used.csv")):
        rows.append(dict(date=r["date"], stamp=r["stamp"], scenario="A8", setting="", strategy=strategy(r["variant"]),
                         decision="clean", reason=r.get("flags", ""), vbat_min="", notes=f"history; variant={r['variant']}; source={r['source']}"))
    for r in csv.DictReader(open(CMP / "flights_excluded.csv")):
        rows.append(dict(date=r["date"], stamp=r["stamp"], scenario="A8", setting="", strategy="", decision="excluded",
                         reason=r["reason"], vbat_min="", notes=f"history; variant={r['variant']}"))
    rows.sort(key=lambda x: (x["date"], x["stamp"]))
    with open(OUT, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["date", "stamp", "scenario", "setting", "strategy", "decision", "reason", "vbat_min", "notes"])
        w.writeheader(); w.writerows(rows)
    print(f"wrote {len(rows)} rows -> {OUT}")


if __name__ == "__main__":
    main()
