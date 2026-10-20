#!/usr/bin/env python3
"""Rebuild sections Overview .. 4 of docs/meetings/2026-10-22.md from experiments/results/unified_overview_2026-10-20/summary.json (keeps section 5 NS2 data, Questions, Next Steps).
  python3 experiments/analysis/build_meeting_results.py"""
import json
from pathlib import Path
REPO = Path(__file__).resolve().parents[2]
S = json.load(open(REPO / "experiments/results/unified_overview_2026-10-20/summary.json")); a8, a1, a4 = S["A8"], S["A1"], S["A4"]
p = REPO / "docs/meetings/2026-10-22.md"; m = p.read_text(); tail = ""
def c(s, k): d = a4[k][s]; return f"{d['mean']:.1f} cm (n={d['n']})" if d["n"] else "–"
def f(x, sign=False): return (f"{x:+.1f}" if sign else f"{x:.1f}").replace("-", "−")
a8n = sum(a8[k]['n'] for k in a8); a1n = sum(a1[k]['n'] for k in a1); a4n = sum(a4[o][k]['n'] for o in a4 for k in a4[o])
rows = []
for o in ("0.05", "0.1", "0.15"):
    cs = []
    for s in ("S0", "S1", "S2", "S3"):
        d = a4[o][s]; cs.append(f"{d['mean']:.1f} ± {d['std']:.1f} (n={d['n']})" if d["n"] > 1 else (f"{d['mean']:.1f} (n=1)" if d["n"] == 1 else "–"))
    rows.append(f"| {float(o):.2f} m | " + " | ".join(cs) + " |")
head = f"""# Meeting Document — 2026-10-22 (date open)

Deadline: 26 Feb 2027 (~18 weeks left)

## Overview
- **Unified controller tree:** one controller with two switches (INDI on/off, NS2 on/off) gives four strategies, validated on both drones: S0 geometric, S1 INDI, S2 geometric + NS2, S3 INDI + NS2.
- **Comparison (pilot flights of 2026-10-19/20, A8 swap, A1 hold, A4 offset line):** S0 is the worst in all three scenarios; S3 is the best or within the spread of the best in all three; S1 (INDI alone) is excellent in A1 and A4 but weaker than S2 in the A8 crossing.
- **A8 crossing dip:** {f(a8['S0']['mean'])} → {f(a8['S1']['mean'])} → {f(a8['S2']['mean'])} → {f(a8['S3']['mean'])} cm for S0 → S1 → S2 → S3.
- **A1 hold, mean z error:** {f(a1['S0']['mean'])}, {f(a1['S1']['mean'])}, {f(a1['S2']['mean'])}, {f(a1['S3']['mean'], True)} cm.
- **A4 (new):** the geometric controllers depend strongly on how far the drones are offset sideways, INDI hardly at all; NS2 cuts the geometric error by a third to two thirds.
- **NS2:** the neural network, retrained on 73 flights (about 19 min inside the gate, see section 5), runs in C at 526 µs per evaluation.
- **INDI is battery-sensitive:** the S1 flights on low batteries (3.4 V loaded) were clearly worse.
- **Data:** {a8n + a1n + a4n} pilot flights in the tables (A8 {a8n}, A1 {a1n}, A4 {a4n}), each with complete uSD logs of both drones; strategy of every flight checked from the logs. A bug found on the way (uSD card full at 100 files made the drones reset at landing) is fixed.

Notation: `z` = measured height, `z_cmd` = commanded height, `e_z = z − z_cmd` (positive = too high). A8 = two drones swap sides (4 crossings), A1 = stacked hold (15 s), A4 = bottom drone shuttles along a line beside the stack (offset 0.05 / 0.10 / 0.15 m). Bottom vehicle, dz 0.5 m, 0.5 m/s, uSD logs only, n = flights.

---

## 1. The four strategies at a glance
| | INDI | NS2 | A8 crossing dip | A1 mean z error | A4 z RMS, offset 0.05 m |
|---|---|---|---|---|---|
| S0 geometric | off | off | {f(a8['S0']['mean'])} cm (n={a8['S0']['n']}) | {f(a1['S0']['mean'])} cm (n={a1['S0']['n']}) | {c('S0','0.05')} |
| S1 INDI | on | off | {f(a8['S1']['mean'])} cm (n={a8['S1']['n']}) | {f(a1['S1']['mean'])} cm (n={a1['S1']['n']}) | {c('S1','0.05')} |
| S2 geometric + NS2 | off | on | {f(a8['S2']['mean'])} cm (n={a8['S2']['n']}) | {f(a1['S2']['mean'])} cm (n={a1['S2']['n']}) | {c('S2','0.05')} |
| S3 INDI + NS2 | on | on | {f(a8['S3']['mean'])} cm (n={a8['S3']['n']}) | {f(a1['S3']['mean'], True)} cm (n={a1['S3']['n']}) | {c('S3','0.05')} |

![A8 dip, A1 mean z error and A4 z RMS per strategy](assets/2026-10-19/overview.png)

- Pilot data: n = 3–7 per cell, one lab, two drones. Bars = mean over flights, whisker = ± std between flights, dots = single flights (A4: whiskers only).
- S3 A8: two further flights on a draining battery are excluded.
- Full tables: `docs/tree_comparison_2026-10-19.md` (§9), per-flight tables `docs/lab_sessions/2026-10-20.md`.

## 2. A8: crossing of the two drones
| | crossing dip (mean) | std between flights | worst crossing | z RMS | motors max |
|---|---|---|---|---|---|
| S0 geometric | {f(a8['S0']['mean'])} cm | {a8['S0']['std']:.1f} | {f(a8['S0']['worst'])} cm | {a8['S0']['zrms']:.1f} cm | 76–80 % |
| S1 INDI | {f(a8['S1']['mean'])} cm | {a8['S1']['std']:.1f} | {f(a8['S1']['worst'])} cm | {a8['S1']['zrms']:.1f} cm | 100 % |
| S2 geometric + NS2 | {f(a8['S2']['mean'])} cm | {a8['S2']['std']:.1f} | {f(a8['S2']['worst'])} cm | {a8['S2']['zrms']:.1f} cm | 79–82 % |
| S3 INDI + NS2 | {f(a8['S3']['mean'])} cm | {a8['S3']['std']:.1f} | {f(a8['S3']['worst'])} cm | {a8['S3']['zrms']:.1f} cm | 100 % |

![height error around the crossing, mean over the crossings per strategy (all flights)](assets/2026-10-19/A8_crossing_profile.png)

- **How the profile plot is made:** all A8 flights of a strategy, 4 crossings each; for every crossing the z error from −2 to +2 s around the zero-crossing of y, steady offset of that flight removed. Thick line = mean over all crossings (n flights × 4), shaded = ± 1 std between crossings.
- **Table and bars:** crossing dip = lowest point of a crossing, averaged per flight; mean = over flights, std = between flights (bars: whiskers = std, dots = flights).
- S1 has the largest spread: its fourth flight ran at 3.4 V loaded and dipped −12.4 cm; the other three −6.3 … −8.1 cm.
- Only the INDI strategies use the full motor range (100 %).

## 3. A1: stacked hold
| | z error mean | std between flights | z RMS | motors max |
|---|---|---|---|---|
| S0 geometric | {f(a1['S0']['mean'])} cm | {a1['S0']['std']:.1f} | {a1['S0']['zrms']:.1f} cm | 89–100 % |
| S1 INDI | {f(a1['S1']['mean'])} cm | {a1['S1']['std']:.1f} | {a1['S1']['zrms']:.1f} cm | 100 % |
| S2 geometric + NS2 | {f(a1['S2']['mean'])} cm | {a1['S2']['std']:.1f} | {a1['S2']['zrms']:.1f} cm | 90–100 % |
| S3 INDI + NS2 | {f(a1['S3']['mean'], True)} cm | {a1['S3']['std']:.1f} | {a1['S3']['zrms']:.1f} cm | 100 % |

![A1 hold: z error over time per strategy, and mean z error per strategy (all flights)](assets/2026-10-19/A1_comparison.png)

- **How the plot is made:** left: z error of every flight over the 15 s hold (window 2–15 s of the flight, time zero = start of the hold); thick line = mean over the strategy's flights at each time, shaded = ± 1 std between flights. Right: mean z error of each flight (dots), bar = mean over flights, whisker = std between flights.
- Geometric strategies sag in the hold and tilt up to 42°; NS2 halves the sag.
- S3 starts about +13 cm high (takeoff overshoot, gone after ~4 s).

## 4. A4: bottom drone shuttles beside the stack (new)
z RMS in cm, mean ± std over flights (n):
| offset | S0 geometric | S1 INDI | S2 geometric + NS2 | S3 INDI + NS2 |
|---|---|---|---|---|
""" + "\n".join(rows) + """

![A4: bottom z error over time, every flight, four strategies](assets/2026-10-19/A4_offsets_pilot.png)

- The closer the bottom drone passes to the upper one, the larger the error for the geometric controllers (S0 17 cm RMS at 0.05 m); NS2 cuts it by a third to two thirds (S2 against S0).
- INDI keeps the error at 3–5 cm at every offset; at 0.15 m all strategies except S0 are at 2–3 cm.
- Offset 0.0 was dropped (it repeats A1 and A8).

"""
old_fbl = []
_ = [l for l in tail.split("\n") if l.startswith("- **FBL controller follow-up questions/missing parts:**")]
if old_fbl:
    tail = tail.replace(old_fbl[0], "- **FBL controller (part of the study):** desk part of the port done (own fit, simulation, C code tested against the authors' code, firmware branch built, not flown yet); waiting for the authors' training script and flight gains")
tail = """## 5. NS2: training data
- All 73 flights together, flown with the geometric controller, labels from the deck RPM.
- **The gate (our own implementation, `ns2_residual.c` in our firmware):** the network only acts, and only trains on data, when a neighbour is above and close: |dx| < 0.2 m, |dy| < 0.2 m, |relative vx| < 1.5 m/s. Outside, its output is zero. Why: the downwash only matters when the other drone is right above; the reference (Neural-Swarm) also filters by a horizontal distance, the values here are ours. Crazyswarm2 / the simulator have no such gate.

""" + (REPO / "experiments/results/ns2_v2_data_summary.md").read_text().strip().replace("## 5. NS2 v2: what was flown\n", "") + """

![NS2 training data: gate time per scenario (left), coverage of vertical gap and horizontal offset (right)](assets/2026-10-19/ns2_v2_data.png)

- **Left:** seconds inside the gate per scenario. R1, A9, A2, A1 give most of it.
- **Right:** seconds of data per cell, horizontal offset (x) against vertical gap (y). Dashed lines = study gaps 0.25 / 0.5 / 0.75 m; red dotted = gate edge (±20 cm in x and y, corners reach 28 cm).
- Vertical gap in the gate: median 0.62 m; horizontal offset: median 0.09 m.
- Leave-one-flight-out: 69 of 73 flights pass.

---

## Questions:
- **NS2 execution context:** today inline in the controller, 100 Hz, 526 µs = 26 % of the 2 ms slot. Your idea: separate low-priority task + queue. I did not find it in any repo — which one did you mean?
- **Decision:** keep inline for the two-drone study, or parallel task now?

## Next Steps:
- **Study:** freeze protocol, start tier-1 two-drone study, then three vehicles
- **Hardware:** new drone pair, NS2 frozen
- **FBL (part of the study):** desk part done, waiting for the authors' training script and gains
"""
p.write_text(head + tail)
