# 24 — Downwash Compensation Comparison (living results table)

**Purpose.** One accumulating table of how each control strategy performs against *real*
2-drone downwash, on the same scenario, so results are directly comparable as controllers are
added. This is the working record that C.4 (Systematic Comparison) will be built from — not
C.4 itself, which needs repeats and statistics.

**Protocol (keep identical for every new row).**

- **Study vehicle (bottom, in the wash):** **`cf5`** since 2026-09-19. **`cf231_active`** rows
  below are historical (18 Sep only); do not mix protocols without labelling.
- **Partner (top):** **`cf_second`**, stock Lee (`controller: 5`) — disturbance source only.
- Both vehicles brushless (CF21BL).
- **Source of record:** merged **uSD @ 500 Hz** (`merge_usd_logs.py --meta --roles`). Radio CSVs
  are monitoring only (`docs/27`).
- **Tracking error:** use **`ctrltarget.*`** from uSD when present; no radio reconstruction for
  thesis figures.
- **Canonical number for a new table row:** `compare_downwash.py --merged-usd` over the
  **scenario duration window** (same policy as `ctrltarget.*`). Phase/crossing RMSE for C.4
  lives in `vehicle_metrics_by_phase` / `run_c4_desk_prep.py`. Catalog “whole log” RMSE in
  `docs/28` is a quick QA column only — do not paste it into **docs/24** if it disagrees.
- **Phases:** report **scenario / crossing** windows via `vehicle_metrics_by_phase` (`docs/27`
  P2), not whole-log RMSE when comparing downwash unless explicitly noted.
- **Repeats:** 3–5 **flights** per (scenario, controller) for C.4 claims — not one flight sliced
  into phases (`aggregate.py` guards).
- Default comparison scenario for baseline rows: **A8** with params from the flight
  `.meta.json` (often `passes=1` on 19 Sep logs; state that in the row).

> ⚠️ **Do not apply `Z_OFFSET_COMPENSATION` when reconstructing the command.** It was removed
> 2026-09-18 (`crazyswarm2` `d824fba`). Runs before that on `cf_second` carry a +0.40 m
> over-command; `cf231_active` never carried it at all.

---

## Results — `cf231_active` (bottom, receiving downwash)

| Strategy / controller | pos RMS x | pos RMS y | pos RMS z | **pos RMS ‖e‖** | max ‖e‖ | roll std | pitch std | yaw std | gyro std x | `a_res_z` peak | run |
|---|---|---|---|---|---|---|---|---|---|---|---|
| **S0 — Geometric** (ours, `c=6 m=0`) | 64.5 mm | 39.8 mm | 70.5 mm | **103.5 mm** | 227.4 mm | 7.14° | 6.16° | 2.07° | 58.95 | −2.81 m/s² | 2026-09-18 18:31:15 |
| **S1 — Full INDI** (ours, `c=6 m=3`) | **5.8 mm** | **8.0 mm** | **34.2 mm** | **35.6 mm** | 145.2 mm | 2.70° | 1.64° | 0.16° | 58.53 | −4.14 m/s² | 2026-09-18 18:39:08 |
| **S1b — Stock INDI** (Bitcraze, `c=3`) | — | — | — | — | — | — | — | — | — | — | *19 Sep cf5, n=1 unstable — not a row* |
| **S1c — Briesewitz INDI** (`c=7`) | — | — | — | — | — | — | — | — | — | — | *not yet flown* |
| **S4 — NA-INDI** (`c=8`) | — | — | — | — | — | — | — | — | — | — | *not yet flown* |
| **S2 — Neural-Swarm2** | — | — | — | — | — | — | — | — | — | — | *needs C.1/C.2 first* |

Reference, same runs — `cf_second` (top, stock Lee, **not** receiving downwash): roll std
1.04–1.06°, peak 2.02–2.33°, pitch std 0.20–0.27°. Confirms the attitude disturbance is
specific to the lower vehicle. Its **z error is not comparable across runs** (see the warning
above): 296 mm at 18:31 with the compensation active, 77 mm at 18:39 without it, commanded
1.25 m both times.

---

## What the first two rows already tell us

**INDI improves nearly every metric 2–13× while measuring a *larger* disturbance**
(`a_res_z` −4.14 vs −2.81 m/s²). Three structural observations:

1. **Lateral is where INDI wins biggest** — x **11.1×**, y 5.0×, yaw **12.9×**.
2. **Vertical is where it wins least** — z RMS only 2.1×, z peak only 1.1× — and **z now
   dominates the residual: 34.2 mm of the 35.6 mm total ‖e‖.** Physically consistent, since
   downwash is primarily a vertical force and the vertical channel is where the uncompensated
   remainder lands. **This is the specific headroom the interaction-force-aware strategies
   must close**, and it is a much sharper target than "reduce attitude error".
3. **Gyro std is essentially unchanged** (58.95 → 58.53 °/s). INDI suppresses the
   low-frequency attitude excursion and the position error but **not** the high-frequency
   content — the standing 6–8 Hz attitude limit cycle is present in both runs and untouched.
   That remains its own separate open problem and should not be conflated with downwash
   rejection.

**Why INDI cannot erase it (structural, not a tuning failure).** INDI is *reactive*: it
estimates the residual from measured acceleration and cancels it, so it cannot act before the
disturbance has been measured — an irreducible lag of at least the filter time constant, with
`fc_bw=206` / `res_fc=80` further attenuating and delaying a fast crossing transient. And
lateral rejection must route through the attitude loop to tilt the thrust vector, far slower
than the direct thrust channel that handles the vertical component.

---

## Status and caveats

- **Indicative, not statistical.** One run per controller. A preview of C.4, not C.4 itself —
  the systematic comparison needs repeats under matched conditions.
- Both rows were flown **before** the 2026-09-18 EKF-reset fix (`crazyswarm2` `8296ad8`), in a
  session where the estimator was intermittently corrupt. The two runs themselves were clean,
  but re-confirming them on a validated platform is worthwhile.
- Next row to fill: **stock Bitcraze INDI (`controller: 3`)** — a fully independent third-party
  implementation that ignores our `indi_gains`/`pos_gains` entirely (same arrangement as the
  stock-Lee pin). It answers directly whether ~35.6 mm / 2.70° is simply what INDI can do
  against this downwash, or whether ours is under-performing. Hover first: its gains are
  compiled defaults for a standard CF2.1, unvalidated on this brushless airframe.

## Results — `cf5` (bottom, uSD, 2026-09-19)

**Protocol note:** Rows below use **merged uSD @ 500 Hz** (`ctrltarget.*`), whole merged window,
`passes=1` (not the 2026-09-18 `passes=2` pair). Study vehicle **cf5** replaces `cf231_active`
for this session. **Indicative — n=2** for geometric vs full INDI only.

| Strategy (cf5) | pos RMSE ‖e‖ (cf5) | run stamps | Source |
|---|---|---|---|
| Stock Lee (both drones `c=5`) | 37–104 mm (3 repeats; one outlier 15-00-55) | 14-58-02, 15-00-55, 15-01-33 | uSD merged |
| **Geometric** (`c=6 m=0`) | **40–42 mm** | 15-04-01, 15-04-41 | uSD merged |
| **Full INDI** (`c=6 m=3`) | **36–37 mm** | 15-06-38, 15-07-11 | uSD merged |

Packages, plots, and repro: **`docs/28_USD_Radio_Matching_and_Session_Analysis.md`**.

---

## Reproducing the numbers

Logs live in `experiments/logs/` as `A8_<drone>_<date>_<time>.csv` with an
`A8_<date>_<time>.meta.json` sidecar carrying each vehicle's resolved per-drone config
(`per_drone`), `anchor`, `t_start_sim` and the scenario params. The analysis is the protocol
above; see `docs/lab_sessions/2026-09-18.md` §1b and docs/07 History (44) for the full
narrative of how these two rows were produced.
