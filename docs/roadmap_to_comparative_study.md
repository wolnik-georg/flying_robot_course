# Roadmap to the 2-drone comparative study (written 2026-10-09, after the lab validation day)

Deadline **26 Feb 2027** (≈ 20 weeks). Strategies (docs/07): **0** geometric baseline · **1** pure INDI (candidates: ours, Omar C + Iz 1.5) · **2** geometric + NS2 residual (`res_sign −1`) · **3** FBL + residual (blocked: authors' code not received). NA-INDI is not compared (supervisor decision).

## 1. What is finished (state 2026-10-09)
| Area | Result | Where |
|---|---|---|
| NS2 (Strategy 2) | 100 Hz works; `res_sign −1` correct: A8 dip −3.4 cm vs −5.9 off vs −11.0 `+1`; A1 oscillation not caused by the network; unified firmware sanity −2.98 cm; SIL validated for A8 (within ≈ 1 cm) | docs/62, 68, 72 |
| Omar INDI z offset | opt-in z integral; **study variant = Omar C + Iz 1.5 (controller 9)**: +2.4 cm steady on A8 over 4 flights / 2 days (exact: +13…+23 cm); Rust and no-integral variants dropped | docs/65, 66, 70, 71, study_parameter_decisions |
| RPM filter | sentinel fix harmless on our INDI (same-day OFF/ON pair); stays ON | docs/67, lab_sessions/2026-10-09 |
| Firmware | one unified build on cf5 (network + `kpos_iz` + filter fix), bench timing 504/533 µs | lab_session_pack_rpm_filter, lab_sessions/2026-10-09 |
| A8 comparison | geometric +0.8 / NS2 −1 +0.3 / ours INDI +1.9 / Omar C + Iz +2.4 cm; dips −5.9 / −3.2 / −9.3 / −10.3; ours tightest laterally (0.8 cm) | docs/71 |
| Analysis tooling | robust crossing detector (`a8_crossings.py`; the old one was wrong in 8/29 flights), one comparison script, SIL showcase + animation | docs/71, 72 |
| Frozen rules | `res_sign` −1 only for `ctrl_mode 0`; integral 1.5; 100 Hz; all scenarios stay; fresh batteries, pose bag, uSD check | study_parameter_decisions |


## 1b. Readiness per controller and open items (status 2026-10-09 evening)
| Controller | Validated with the final config | NOT yet flown with the final config |
|---|---|---|
| S0 geometric | A8 (4 flights, +0.8 cm), A1 | A2, A3, A4, A5, A7 (the C.1 flights used an older gain setup, so they are not study data) |
| S2 NS2 (`res_sign −1`) | A8 (−3.4 cm), A1 (stable, 3 flights), unified-firmware sanity (−2.98 cm) | A2, A3, A4, A5, A7 |
| S1 Omar C + Iz 1.5 | A8 only (4 flights, 2 days, +2.4 cm) | A1 and all other scenarios; deck RPM never tried on other moving 2-drone scenarios |
| Ours INDI (supplementary) | A8 only (+1.9 cm) | everything else; weak on A1 |
**Decisions still open:** (1) Pure INDI — Omar C + Iz 1.5 expected (supervisor); (2) NS2 retrain or keep — must be decided **before the shakedown**, then weights frozen; also tier 2/3 go/no-go, exact speed↔parameter mapping, ours as supplementary column.
**Work still open before data collection:** logging changes (SD `config.txt` Omar C channels; `run_formation` meta with the full yaml block + hashes), tag yaml commits `study-S0/S1/S2`, extend `metrics.py`/`aggregate.py` to the sweep cells, the shakedown session (every scenario × controller once, throughput, speed 0.5 feasibility), protocol v1.0 freeze.
**Verdict:** ready for the shakedown, not yet for the data collection.

## 2. Next steps, in order
**Phase A — desk, now (no lab needed)**
1. **Study protocol** (`docs/comparative_study_protocol.md`): scenario list (A1, A2, A3, A4, A5, A7, A8, C5 — the 2-drone library; A6/C4 dropped), controllers/strategies and their frozen configs (yaml + firmware sha), repeats (≥ 3 clean flights per scenario × controller), interleaved order, metrics (steady z error, z RMS, crossing dip, lateral RMS, max tilt, saturation, vbat) with the fixed definitions of docs/71, abort/battery/pose-bag rules, file naming, exclusion rules (crash, pose swap, abort — always listed, never silent).
2. **"Pure INDI" decision** (you + supervisor): ours vs Omar C + Iz 1.5. Evidence: A8 — same level (+1.9 vs +2.4 cm), ours tighter (z sd 0.5 vs 1.8 cm, lateral 0.8 vs 3.0 cm); ours weak on A1 (saturation, oscillation); Omar C needs the optical deck and is unfiltered (no dropouts in 12 flights). Option: carry both into the shakedown and decide with data.
3. **Scenario readiness check** per controller: which scenarios have any flight with the final configs (only A8 for all; A1 for geometric/NS2/ours); is the NS2 network covered by its training bank for each scenario; scenario params fixed (dz, speed, height).
4. **One processing script for any session** (copy/`cmp`/archive helper, merge, per-flight metrics, per-cell table, figures) built on `a8_variant_comparison.py` / `a8_crossings.py`, generalised from A8 to all scenarios (crossings are A8-specific: other scenarios need their own event definition — e.g. stack hold for A1).
5. **Supervisor items:** FBL follow-up (professor already wrote, waiting), meeting document (yours: Neural-Swarm2 section), Strategy 3 status.
6. **Writing:** Ch. 6–9 skeleton from docs/62, 64, 68–72 (NS2 result, INDI comparison, RPM filter, simulation).
7. Optional desk: why our INDI level moved from +4.2 (10-02) to ≈ +1.9 cm; `Kpos_Iz` scaling between the Omar ports (no longer needed — Rust dropped); harness fix so our INDI can run in the SIL; bench latency + logging patch.

**Phase B — lab session "shakedown" (1 session)**
- Fly every scenario once per candidate controller (geometric, NS2 −1, ours INDI, Omar C + Iz 1.5) to find failures *before* the data collection: INDI on A1 (saturation), the network on the other scenarios, Omar C deck RPM in 2-drone scenarios, tracker pose swaps. Gains stay frozen; failures become documented exclusions or scope decisions (supervisor).
- Output: a go/no-go table per cell (scenario × controller).

**Phase C — data collection (several lab sessions)**
- Rough size: 8 scenarios × 4 controllers × ≥ 3 clean flights ≈ 100 flights (+ repeats for failures); estimated 6–8 lab sessions at ≈ 12–15 flights per session — **rough estimate, to be replaced by the protocol and the shakedown numbers**.
- Every session: fresh batteries (rest ≥ 4.1 V), pose bag, `check_usd_deck.py` on both drones, unified firmware ON, yaml per controller from the protocol, flight log table in `docs/lab_sessions/<date>.md`, cards `cf_second` first, then processing with the one script.
- Interleave controllers inside a session (not all flights of one controller in a row) to decouple battery/day effects.

**Phase D — analysis and writing (≈ Dec–Feb)**
- Per-cell tables and figures, statistics across flights, SIL comparison where valid, discussion of limits (A1, SIL gaps, battery dependence), chapters 6–9, final presentation material.
- Proposed calendar (adjust with the supervisor): Oct — protocol + decisions + shakedown; Nov — data collection; Dec — analysis + chapters; Jan — writing + gap-filling flights; Feb — finish before 26 Feb.

## 3. Open risks
Ours INDI on A1 · NS2 not validated on A1 (SIL pessimistic, hardware stable) · pose swaps (battery-related so far) · cf5 uSD card lost files once · Omar C deck RPM dropouts possible in 2-drone scenarios · Strategy 3 blocked · several variants have only 2–4 flights so far.
