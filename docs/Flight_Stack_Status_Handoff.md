# Flight Stack Status — Engineering Handoff

**Thesis:** *Comparison of Control Strategies for Interaction-Force Aware Multirotor Teams*
**Platform:** Crazyflie 2.1 Brushless (41 g) · Crazyswarm2 / ROS 2 Humble · OptiTrack · out-of-tree controller in `no_std` Rust
**Date:** 24 August 2026 · **Purpose:** technical status for discussing next steps

---

## 0. The one-paragraph summary

Every piece of software needed to start the experimental campaign exists and has been verified in
simulation. **Nothing on the critical path is a software task any more** — the project is waiting on
lab access. The remaining code work (wiring strategies 4 and 6) sits *downstream* of three flight
phases and cannot sensibly be done first, because the model it consumes does not exist until real
flight data trains it.

The single largest risk is not incompleteness — it is that **three flight-code changes alter control
behaviour and have never flown**, and none of them is detectable in single-drone flight.

---

## 1. Controllers

Strategy 0 is the uncompensated geometric reference and is *not* one of the seven — without it,
"how much did compensation help" has no denominator.

| # | Strategy | Uses NN residual | State |
|---|---|---|---|
| 0 | Geometric baseline | no | ✅ Flying (reference condition) |
| 1 | Pure INDI | no — reacts to *measured* residual | ✅ Flying |
| 2 | Geometric + NN residual | yes, feedforward (`rnn.en`) | ⚠️ Implemented, **never flown** |
| 3 | FBL + NN residual | yes | ⬜ Blocked — FBL code still with the authors |
| 4 | Hybrid neural-augmented INDI | yes, alongside INDI measurement | ⬜ Not wired |
| 5 | Residual RL | separate policy net | ⬜ Deliberately deferred |
| 6 | Learning-based MPC | yes, as horizon prediction model | ⬜ Not wired |
| 7 | Geometric + residual RL | separate policy net | ⬜ Deliberately deferred |

**Scoreboard:** 2 flying · 1 implemented-unflown · 2 not wired · 1 externally blocked · 2 deferred.

**Why this is a fair comparison for 0/1/2/4:** these are not four controllers but four settings of
one controller, differing only in which residual term is active in the desired-acceleration vector.
That is verifiable against source, not asserted. **It does not extend to 3 and 6**, which are
structurally different controllers.

> Strategies 2 and 4 can run measured and predicted residual *together*. Double-counting is a real
> risk and is precisely what Strategy 4 exists to measure — so it is deliberately not prevented in
> code.

---

## 2. ⚠️ Three unflown flight-code changes — the gating risk

All three were found in simulation. **None is detectable in single-drone flight**, because `a_res`
is ~0 with only one vehicle.

| # | Change | What it does | Evidence |
|---|---|---|---|
| 1 | **Residual sign fix** | Position INDI was **adding** the residual instead of subtracting — it *reinforced* every unmodelled force | Measured **exactly 2.00×** on a known 20 mN disturbance; 1.89–2.10× across all formation scenarios. Now cancels to 0.0 mm |
| 2 | **`a_res` gating fix** | `a_res` read **exactly zero** whenever geometric was selected (RPM source gated on `mode != 0`), silently blocking all Geometric+NN training data | Probe: `experiments/analysis/probe_residual_sign.py` |
| 3 | **`rnn.en` compensation** | Feeds NN prediction into the position loop (Strategy 2). Default off — but it **changes a control term** | Sim only |

**Implication:** the sign fix means every pre-fix INDI number in the repo is obsolete. Hover before
anything else.

---

## 3. Simulation

| | |
|---|---|
| What runs | Geometric + all 3 INDI modes, as the **same Rust source that flies** (SIL bindings) |
| Interaction model | Neural-Swarm2 downwash; 2-drone downwash reproduced end-to-end in ROS on both controllers |
| Role in the thesis | **Used only to disprove, never to validate a method** — the simulated interaction comes from a learned model of the same family being fitted |

**Traps worth knowing:**

- Five sim-fidelity bugs had to be fixed first — missing accelerometer, wrong compiled airframe,
  chained motor calibrations, 2 kHz call with a ms tick, and `firmware_params` never applied.
  **Each one looked like a controller/tuning fault.** Check the startup line `applied N
  firmware_params` before blaming a control law.
- **Verification must use `experiments/analysis/verify_formation_sim.py`** (record_states + sidecar
  JSON). The runner's own report and `/pose` **do not work in sim**.
- Sim uses `kv_xy = 8.0`; **hardware keeps 5.0**. Confirm the readback before flying.
- Sim depends on ~110 **uncommitted** lines in `crazyflie-firmware/bindings/` — that tree tracks
  bitcraze upstream and is deliberately not forked. Patch preserved at
  `firmware_app/host/cffirmware_bindings.patch`.

**Open, paused, not blocking:** the simulator needs ~2× hardware's position damping to behave.
Time-boxed and parked — `KI_P`, rotor drag, inertia and airframe all refuted; motor lag explains
~40%.

---

## 4. Formation library & multi-robot execution

| | |
|---|---|
| Library | **16 scenarios frozen** — A1–A8 (pairs), B1–B3, C1–C5 (controls) |
| Sim validation | **34 cases = 16 scenarios × 2 controllers → 33 pass, 1 expected, 0 defects** |
| Control cases | Coplanar C1/C2/C3 = **0.0 mm on both controllers** — this is what makes the positive results meaningful |
| Geofence | Real: x ±1, y ±2, z 0.30–1.70. All 19 configs fit via `--auto-center --rotate 90` |
| Runners | `run_formation.py` (per-robot trajectories) — validated; `formation_flight.py` (shared trajectory) — **exists, never run** |

**Upload path:** Mode E (stock `uploadTrajectory`, multi-robot) is the standard. Mode D
(`--onboard`) is frozen byte-identical and still required for aggressive manoeuvres. Modes are
deliberately **not** bit-identical — frame convention is pinned per mode. Do **not** use
`--rest-to-rest` (measured worse).

**⚠️ Mode E maturity gap:** Mode E + full INDI has **180 hover flights but only 1 trajectory
flight.** Validate figure8 Mode D → figure8 Mode E → circle before trusting it.

**Two landmines:**

- **Poly4D CSVs need `%.12g`, not `%.6f`.** Six-decimal truncation turned 0.21 m/s² into 241 m/s²
  and crashed a drone. The Rust exporter escapes this only because its pieces are < 1 s.
- **Any stateful controller needs a per-vehicle state swap**, or the multi-drone simulator silently
  shares its filters between vehicles.

**Known coverage gaps:** yaw fixed at zero · no vertical relative motion except A7/C4 · nothing
above 3 robots.

---

## 5. Logging

| | |
|---|---|
| **The thesis signal** | `indi.a_res_*` = `a_meas − a_model` = f_res/m — logged in **every** controller mode on purpose, since Geometric+NN needs non-INDI training data |
| uSD | 500 Hz × ~35 variables per drone (incl. `rnn.pred_*`), **unaffected by drone count** — this is the dataset |
| Radio | ~811 pkt/s per drone vs ~1000 pkt/s per dongle **total**, 6 floats/topic max → **saturates at 2 drones**. Monitoring only |
| Sync | Broadcast start so per-drone logs share an origin; `merge_usd_logs.py` prints the measured offset |

**Two silent-data-loss bugs were found and fixed** in the uSD path: `decode_usd_log.py` `RENAME` now
maps all thesis channels, and `merge()` raises instead of skipping.

**⚠️ `a_res` reads exactly 0.0 without an RPM source. Zero means no thesis data at all** — verify on
the very first flight.

---

## 6. Residual-learning infrastructure — all three steps done

| Step | What exists | Verification |
|---|---|---|
| Onboard inference | Deep-sets MLP, **987 weights**, `no_std` Rust; `rnn.*` upload protocol refusing partial uploads; prediction logged every tick regardless of `rnn.en` | 10/10 numerical checks vs an independent reference |
| Training pipeline | uSD loader, PyTorch model, provenance, **normalisation folded into layer 1 at export**, ROS uploader | 12/12 checks end-to-end against the *compiled* controller |
| Sim dry run | collect → train → upload → enable → measure, through the lab's real code paths | 90% / 76% of `a_res` predicted open-loop; held-z separation error **4.66 → 0.63 mm** |

**Architecture:** `a_res = ρ(Σⱼ φ(rel_j))` — permutation-invariant, so team size is not baked in.
Inference onboard, training offline. Peer position is already available onboard via
`peerLocalizationGetPositionByID()`, so the network's main input is free.

> **Read the dry-run result correctly:** the residual it learned was *generated by a model of the
> same family being fitted*. It proves the plumbing, **not the method**.

---

## 7. Where the work stands

```
PREPARATION ████████████████████ 100%  ✅ finished
C.0 Hardware Gate  ░░░░░░░░░░░░░░░░░░  ⬅️ NEXT — blocked on lab access
C.1 Data collection ░░░░░░░░░░░░░░░░░  blocked by C.0
C.2 Train model     ░░░░░░░░░░░░░░░░░  pipeline ready, needs real data
C.3 Integrate 4 & 6 ░░░░░░░░░░░░░░░░░  ← the only remaining software
C.4 Comparison      ░░░░░░░░░░░░░░░░░  the thesis result
```

### What's left, by type

| Type | Items |
|---|---|
| **Lab measurement** | Tape-measure the flight volume (scenarios sit within ~10 cm of walls); hardware inventory; measure uSD sync |
| **Bench** | Flash firmware; confirm `kv_xy` reads back **5.0**, not the sim's 8.0 |
| **Flight** | Single-robot ladder (figure8 Mode D → Mode E → circle); validate the 3 unflown fixes; 2-robot at large separation (A1 Δz 0.75→0.50, then A3); confirm `a_res` non-zero and correctly signed |
| **Decision** | **Freeze the gains** — re-tuning later would measure tuning effort, not the methods |
| **Software** *(downstream)* | Wire strategies 4 and 6; strategy 3 when FBL code arrives; 5 and 7 last |

**Ordering is a constraint, not a suggestion.** The single-robot rungs exist to separate "the
migration broke something" from "the interaction broke something." If a two-robot flight is the
first thing that fails, those two causes are indistinguishable.

---

## 8. Six design questions — CLOSED by external review (2026-08-24)

| # | Question | Decision |
|---|---|---|
| 1 | C.1 scenario order | **Changed — A4 mandatory**, A7 or a lateral translate strongly preferred. A1+A3+A4 is the minimum viable dataset; C.2 does not start without all three |
| 2 | Frozen shared gains | **Keep frozen and shared.** Document the residual risk for strategies 3/6 under threats to validity |
| 3 | Strategy 4 form | **Keep the reference form** (NN bulk + incremental remainder). Open-loop model checks mitigate masking |
| 4 | 2 vs 3 robots | 2 suffice for the primary ranking (RQ1); **3 required for the transfer-penalty claim (RQ3)** — do not descope the B scenarios |
| 5 | Strategies 5/7 (RL) | **Explicitly deferred / stretch, not core.** No RL environment work now |
| 6 | Mode E maturity | **3-rung ladder is necessary and sufficient**: figure-8 Mode D → figure-8 Mode E → circle, then multi-robot |

**Explicitly not to be done now:** wire Strategy 4 or 6 · per-method gain tuning · RL environment
work · further simulator fidelity archaeology · results chapters.

### C.0 Hardware Gate — ordered

| Step | Action | Pass criterion |
|---|---|---|
| C.0.1 | Inventory + tape-measure volume + geofence | A/B scenarios fit with margin |
| C.0.2 | Flash firmware; confirm `kv_xy` readback | **5.0**, not 8.0 |
| C.0.3 | Single-robot hover, residual log live | `a_res` near 0, not stuck |
| C.0.4 | Known small disturbance or 2-drone large-Δz hover | Residual **cancels**, does not amplify |
| C.0.5 | figure-8 Mode D → Mode E → circle | Clean tracking, no oscillation |
| C.0.6 | 2-robot A1 at Δz 0.75 m then 0.50 m | Geometry held; `a_res` non-zero and correctly signed |
| C.0.7 | **Freeze gains** | Written decision, no further tuning |

Only after C.0.7 → C.1. The flight-day card for steps C.0.2–C.0.4 is at the top of
[`11_Hardware_Readiness_Checklist.md`](11_Hardware_Readiness_Checklist.md).

---

## 9. Reference map

| Doc | Contents |
|---|---|
| `07_Thesis_Progress_Checklist.md` | **Single source of truth** for status |
| `08_Trajectory_Upload_Paths.md` | Mode D/E engineering detail |
| `09_Simulation.md` | Simulator setup and fidelity traps |
| `10_Formation_Library.md` | The 16 scenarios |
| `11_Hardware_Readiness_Checklist.md` | C.0 checklists A/B/C |
| `12_Sim_Formation_Validation_Report.md` | 34-case validation + the two bug findings |
| `13_Residual_Learning.md` | Network, pipeline, dry run |
| `14_Repository_Map.md` | Where everything lives |

*Note: the thesis manuscript, bibliography and paper library are deliberately local-only and were
purged from git history on 2026-08-24. This document covers the engineering track only.*
