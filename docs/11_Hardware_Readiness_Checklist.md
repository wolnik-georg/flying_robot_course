# Hardware Readiness Checklist — the operational detail of **C.0**

**Last updated:** 23 August 2026

> ### ⬅️ **This is the next actionable phase of the thesis.**
>
> **C.0 — Hardware Gate** is step 1 of the five-step Core Thesis Workflow in
> [`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md). **Software preparation is
> finished**; nothing in software blocks these items. The project is waiting on a drone.
>
> C.0 is not an experiment. It is the check that nothing is broken, so that everything measured
> afterwards means something. **No data collected before C.0 passes counts.**
>
> **C.0 runs in three strictly ordered stages** (confirmed 2026-08-23):
>
> | Stage | What | Note |
> |---|---|---|
> | **1** | Validate the controllers still behave — single robot, geometric then INDI | Cannot confirm the three unflown changes are *correct*, only that they are not catastrophic |
> | **2** | Retune the brushless attitude loop **if needed**, then **FREEZE THE GAINS** | The retune is allowed **here and only here**. The freeze is the last moment any gain may change |
> | **3** | Two robots at large separation; confirm `a_res` non-zero; check uSD sync | The first flights where the residual sign fix is observable at all |
>
> Tuning before validating means tuning against an unknown fault. Flying two robots before the
> freeze means the gains move underneath the dataset.

Everything that must be true before the first real flights, and the order to fly them in.
This document does not authorise anything. Each item is either a measurement you take or a
decision you make.

Status lives in [`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md); this
is the operational detail.

**What C.0 unlocks:** C.1 Residual Data Collection → C.2 Train the Residual Model →
C.3 Integrate the Strategies → C.4 Systematic Comparison.

---

## Why the order matters

Nothing in this project has flown since the Mode E migration. The simulator has caught
several things that would have been discovered the hard way, but it is **not** a substitute
for a flight, and it currently runs at different position gains than hardware (see
[`09_Simulation.md`](09_Simulation.md)). So the ladder starts with the least risky flight
that can still fail informatively, and each rung must pass before the next.

**Three flight-code changes have never flown and are cleared here.** All three alter control
behaviour and **none is detectable in single-drone flight**, because `a_res ≈ 0` with no
neighbour present:

| # | Change | Risk if it is wrong |
|---|---|---|
| 1 | **Residual sign fix** | Position INDI previously *reinforced* every unmodelled force at exactly 2.00×. The fix inverts a control term |
| 2 | **`a_res` gating fix** | `a_res` was dead under geometric, silently blocking all Geometric+NN training data |
| 3 | **`rnn.en` compensation** | The learned residual can now feed the position loop. Default off → byte-identical, but it is a control term. **Leave it off until the rest of C.0 has passed** |

**Do not skip to two robots.** The single-robot rungs exist to separate "the migration
broke something" from "the interaction broke something". If a two-robot flight is the first
thing that fails, those two causes are indistinguishable.

---

## Stage 0 — Bench, no propellers turning

| # | Item | Pass criterion |
|---|---|---|
| 0.1 | **Measure the flight volume** | Real x/y/z limits written into `formations/safety.py` `FLIGHT_SPACE`, replacing the placeholder |
| 0.2 | Hardware inventory | Working brushless drones, decks, charged batteries, spare props — counted |
| 0.3 | Firmware bindings decision | `crazyflie-firmware/bindings/` changes committed somewhere durable, or the patch confirmed re-appliable |
| 0.4 | Flash firmware | `cd flying_drone_stack/firmware_app && make cload` completes |
| 0.5 | Confirm gains on the drone | `indi_gains.*` and `pos_gains.*` read back as `crazyflies.yaml` intends — **`kv_xy` must be 5.0, not the simulator's 10.0** |
| 0.6 | Confirm `clamp_en` | Currently `11` (tilt clamp OFF, set for inverted-loop work). Decide deliberately |

### 0.1 — the flight volume, and the floor we choose

Two different limits, deliberately separated in `formations/safety.py`:

```python
FLIGHT_SPACE          = dict(x=(-1.0, 1.0), y=(-2.0, 2.0), z=(0.0, 1.70))  # where mocap sees
Z_FLOOR_DEFAULT       = 0.30    # where we choose to fly formations
Z_FLOOR_GROUND_EFFECT = 0.10    # only when ground effect is the measurement
```

**The physical volume** is what the cameras track — z runs to the floor. Confirmed 2026-08-23 and
consistent with an independent statement from 2026-07-27 (x and z identical, y then quoted as
±2.1; the conservative ±2.0 is applied). Still not tape-measured.

**The operational floor is a separate decision.** Near the ground a rotor's downwash reflects and
pushes the vehicle up — ground effect, roughly the same magnitude as the inter-vehicle downwash
this thesis measures, and *not* the quantity under study. A formation at 0.15 m would contaminate
every residual with a second unmodelled force having nothing to do with the other drone. 0.30 m is
several rotor diameters up and comfortably clear.

**But ground effect is itself a residual force**, and this project measures residual forces. C5
exists to characterise it in isolation and has to go low to see anything, so it gets
`--z-floor 0.10` rather than being treated as a safety exception. Whether the same learned models
can compensate it is a legitimate extension — see [`10`](10_Formation_Library.md).

> `--auto-center` centres **horizontally only**. Altitude is an experimental parameter, not
> something to optimise for clearance: vertical centring silently lifted C5 from 0.15 m to 0.85 m,
> which is a different experiment. If a scenario does not fit at the requested `--height`, the
> check refuses and says so.

**Heights that fit** (with `--auto-center --rotate 90`): most scenarios at `--height 1.0`;
A1 Δz 0.75 needs 0.85; A4, B1, B2, B3, C4 need 0.9; A7 needs 0.5; C5 flies at 0.15.

### ⚠️ 0.5 — the simulator and hardware run different position gains

| | `kv_xy` | ζ |
|---|---|---|
| `crazyflies.yaml` (hardware) | **5.0** | 0.31 |
| `crazyflies_sim.yaml` (simulation only) | 10.0 | 0.62 |

The simulator needs about twice the damping of the real vehicle for reasons not yet
understood. **Never copy the simulator value onto hardware**, and never assume a
sim-validated scenario has been validated at the gains that will fly.

---

## Checklist A — Hardware inventory

Everything that must be present **and confirmed working**, not merely present. Tick only after a
power-on check, not from the shelf.

### Vehicles

- [ ] **2 × Crazyflie 2.1 brushless (CF21BL)** — the minimum for the two-robot protocol
- [ ] **3rd CF21BL** — required for B1/B2/B3 and C1–C3; the three-robot scenarios cannot run without it
- [ ] Spare airframe or a full set of spare parts (motors, arms) — a crash mid-campaign otherwise stops everything
- [ ] Each vehicle powers on, connects, and holds a stable hover under the stock controller
- [ ] All vehicles are the **same** platform. The gains block is shared across every robot, so a mixed pair would fly one of them on the wrong mass, `kt` and inertia

### Decks — per vehicle

- [ ] **Micro SD deck** on each vehicle. This is the thesis dataset; radio cannot carry two drones at full rate
- [ ] **Rotor-speed source** on each vehicle — optical RPM deck, or DShot telemetry on the brushless platform. **Without it `a_res` reads exactly zero and there is no thesis data**
- [ ] Marker set fitted and each vehicle trackable by the mocap, with a distinct rigid-body definition

### Power

- [ ] At least **4 charged batteries per vehicle** — a formation flight burns two vehicles' worth per run
- [ ] Multi-bay charger, so charging is not the bottleneck in a session
- [ ] Batteries labelled and cycle-tracked; a sagging cell changes thrust and therefore the residual

### Logging and ground station

- [ ] **uSD card per vehicle**, formatted, with `usd_thesis_config.txt` copied to the card root as `config.txt`
- [ ] Card reader on the ground-station machine
- [ ] `check_usd_deck.py` confirms each deck is detected
- [ ] Crazyradio dongle(s). Two drones at full log rate exceed one dongle — either drop the rates or use a second dongle on another channel

### Environment

- [ ] Mocap calibrated, covering the whole intended flight volume
- [ ] Netting or a defined safe perimeter
- [ ] Physical kill switch / emergency stop reachable by the operator

---

## Checklist B — C.0 acceptance criteria

**Hardware validation of the two unflown flight-code fixes.** Both change or enable a control
path and neither has ever been in the air. C.0 is not an experiment — it is the check that nothing
is broken, and **no data collected before it passes counts**.

### Required flights, in this order

| # | Flight | Controller | Minimum |
|---|---|---|---|
| 1 | Hover 30 s at 1.0 m | Geometric | Establishes the baseline is unchanged |
| 2 | figure-8, `--kt 0.05` | Geometric | Matches the recorded baseline RMSE |
| 3 | **Hover 30 s at 1.0 m** | **Corrected full INDI** | **The gate — first flight of the sign fix** |
| 4 | **figure-8, `--kt 0.05`** | **Corrected full INDI** | Sign fix under real dynamics |
| 5 | A1 at Δz 0.75 m, 2 robots | Geometric | First interaction, widest separation |
| 6 | A1 at Δz 0.75 m, 2 robots | Corrected full INDI | Residual actually being used |

Flight 3 comes before flight 4, and 4 before 5. **Do not skip to a formation.**

### What must be checked in the logs

**Residual signal — the whole point**

- [ ] `indi.a_res_{x,y,z}` is **non-zero** in flight. Zero means no rotor-speed source and no thesis data
- [ ] In single-drone hover (flights 1–4), `|a_res|` is **small** — there is no other vehicle, so a large residual means the thrust model is wrong, not that a disturbance exists
- [ ] In flight 5, the **lower** drone shows a **negative** `a_res_z` — downwash pushes it down. A positive value means the sign convention is inverted somewhere in the chain
- [ ] `a_res` is non-zero under **geometric** too (flights 1, 2, 5). This is the second bug fix; zero here means Geometric+NN training data is still impossible
- [ ] Noise on `a_res` is low enough to be usable — judge against the ~0.5 m/s² magnitude seen for a 5% -of-weight disturbance in simulation

**No new instability from the sign fix**

- [ ] No sustained oscillation in position or attitude. Specifically check for a **~1–1.5 Hz position oscillation** — the sim showed the position loop ringing at its own frequency at these gains
- [ ] Attitude stays within normal bounds; no growing tilt envelope over the flight
- [ ] Motor commands are not persistently saturated
- [ ] INDI hover RMSE is **no worse than geometric hover RMSE**. The sign fix should improve or leave disturbance rejection unchanged, never degrade it

**Tracking**

- [ ] figure-8 RMSE under corrected INDI is within **20%** of the geometric baseline
- [ ] Two-robot A1 achieved separation is within **50 mm** of commanded

### Pass / fail

**PASS** — all six flights complete without incident, every box above ticked, and `a_res` is
non-zero, correctly signed and usable under both controllers.

**FAIL — stop and investigate, do not proceed to C.1 — if any of:**

- `a_res` reads zero in any flight → no rotor-speed source; the entire dataset would be empty
- `a_res_z` on the lower drone is **positive** in flight 5 → a sign convention is still wrong somewhere
- INDI hover is worse than geometric hover → the sign fix has not done what the derivation says
- Any sustained oscillation appears that was not present before the fix
- Any crash

> A failure here is informative and cheap. A failure discovered after a data-collection campaign
> is neither.

---

## Checklist C — Freeze-the-gains criteria

The method comparison is only fair if every strategy flies the **same vehicle configuration**.
Re-tuning between methods measures tuning effort, not the methods.

### Required before freezing

- [ ] C.0 passed in full (Checklist B)
- [ ] Single-drone figure-8 flown **at least 3 times** under the final configuration
- [ ] Two-drone A1 at Δz 0.50 m flown **at least 3 times**
- [ ] Two-drone A3 at Δz 0.40 m flown **at least twice**
- [ ] All flights on **fully charged** batteries — a sagging pack changes thrust and biases the residual

### Acceptance metrics

| Metric | Threshold | Why |
|---|---|---|
| Single-drone figure-8 XY RMSE | ≤ 3.5 cm, spread ≤ 1 cm across repeats | The recorded best is 2.4 cm; this allows margin without accepting a regression |
| Two-drone A1 achieved separation | within 50 mm of commanded | Matches the simulation tolerance |
| Repeat-to-repeat variation in separation | ≤ 15 mm | If the same flight is not repeatable, no comparison between methods is either |
| Position oscillation at 1–1.5 Hz | not visible in the logs | The known thin-margin failure mode |
| `a_res` magnitude on repeated identical flights | consistent within 20% | The measurement itself must be repeatable before it can be trained on |

### What is frozen

Once the boxes above are ticked, record the exact values and **change nothing thereafter**:

- [ ] `indi_gains.*` — `kr`, `kw`, `kr_z`, `kw_z`, `fc_bw`, `mass`, `kt1`–`kt4`, `filt_order`, `filt_tau`, `clamp_en` and the clamp limits
- [ ] `pos_gains.*` — `kp_xy`, `kp_z`, `kv_xy`, `kv_z`
- [ ] Firmware build hash
- [ ] Vehicle mass as flown, including every deck
- [ ] Geofence values

**After this point no further tuning is permitted.** If a method appears to need different gains,
that is a *result about the method*, and it is reported as such — not fixed by re-tuning.

> The one legitimate exception: a **safety-critical** discovery, such as an instability that risks
> a crash. If that happens, the freeze is broken deliberately, the reason is recorded, and **every
> method flown before it is re-flown** under the new configuration. Partial re-flights invalidate
> the comparison.

---

## Residual data collection order (C.1)

Decided 2026-08-23. Flown under **geometric** control — it does not compensate, so the residual is
observed rather than partly cancelled by the controller being measured.

| Order | Scenario | Separation | Why here |
|---|---|---|---|
| 1 | **A1** vertical stack hover | Δz 0.75 m → 0.50 m | Widest and least dynamic. Both vehicles stationary, so any displacement is the interaction itself |
| 2 | **A3** static-top | Δz 0.40 m → 0.30 m | Adds motion to one vehicle only. Captures wash entry and exit, not just the steady level |
| 3 | **A4** offset stack | Δz 0.60 m, offset 0.10 m | Partial overlap — the asymmetric regime a model needs and cannot infer from aligned cases |

**Only after all three are clean** does anything closer follow — A2 and A5 for constant-separation
tracking, then A7's merge, and the `--allow-extreme` scenarios (A6, C4) last of all.


---

## Stage 1 — Single robot, Mode D baseline

Mode D is the frozen predecessor and the only path with a flight history. It establishes
that the airframe, gains and tracking still behave as they did.

```bash
ros2 run crazyflie_examples flight -- --trajectory figure8 --mode 1 --kt 0.05 --onboard
```

**Pass:** RMSE consistent with the recorded baseline; no oscillation; `indi.a_res_*`
non-zero (needs an RPM source — zero means no telemetry and no thesis data).

---

## Stage 2 — Single robot, Mode E — the migration test

Same trajectory, same gains, **without** `--onboard`.

```bash
ros2 run crazyflie_examples flight -- --trajectory figure8 --mode 1 --kt 0.05
```

**Pass:** RMSE ≈ Stage 1. This is the gate the whole Mode E migration rests on.

> If Stage 2 does not match Stage 1, **stop**. The offline equivalence analysis said they
> should match; a mismatch means that analysis missed something.

Then `--trajectory circle --kt 0.1` to confirm rest-to-rest behaves (no swing-out at start).

---

## Stage 3 — Single robot, watch the stability margin

The tuning notes record `kv_xy=4` crashing 2 of 2 flights and `kv_xy=5` flying. That is a
thin margin, and the simulator — even allowing for its 2× discrepancy — is oscillating at
the position loop's own frequency at these settings.

**Before adding a second drone**, fly Stage 2 again and look specifically for a low-frequency
(~1–1.5 Hz) position oscillation in the logs. If it is visible even at low amplitude, raising
`kv_xy` toward 7 (ζ=0.44, an already-flown value per the notes) buys margin at a small RMSE
cost — the notes record 2.6/2.8/2.6 cm at `kv=7` against 2.4/2.3 cm at `kv=5`.

**This is a judgement call for you, not the agent.** Formation flight adds downwash to the
same loop.

---

## Stage 4 — Two robots, large separation only

Only after Stages 1–3 pass and Stage 0 is complete.

```bash
# always dry-run first: prints the plan, commands nothing
ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.75 --check

# then fly, starting WIDE
ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.75 --brushless
```

Order: **A1 at Δz 0.75 → A1 at 0.50 → A3 at 0.40 → A3 at 0.30**.

- A1 (both hovering) is the least dynamic thing two drones can do
- A3 (top hovers, bottom traverses) adds motion to only one vehicle
- Everything else waits

**Do not go below Δz = 0.5 m on the first day.** The tighter scenarios (A6 at 0.08–0.10 m,
A7 merging to 0.10 m, C4 docking) are `--allow-extreme` gated for a reason and should be
last, after the residual measurement is confirmed working and the tracking margin is known.

**Check on the first two-robot flight:**

| | Why |
|---|---|
| `indi.a_res_*` non-zero on both | Zero means no RPM source — no thesis data |
| Measured uSD sync from `merge_usd_logs.py` | The few-ms figure is predicted, never measured on real logs |
| Achieved vs commanded separation | `run_formation` prints it on landing |
| Any ~1–1.5 Hz position oscillation | The margin question from Stage 3, now with downwash |

---

## What the simulator is and is not for

**Use it for:** formation geometry correctness, multi-robot logging and procedure, catching
crashes before they cost hardware, developing scripts.

**Do not use it for:** claiming one controller tracks better than another. It has no motor
lag, no EKF, no sensor noise, an interaction model fitted to someone else's vehicle, and
currently different position gains. A controller comparison is a hardware result.

Confirm before each session that the scenarios still pass their geometry checks:

```bash
python -m crazyflie_examples.formations.scenarios --self-test    # 38 cases
ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.75 --check
```
