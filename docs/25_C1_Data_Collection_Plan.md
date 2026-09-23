# 25 — C.1 Residual Data Collection Plan (Neural-Swarm2, Strategy 2)

Pre-decided so lab time is spent flying, not deciding. Fly top-down; every block is useful on
its own, so stopping early still yields a trainable set.

---

## Progress — 2026-09-23 lab (+ 21 Sep bank)

Training-eligible uSD: `experiments/logs/c1_2026-09-23_merged/manifest_2026-09-23_c1.json`
(**19** merges on 23 Sep) plus `experiments/logs/c1_2026-09-21_merged/manifest_2026-09-21_c1.json`
(**5** training + 3 A8 QA — training/QA split for 21 Sep is **not** in that manifest JSON; see
`experiments/logs/c1_2026-09-21_merged/training_eligible_crosswalk.json`). Session log:
`docs/lab_sessions/2026-09-23.md`.

| Plan row | Status | Manifest / note |
|---|---|---|
| A-1 **A7** | **3/3** | 23 Sep `19-11-19` … `19-13-55` |
| A-2 **A1** dz 0.20 ×2 | **1/2** | 21 Sep `13-25-10` — optional 2nd rep |
| A-3 **A1** dz 0.30 ×2 | **2/2** | 23 Sep `17-17-26`, `17-18-48` |
| A-4 **A1** dz 0.50 ×2 | **2/2** | 23 Sep `17-36-06`, `17-37-26` |
| A-5 **A1** dz 0.75 ×1 | **1/1** | 21 Sep `12-51-16` |
| B-1 **A3** 0.30 ×2 | **2/2** | 21 Sep |
| B-2 **A3** 0.50 ×2 | **2/2** | 21 Sep + 23 Sep B-2 reps |
| B-3 **A3** 0.40 ×1+ | **2/2** | 23 Sep (extra rep OK) |
| B-4 **A2** | **2/2 merged** | 23 Sep `19-21-05`, `19-27-03` (legacy circle profile) |
| C-1/C-2 **A4** | **Not flown** | Next lab priority |
| D-1…D-3 **C5** | **Done** | 23 Sep — use CLI **`--height`**, not `--z` |

Next lab: **A4 ×4** → optional **A1 @ 0.20** 2nd rep. Pull/rebuild
**crazyswarm2** for **A2 lab defaults** (`height 0.45`, `radius 0.40`, `rotate 90°`). See
`docs/next_flight_card.html`.

**Coverage grid (2026-09-21 desk) — ⚠️ SUPERSEDED by the "Progress" table above.** Kept verbatim
as a dated snapshot (do not edit its rows); every "Not flown"/"Unmergeable" cell below was
resolved on 23 Sep — see the current status in the table at the top of this file instead. Compare-downwash catalog:
[`24_Downwash_Compensation_Comparison.md`](24_Downwash_Compensation_Comparison.md) § Catalog —
2026-09-21. Figure:
`experiments/analysis/out/c1_2026-09-21/c1_coverage_2026-09-21.png`.

| Block | Cell (plan) | Reps needed | On disk 2026-09-21 | Status |
|---|---|---|---|---|
| A-2 | A1 dz 0.20 | 2 | `13-25-10` (1) | **Partial** — refly 2nd |
| A-3 | A1 dz 0.30 | 2 | `12-40-41`, `13-27-27` meta only | **Unmergeable** — ~20 cm / **15.6 cm** 3D RMS on best bins (`a1_z_diagnostic.json`) |
| A-4 | A1 dz 0.50 | 2 | `12-43-34`, `13-28-49` meta only | **Unmergeable** — ~18 / **35.7 cm** 3D RMS |
| A-5 | A1 dz 0.75 | 1 | `12-51-16` (1) | **Done** |
| A-1 | A7 dynamic | 3 | — | **Not flown** |
| B-1 | A3 dz 0.25 v 0.30 | 2 | `13-00-57`, `13-02-56` | **Done** |
| B-2 | A3 dz 0.25 v 0.50 | 2 | `13-04-34` (1); `13-05-58` no cf5 uSD | **Partial** |
| B-3 | A3 dz 0.40 v 0.40 | 1 | — | **Not flown** |
| B-4 | A2 same-path | 2 | — | **Not flown** |
| C-1/C-2 | A4 offset | 4 | — | **Not flown** |
| D-1…D-3 | C5 ground | 3+ | — | **Not flown** |
| — | A8 QA | — | 3 merges in manifest | **QA only** (not training) |

---

## The constraint that drives everything: the proximity gate

`model.neighbour_gate` replicates the reference's own cutoff literally:

```
|dx| < 0.2 m   AND   |dy| < 0.2 m   AND   |dvx| < 1.5 m/s      # dz is NOT tested
```

**A neighbour only becomes a training input when it is within ±20 cm laterally.** Vertical
separation is unbounded — `dz` is deliberately not gated. Two consequences:

- **Scenarios that keep the two vehicles vertically aligned are the high-yield ones.** Lateral
  offset beyond 20 cm gates the neighbour out entirely and the row degenerates to "no
  interaction", which is legitimate data but carries no downwash signal.
- ⚠️ **A8 is a poor collection scenario**, despite being an excellent *demonstration* one. With
  `span=1.0 m` the vehicles are laterally separated for most of the run and only pass through
  the gate briefly at each crossing. Keep flying A8 for the comparison table (docs/24); do not
  rely on it for training data.

**Second hard constraint:** only **cf5** (study / bottom drone) produces usable ego rows.
`cf_second` is pinned to stock Lee, has no RPM source, and its `a_res` is identically zero —
verified 2026-09-16 when `dataset.py` correctly excluded it as ego while still using it as the
neighbour. So **cf5** must be the **bottom** vehicle (the one in the wash) in every downwash
block. (`cf231_active` is bench-only since 2026-09-19.)

**What the model consumes** (`dataset.build` → `NeuralSwarm2`):
`rel` = peer − own, position *and* velocity, world frame (6 per neighbour) · `ground` =
`[0 − own_z, −own_vx, −own_vy, −own_vz]` · target `y` = measured `a_res_z`.
So the state space to cover is **dz**, **relative velocity**, **small lateral offset**, and
**own altitude** (for the ground term).

---

## Flight plan

Durations are scenario time; add ~25 s per flight for arm/ramp/land. Estimate ~6–8 flights per
battery pair.

### Block A — dz coverage (do first, highest value)

| # | Scenario | Params | Reps | Why |
|---|---|---|---|---|
| A-1 | **A7** dynamic merge | `--dz-start 1.10 --dz-end 0.10 --speed 0.30` | **3** | **The single most valuable run.** Sweeps `dz` continuously through the whole range *while staying laterally aligned*, so every sample is inside the gate. One flight covers what a dozen static holds would. |
| A-2 | **A1** static stack | `--dz 0.20 --hold 15` | 2 | Static reference at close range, zero relative velocity — isolates the pure `dz` dependence. |
| A-3 | **A1** static stack | `--dz 0.30 --hold 15` | 2 | " |
| A-4 | **A1** static stack | `--dz 0.50 --hold 15` | 2 | " |
| A-5 | **A1** static stack | `--dz 0.75 --hold 15` | 1 | Upper end — where `Fa` should be approaching zero. Negative examples matter. |

### Block B — relative velocity

| # | Scenario | Params | Reps | Why |
|---|---|---|---|---|
| B-1 | **A3** static-top | `--dz 0.25 --speed 0.30 --passes 4` | 2 | Bottom translates under a hovering top: sweeps `dvx` through the gate repeatedly at close range. |
| B-2 | **A3** static-top | `--dz 0.25 --speed 0.50 --passes 4` | 2 | Same geometry, faster — separates velocity dependence from position dependence. |
| B-3 | **A3** static-top | `--dz 0.40 --speed 0.40 --passes 4` | 1 | Mid `dz`, mid speed — fills the interior of the grid. |
| B-4 | **A2** same-path tracking | `--dz 0.30 --path circle --speed 0.4` | 2 | Both vehicles moving together: non-zero own-velocity with a *stationary* relative state. Distinguishes "neighbour moving" from "I am moving". |

### Block C — small lateral offset (inside the gate)

| # | Scenario | Params | Reps | Why |
|---|---|---|---|---|
| C-1 | **A4** offset stack | `--dz 0.30 --offset 0.10 --axis y --motion lemniscate` | 2 | `offset 0.10` sits inside the 0.2 m gate, so this teaches the lateral falloff *within* the gated region — otherwise the net only ever sees near-zero `dx/dy`. |
| C-2 | **A4** offset stack | `--dz 0.50 --offset 0.15 --axis x --motion lemniscate` | 2 | Other axis, larger offset, still gated in. Guards against an axis-aligned bias. |

### Block D — ground effect (`phi_G`)

| # | Scenario | Params | Reps | Why |
|---|---|---|---|---|
| D-1 | **C5** near-ground pass | `--z 0.10 --speed 0.25 --passes 2` | 2 | Single drone, no neighbour. `phi_G` is a *separate* input branch and needs its own data, or the ground term is trained on nothing. `z_floor` defaults to 0.0 precisely so these rows survive. |
| D-2 | **C5** near-ground pass | `--z 0.15 --speed 0.25 --passes 2` | 2 | " |
| D-3 | **C5** near-ground pass | `--z 0.25 --speed 0.25 --passes 2` | 1 | Upper end, where ground effect should be fading. |

### If time remains

| # | Scenario | Params | Why |
|---|---|---|---|
| E-1 | **A7** again | `--dz-start 1.10 --dz-end 0.10 --speed 0.50` | Faster merge — adds `dvz` coverage the 0.30 runs don't reach. |
| E-2 | **A1** | `--dz 0.15 --hold 15` | Closer than Block A goes. Strong signal, tighter margin — only with everything else banked. |
| E-3 | **A6** extreme stack | `--dz 0.10` + `--allow-extreme` | Strongest achievable signal. **Gated in the library for a reason** — last, deliberately, and only if the session has been clean throughout. |

---

## Minimum viable set

If the session is short, **Block A (10 flights) + B-1/B-2 (4) + D-1/D-2 (4) = 18 flights** is
enough to train a defensible first model: `dz` covered statically and dynamically, velocity
covered at two speeds, ground effect covered at two heights.

**Absolute floor if things go badly: A-1 ×3 and D-1 ×2.** A7 alone spans the entire `dz` range
inside the gate, and C5 is the only source of ground-term data.

---

## Per-flight checklist

1. `check_usd_deck.py` before flying — always, not only when troubleshooting.
2. Confirm the pre-takeoff EKF-vs-mocap gate prints a small `|err|` for both vehicles
   (added 2026-09-18, `crazyswarm2` `8296ad8`).
3. **cf5** must be the **bottom** vehicle and must be running **geometric**
   (`ctrl_mode: 0`) — the residual is a *measurement*, and collecting it under INDI would mean
   learning a residual the controller is simultaneously cancelling.
4. Pull the uSD card with `copy_usd_log.py` (verified sha256, wall-clock names), never `cp`.
   After flash with `usd.runTag`, confirm printed tag matches `meta.json` `usd_run_tag`.
5. Merge: **tagged logs** — `merge_usd_logs.py --run-tag … --archive-t1 … --archive-t2 …`
   (+ `--meta` for RMS quality printout). **Pre-tag archives** — `merge_usd_logs.py --meta
   --roles`; aborts above 15 cm RMS (pairing search). See `docs/28`, `docs/39`.

---

## After collection

```bash
cd flying_drone_stack/tools/residual
python3 train.py <merged_*.csv ...> -o weights/c1_geometric.npz
```

Pipeline verified end-to-end 2026-09-18 (`fold check: max |trained − exported| = 2.27e-07`), so
this is a one-liner, not a debugging session. Watch the printed `baseline (predict zero)`
reduction — if the trained model does not clearly beat predicting zero, the dataset lacks
gated samples, and the most likely cause is too much lateral separation (see the gate above).

---

## Does NA-INDI (`controller=8`) need the same thing? **No — and that is itself a finding.**

`naindi_hybrid.rs` carries Cobo-Briesewitz's **own real trained weights**, extracted from their
compiled `nn.c`, so it needs no data collection and no training pipeline. But its input vector
is **19-dimensional and own-state only** — rotation-matrix columns, EKF acceleration, velocity,
gyro, and the four motor PWM ratios. **There is no relative or neighbour state in it at all**,
and it was trained on their single-vehicle payload-swing experiment.

The consequence is structural, not a tuning matter: **controller=8 cannot represent downwash as
a function of relative geometry, because relative geometry is not among its inputs.** It can
only learn whatever residual correlates with the vehicle's own state. It is therefore a
faithful reproduction of *their* method, and a legitimate comparison point, but it is not an
interaction-force-aware controller in the sense Strategy 2 is — and the thesis should say so
plainly rather than presenting the two as like-for-like.

**Can their network be retrained on our downwash data?** Mechanically, yes — and the uSD
logging already carries everything it would need (`motor.m1-4` and `gyro.*` for the inputs,
`a_res` for the label), as `naindi_hybrid.rs`'s own module doc notes. But it is worth being
precise about what such a retrain could and could not learn, because the distinction matters
for the thesis:

- It **could** learn a mapping from own-state to residual — i.e. "when my own accelerometer,
  gyro and PWM look like this, a residual of about this size is usually present".
- It **could not** distinguish downwash from a neighbour 30 cm above from any other disturbance
  producing a similar own-state signature, because relative geometry is not in its input.
- Most importantly, it would be **correlative rather than anticipatory**. INDI already *measures*
  the residual from own state; a network predicting the residual from the same own state is
  largely approximating something already available. The value of Strategy 2's formulation is
  that it predicts from *relative geometry*, so it can act **before** the disturbance shows up
  in the vehicle's own measurements.

So a retrained own-state net is a legitimate experiment, not a useless one — but it does not
recover what neighbour-aware input provides. Worth stating as a limitation in Ch. 2/6 and, if
anything, as a motivation for Strategy 2's design.

> C.1 collection is geometric only (`ctrl_mode=0`, `rnn.en=0`). The hybrid appendix below is
> obsolete (2026-09-22).

### …so does a retrained downwash variant need a new controller slot? **No — it already exists.**

The natural next thought is a `controller=9`: an NA-INDI variant retrained from scratch on
downwash data, leaving `controller=7`/`controller=8` byte-untouched as faithful reproductions.
That instinct is right about keeping 7 and 8 pristine — but the variant itself needs no new slot.

`lib.rs` already computes **both** residuals and adds **both** into `f_d`:

```rust
.add(a_indi.scale(res_sign))   // measured, INDI
.add(a_nn.scale(res_sign));    // predicted, Neural-Swarm2 deep-sets net
```

with the intent stated outright in the source: *"the two are deliberately allowed to be on
together. Double-counting is a real risk and is exactly what strategy 4 exists to measure; it is
not prevented here, because preventing it would remove the comparison."*

So the interaction-aware hybrid is:

| Strategy | Configuration |
|---|---|
| **2** — Geometric + NN residual | `controller=6`, `ctrl_mode=0`, `rnn.en=1` |
| **4** — Hybrid: INDI *measured* + NN *predicted* | **`controller=6`, `ctrl_mode=3`, `rnn.en=1`** |
| 4 (reference reproduction) | `controller=8` — the authors' own-state net, untouched |
| 1 — Pure INDI | `controller=6, ctrl_mode=3` (ours) · `controller=7` (theirs, untouched) |

This is **architecturally better than a retrained `controller=8`** for the purpose, because the
Neural-Swarm2 network is permutation-invariant and consumes relative neighbour position and
velocity — the correct input space, which their own-state MLP structurally lacks.

**Practical consequences: no `controller=9`, no second training pipeline, no second collection
campaign.** The same C.1 dataset and the same C.2 training run produce weights serving *both*
Strategy 2 and Strategy 4 — they differ only by a runtime flag, which also makes them a genuinely
clean A/B on identical weights.
