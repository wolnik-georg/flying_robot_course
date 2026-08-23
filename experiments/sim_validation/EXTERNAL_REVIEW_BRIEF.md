# External Review Brief — Formation Flight Simulation Validation

*Self-contained. Written to be handed to a reviewer with no access to the repository.*

---

## 1. Context

Master's thesis: **"Comparison of Control Strategies for Interaction-Force Aware Multirotor
Teams."** Two or three Crazyflie 2.1 brushless quadrotors fly in close formation; when one
vehicle is above another, its rotor downwash pushes the lower one down. The thesis compares
control strategies by how well they reject that interaction force.

The measured quantity is the **residual acceleration**

```
a_res = a_meas − a_model = f_res / m
```

where `a_meas` comes from the accelerometer and `a_model` is the thrust the controller
believes it commanded, reconstructed from measured rotor speed (`Σ kt_i · RPM_i²`). In close
formation `f_res` is dominated by downwash. Every residual-learning method in the plan
(Neural-Swarm2, aggregate downwash, flatness-preserving residual) trains on this signal.

**Nothing has flown.** All results below are simulation.

---

## 2. What was built and validated

A library of **16 formation scenarios**, each flown end to end in the Crazyswarm2 ROS
simulator under **two controllers**:

- **Geometric SE(3)** (Lee et al. 2010) — no disturbance-rejection mechanism
- **Full INDI** (Tal & Karaman 2020) — position + attitude incremental nonlinear dynamic
  inversion, which uses `a_res` to cancel unmodelled forces

Both are the *same Rust source that runs on the drone*, cross-compiled for the host and
linked into the firmware's SWIG bindings — not a reimplementation. The simulator uses the
**Neural-Swarm2 pretrained deep-sets downwash model** for the interaction force.

---

## 3. The 16 scenarios

Robot 0 is always the **lowest** of a vertical stack — the one receiving downwash.
Δz = vertical separation. ⚠ = safety-gated, requires an explicit `--allow-extreme` flag.

### Two robots (Priority A)

| ID | Scenario | Geometry | Purpose |
|----|----------|----------|---------|
| A1 | Vertical stack hover | Δz ∈ {0.25, 0.50, 0.75} m, Δx = Δy = 0, hover ≥ 10 s | Baseline. Nothing moves, so any displacement is interaction, not tracking error |
| A2 | Vertical stack tracking | Δz ∈ {0.20–0.50} m, both follow the *same* path (line at 0.3–0.5 m/s, or circle d = 1.5 m, T = 7.5 s) | Constant separation while both vehicles move |
| A3 | Static-top | Top hovers; bottom shuttles underneath at 0.4 m/s, Δz ∈ {0.20, 0.30, 0.40} m | Fixed wash source, lower vehicle sweeps through — captures entry/exit transients |
| A4 | Offset stack | Δz ∈ {0.50, 0.60, 0.80} m, lateral offset ∈ {0.10, 0.20} m, bottom flies a lemniscate | Partial overlap; lower vehicle repeatedly crosses the wash edge |
| A5 | Reverse-circle stack | Both on a d = 1.5 m circle, T = 7.5 s, **opposite rotation**, Δz = 0.50 m | Lateral offset sweeps 0 → 2r → 0 twice per lap at constant height and speed |
| A6 ⚠ | Extreme stack | Δz ∈ {0.08, 0.10} m, straight line only, 0.2 m/s | Strongest interaction, smallest error budget |
| A7 ⚠ | Dynamic merge | Δz 1.10 → 0.10 m while both translate, lateral alignment held ≈ 0 | One continuous sweep of the whole separation range — gives a model the gradient, not just levels |
| A8 | Vertical swap | Fixed heights, horizontal side exchange; at crossing Δz ∈ {0.20, 0.25} m and horizontal offset → 0 | Wash arrives as a transient with a known arrival time (Neural-Swarm geometry) |

### Three robots (Priority B)

| ID | Scenario | Geometry | Purpose |
|----|----------|----------|---------|
| B1 | I-stack | Pure vertical alignment; top–centre gap 0.20 m, centre–bottom ∈ {0.20, 0.30, 0.40} m, holding formation on a shared path | Bottom vehicle in the combined wash of **two** — tests whether pairwise superposition holds |
| B2 | V-stack | Top and centre laterally offset by r = 0.10 m; bottom centred beneath the offset pair | Asymmetric superposition, which B1 cannot produce |
| B3 | Vertical swap (3) | Fixed heights, neighbour Δz 0.22 m, cyclic shift along a row | Two staggered crossings at two different height differences in one flight |

### Coverage and control cases (Priority C)

| ID | Scenario | Robots | Purpose |
|----|----------|--------|---------|
| C1 | Side-by-side coplanar | 3 | **Control case** — same height, so near-zero interaction expected |
| C2 | Leader-follower line | 3 | Wake rather than wash |
| C3 | Equilateral triangle | 3 | Coplanar three-body control case |
| C4 ⚠ | Docking approach | 2 | Simultaneous lateral (1 m) and vertical (0.50 → 0.10 m) closure, ProxFly-style |
| C5 | Near-ground pass | 1 | Ground effect alone, so it can be separated from downwash |

---

## 4. How each run is verified

This matters for judging whether the results mean anything.

1. Each robot has a **role**, a **slot** (fixed offset from a formation anchor) and a
   **curve** (motion relative to that slot). Commanded relative geometry is therefore
   `(slot_i − slot_j) + (curve_i(t) − curve_j(t))`.
2. Curves are compiled to **piecewise degree-7 polynomials** matching position, velocity,
   acceleration and jerk at both ends of every segment — 8 conditions, 8 coefficients,
   exactly determined, C3 continuous. Uploaded through the stock Crazyswarm2
   `uploadTrajectory` path.
3. `check_spec()` verifies, offline, that each scenario produces the geometry its own
   documentation claims. **38 parameter combinations, all passing.**
4. A safety gate checks commanded speed, acceleration, inter-robot separation and geofence
   containment *before* upload, and refuses rather than warns.
5. The runner records the **exact simulation-clock instant** the trajectory starts. After the
   run, the recorded vehicle states are compared against the rebuilt commanded geometry over
   that exact window.
6. **Pass criteria:** mean |Δz error| < 50 mm, mean horizontal error < 80 mm, no divergence
   (automatic fail above 60° tilt), ≥ 90% of the trajectory recorded.

**The simulator is deterministic** — nine geometric cases reproduced to 0.1 mm across a
rebuild, so a re-run is a genuine regression test.

---

## 5. Results — 34 cases

| Scenario | N | Geometric | INDI |
|---|---|---|---|
| A1 stack hover, Δz 0.50 | 2 | PASS — 27.6 mm | PASS — **0.0 mm** |
| A1 stack hover, Δz 0.25 | 2 | PASS — 41.5 mm | PASS — **0.0 mm** |
| A2 stack tracking | 2 | PASS — 38.4 mm | PASS — **0.1 mm** |
| A3 static-top | 2 | PASS — 3.5 mm | PASS — **0.6 mm** |
| A4 offset stack | 2 | PASS — 2.0 mm | PASS — **0.2 mm** |
| A5 reverse-circle | 2 | PASS — 4.2 mm | PASS — **0.5 mm** |
| A6 extreme, Δz 0.10 | 2 | PASS — 3.2 mm | PASS — **0.4 mm** |
| A7 merge 1.10 → 0.10 | 2 | PASS — 25.5 mm | PASS — **0.3 mm** |
| A8 vertical swap | 2 | PASS — 2.1 mm | PASS — **0.3 mm** |
| C4 docking | 2 | PASS — 7.2 mm | PASS — **0.2 mm** |
| B1 I-stack | 3 | **EXPECTED — 58.7 mm** | PASS — **0.1 mm** |
| B2 V-stack | 3 | PASS — 42.6 mm | PASS — **0.1 mm** |
| B3 vertical swap (3) | 3 | PASS — 3.9 mm | PASS — **0.7 mm** |
| C1 side-by-side | 3 | PASS — 0.0 mm | PASS — 0.0 mm |
| C2 leader-follower | 3 | PASS — 0.0 mm | PASS — 0.0 mm |
| C3 triangle | 3 | PASS — 0.0 mm | PASS — 0.0 mm |
| C5 near-ground pass | 1 | PASS (no divergence) | PASS (no divergence) |

Values are mean |Δz error|. **33 pass, 1 expected, 0 defects.** Coverage 100% on every run;
largest tilt anywhere 6.4° (the commanded bank for A5's circle). C5 has no pair, so no
separation error is defined.

### Geometric sag scales with wash exposure

| Wash exposure | Geometric sag |
|---|---|
| None — coplanar (C1/C2/C3) | 0.0 mm |
| One source, 0.50 m above | 27.6 mm |
| One source, 0.25 m above | 41.5 mm |
| Two sources, laterally offset (B2) | 42.6 mm |
| Two sources, vertically aligned (B1) | 58.7 mm |

Monotonic in every direction it should be.

### The one non-pass

B1 under geometric misses the 50 mm tolerance at 58.7 mm. Recorded as **EXPECTED**, not
FAIL: geometric has no mechanism to reject downwash, so in a wash-dominated scenario its
tracking error *is* the disturbance.

`EXPECTED` must be earned — the audit requires (a) the case is listed with a written reason,
(b) it did not diverge and was fully recorded, and **(c) the same scenario under INDI is
within tolerance.** Condition (c) is the guard: a genuinely broken scenario would fail under
INDI too. B1/INDI is 0.1 mm.

---

## 6. Two bugs found in flight code

### 6.1 The residual was added instead of subtracted

From `m·a = f_thrust + f_res + m·g`, holding a desired acceleration requires thrust
`m·a_des − m·g_vec − f_res`, so the desired-acceleration vector must carry **−a_res**. The
code added it. Position INDI therefore **reinforced** every unmodelled force instead of
rejecting it.

Measured against a known 20 mN downward force (≈5% of vehicle weight):

| Controller | Before | After |
|---|---|---|
| Geometric (no residual feedback) | 10.2 mm sag | 10.2 mm |
| Position INDI | **20.3 mm — exactly 2.00×** | **0.0 mm — cancels** |

The same factor appeared across every formation scenario: 1.89× to 2.10×. The *measurement*
was always correct (−0.4871 m/s² against an exact −0.4878); only its application was
inverted.

**Why no flight caught it:** with a single drone `a_res ≈ 0`, and the sign of a zero term is
invisible. It only manifests once another vehicle's downwash exists — the exact regime the
thesis studies, and the one never flown.

### 6.2 The residual was dead under geometric control

Rotor speed was read only when the controller mode was non-zero, so `a_res` was forced to
exactly zero whenever the geometric controller was selected. The plan requires collecting
Geometric+NN training data *while flying geometric*; that path produced nothing.

**Both fixes are unflown.** The first changes a control term on real hardware.

---

## 7. What is NOT validated — please attack these

1. **This is simulation.** No hardware flight has occurred at any point in the project.
2. **The interaction model is borrowed** — Neural-Swarm2's pretrained network, fitted to
   their vehicle, not ours. INDI is cancelling a disturbance the simulator itself generates.
3. **The simulator is not the drone.** No EKF, no sensor noise, no radio latency. It does
   model first-order motor lag (44 ms).
4. **Gains differ between sim and hardware.** The simulator needs roughly twice the position
   damping (`kv_xy` 8.0 vs 5.0) for reasons not yet understood. Motor lag closed ~40% of the
   gap; the position integral term, rotor drag, inertia and airframe have each been tested
   and refuted as explanations.
5. **The flight volume is an estimate**, not a measurement: x −1…1 m, y −2…2 m, z 0.30…1.70 m.
   It is corroborated by an independent statement a month earlier, but never tape-measured.
6. **Hardware stability margin is thin** — the tuning record shows `kv_xy = 4` crashed 2 of 2
   flights and 5 is what flies. Formation flight adds downwash to that same loop.

### Known coverage gaps in the scenario set

- **Yaw is fixed at zero in every scenario.** A yawing vehicle reorients its wake; whether
  that matters at these separations is untested.
- **No vertical relative motion** except A7 and C4. A vehicle climbing *through* another's
  wash is a distinct case from translating through it.
- **Nothing above three robots.**
- **No sustained close-proximity flight** — every scenario is under 30 s.
- **No deliberate disturbance-rejection test** (e.g. a commanded step in separation while
  close).

---

## 8. Questions for the reviewer

Ordered by how much damage a wrong answer would do:

1. **Is the scenario set complete for a 2- and 3-robot downwash study?** The argument: A1–A8
   cover separation, overlap and transients for two vehicles; B1–B3 add superposition;
   C1/C3/C5 are the control cases without which a positive result is uninterpretable. Are the
   gaps in §7 the right ones to have left open?
2. **Does the `EXPECTED` verdict hide a real failure?** The guard is that the same scenario
   must pass under INDI. Is that strong enough?
3. **Are the tolerances defensible?** 50 mm vertical / 80 mm horizontal were chosen before
   the sag magnitudes were known. A1 at Δz 0.25 already consumes 41.5 mm of the 50 mm budget
   on downwash alone. Should the criterion be relative to a predicted sag instead?
4. **Is the sign fix correct?** Derived from `m·a = f_thrust + f_res + m·g` and confirmed
   against an injected force in simulation. Never flown.
5. **Does anything here justify a controller claim?** It should not — see §7. Is that
   caveat stated strongly enough, or is the 2–3 order-of-magnitude result likely to be
   misread?
6. **What would you fly first, and what would you measure to know the sign fix is right on
   hardware?**

---

## 9. Planned next steps (for comment)

Before the core experimental work:

1. Tape-measure the flight volume and set the real geofence
2. Resolve firmware binding provenance (~110 lines currently uncommitted in an
   upstream-tracking tree)
3. Hardware inventory — drones, decks, batteries, SD cards
4. **Single-robot ladder:** flash → figure-8 on the legacy path → figure-8 on the current
   path (this migration has never flown) → circle
5. Re-validate INDI hover after the sign fix — treat as a new configuration
6. First two-robot flights at **large separation only**: A1 at Δz 0.75 → 0.50, then A3
7. Confirm `a_res` is non-zero in flight (needs a rotor-speed source; zero means no data)
8. Freeze the gains before the comparison, so the study measures the methods and not tuning
   effort

Then: collect residual data → train the NN models → integrate the seven control methods →
run the systematic comparison.
