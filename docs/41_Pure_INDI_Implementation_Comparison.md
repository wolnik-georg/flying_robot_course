# 41 — Pure INDI: four implementations compared

**Status: investigation only, no decision made, nothing executed.** Triggered by the supervisor
sharing his own working repos (`~/Desktop/crazyflie-firmware-omar`,
`~/Desktop/crazyswarm2-omar`) as a source-of-truth reference, in the context of the still-open
6.3 Hz oscillation question ([[project_indi_oscillation_investigation]]). Purpose: understand what
actually differs between the four "pure INDI" implementations we have access to, before deciding
whether to (a) close specific gaps against Omar's version, (b) fly/reuse his controller as-is, or
(c) port it to Rust the way Briesewitz's was ported. **None of those three is chosen here.**

---

## 0. The four variants, at a glance

| # | Name | Language / where it lives | Architecture | Ever flown (this project) |
|---|---|---|---|---|
| **1** | **Stock Bitcraze INDI** | C, `controller_indi.c` + `position_controller_indi.c` in `~/Desktop/crazyflie-firmware` (`stabilizer.controller=3`) | Cascaded **rate-INDI** (Tal & Karaman style): incremental control on **actuator commands**, no explicit residual-force term, no RPM feedback at all | 1 flight, 19 Sep — unstable, gains untuned (S1b) |
| **2** | **Our INDI** | Rust, `firmware_app/src/lib.rs` (`stabilizer.controller=6`, `indi_gains.ctrl_mode=3`) | **Force/torque-residual INDI** on top of a Lee/SE(3) geometric controller: `a_res = a_meas − a_model` from RPM², fed back into the desired-acceleration vector | Flying (S0/S1), full INDI 2-drone A8 |
| **3** | **Briesewitz / NA-INDI plain INDI** | C, `controller_lee.c` in `~/Desktop/NA-INDI-firmware` — ported to Rust as `firmware_app/src/naindi.rs` (`stabilizer.controller=7`, `use_nn=0`) | Same family as #2 (residual INDI on a Lee controller), but **RPM-force based**, includes gyroscopic coupling term, uses **unfiltered gyro** for the moment residual | Sim-clean since 09-18; **never flown** (S1c) |
| **4** | **Omar's INDI** | C, `controller_lee.c` in `~/Desktop/crazyflie-firmware-omar` — config in `~/Desktop/crazyswarm2-omar` | **Structurally the same family as #3** — same `controller_lee.c` lineage, `self->indi` bit-flag switch (bit0=position, bit1=attitude) — but independently tuned/simplified, **no gyroscopic coupling term**, uses **filtered gyro** | Supervisor's own hardware, presumably flown by him; not flown on our airframe |

**Headline structural finding:** #1 (stock) is architecturally a *different species* of INDI from
#2/#3/#4 — it never separates a residual force/torque and never reads RPM. #2, #3, and #4 are all
descendants of the same `controller_lee.c` idea (geometric SE(3) + measured-vs-modelled
acceleration/torque residual), which is worth stating plainly in the thesis rather than treating
"INDI" as one algorithm with four gain sets.

---

## 1. Where each file lives (for anyone continuing this)

```
Stock (#1):        ~/Desktop/crazyflie-firmware/src/modules/src/controller/controller_indi.c
                    ~/Desktop/crazyflie-firmware/src/modules/src/controller/position_controller_indi.c
                    ~/Desktop/crazyflie-firmware/src/modules/interface/controller/controller_indi.h  (gains, OUR retuned copy)

Ours (#2):          flying_drone_stack/firmware_app/src/lib.rs

Briesewitz (#3):    ~/Desktop/NA-INDI-firmware/src/modules/src/controller/controller_lee.c
                    (our faithful port: flying_drone_stack/firmware_app/src/naindi.rs)

Omar (#4):          ~/Desktop/crazyflie-firmware-omar/src/modules/src/controller/controller_lee.c
                    ~/Desktop/crazyswarm2-omar/crazyflie/config/crazyflies.yaml   (controller=5, ctrlLee.indi param)
                    ~/Desktop/crazyflie-firmware-omar/configs/cf21blrpm_defconfig  (his brushless target, mass 42.7g)
```

**Confirmed by direct `diff`:** Omar's `controller_indi.c` / `position_controller_indi.c` (the
*stock* INDI files) are **byte-identical** to our own unmodified stock copy — he has not touched
stock INDI at all. His actual custom controller lives entirely in `controller_lee.c`, selected via
`controller: 5` (`ControllerTypeLee`) + a runtime `ctrlLee.indi` bit-flag, **not** via
`stabilizer.controller=3`. This is architecturally identical to how Briesewitz's `use_nn`/`indi`
switches work inside the same file, and to our own `ctrl_mode` switch inside `lib.rs` — three
independent groups converged on "one controller module, runtime-selectable INDI on/off" as the
pattern.

---

## 2. Gain comparison (outer position loop + inner attitude loop)

| Gain | Stock (#1) default | Ours (#2), active block | Briesewitz (#3) | Omar (#4) |
|---|---|---|---|---|
| Position P (`Kpos_P` / `KP`) | n/a (separate `K_xi_*`, not diffed here — different loop shape) | 28.0 (xy) / 30.0 (z) | 12.0 (all axes) | **7.0** (all axes) |
| Position D (`Kpos_D` / `KV`) | n/a | 6.0 (xy) / 14.0 (z) | 10.5 (all axes) | **4.0** (all axes) |
| Position I | n/a | 0.05, limit 2.0 | 2.0, limit 100 | **0.0, limit 2** (integral effectively off) |
| Attitude `KR` | n/a (rate-INDI has no explicit KR) | 0.010 (all axes) | 0.007 / 0.007 / 0.01 | 0.007 / 0.007 / **0.008** |
| Attitude `Komega`/`KW` | n/a | 0.00110 (xy) / 0.00138 (z) | 0.002 (all axes) | **0.00115 / 0.00115 / 0.002** |
| Attitude `KI` | n/a | (position-loop only; no separate attitude KI) | 0.01 (all axes) | 0.03 (all axes) |
| Mass | n/a | 0.027 kg (CF2.1+Flow) | hardcoded 0.034 kg | `CF_MASS` (platform define — **airframe-aware**, 42.7 g on his brushless config) |
| Rate-INDI `g1_p/q/r`, `g2` (#1 only) | 0.0066 / 0.0052 / 0.0015 / 0.0000435 | n/a (different architecture) | n/a | Omar's *stock* copy is untouched → same as column 2 |
| Rate-INDI filter cutoff (#1 only) | 8.0 Hz (stock) / **70.0 Hz (our retune)** | n/a | n/a | 8.0 Hz (his stock copy, unmodified) |

**Reading this table:** every implementation uses **different position and attitude gains**, and
none of the three residual-INDI variants (#2/#3/#4) agrees with either of the other two on mass
handling — we hardcode our real mass, Briesewitz hardcodes *their* airframe's mass (0.034 kg,
wrong for us), Omar reads the platform's compile-time mass constant (right idea, but his constant
is for *his* airframe, not ours). None of this alone explains the oscillation; it does mean "just
copy Omar's gains" is not a drop-in — his gains were tuned for his own vehicle and mass.

> **Update 2026-09-23, controller=9 only:** operator decision reversed the "his constant is for
> his airframe" framing above for this specific port — `CF_MASS` on `controller=9`'s build is
> now set to Omar's own brushless-established `42700` (42.7g, his `cf21blrpm_defconfig`), not
> this project's independently measured 41.0g. Rationale: use exactly the airframe constant set
> he established for brushless throughout, not a mix of his port + our own measurement —
> consistent with `MOTORRPM2FORCE`/`THRUST2TORQUE`/`ARM_LENGTH` already being his values
> verbatim. Position/attitude gains remain his untouched defaults either way (§8's "what
> remains" list). Re-verified after the change: numerical test still 6/6 to `d=0.00e+00`, SIL
> height-tracking still exact — see `omar_indi_reference_build_notes.md`.

---

## 3. Position-residual computation (`a_indi` / `a_res`) — algorithm-level diff

| Aspect | Ours (#2) | Briesewitz (#3) | Omar (#4) |
|---|---|---|---|
| Model term | `a_model` = Σ(kt·rpm²) rotated to world + gravity | `a_rpm` = same idea, rotated, gravity | `a_rpm` = same idea, uses `MOTORRPM2FORCE` (single scalar, not per-motor `kt`) |
| Measured term | `a_meas` = rotated body accel + gravity | `a_imu` = same | `a_imu` = same |
| Per-motor force source | `kt1..kt4` (individually calibrated) | `kappa_f[4]` (per-motor) **+ optional PWM-derived RPM estimate** (`indi & 4` bit) | single scalar `MOTORRPM2FORCE` — **not per-motor**, no PWM fallback |
| Outlier clamp | none by default (`res_clamp` param, off) | `vclampnorm(·, 10)` on both `a_rpm` and `a_imu` | **none** |
| Filter | optional Butterworth on both sides (`res_fc` param, off by default) | Butterworth **80 Hz** on both `a_rpm`/`a_imu` | Butterworth, single **30 Hz** cutoff shared by accel *and* torque filters |
| Residual definition | `a_res = a_meas − a_model` | `a_indi = a_imu_filtered − a_rpm_filtered` (= meas − model, **same direction as ours**) | `a_indi = a_rpm_filtered − a_imu_filtered` (= **model − meas, opposite direction**) |
| How it enters `f_d` | `f_d = a_d + a_indi·res_sign` — **default `res_sign=+1` → ADDS a_res** (the flight-proven-but-undecided default; see `[[project_controller_validation_2026-09-09]]`) | `F_d = a_d − a_indi − a_nn` → **SUBTRACTS** the meas-minus-model residual | `F_d = a_d + a_indi` → adds *his* `a_indi`, but since his `a_indi` is model−meas (opposite sign from ours/Briesewitz's), **this nets to the same physical direction as Briesewitz: `a_d − (meas−model)`** |

### ⚠️ The one finding worth flagging first: sign convention

Converting Omar's and Briesewitz's residual terms into **our own sign convention**
(`a_res = a_meas − a_model`, positive = "vehicle is accelerating less than the motors alone would
predict, e.g. under downwash"):

| Implementation | Net effect on desired acceleration |
|---|---|
| **Briesewitz / NA-INDI (#3)** | `a_d − a_res` (subtracts) |
| **Omar (#4)** | `a_d − a_res` (subtracts — same direction, reached via the opposite-sign `a_indi` definition above) |
| **Ours, currently flying default (`res_sign=+1`)** | `a_d + a_res` (**adds**) |
| **Ours, derivation-correct but never-flown (`res_sign=-1`)** | `a_d − a_res` (subtracts — agrees with both #3 and #4) |

**Two independent implementations we did not write (Briesewitz's and now Omar's) both subtract the
residual, agreeing with our own from-first-principles derivation (`m·a = f_thrust + f_res + m·g`
⟹ desired accel should carry `−a_res`) — and disagreeing with what we currently fly by default.**
This doesn't prove `res_sign=-1` fixes the 6.3 Hz shake (the 09-12 test was inconclusive — see
`[[project_indi_oscillation_investigation]]` — and a lagged-`Rd` dynamic-stability mechanism is a
separate question from the steady-state sign), but it is now a 3-way-independent convergence
result, which is a much stronger piece of evidence than it was before Omar's code existed to check
against. Worth stating explicitly to the supervisor.

---

## 4. Attitude-residual (moment INDI) — algorithm-level diff

| Aspect | Ours (#2) | Briesewitz (#3) | Omar (#4) |
|---|---|---|---|
| Torque model source | RPM² × per-motor kt, arm, t2t (our own calibrated mixer) | RPM² × `kappa_f`, hardcoded `t2t=0.006`, `arm=0.707·0.046` (their airframe) | RPM² × `kappa_f`/`MOTORRPM2FORCE`, uses **firmware's own `THRUST2TORQUE`/`ARM_LENGTH`** (platform-derived, more portable) |
| Angular-accel measurement | filtered gyro finite difference | **unfiltered gyro** (`gyroNoLpf`, added specifically to port this faithfully — see `firmware_app/host/LOCAL_MODIFICATIONS.md`) finite difference | **filtered gyro** (`sensors->gyro`, the normal LPF'd signal) finite difference |
| Gyroscopic coupling term (`ω × Jω`) | present (`omega_dot_filt` path includes standard rigid-body correction — verify against lib.rs if reused) | **explicitly included**: `tau_imu = J·α − ω×(J·ω)` | **absent** — `tau_gyro = J·α` only, no cross term |
| Residual clamp | none by default | `vclampnorm(·, 0.006)` | none |
| Filter cutoff (moment) | shared with position path (`fc_bw`) | **frequency-selective**: 40 Hz for roll/pitch torque, 10 Hz for yaw torque | single 30 Hz for all three axes |

**Reading this:** Briesewitz's version is the most physically complete (gyroscopic coupling term,
unfiltered gyro for a less-lagged derivative, frequency-selective filtering, outlier clamping).
Omar's is the most simplified — closer in spirit to a "minimum viable INDI" than to a faithful
Tal & Karaman-style implementation. That's not necessarily wrong (simpler can be more robust on
real hardware, and his presumably flies), but it means "port Omar's version" and "port Briesewitz's
version" are not interchangeable choices — they differ on a real physics term (the gyroscopic
coupling), not just tuning.

---

## 5. Filtering — cutoff frequencies side by side

| Signal | Ours (#2) | Briesewitz (#3) | Omar (#4) |
|---|---|---|---|
| Position residual (accel) | off by default (`res_fc=0`); when enabled, one shared cutoff | **80 Hz** | **30 Hz** |
| Moment residual, roll/pitch | shares `fc_bw` (currently 70 Hz on brushless) | **40 Hz** | 30 Hz (shared with accel) |
| Moment residual, yaw | shares `fc_bw` | **10 Hz** (deliberately lower — yaw is noisier/less critical) | 30 Hz (same as roll/pitch — no yaw-specific treatment) |
| Rate-INDI actuator filter (#1 only) | n/a | n/a | n/a (Omar doesn't use rate-INDI) |

Note our own oscillation sits at **6.3 Hz** (`[[project_indi_oscillation_investigation]]`) — well
below every cutoff in this table, including the tightest (10 Hz). None of the three
implementations' filtering alone would suppress a 6.3 Hz mode; this table rules filter-cutoff
mismatch *out* as a likely single cause, it doesn't point at one.

---

## 6. What we still don't know about Omar's setup

- **Whether `crazyswarm2-omar`'s yaml actually enables `ctrlLee.indi` in flight**, or just declares
  the parameter group — the snippet found (`controller: 5`, `ctrlLee.indi: 0`) shows it **disabled**
  in the committed config. Need to check whether he flips it live via cfclient / a launch script,
  or whether a different yaml/branch has it on. Don't assume "his pure INDI flies well" from this
  repo alone without confirming the flag is actually set to 1 or 3 on his hardware runs.
- **Which physical airframe** the flown config corresponds to — `cf21blrpm_defconfig` (brushless,
  42.7 g) vs `cf2rpm_defconfig` (standard, 36.4 g) — both exist; no in-repo evidence yet of which
  one he actually flies day to day.
- **No commit history available** — both `crazyflie-firmware-omar` and `crazyswarm2-omar` are
  plain directories, not git repos (`git status` fails in both), so there's no way to see when his
  gains were last touched or whether they're mid-tuning.
- **No thrust/actuator model documentation** the way we have `bench_actuator_2026-07-22...csv` for
  our own retune — unclear whether his `MOTORRPM2FORCE` constant and default `g1/g2` were bench-
  calibrated for his airframe or left at stock too.

---

## 7. Two roads — not decided here

Per your framing, this doc stops at understanding. The two options on the table for a follow-up
ticket:

1. **Close the gap** — take the differences in §3–§5 (sign convention, gyroscopic coupling term,
   filter cutoffs, mass handling) one at a time against our own INDI and see which one, if any,
   moves the 6.3 Hz shake. The sign convention (§3) is the cheapest to test — it's already a
   runtime param (`indi_gains.res_sign`), no reflash needed.
2. **Reuse or port Omar's controller** — either fly his `controller_lee.c` as-is (needs re-gaining
   for our mass/airframe, and his stock-INDI files are irrelevant since they're untouched), or port
   it to Rust the way `naindi.rs` was done for Briesewitz's version. Given §4's finding that Omar's
   version is *physically simpler* than Briesewitz's (no gyroscopic coupling term) and we already
   have a faithful Briesewitz port sitting unflown at `controller=7`, a natural first question for
   that road is: **is there anything in Omar's version NOT already covered by trying `controller=7`
   on hardware?** — worth answering before committing to a second port.

No execution taken on either path.

---

## 8. controller=9 — scaffolded, verified, sim-complete (2026-09-22/23)

> **Note on the number:** `docs/26_Controller9_NA_INDI_Retrained.md` once reserved
> `controller=9` for an unrelated, never-built concept (a retrained NA-INDI network). That plan
> is out of scope (`docs/34`, 2026-09-22 supervisor decision) and was never coded, so the number
> was free to reuse here — see `docs/26`'s own updated banner for the full disambiguation.

Per operator instruction, road 2 (§7) was executed for the **structural integration only** —
this was never a decision to pursue the controller, only to build and validate it to the point
every other controller in this comparison has reached. Full detail and diagnostic trail:
`firmware_app/host/omar_indi_reference_build_notes.md`. This section is the summary.

**Built, kept in C, not ported to Rust.** `controller_omar_indi.c`/`.h` are a literal copy of his
`controller_lee.c`/`.h` — only identifier names changed (to avoid linking against our own stock
`controller_lee.c`). Verified **byte-identical** to his source (`diff` after normalizing the
renamed identifiers back, exit 0) — no control-law line touched. Wired in as its own isolated
controller slot, `stabilizer.controller=9` (`ControllerTypeOot4`), same 3-file Kconfig pattern
as `controller=7`/`8`. Three small, non-behavioral backports were needed to compile at all
(`MOTORRPM2FORCE` + an unconditional `THRUST2TORQUE` in `platform_defaults_cf21bl.h`, `vadd5()`
in `math3d.h`, `setpoint_t.attitudeAcc` in `stabilizer_types.h` — all plain upstream constants,
none of them his customization, confirmed by cross-checking `NA-INDI-firmware`). Compiles clean
for the real brushless target (`make DRONE=bl`).

**Numerically verified**: 6/6 hand-picked test vectors match his own compiled build to
`d=0.00e+00` — bit-for-bit, not just within tolerance.

**Both dispatch paths exercised** — not only the explicit-self functions every test calls, but
also the real firmware's `CRAZYFLIE_FW`-gated wrapper (`controllerOutOfTree4*`), which is never
even compiled into the host build and had never actually been executed by anything before an
isolated standalone compile confirmed it threads through correctly.

**Sim-complete, both interaction models**: clean (zero NaN, zero divergence, correct height
tracking, no cross-vehicle contamination) across single-drone × 3 trajectory shapes, 2-drone and
3-drone with no interaction model (`backend: np`), and 2-drone and 3-drone under **real coupled
downwash** (`backend: neuralswarm`) — the C1 scenario (3 drones, coplanar) achieved *exact*
station-keeping (`[0,0,0]` realized geometry on all three pairs); A1 (2 drones, vertical stack)
showed perfect lateral alignment with a modest (~14cm) vertical offset from the requested
separation, plausibly real downwash coupling rather than a defect.

**Two real integration bugs found and fixed along the way, neither in the ported control law
itself**: (1) the plant-mass sync defaulted to this project's own mass/kt globals, which this
controller never reads — would have flown the plant ~4% off the controller's model; (2) the
plant's PWM→force inversion (`self.kt`/`self.thrust_max`) was never set for this controller at
all, silently substituting a generic wrong thrust curve and producing a reproducible steady-state
height deficit. Both root-caused via a standalone diagnostic harness before being accepted as
findings, the same method (simplified harness disagrees with the real SIL → bug is in the SIL,
not the port) that previously found the `state.acc` bug for controller=7/8.

**What remains, and cannot be closed without a real drone**: any actual flight (the entire point
of the C.0 gate every other controller here goes through — sim-clean is a precondition, never a
substitute), gain tuning for this project's real airframe (every run above used his default
gains, untouched), and independent bench verification of `MOTORRPM2FORCE` (this project's own
brushless `kt` was never bench-calibrated either, so there is nothing to cross-check his number
against — unlike `arm_length`/`THRUST2TORQUE`, which *are* independently confirmed identical, see
the table in §2). **Never flown, not queued to fly.** Nothing here changes the "no decision
made" status of §7 above — this section only establishes that everything achievable without
hardware has now been done, to the same standard `controller=7`/`8` were held to.
