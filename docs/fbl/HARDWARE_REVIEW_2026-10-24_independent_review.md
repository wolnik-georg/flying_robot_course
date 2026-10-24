# Independent review of `HARDWARE_REVIEW_2026-10-24.md`

Scope: every claim checked against the bundle (`~/Desktop/learned_fbl_core`), the paper (`docs/papers/hsieh2026flatness.pdf`), the firmware tree (`~/Desktop/crazyflie-firmware`) and the 12 raw uSD logs. Nothing flown, flashed, committed or edited apart from this file and three new `experiments/analysis/review_fbl_*.py` scripts.

Confidence: **V** = verified (code read / exact reconstruction / rerun), **S** = strong evidence (data, quantitative), **H** = hypothesis, **U** = unknown / not published.

**Short version**
- The main conclusion holds, and the logs back it more strongly than the doc says: the slow part of the measured rate-tracking error, put through the Eq. 17/18 closed loop, reproduces the large A4 / A8 y errors with corr 0.94–0.99 in 9 of 10 flights (rms ratio 0.67–1.04; exception 14:54, corr 0.65) (§b1).
- Corrected:
  - "z best in A1" is wrong (S1 is better).
  - The yaw-drift attribution is only partly right: yaw has no feedback, but much of the drift comes from Eq. 10's own commanded z rate, not from the rate error.
  - The "steady torque bias" is not a fixed property of the drone: its sign changes between flights for roll and yaw.
  - The state does not update at 500 Hz (EKF steps about every 5.6 ms).
  - "Latency ~0" is wrong.
- 1:1 change list: §c. Main new fact: the stock CF21BL rate PID equals kr ≈ 103 1/s roll/pitch and ≈ 15 1/s yaw in FBL units, vs. 59.5 / 35 flown. The inner loop that 1:1 needs is about 1.7× stiffer than preset 1, and it has an integral and a D term.

---

## a. Claim check

| # | Claim (doc §) | Verdict | Evidence | Conf. |
|---|---|---|---|---|
| 1 | 12 complete logs, 3 trips: preset 1 2/5, `_kr` 1/3, `_kp` 0/4 (Overview) | confirmed | `fbl_hw_overview.py` rerun, csv byte-identical to committed | V |
| 2 | "z as good as or better than S0–S3 (best in A1 and A8)" (Overview) | **corrected** | A1: S1 2.1 cm < FBL 4.3 / 6.5 cm. A8: FBL `_kp` 2.5 vs S2 2.6 (tie). Best only in A4 0.125 (`_kp` 1.8). Correct wording: "z in the S1–S3 range; best in A4 0.125, tied in A8, worse than S1 in A1" | V |
| 3 | §2 table, all numbers (y/z rms, yaw mean / max, tilt 2–11°) | confirmed | rerun; e.g. A4 0.125 preset 1 y = mean(79.4, 80.3) | V |
| 4 | §1 gains identified; 14:44 = preset 1, 14:54 = `_kr` | confirmed | `fbl_replay_check.py` rerun: 14:44 kr 59.5 / kp 14.9; 14:54 kr 29.75 / kp 13.8. The kr fit is an identity check (τ = J kr werr exactly), so it settles the block. Position-gain fits are within 2–6 % of nominal (the scripts use M = 0.043, flown 0.0427: −0.7 %, negligible). Branch heads support it: `study_FBL` was moved to the `_kp` gains at 14:35 (crazyswarm2 a73b4fa), so `study_FBL` ≡ `study_FBL_kp` today | V |
| 5 | §5 yaw not held; S0 same day within ±2.3° | confirmed | csv | V |
| 6 | §5 a_cmd reconstruction from `rpyd` is exact; dI/dt = e | confirmed, stronger | z column of R_des = F_d/\|F_d\| independent of yaw (`mcolumns(xb,yb,zb)`, `controller_omar_indi.c` FBL block); `Rot.from_euler("ZYX",[y,p,r])` matches `quat2rpy`. **Independent proof:** thrust / (F_d·z_b) gives the controller mass 0.04270 kg in every sample of every flight (p5 = p95), with reconstruction error < 0.05 mN. I_check_rms 0.003–0.03 m s | V |
| 7 | commanded tilt 33–66°, a_cmd,xy 9–52 m/s² rms, actual tilt 1–11° | confirmed | `integral_windup.csv` rerun | V |
| 8 | thrust coupling 14–70 mN std, total 28–88, corr 0.4–0.75 | confirmed (17–70 / 28–88 / 0.37–0.76) | `review_fbl_thrust_coupling.py` | V size; H that it drives the A1 oscillation |
| 9 | kiI 3–13 m/s² at log start | confirmed (2.9–12.9) | same script | V |
| 10 | plan feedforward reaches setpoint; desired yaw 0 | confirmed | `crtp_commander_high_level.c:381-397`; logged rpyd_z within ±0.03° | V |
| 11 | §6 r1: Eq. 17/18, integral, α-β-γ (β 0.4, γ 0.1 per update) same as bundle | confirmed | `fbl_law.c:24-32` vs `fbl_control.py:239-245`, `acceleration_estimator.py:236-244`; `make test_law` rerun: ALL PASSED (6×400 steps). Differences: the first integral step is 0 (bundle: t − 0); the bundle's α is unused in both | V |
| 12 | r2: thrust on measured z_b, Eq. 9 rates, Eq. 10 | confirmed | `fbl_law.c:76,84,91` vs `fbl_control.py:48-60`; on hardware confirmed by the exact mass recovery (#6) | V |
| 13 | r3: bundle P-only kr 238/140; paper hw = thrust + rates to the onboard PID | confirmed | `fbl_control.py:145-149,63`; paper Sec. 5.2 ("configured to output thrust and angular velocity … rely on the high-frequency PID controller onboard") | V |
| 14 | r4: stock CF2 rate PID 250/500/2.5, yaw 120/16.7 | confirmed; **missing detail** | `platform_defaults_cf2.h:96-112`; i-limit 33.3 / 166.7; D on the measurement, not on the error (`utils/src/pid.c`, version-dependent). CF21BL: 200/400/2.5, yaw 120/16.7 (`platform_defaults_cf21bl.h:93-109`). In physical units see §c | V (theirs U) |
| 15 | r5: yaw rate integrated into a yaw setpoint, angle loop on it | confirmed + detail | `controller_pid.c` (modeVelocity yaw → `attitudeDesired.yaw += rate·dt` → yaw attitude PID 6/1/0.35 → rate PID). The yaw-rate command is **not** passed as rate feedforward; only the integrated setpoint is. `manual` packet: yaw always rate (`crtp_commander_generic.c` `manualDecoder`). rpyt: yaw rate by default, **sign inverted** (`crtp_commander_rpyt.c`, "legacy rate input is inverted"). Missing from the doc: the `cppmEmu` packet (type 3) also has a rate mode | V |
| 16 | r6: stock interfaces take thrust as uint16 | confirmed + detail | rpyt: <1000 → 0, clamp 60000, thrust lock until a 0 packet. With `CONFIG_ENABLE_THRUST_BAT_COMPENSATED` (Kconfig default y, CF2 and CF21BL) uint16 is **linear in force**: F_motor = u/65535·THRUST_MAX (`motors.c:181ff`), the same mapping our force-torque path uses (`power_distribution_quadrotor.c` ForceTorque). Their THRUST_MAX would be 0.18 N (upgrade kit). So the thrust interface can be made physically identical; their laptop-side N→uint16 map is U | V / U |
| 17 | r7: torque residual not compensated on paper hw; 0 in ours | confirmed | paper 5.2; `controller_omar_indi.c` `res_ang = 0`. Note: the bundle's ROM **includes** a torque term (`downwash_residual.py:302-324`), sim only | V |
| 18 | r8: paper hw ROM C_D 0.236 + 4-neuron force NN; ours ROM-z only, C_D 0.55 | confirmed + detail | paper 5.2. Bundle runtime ROM uses C_D **0.3** (`nominal_dynamics.py:13`; paper 5.1 also 0.3); sim truth 0.55 in code vs **0.7 in the paper** (5.1). Our table uses arm 0.05 / m 0.0427 instead of 0.046 / 0.03: **per-unit-C_D ROM is 15 % stronger** (z0 0.3–0.6). Magnitude at A1 dz 0.45, r0 0: ours 0.55 → 2.86 m/s²; authors 0.236 on their constants → 1.06 m/s² (**2.7×**) | V |
| 19 | r9: gains, "Deployed" in code; ours 0.25× | confirmed | `fbl_control.py:132`; `fbl_apply_preset` | V |
| 20 | r10: 100 Hz sim / 400 Hz radio / 500 Hz onboard | confirmed + correction | paper 5.1 / 5.2. **But the state does not update at 500 Hz:** in the logs `vx` changes in 40 % of 500 Hz rows (median 5.6 ms), `est_q` in 31 % (5.9 ms), mocap in 10 % (9.9 ms). EKF prediction is 100 Hz (`estimator_kalman.c:119`). The α-β-γ filter at 500 Hz therefore sees mostly zero innovations, so "bandwidth ×5" is not the full story | V |
| 21 | r11: their state = Vicon 120 Hz | confirmed; ours "500 Hz" corrected to ~100–180 Hz effective | as #20 | V |
| 22 | r12: our latency "onboard, ~0" | **corrected** | own state = mocap → PC → radio → EKF, the same chain type as theirs minus the command uplink; not ~0. Only controller → motors is onboard | V (sizes U) |
| 23 | r13: peer by radio ~100 Hz, velocity by differencing | confirmed (code) | `fbl_residual.c:94` (ms timestamps → ±10 % velocity quantisation at 10 ms; affects only the residual jerk). Rate not re-measured (peer fields not in the uSD set) | V code / U rate |
| 24 | r14: Omar < 0.15 m, latch 60°, w_max 20 rad/s, thrust clamp | confirmed | `controller_omar_indi.c` cfg init (z_on 0.15, z_off 0.10 hysteresis), lines 170, 201ff; non-finite → trip | V |
| 25 | r15: 36 g CF2.1 + upgrade, brushed vs CF21BL ~43 g | confirmed (flown mass 42.7 g) | paper 5.2; #6 | V |
| 26 | r16: test cases static/stacked 0.3/0.4 m, 0.4 m/s; A4 outside | confirmed | paper 5.2 | V |
| 27 | bundle note "yaw torque bias causes yaw drift" | confirmed | `fbl_control.py:228-231` | V |
| 28 | §7 yaw drift: P-only needs a steady yaw rate error; "rate error = τ/(J kr) in every flight [V]" | **partly corrected** | That equation is the P law itself, true by construction, so it is not evidence for a cause. Yaw budget (`review_fbl_yaw_budget.py`, gap-free steps, Euler kinematics): the gyro integral matches the estimated yaw change (kinematics consistent). The rate-error part dominates in A8 / A4 `_kp` (e.g. 15:01 +27 of +33°). In A1 (14:30, 14:57) Eq. 10's own commanded z rate drives yaw (−36°, +69°), opposite to the error part. Without yaw feedback, Eq. 10 holds y_b·h constant, not ψ, and with tilt oscillation and ψ ≠ ψ_d this commands yaw motion (second order in tilt). The root cause, **no yaw-angle feedback** (row 5), stays V. "Torque bias → rate error → yaw" is S only for the low-oscillation flights | V/S |
| 29 | §7 "cf5's steady body torque" (S0 −70…+56 yaw, pitch to −490 µN m) | numbers confirmed; **interpretation corrected** | S0 means (13 flights): roll −28…+387, yaw −70…+56 (**sign changes**), pitch −488…+98 (10/13 negative). FBL means vary sign the same way. So it is a slowly varying torque need (flight, battery, downwash in A1 / A4), not a fixed bias; only pitch is close to consistent. The mechanism (P-only cannot supply any slow torque without a rate error) does not depend on this | V |
| 30 | §7 lateral offset: "time correlation 0.55–0.88 [S]" | **not reproducible; superseded** | no script in the repo produces it (`lateral_acc_chain.png` / `yaw_mechanism.png` have no generator). Replacement `review_fbl_lateral_chain.py`: jerk deficit d = (T/m) R (−werr_y, werr_x, 0), put through e‴ + kv e″ + kp e′ + ki e = d from the logged initial state. y error, rate error low-passed at 0.3 Hz: corr 0.94–0.99, rms ratio 0.67–1.04 in 9 of 10 A4 / A8 flights (14:54: corr 0.65, ratio 2.4; full band corr 0.89); A1 not explained by the slow part (corr ≤ 0.2; there the error is the 1 Hz oscillation, full-band corr 0.56–0.94) | S |
| 31 | §7 integral wind-up = steady position error the jerk path does not remove | confirmed, mechanism exact | the jerk path integrates d(a_cmd)/dt, so a_cmd − a_real = const + ∫(rate-error jerk). ki·I absorbs it. See also b3 | V |
| 32 | §7 A1 oscillation from coupling | H (unchanged) | size V (#8); no closed-loop reproduction | H |
| 33 | §7 A4 0.125 trips: jet edge, roll torque | partly re-checked | mean roll torque −1.36 / −1.62 mN m, mean roll rate error −55 / −65 °/s in both trips; the y drift is again fully explained by the rate error (corr 0.99, #30). Cause vs effect still open; side offset 0.04–0.05 m not re-checked | S / H |
| 34 | torque-bias simulation | fair as a **mechanism** test, not as a stock-PID emulation | signs right (plant torque = τ − τ_hold, so the controller must supply +τ_hold); plant J / M equal the controller's. Limits: (1) rate "PI" uses FBL's kr as P (stock P ≈ 1.7× / 0.4× for roll-pitch / yaw, §c), no D, no i-limit; (2) yaw hold = J kr_z·6·Δψ, stiffer than stock (rate P ≈ 15 1/s); (3) starts in hover with FBL on: no handover, no takeoff integral (b3); (4) constant bias, while hardware varies per flight (#29); (5) 44 ms motor lag (see b5) | V (code read) |
| 35 | §8 b "thrust only approximately" | **corrected** | with battery compensation the uint16 path is force-linear on both platforms (#16), so the onboard thrust can be the same physical command. Only their laptop map is U | V |
| 36 | §8 a "stock gains for brushed CF2.1; ours CF21BL 200/400" | confirmed + physical comparison | §c1 | V |

---

## b. Missed items

1. **The lateral error is quantitatively the inner-loop jerk deficit** (#30). This is the strongest data point for the root cause and should replace the 0.55–0.88 line. It does not show where the slow torque comes from (#29), only that P-only turns it into position error. [S]
2. **Yaw:** two drivers, both possible only because yaw has no feedback: rate error, and Eq. 10's commanded z rate when ψ ≠ ψ_d under tilt oscillation. Mean body z rate ≠ mean yaw rate (14:57: ω_z 4.9 °/s vs ψ̇ 1.2 °/s, coning). [V]
3. **Switch-on mismatch is stored in the integral.** a_cmd − a_real is never driven to zero in xy (only the jerk path acts), so any mismatch at FBL switch-on (0.15 m, climbing, Omar's tilt) persists, and ki·I settles to cancel it. Part of the 3–13 m/s² at log start may be this handover offset, not torque. It cannot be split, because uSD starts after takeoff. The authors' structure has the same property; their start procedure is U. [V mechanism / H share]
4. **Physical size of the stock rate PID** (§c1): CF21BL kp 200 = 2.47e-3 N m/(rad/s) = kr 103 1/s in FBL units (J 2.395e-5). Yaw 120 = 4.8e-4 N m/(rad/s) = kr_z 15 1/s. Flown: 59.5 / 29.75 / 35. CF2 + upgrade kp 250 = 2.56e-3 N m/(rad/s), so the CF21BL default is the same torque gain within 4 %. The bundle's kr 238 on its J 1.43e-5 is 3.4e-3, 1.3× the CF2 stock. [V arithmetic; assumes battery compensation on]
5. **The tuning sim's claim "authors' gains unstable with 44 ms lag" now matters.** The 1:1 inner loop is ~1.7× stiffer than preset 1 in P, plus I and D. The stock BL PID flies the CF21BL at these gains every day, which hints that the 44 ms first-order lag is pessimistic or that the D term matters. Check this in simulation with the real stock rate PID before any flight. [H]
6. **Inertia is not settled:** FBL uses 2.395e-5 / 3.235e-5 (`common/vehicle.py`, OOT value), Omar uses 1.657e-5 / 2.926e-5 for the same drone (45 % apart in J_x). With P-only the physical gain is J·kr; with the stock path J does not enter. [V]
7. **Effective state rate ~100–180 Hz**, not 500 Hz (#20); the attitude used for the thrust projection is stepwise too. Close to the authors' 120 Hz Vicon. [V]
8. **Sign and unit traps in the stock path:** the rate PID uses −gyro.y, so pitch rate desired = −ω_y; rpyt inverts the yaw rate; deg/s; thrust uint16 = F/(4·THRUST_MAX)·65535; `control->yaw = −yaw`. [V]
9. **CrazyROS = crazyflie_ros (ROS1):** its `cmd_vel` is the legacy rpyt commander. That makes "rpyt with stabModeRoll/Pitch = RATE" the most likely interface, but this is not stated. [H]
10. `yaw_rate` fed to Eq. 10 is the HLC's body ω_z (`crtp_commander_high_level.c:384`), not ψ̇_ref; ≈ 0 for these plans. [V, negligible]
11. **C_D is not transferable:** constants differ (+15 %), and the identification method differs (theirs: Chee et al. 2024 sys-ID; ours: KNODE fit). Compare the predicted acceleration, not C_D (#18). [V]
12. uSD write gaps up to 0.24 s (0.1–1.4 s per flight); integrals over raw `diff(t)` are fine, but uniform-dt analyses must interpolate. [V]
13. Battery: vbat sags to 3.1–3.2 V in both tripped (14:44, 14:46) and clean (14:50) flights, so it does not separate them. [V]
14. Cosmetic: the `fbl_hw_overview.py` docstring still calls rpyd a "logging-side value". [V]

---

## c. The 1:1 change list (final)

**c1. Inner loop → the stock rate path (replaces Eq. 19).** Possible onboard.
- Where: `controller_omar_indi.c` `controllerOmarFbl()`, from `fbl_law_command` (l. 199) to `controlModeForceTorque` (l. 206). Keep `fbl_law_command` for thrust + w_des; ignore its torque.
- Init: call `attitudeControllerInit(1/ATTITUDE_RATE)` in `controllerOmarIndiInit` (it is otherwise called only by the PID / INDI controllers; it is guarded by `isInit`). At each FBL switch-on: `attitudeControllerResetAllPID(roll, pitch, yaw)` and yaw setpoint ψ_sp = current yaw (stock does this only at thrust = 0; for the in-air switch-on, continuity is the closest equivalent; the authors' start is U).
- Each 500 Hz tick (what `controller_pid.c` does in rate mode):
  - ψ_sp = capAngle(ψ_sp + deg(w_des_z)·dt)
  - `attitudeControllerCorrectAttitudePID(...)`: keep only the yaw output, then reset the roll/pitch attitude PIDs
  - `attitudeControllerCorrectRatePID(gyro.x, −gyro.y, gyro.z, deg(w_des_x), −deg(w_des_y), yawRateDesired)`
  - `attitudeControllerGetActuatorOutput(&roll, &pitch, &yaw)`; yaw = −yaw
  - `controlModeLegacy`, thrust = T/(4·THRUST_MAX)·65535
- Gains: CF21BL stock (`pid_rate.*` 200/400/2.5, yaw 120/16.7/0; `pid_attitude.yaw_*` 6/1/0.35; i-limits 33.3 / 166.7; rate LPF off). In torque units this equals the authors' CF2 + upgrade defaults within 4 % (b4). It cannot be identical in angular-acceleration terms (our J is larger, b6).
- Keep ours (study setup): w_max clamp, tilt latch, Omar below z_on. Old path behind a parameter, bit-identical.
- Before any flight: host test against `controller_pid.c` rate mode (same inputs → same int16 outputs), then sim with the real stock PID (b5).

**c2. Thrust.** Physically identical via c1 (force-linear uint16 with battery compensation). Their laptop map and battery handling: U.

**c3. Torque residual.** Stays 0; matches the paper's hardware.

**c4. Force residual.** ROM + 4-neuron force NN trained on our data, collected like theirs: static + stacked, 0.4 m, 0.4 m/s, about 28 s, under FBL without NN. Needs data + training (KNODE task of 10-22). C_D identified on our vehicle; a 1:1 parameter value does not exist (b11).

**c5. Gains.** The bundle's "Deployed" values (56/38/48, 45/21.6/3) are the only source. Whether they are the hardware gains is U. Re-check them in sim with c1 before use (b5).

**c6. α-β-γ.** Same constants. Optional and closer to a measurement-rate filter: update only when the EKF velocity changes (dt = time since the last change). Their filter rate (120 / 400 Hz) is U.

**c7. Not identical, document only:** laptop vs onboard, Vicon vs EKF, radio command latency, firmware version, vehicle (brushless, 42.7 g, J), test cases (A4 outside their regime), FBL from 0.15 m during takeoff.

---

## d. Open facts for the authors
- Firmware version / commit and Kconfig: platform CF2 + `CRAZYFLIE_THRUST_UPGRADE_KIT`? battery compensation on? `pid.c` with D on the measurement?
- Interface: crazyflie_ros `cmd_vel` (rpyt) with `flightmode.stabModeRoll/Pitch = 0`? or `manual` / `cppmEmu`? Which signs for the pitch and yaw rate? Was ω_z of Eq. 10 sent as the yaw rate? Controller = 1 (PID)? Any changed `pid_rate` / `pid_attitude` parameters?
- N → uint16 map (THRUST_MAX used?), battery handling.
- Vehicle values in the hardware controller (m 0.036 or 0.03? J?).
- Hardware gains; were the "Deployed" values flown?
- Velocity source and the rate of FBL and α-β-γ (120 or 400 Hz; repeated Vicon samples?), latency compensation.
- Start / stop: takeoff with FBL or with another controller, switch-on height, integral and estimator reset, anti-windup.
- C_D 0.236 identified with which ROM constants (arm 0.046, m 0.03)? NN weights / training data.
- Was the ROM torque term also off on hardware?
- Hardware RMSE: 3D (as defined in 5.1) or z?
- Sim truth C_D: 0.7 (paper) or 0.55 (code)?

---

## e. Assessment
The root-cause story holds: the authors' control law is ported faithfully (code + host tests + exact log reconstruction), but its P-only rate loop (Eq. 19) is not what they flew. On cf5 any slowly varying torque need then becomes a steady rate error. Through the jerk path, that rate error becomes a lateral position error and a growing xy integral, and it explains the large A4 / A8 y errors almost completely [S]. Yaw has no feedback in this structure, so it drifts [V]. The doc overstates two points. The torque is not a fixed cf5 bias (sign changes per flight), and the yaw drift is not only rate error. Neither point changes the remedy. The A1 1 Hz xyz oscillation and the A4 0.125 trips are not yet explained [H]. The 1:1 inner loop (c1) is about 1.7× stiffer than the flown preset, so it needs the sim check with the real stock PID before hardware.

---

## Reproduce
```
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/fbl_hw_overview.py --out <scratch>      # = committed csv
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/fbl_replay_check.py --out <scratch>
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/fbl_integral_windup.py --out <scratch>
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/review_fbl_thrust_coupling.py          # mass 0.0427, coupling, kiI at start
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/review_fbl_yaw_budget.py               # yaw budget (#28)
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/review_fbl_lateral_chain.py            # lateral chain (#30)
# C port, in a scratch copy with PYTHONPATH=experiments/fbl:experiments/fbl/stage3_port:
make test_law            # ALL PASSED (rates max err 1.1e-2 rad/s, worst/tol 0.23; torque worst/tol 0.69)
test_fbl_residual        # ALL PASSED, 3077 cases (existing vectors; regeneration needs the repo-relative path)
diff: experiments/fbl/stage3_port/c/fbl_law.c, fbl_residual.c == firmware tree copies
```
Not re-run: `torque_bias_sim.py` (code read only), §4 top-drone geometry, A4 side-offset 0.04–0.05 m.
