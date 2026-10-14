# Follow-up 5 — Last examiner pass (Chapters 1–5)

Read as an examiner who has not seen the project, on the revised chapters after Tasks 1–4. Before this pass started I
fixed three defects I found during it: a sign error in Ch5 (the lower vehicle "sags, closing the gap"; it widens the
vertical gap; now consistent with $e_{\mathrm{sep}}$), and two pieces of Ch1 prose that a Phase 4 marker had commented
out (`followup_diff_audit.md` §6b). The DECISION rows are not repeated here. **D-11** (which INDI controller Strategy 1
is) and **D-03** (gap) remain the two largest risks.

## The five weakest passages that remain

| # | Location | Why it is weak | Concrete fix (ready to paste; not applied, because each changes an argument) |
|---|---|---|---|
| W1 | Ch1 §1.1 L54 "Formation flight happens closer than that."; Ch2 §2.2 L172 "Formation flight operates in exactly that near field --- our observation, not theirs" | Bauersfeld et al.'s far field begins about 2.5 motor distances below the vehicle. For the Crazyflie 2.1 Brushless that is about 25 cm if the motor distance is the diagonal (10 cm), and about 18 cm if it is between adjacent motors (7 cm); the paper does not say which, so this is `[VERIFY]`. Most default separations of Table 5.1 (A1 0.50 m, A2/A3 0.30 m, A4 0.60 m, A5 0.50 m, A8 0.25 m) lie *beyond* that, so the sentence is false for most of the library. The argument for learning still holds, because the jet model gives the time-averaged flow in hover, not the force on a moving follower, but it rests on that reason, not on the near field. | Ch1: replace with `The tightest formations of interest are closer than that, and at every separation the model describes the time-averaged flow below a hovering vehicle rather than the force on a neighbour that moves through it.` Ch2 L172–174: replace `Formation flight operates in exactly that near field --- our observation, not theirs --- which is the practical argument for learning the residual from data rather than deriving it.` with `The tightest formations operate in that near field, and even beyond it the model gives the time-averaged magnitude of the flow in hover, not the instantaneous force on a moving follower --- our observation, not theirs --- which is the practical argument for learning the residual from data rather than deriving it.` |
| W2 | Ch5 §5.3.1 L177 "C1--C3 … configurations where no significant inter-vehicle interaction should exist"; Table 5.1 caption; Table 5.2 "Null scenarios C1--C3 bound it" | C2 (leader–follower line at 0.4 m/s) is described in the same table as probing "the wake behind a moving vehicle". A scenario designed to show an interaction cannot also define the measurement floor. A finding in C2 would be read as model error by §5.3.1 and as wake by Table 5.1. | Ch5 L177: replace `C1--C3, the coplanar controls in which the vehicles fly side by side at equal height,` with `C1 and C3, the coplanar controls in which the vehicles hover side by side at equal height,`, and add after the paragraph `C2, in which the vehicles move in line at equal height, is a control for downwash but not for interaction: it probes the wake behind a moving vehicle and is therefore not used to set the measurement floor.` In Table 5.2 and §5.8 replace `C1--C3` / `three null scenarios` with `C1 and C3` / `two null scenarios`. |
| W3 | Ch1 §1.5 RQ3 L192–193 "It is not a test of whether superposition holds: it does not, and that is already established" | This is stronger than the cited sources. Gielis et al. report a *linear regime well estimated by linear summation* next to a nonlinear one (Ch2 §2.4 now says "regime-dependent"), and Hsieh et al. (2025) cite the same data in support of pairwise summation. | Replace `it does not, and that is already established~\cite{…}` with `it does not hold in general, which is already established~\cite{shi2020neuralswarm,gielis2023aggregate}` (wording only; the RQ itself is unchanged). |
| W4 | Ch5 §5.7 L534–536 "battery discharge changes the thrust constant slightly within a flight, which appears as a slow drift in the measured residual; the effect is bounded by the null scenarios"; Table 5.2 row 3; Ch3 assumptions table "Battery discharge" | The thrust in $a_{\mathrm{model}}$ is reconstructed from *measured* rotor speeds (Ch3 eq. model-ares). Battery voltage changes how fast the rotors turn for a given command, not the thrust per rotor speed, so the stated mechanism is not obvious and no source is given. "Bounded by the null scenarios" is unexplained, because a within-flight drift is not what a null scenario measures. | Ch5 L534–536: replace with `And the thrust model is identified once; any slow change of the thrust per rotor speed within a flight, for example with motor temperature or battery state, would appear as a drift in the measured residual. The null scenarios show whether such a drift is present but do not remove it.` Adjust the Table 5.2 cell to `A slow drift of the thrust model within a flight is not compensated`. |
| W5 | Ch5 §5.5.4 prediction error $\epsilon_{\mathrm{pred}} = \operatorname{RMS}_W(\hat a_{\mathrm{res},z} - a_{\mathrm{res},z})$ | The two signals have different delays. The prediction is computed at about 100 Hz from broadcast positions with finite-difference velocities (Ch4 §4.4), while the measurement is filtered (Ch3 §3.3.1). A sample-by-sample difference therefore includes a lag term, which inflates $\epsilon_{\mathrm{pred}}$ most in the transient scenarios (A8, B3), exactly where RQ2 looks. | Add after the definition: `Both signals are taken as logged; because they carry different delays, $\epsilon_{\mathrm{pred}}$ is also reported after shifting the prediction by the lag that minimises it, and the lag is reported with it.` and extend the §5.5.4 `\todo` with `the lag treatment for $\epsilon_{\mathrm{pred}}$`. |

Runner-up (not in the five): Ch3 L494 "8 m/s² … comfortably above the interaction accelerations expected". This is
supported only indirectly, by the rig-based ≈3.4 m/s² of Kiran et al. quoted in Ch3 L158; add "(about 3.4 m/s² on the
rig of Kiran et al., Section~\ref{sec:model-dynamics})".

## Questions a student would still ask

### After Chapter 3
1. **"The residual is a 3-D vector, but the network predicts only its vertical component. What happens to the lateral
   residual in the offset scenarios (A4, A5), and where is it measured?"** Ch4 §4.4 says the horizontal residual is not
   compensated by Strategy 2; Ch3 does not say so. Fix, after the sentence on the scalar output in §3.4.3: `The
   horizontal components of the measured residual are logged but have no predicted counterpart; Strategy~2 leaves
   them to feedback.`
2. **"How are $m$ and $k_{t,k}$ obtained, and how would I recognise that they are wrong?"** Ch3 shows what a thrust-constant
   error looks like (−0.49 m/s²) but not where the constants come from. Ch5 says only "identified once and frozen".
   Fix (Ch3 §3.2, one sentence): `The mass is weighed and the thrust coefficients are identified per rotor before the
   campaign (Chapter~\ref{ch:setup}); a residual in the null scenarios is the check that they are right.` The author
   must confirm the identification method; it is not in the read code.
3. **"Why are the measured residual (filtered, 1 kHz) and the prediction (≈100 Hz, from broadcast positions) comparable
   sample by sample?"** See W5; the same fix answers it.

### After Chapter 4
1. **"What do the INDI gains mean physically?"** Units are given, but not the interpretation. The code documents
   $K_R^{\mathrm{INDI}} = \omega_n^2$ and $K_\omega^{\mathrm{INDI}} = 2\zeta\omega_n$ for the attitude error dynamics
   (`lib.rs:250`, comment above the INDI parameters: "KR [1/s²]: ωₙ = √KR. KW [1/s]: ζ = KW/(2·ωₙ)"). Fix after eq. (ctrl-alpharef): `With the increment
   achieving $\alpha_{\mathrm{ref}}$, the attitude error obeys approximately $\ddot e + K_\omega^{\mathrm{INDI}}\dot e +
   K_R^{\mathrm{INDI}} e = 0$, so $K_R^{\mathrm{INDI}} = \omega_n^2$ and $K_\omega^{\mathrm{INDI}} = 2\zeta\omega_n$ set the
   natural frequency and damping.` (Linearised, small-angle; label it as such.)
2. **"Table 4.1 says Strategy 2 needs no rotor speed at run time, but Chapter 5 measures the remaining residual under
   every strategy. Does a Strategy 2 flight need rotor speeds after all?"** Yes, for the *evaluation*, not for the
   control. Fix in the Table 4.1 footnote: append `The evaluation in Chapter~\ref{ch:setup} still records the measured
   residual, and therefore rotor speeds, on every flight.` This also belongs in the RQ4 cost account.
3. **"If Strategy 1 is 'the full configuration', why does Chapter 4 also describe position-only and attitude-only INDI,
   and could the attitude increment alone explain any Strategy 1 advantage?"** Ch4 calls them diagnostic, and Ch6 §6.5
   uses them. The student still lacks one sentence on *why* each loop could matter. Fix after L387: `The position
   increment acts on the residual force, the attitude increment on the residual moment and on the vehicle's own
   attitude-loop model error; the diagnostic configurations separate the two.` (This depends on D-11: under option (b)
   the diagnostic bits do not exist in the same form.)
