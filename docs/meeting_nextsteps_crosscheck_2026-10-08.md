# Cross-check: "Next steps" of the 2026-10-05 meeting doc vs. state 2026-10-08 evening
(The meeting doc itself is yours and untouched.)

| # | Meeting next step | Status | Evidence / what remains |
|---|---|---|---|
| 1 | Test NS2 100 Hz in lab — do crashes still happen? | **DONE** | 10-05: 100 Hz works, crashes were tracker pose swaps + battery. 10-08: 10 network-on flights (A8 ×4, A1 ×3 + earlier), no crash. `docs/68` |
| 2 | Decide which INDI variant | **OPEN (supervisor)** | Data ready: ours / Omar C + Iz / Omar Rust + Iz (A8: ours +3.4 cm, Rust +1.0, C +1.5…+2.2; A1: only without the integral, ours worst). `docs/69`, `docs/70`, `docs/56` |
| 3 | Fix constant z error of Omar C/Rust | **DONE** | opt-in integral, study value 1.5 for both; hardware A8 confirmed. `docs/65`, `66`, `69`, `70`, `study_parameter_decisions.md` |
| 4 | George's INDI oscillation cause | **OPEN (low expectation)** | Closed parts: A1 oscillation of the geometric cf5 is NOT NS2 (`docs/68`); ours at kr 2400 sits on a delay knife edge, 96 % saturated samples in 10-02 A1 (`docs/56`, `64`). Remaining lab items: bench command→thrust latency, logging patch, A1 package flights |
| 5 | Decide scenarios for the study | **DONE** | all prepared scenarios stay for 2 drones, A1 included (decision 10-08). Open detail: scenario × controller matrix and repeats |
| 6 | FBL email follow-up | **DONE, waiting** | professor followed up; Strategy 3 blocked on the authors' code |
| 7 | INDI comparison: fix z integral / z error | **DONE** | = #3 |
| 8 | Replay same inputs through all controllers | **DONE** | `docs/64`: Omar C ≡ Rust (1e-7 / 1e-9); ours differs; A1 invalid due to motor saturation |
| 9 | RPM filter in the control loop + validate on real flights | **PARTIAL** | filter exists since 09-29 (DShot users: ours, Omar Rust); extracted + unit-tested; sentinel fix flag; Rust flights with fix ON: no detectable effect (ratio 1.01). **Open: our INDI ON (and optionally OFF/ON pair) → `docs/lab_session_pack_rpm_filter.md`** |
| 10 | Prepare NS2 sim for next session | **DONE** | `docs/62`: validated for A8 dips (SIL −2.3 vs hardware −3.15 for `res_sign=-1`); not validated for A1 (SIL too pessimistic) |
