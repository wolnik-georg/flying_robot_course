# DiffCoTune protocol change — decisions 2026-11-23

## Why
- **S1 round 2 (23 Nov):** sustained ~4.2 Hz attitude oscillation on A8/A4/CIRCLE/FIG8.
  - tilt 42–61°, 3–7 s above 25°;
  - the preview predicted 2.07 cm.
- **S0 and S2 round 5:** flipped on hardware after the preview passed (KR/Kw ≈ 15).
- **Common cause:** the tuning cost is position only, and the simulator fit ignores angular rate. Neither sees attitude misbehaviour.

## Decisions (user, 23 Nov)
| # | Change | Before | After | Source |
|---|---|---|---|---|
| 1 | Cost minimized, per scenario | 3-D position error (RMS) | position error + attitude error | DiffTune hardware loss: ‖p − p̄‖² + tr(I − R̄ᵀR)/2 |
| 2 | Simulator fit (J_sysId) states | position, velocity, attitude (`p, v, q`) | + angular rate (`p, v, q, w`) = all rigid-body states | DiffCoTune: closed-loop states |
| 3 | Fitted simulator parameters | thrust_gain, motor_lag, downwash_scale, plume_scale; rpm_delay fixed at 4.05 ms (decision 14 Nov) | same 4 for all; **rpm_delay fitted for S1/S3** (only INDI uses measured RPM) | DiffCoTune: fit the physical parameters that matter for each robot |
| 4 | Gain step per round | ±25 % | **±10 % per round, every round: θ_new ∈ [0.9 θ, 1.1 θ], linear as in DiffTune, not log-symmetric** (user, 23 Nov; protocol v2.1) | DiffTune hardware trust region θ_{j+1} ∈ [0.9 θ_j, 1.1 θ_j] |
| – | Mass, inertia, force cap, 4 ms command delay | fixed | **unchanged** | measured / decision 9 Nov |

## Consequences
- **All controllers (S0–S3, FBL) are retuned under the new protocol.** The rounds flown before 23 Nov 18:00 are the old protocol; they are kept as data and are not mixed with the new results.
- **Order (user preference):** S1 + S3 first, then S0 + S2, with FBL under the same changes.
- **Simulator parameters vs the papers:**
  - the 4 (+1) parameters are our own choice for the quadrotor; DiffCoTune has no quadrotor and DiffTune fits no simulator;
  - downwash and plume are needed because the thesis is about two drones.

## Session 23 Nov evening: protocol v2 at ±25 % (kept as data, negative result)
- **Flown (ours2, ±25 %):** S1 r1 (new best, 2.82 cm), S3 r1 (best on total cost, 3.65 cm), S0 r1 (marginal best), S2 r1 (8.22 cm, best).
- **S1 r2 gains** repeated the attitude P of the v1 set that oscillated, with lower rate P (KR/Kw 8.29 vs 7.9); the preview passed again. The flights showed a large oscillation (operator observation, logs pending).
- **Conclusion:** angular rate in the fit + attitude in the cost did not stop the tuner from reaching the unstable region within two ±25 % steps. Hence decision 4 (±10 %).
- **Desk steps:** S3 r1 finished (kept as data); the remaining running/queued v2 desk steps (S0 r1, S2 r1, ours2 regression) were stopped by user decision to free the PC for v2.1. All v2 files are kept under `lab/v2/` and `usd_merged/rounds_v2/`.

## Next session: protocol v2.1, full restart
- Method ours2 + ±10 % per round, new state root (e.g. `lab/v21/`) and log folder (`usd_merged/rounds_v21/`).
- Round 0: reuse the flown default-gain round 0 flights of all four controllers (same gains, same flight protocol; user, 23 Nov night) → desk steps → round 1 gains for all four, my gain check, then rounds 1–5 in the lab.
- Order: S1 + S3 first (user preference); S0 / S2 in free slots. Max 2 desk steps at once.

## Open (for the lab agent to report, not to decide)
- How the attitude term enters: the DiffTune form with weight 1 is the default. Any other weighting is a user decision.
- The real cost for keep-best uses the same position + attitude cost; the 3-D position RMS is still reported.
- The size of the gyro term in J_sysId (earlier `lab/gyro_check.py`: 1.5–17 rad²/s² on S0 flights). Report its share before any weighting.
