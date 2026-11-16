
Note on theta_best: in A and B the best flown set by real cost is theta_1 (round 1); in A + B it is theta_3, so θ_best and θ₃ are the same gain set there (identical rows).

## 7. Recommendation for the full run (for you to decide)
- **A + B is the only variant that passes criteria 1, 2 and 3** (real cost 10.16 / 8.53 / 6.86 cm after theta_0 19.80; no floor or tilt failure after theta_0; final evaluation tuning 5.60 cm and test 6.89 cm against default 15.29 / 9.04). It is also the only variant whose cost keeps falling with the rounds, and its beta error after the round-2 fit is the smallest of the three (about 9-17 %), though still not small.
- **A alone** is close (8.70 / 8.86 / 11.38 cm; criterion 1 fails by 1.7 cm against the 1 cm margin; criteria 2 and 3 pass). **B alone** does not help: it reproduces the Stage 3b blow-up in round 3 (97 cm, four flights failed); the limit is hit in 20-28 of 50 steps for several gains, so it only slows the step, it does not stop the drift of theta in a chaotic region without noise.
- Caveat: one run per variant, one set of flight seeds. A + B beating A by 1.7 cm in round 3 is within what a second run could change; I did not measure run-to-run spread. The cause of the Stage 3b blow-up in (d) and B is not isolated (not stated): noise in the tuning cost is what removed it here, which fits the reading that the noise-free model overfits the floor-touching, chaotic regime, but I did not test that directly.
- If a full run is wanted: A + B, and the budget is about 18-19 min per round on the CPU pool (about 19 min round 0, 18 min later), so 6 rounds are about 1 h 50 min. Whether 6 rounds are worth it is open: A + B was still improving at round 3 (6.86 cm), but the pass criteria were only set for 4 rounds.
- Open or not measured: the gradient check with noise failed its gate at both steps (default point, KRxy 1.68 % at h = 1e-6); the model with round-3 gains of (d) is chaotic in frsim itself, so cells like that cannot be matched; 8 workers x 1 row not run (memory); the speed numbers for GPU include compile in the first iteration.

## 8. Process notes
- First forward check (23:52, 12 Nov) briefly reached load 70 (torch threads); I stopped it by exact PIDs and set 2 torch threads in every worker. Load was above 22 for about a minute. All later starts logged load 3.3-6.8 one minute after start (`logs/chain.log`).
- `speed.py` once started its own pools in the spawned workers (no `__main__` guard, load 38); stopped by exact PIDs, guard added, failed log kept as `logs/speed_cpu_FAILED_noguard.log`.
- The chain's first load gate used a decimal comma in awk and never passed; fixed with `LC_ALL=C`, chain restarted. No result affected.
- Inductor crashes for compiled CPU batches with more than one row (`try_loop_split`); worked around by replacing that function with the identity in `s0_model._patch_inductor_loop_split` (loop structure only, J matches eager and GPU).
- Keep-best fix of `run_loop` confirmed: round 0 = 19.80 cm and theta_1 identical to Stage 3b.
- Noise seeds of variant A tuning: 5000+2l and 5001+2l (my choice; not the flight seeds 0-33 and not the evaluation seeds 1000-1004).
- Nothing outside `experiments/diffcotune/` and the two docs was changed; no commit.
