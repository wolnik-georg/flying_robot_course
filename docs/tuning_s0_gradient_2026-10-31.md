# Auto-tuning: gradient tuning of S0 through a differentiable copy (2026-10-31)

Paper methods applied: von Prittwitz 2024 (gradient descent through a simulated rollout, growing time windows) and DiffTune 2024 (projected update inside a feasible box). Scope: S0 only, option A (solo drone, no downwash, one gain set), plant results only, nothing flown. The C firmware is unchanged.

## 1. What was built (`experiments/tuning/diff/`, GPU)
| File | Content |
|---|---|
| `s0_torch.py` | S0 controller (`controller_omar_indi.c`, indi 0, NS2 off) and mixer (force/torque, idle floor 7000 counts, cap) in PyTorch, batched |
| `s0_plant_torch.py` | Plant of `plant_core.py` tree mode at 1 ms: command delay 4 ms, motor lag, quadratic thrust law with `fmax`, state delay, noise, ground contact, Omar body inertia (the S0 jobs use `J='omar'`); block-wise function for `torch.compile` |
| `traj.py` | Solo trajectories: circle, figure-8, move/landing, takeoff (speeds 0.1-0.5 m/s) |
| `train_s0.py`, `run_diff_s0.sh` | Window-doubling training (1/2/4 s), Adam on log gain factors, step <= 10 %, projection to the box of `run_job.S0_PARAMS`, CUDA graphs |
| `check_s0_equiv.py`, `check_plant_equiv.py` | Equivalence checks against the firmware bindings |
| `eval_diff_s0.py`, `rank_diff_s0.py` | Evaluation / ranking in the real firmware plant |

## 2. Equivalence with the firmware (noise off)
| Check | Result |
|---|---|
| Controller + mixer, 3 random sequences | thrust 1e-7 N, torque 3e-8 N m, motor force within one uint16 step |
| Closed loop, full 23 s episode (A8 / A1 / A4; lag 30-60 ms; state delay 0-60 ms) | max position difference 0.02-0.03 mm |

Gradients are finite and CPU = GPU. Speed: about 0.2-0.5 s per gradient step for 256-512 windows of 2-4 s (about 40x faster than eager PyTorch; first step 60-75 s for compile + graph capture).

## 3. Problems found and fixed on the way
| Problem | Fix |
|---|---|
| Compiled backward gave NaN | tilt statistic with `acos` fed a zero gradient; detached |
| History gather in the compiled block | gathered outside the block |
| CUDA graph out of memory / silent exit | 256 rows for the 4 s stage; `torch._dynamo.config.cache_size_limit = 64`. Note: `nvidia-smi` fails (driver/library mismatch) and PyTorch's out-of-memory path then asserts; avoid memory pressure |
| First results did not transfer | plant inertia was the FBL one; S0 jobs use Omar's (fixed) |
| Gains near the stability edge | mean loss let rare divergence pass; now 0.5 mean + 0.5 worst-25 % rows, tilt hinge above 20 deg, 30 % stress rows (lag 50-64 ms, state delay 40-60 ms) |

## 4. Result: 12 gradient candidates (`s0x`), ranked in the firmware plant
Selection: tuning cells (`TUNING_CELLS_V2`) x 9 full variants, solo. Only seeds 9, 6, 0 have no crash (score 2.36 / 2.87 / 5.86; baseline 2.19; the others 8.8-17.1 with 1-6 crashes).

Study cells, 3-D RMS cm (nominal / ffs 0.4 / md 40 / lag 45; full variants with crashes of 9):
| Gains | Solo | Solo full | With downwash nominal / full |
|---|---|---|---|
| Baseline | 2.2 / 2.5 / 2.3 / 2.2 | 2.4 (0) | 11.8 / 17.8 |
| gradient s9 | 2.4 / 2.7 / 2.5 / 2.4 | 2.6 (0) | 11.2 / 19.7 |
| gradient s6 | 1.5 / 1.6 / 1.6 / 3.0 | 2.8 (0) | 20.8 / 28.3 |
| gradient s0 | 1.8 / 1.9 / 5.5 / 1.9 | 5.8 (0) | 27.7 / 30.9 |
| CMA-ES pick A8 (`s0q`, tuned with downwash) | 0.6 / 1.2 / 0.9 / 0.6 | 1.1 (0) | 9.6 / 13.5 |

Gains: s6 {"Pxy": 6.5335, "Pz": 20.725, "Dxy": 8.165, "Dz": 11.529, "Iz": 0.19569, "KRxy": 0.016755, "KRz": 0.0025424, "Kwxy": 0.0019176, "Kwz": 0.0014682, "KIxy": 0.02}; s9 {"Pxy": 7.2132, "Pz": 7.2132, "Dxy": 4.1218, "Dz": 4.1218, "Iz": 1.4557, "KRxy": 0.0072132, "KRz": 0.0077636, "Kwxy": 0.001116, "Kwz": 0.0019409, "KIxy": 0.030914}.

## 5. Reading
- The pipeline works and is equivalent to the firmware, but the gradient result is modest (best about 30 % better than the baseline on nominal solo cells) and does not beat the black-box result, which optimises full episodes (takeoff, flight, landing) in the firmware. Gradient windows are at most 4 s; slow instabilities are only seen by the full-episode score.
- Gradient gains were not trained with downwash: they get worse with it (expected for option A).
- Earlier runs (`s0`, `s0w`, `s0v`) are kept for the record; they were trained on a narrower plant or the wrong inertia and crashed in the full variants.

## 6. Next
- Longer horizon (chain 4 s windows with a carried state) or accept as is for S0.
- FBL needs its own differentiable copy (reduced-order model, network, stock inner loop).
- Hardware confirmation not touched. Files: `out/diff_*_seed*.json`, `out/diff_*_eval.json`, `out/diff_s0x_rank.json` (logs and markers not committed).
