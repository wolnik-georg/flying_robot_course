# Analysis of the shared repo `IMRCLab/differentiable-quadcopter` (2026-10-28, corrected after the independent review)

Source: <https://github.com/IMRCLab/differentiable-quadcopter>, cloned on 2026-10-28 (HEAD `3650d09`, last commit 2024-10-24 "Merge pull request #3 … differentiable-lee-controller"). First version: read only. Corrected on the same day with the findings of [`tuning_pipeline_review_2026-10-28.md`](tuning_pipeline_review_2026-10-28.md) §1b; every changed statement was re-checked in the clone. The review itself ran `gradcheck` and a 5-epoch training run in a throwaway venv and loaded the shipped checkpoints; nothing was run for this correction. Companion to the 3-page note [`papers/summaries/vonprittwitz2024diffquad.md`](papers/summaries/vonprittwitz2024diffquad.md). Plan that uses it: [`tuning_pipeline_plan_2026-10-28.md`](tuning_pipeline_plan_2026-10-28.md).

**Conclusion first:** the repo is a **starting template, not a model of our S0**. It lacks the integrals, the ω̇_d term, 500 Hz, the 44 ms motor lag, the thrust map, our mass (42.7 g) and the downwash. The shipped checkpoints do not reproduce Table I of the abstract.

## 1. What is in it (2457 lines of Python)
| File | Content |
|---|---|
| `quadrotor_pytorch.py` | `QuadrotorAutograd`: Crazyflie 2.0 rigid body, dt 0.01 s, quaternion state (13), input = four rotor forces, mixer `B0` (arm 0.046 m, thrust-to-torque 0.006). Position, velocity and rates use explicit Euler; the quaternion uses the exponential map with the world-frame rate (`qintegrate`), which is correct. Attributes are `.m` and `.I`. Has a `gradcheck` in `__main__` (passes, but only at hover with zero force and rate). |
| `controller_pytorch.py` | `ControllerLee`: thrust from `m·(a_ref + g − kp·ep − kv·ev)·R·e3`, desired rotation from the force direction and yaw, rate reference from the jerk, torque `−kr·er − kw·ew + ω×Iω − I(ω̂ Rᵀ R_d ω_d)`. **No integral term.** Uses jerk, not snap. The code drops the −RᵀR_d ω̇_d term of the abstract's eq. 2b. The attitude error matches Lee and our firmware (`vee_so3` symmetrises, `0.5·vee_so3(...)` = eR). |
| `train_lee_controller.py` | Gains `kp, kv, kw, kr` (3 each, = 12) as `nn.Parameter` (mass and inertia are also Parameters, frozen, and get clamped too, harmless). Window dataset from spline setpoints, window doubling on plateau, per-element gradient clip, gain clamp ≥ 1e-8. |
| `evaluate_lee_controller.py`, `test_lee_controller.py`, `simulate_trajectories.py`, `trajectories.py`, `plot_figure8.py`, `visualize.py` | Evaluation on helix, circle, figure-8, random waypoints (`*.csv` spline coefficients); `results.yaml`. `test_lee_controller.py` has **no assertions** (it prints). |
| `train_system_id.py` | Fits mass and inertia to real Crazyflie logs (`data/`, `data/readme.txt`: "weight: 39.0g"). **Stale:** it reads `self.quad.mass` / `self.quad.J`, but the model class has `.m` / `.I`, so it does not run with the current model. |
| `quadrotor_jax.py`, `quadrotor_np.py`, `controller.py` | Alternative implementations (JAX, NumPy reference of the controller; `max_u` is used only in the NumPy one). |
| `checkpoints/` | Two trained gain sets (`lee_controller_best_run.pt`, `…_noisy.pt`). |

## 2. Recipe in the code vs. the abstract (differences matter for reproduction)
| Item | Abstract (vonprittwitz2024) | Code default (`train_lee_controller.py`) |
|---|---|---|
| Initial window | T0 = 3 | `--window-size 3` |
| Patience before doubling | P = 5; Alg. 1 resets the counter only after a doubling | `--num-epochs-plateau 50`; the counter resets on **any** improvement > 1e-4 |
| Maximum window | Tmax = 400 | capped at **350** |
| Gradient clip | ±10 % of the current value | `--clip-gradient-norm 1.0`, but it clamps the **gradient** to ±1.0·\|θ\| (l. 185), not the step. With SGD lr 1e-3 each step changes a gain by at most **0.1 %** (about 3 % per epoch with 31 batches; a 5-epoch run showed kw × 0.999³¹ per epoch). With Adam the clip hardly limits the step. |
| Optimizer | not stated | `SGD`, lr 1e-3 (Adam optional) |
| Loss | MSE, weights 1 | `--loss-fn MSE` default. The `--*-weight` arguments **do not affect the gradients**: l. 180 backpropagates the unweighted sum; the weights enter only the plateau test (l. 318). |
| Gain init | all 1 | `--gain-init ones` (also `random`, `pretuned` = kp 9, kv 7, kw 0.0013, kr 0.0055 = the abstract's hand-tuned row) |
| Epochs | 500 | `--epochs 500` (the README example uses 5000) |

Consequence: the abstract's numbers cannot be assumed to come from these defaults. **The shipped checkpoints do not reproduce Table I:** `lee_controller_best_run.pt` (epoch 4999, window 192, SGD lr 1e-3) holds kp ≈ 1.00 / 1.00 / 1.00, kv ≈ 1.00 / 1.01 / 1.00, kw 0.0016 / 0.0014 / 0.0014, kr 0.0082 / 0.0070 / 1e-8; Table I has Kp 2.56 / 2.48 / 1.01 and Kv 9.51 / 11.28 / 1.01. The Table I recipe cannot be recovered from the repo.

## 3. What the model does **not** contain (relevant for our hardware)
- No motor dynamics (we measured about 44 ms lag on the brushless drone), no command delay, no thrust saturation, no drag, no battery effect, no state estimator or latency; the controller sees the true state.
- Mass 0.034 kg and inertia of a Crazyflie 2.0; ours is 0.0427 kg brushless (`docs/study_parameter_decisions.md`).
- Windows start **on the reference in position and velocity** (the initial state is built from the first setpoint), but the attitude starts at identity with zero rate. Every window restarts from the reference, so drift never builds up beyond one window.
- The "noise" option perturbs only the position update (`pos += v·randn·dt`, under `no_grad`), not sensors; it is zero at hover.
- Two drones, downwash, integral action (our S0 has a z-position integral, clamped at ±2, and an attitude integral `KI`), INDI, learned residual, a different control law (FBL): none of this exists.

## 4. Euler artefact: the 100 Hz step shapes the tuned gains
With explicit Euler on ω̇ = −(kw/I)·ω a step is stable only if dt·kw/I < 2, i.e. kw < 0.0033 (roll/pitch) or 0.0059 (yaw) at dt = 0.01 s (analytic, strong evidence).
- The start kw = 1 is about 300× above that bound, so "an uninformed start diverges" is largely an integration artefact, not a property of the vehicle.
- The auto-tuned Kω (0.0026 / 0.0020) sits at 80 % / 60 % of the bound: it is shaped by the 100 Hz step. Our firmware runs at 500 Hz.
- Yaw gains collapse to the clamp (kr_z = 1e-8 in both checkpoints, Table I KR_z 1e-4) because the figure-8 never excites yaw. Such gains would leave yaw uncontrolled on hardware.

## 5. Quality notes
- Comments such as "I don't know" (docstring of `thrustCtrl`) and a README that ends in "TODO" mark it as research code. The comment "this line might be wrong" at `eta = force @ B0.T` is a **false alarm**: for row-vector batches `force @ B0ᵀ` equals `B0·force`.
- The CI workflow `.github/workflows/benchmark.yml` is manual (`workflow_dispatch`) and uploads `results/`, which training never writes.
- Bugs (run in the review): `--visualize-trajectory False` crashes at the end (`args.plot_errors` does not exist, l. 363); an unsupported `--gain-init` builds a `ValueError` without raising it (l. 276); `gradcheck` runs only at hover.
- Not verified: whether the dynamics match the real Crazyflie at 500 Hz (`train_system_id.py`, which no longer runs, is the check the authors apparently used).

## 6. What it is good for here
- A reference for the **window-doubling schedule** and the gradient safeguards (with the caveats in §2).
- Documentation of what the repo does: about one day of effort, not "match Table I" (plan WP1).
- Our own S0/S1 controller is `controller_omar_indi.c` (controller 9) with `indi 0` / `indi 3`; its tunable parameters are in the plan, §3. Building a differentiable S0 would mean re-implementing that controller, not adapting this repo.
