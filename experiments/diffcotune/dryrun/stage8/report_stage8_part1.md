## 1. Paper settings (DiffCoTune, Krishna et al. 2025) and DiffTune facts

Source: `docs/papers/krishna2025diffcotune.pdf` (read only, text extracted page by page, PDF pages 1–8) and the summary `docs/papers/summaries/krishna2025diffcotune.md`. Page numbers are the PDF page numbers.

### 1.1 DiffCoTune (arXiv 2505.24068v1), verbatim or near-verbatim

| Setting | What the paper states | Page |
|---|---|---|
| Task cost | "c(x_t, x_t*) := ‖x_t − x_t*‖₂², where x_t* is the desired state at time t. For states that evolve in different geometric spaces (e.g., position in R³ and quaternions in S³), we compute an unweighted sum of the individual cumulative norm squared errors." J_task(θ, β) = (1/T) Σ c(x_t, u_t), eq. 5 | 3, 4 |
| System-identification cost | J_sysId = (1/T) Σ ‖x_t − x_t^sys‖², closed-loop states of simulator and target under the same controller, eq. 3 (same unweighted sum over state parts) | 3, 4 |
| Update strategy used | Split-Alternate: β first (θ fixed), then θ in the updated simulator, "a maximum of K/2 epochs for both the β and θ updates" (Alg. 3) | 4 |
| Optimiser | Adam [42] "for computing the gradient update in Alg. 2 and 3"; Optax [45]. **Learning rate: not stated** (the text only says "identical design choices (learning rate and maximum epochs, K) as the combined variant" in the ablation) | 4 |
| Epochs | "We take K = 100" (so K/2 = 50 per inner loop) | 4 |
| Stop rule | termination condition C := 1(\|J_k − J_{k−1}\| < 10⁻³) ∨ 1(J_k > J_{k−1} + 10⁻³), "to terminate under convergence or divergence due to unbounded updates"; units of J: those of the cost as defined (no normalisation stated) | 4 |
| Iterations on the target | "limit to a single rollout in the target system per iteration and maximum tuning iteration of L = 5"; Alg. 1 loops l = 0 … L (= 6 rollouts) | 3, 4 |
| Weights | Combined variant only: w_task = w_sysId = 1; "refrain from setup-specific tuning" | 4 |
| Sensor noise in the tuning | **Not mentioned.** The word "noise" appears only in the introduction (uncertainties from "unmodeled dynamics, noise") and in the quadruped hardware experiment (noisy velocity feedback filtered with a 5th-order Savitzky–Golay filter on the hardware) | 1, 5 |
| Trust region | **Not mentioned** | – |
| Projection | Biped MPC: θ = entries of diag(Q); "we add a clipping operator to our gradient update (like [27]) to project θ into a feasible set (Q ⪰ 0)" | 5 |
| Keep-best | Alg. 1: θ_best replaced when J_task(θ_best, sys) ≥ J_task(θ_{l+1}, sys) — needs the real cost of θ_{l+1} | 3 |
| Other | one gain set per task; no quadrotor example (Table I); JAX / MJX / Flax / Optax; no repeated seeds reported | 4 |

### 1.2 DiffTune (Cheng et al., IEEE T-RO 2024) — ANOTHER PAPER, not DiffCoTune (same group, quadrotor hardware)

Pages are those of `docs/papers/cheng2024difftune.pdf`. These are facts about DiffTune only; none is a DiffCoTune setting.

| Setting | DiffTune | Page |
|---|---|---|
| Update | projected gradient descent θ ← P_Θ(θ − α ∇θ L) (plain gradient descent in raw gain units, not Adam) | 5 |
| Hardware trust region | "learning rate α = 0.1 together with gradient clipping such that each update is within 10 % of the current parameters θ_j, i.e. θ_{j+1} ∈ [0.9 θ_j, 1.1 θ_j]" | 11 |
| Hardware loss | L = ‖p − p̄‖² + tr(I − R̄ᵀR)/2 (position + attitude), horizon 7 s | 11 |
| Hardware budget | 10 trials | 11 |
| Sensor noise in its simulations | zero-mean Gaussian noise added to the simulation (std position 0.1 m, velocity 0.1 m/s, rate 1e-3 rad/s per the summary) | 9 |
| Learning rates compared | 0.1 / 0.01 / 0.001 (Fig. 14: faster with larger α, oscillation at 0.1); simulation runs with α = 0.01 and up to 50 trials | 13, 14 |
| Claim | "learning rate as the sole hyperparameter" (p. 10, their statement) | 10 |

DiffTune's learning rates are for plain gradient descent on raw gains and do not transfer to our Adam on log-ratios.
