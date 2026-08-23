# Literature Matrix – Interaction-Force Aware Multirotor Control

> ⚠️ **This table is a navigation aid, never a citation.** Two misattributions in this project came
> from treating our own summaries as sources. Entries marked ✅ were checked against the PDF on
> 2026-08-23; the rest are unverified. Quotable claims with their evidence:
> [`20_Verified_Claims.md`](20_Verified_Claims.md). Rules: [`17`](17_Source_Ledger_and_Citation_Discipline.md).

| Paper | Year | Residual / Force Model | Controller Type | How compensation is done | Formations & min. separation | Key Metrics | Onboard / Compute notes | Key Assumptions | Relevance to our thesis |
|-------|------|------------------------|-----------------|---------------------------|------------------------------|-------------|-------------------------|-----------------|------------------------|
| Neural-Swarm (Shi et al.) | 2020 | Permutation-invariant DNN | Nonlinear tracking | Learned residual as feed-forward | Trained on 2/3/4 CFs; **25 cm** vertical separation ✅verified | Height tracking error | Real-time on MCU | Relative states known | Foundational predictive architecture |
| Neural-Swarm2 (Shi, Hönig et al.) | 2022 | Heterogeneous Deep Sets + spectral normalisation ✅ | Nonlinear tracking + planning | Learned residual in planning & control | 16 robots, **24 cm** vertical ✅verified | Tracking error, density | Efficient | Relative states available | Core predictive reference |
| Aggregate Downwash (Gielis et al.) | 2023 | Graph-based learned aggregation | Force modelling (test rig, not free flight) ✅ | Non-linear force aggregation | Up to 4 vehicles ✅ | Force prediction accuracy | ⚠️unverified — "usable onboard" not supported | Homogeneous vehicles | Core force modelling |
| SO(2)-Equivariant Downwash (Smith et al.) | 2023 | SO(2)-equivariant NN | Residual force learning | Equivariant prediction | 2 vehicles, close proximity | Tracking improvement (36%/56%) | Lightweight | Rotational symmetry | Efficient residual model |
| Flatness-Preserving Residual (Hsieh et al.) | 2026 | Physics-informed residual (flatness-preserving) | FBL + residual | Feed-forward in FBL | 2-quadrotor stacked | Tracking vs NMPC, compute | Very lightweight | Flatness preserved | Core FBL + NN method |
| Neural-Augmented INDI (Cobo-Briesewitz et al.) | 2026 | NN approximates INDI residual | Hybrid | NN + sensor-based INDI | Single vehicle + payload | Tracking error | Real multirotor | Can remove RPM sensors | Core hybrid paper |
| Cascaded INDI (Smeur et al.) | 2018 | Sensor-based residual | Cascaded INDI | Incremental compensation | Single MAV + wind | Disturbance rejection | Very lightweight | Good acceleration estimates | Foundational reactive method |
| Tal & Karaman INDI | 2018/21 | INDI for accelerations | INDI + differential flatness | Incremental inversion | Aggressive single quadrotor | High-speed tracking error | Real-time | Motor speed helpful | Primary INDI implementation reference |
| ProxFly (Zhang et al.) | 2024/25 | Residual RL (no explicit force model) | Residual RL on cascaded controller | Residual thrust + body rates | Close proximity + docking | Position & attitude RMSE | Lightweight MLP | Domain randomisation | Best pure-learning stretch |
| L1 KNODE-DW MPC (Hsieh et al.) | 2025 | Neural ODE residual | Learning-based MPC + L1 adaptive | Residual inside MPC | 3-quadrotor tight formations | RMSE, max vertical error | Heavier (MPC) | Good residual model helps | Strong recent multi-robot baseline |
| RSS 2025 Downwash Characterisation (Kiran et al.) | 2025 | Experimental force/moment/PIV | Characterisation | — | Multi-quadrotor interactions | Forces, moments, flow | — | — | Best physical insight |
