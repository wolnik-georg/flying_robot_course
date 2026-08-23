# Master Thesis Topic Suggestions
## Controls, Motion Planning & Autonomy for Crazyflie Drones

**Student**: [Your Name]  
**Focus**: Controls + Motion Planning with lightweight learning support  
**Hardware**: Crazyflie (brushless) + indoor motion capture system  
**Timeline**: 6-month Master's thesis  
**Date**: July 2026

---

## 1. Top 5 Recommended Ideas

These ideas were selected as the strongest balance of:
- Feasibility for a 6-month thesis on Crazyflie hardware
- Strong core in **controls and motion planning**
- Lightweight learning used only as a supporting tool
- Clear real-world problem with measurable impact
- Good novelty potential while building on existing work (including Cobo-Briesewitz et al. (2026) LINDI/NA-INDI)

| Rank | Topic | Drone Type | Core Focus | Specific Problem It Solves | Usefulness / Impact if Successful | Differentiation vs. Recent Work (e.g. LINDI/NA-INDI) |
|------|-------|------------|------------|----------------------------|-----------------------------------|-----------------------------------------------------|
| **1** | Hybrid INDI for Highly Aggressive Flight + Wind/Disturbances | Single | Controls (INDI) + Light Residual Learning | High-speed aggressive maneuvers lose accuracy in wind/gusts due to unmodeled effects; limits real-world deployment. | Faster/reliable inspection (wind turbines, bridges), search & rescue, cinematic shots outdoors; broader use of small agile drones in variable weather. | Targets **higher aggression + dynamic wind** with **lighter/online adaptation** (vs. offline NN on moderate indoor trajectories). Builds on [Tal & Karaman (2018)](https://arxiv.org/abs/1809.04048) aggressive INDI and [Cobo-Briesewitz et al. (2026)](https://arxiv.org/abs/2503.09441) LINDI/NA-INDI. |
| **2** | Iterative Learning for Minimum-Snap Trajectory Refinement | Single (or small multi) | Motion Planning + ILC | Classical min-snap trajectories have repeatable real-hardware errors that are not automatically corrected at the planning level. | Easier programming of reliable aggressive maneuvers for drone shows, film, and delivery; reduces manual tuning. | Focuses on **planner-level iterative improvement** (orthogonal to controller residual learning). Related to [Iterative Learning Control for Quadrotors](https://www.mathworks.com/help/slcontrol/ug/quadrotor-trajectory-tracking-using-ilc.html) and minimum-snap planning literature. |
| **3** | Lightweight Online Adaptive Compensation for Varying Payloads | Single | Controls (INDI) + Simple Online Adaptation | Unknown/shifting payloads during flight cause large tracking errors; fixed or offline models cannot adapt in real time. | Enables safe delivery/inspection drones with changing loads without manual re-tuning. | Emphasizes **light non-NN online adaptation** for **varying** payloads (vs. fixed slung payload with offline training). Related to recent adaptive payload estimation works (e.g. L1-adaptive and ESO-based methods, 2024–2026). |
| **4** | Lightweight Learned Correction for Small Multi-Drone Aggressive Planning | Small Multi (2–4) | Motion Planning + Light Learned Layer | Downwash/interference in confined aggressive multi-drone flight leads to instability or overly conservative paths. | Reliable coordinated aggressive flight in small spaces (e.g., shows, inspection swarms). | **Planning-level** light correction + safety margins (vs. full controller learning in close-proximity works). Builds on [Preiss et al. (2017)](https://arxiv.org/abs/1704.04852) downwash-aware planning and [Shi et al. (2020) Neural-Swarm](https://arxiv.org/abs/2003.02992). |
| **5** | Degradation-Aware Replanning for Small Multi-Drone Teams | Small Multi (2–4) | Motion Planning + Light Adaptation | Actuator degradation causes mission failure; few systems handle graceful real-time re-planning. | More reliable swarms for long missions; safer operation with partial failures. | Focuses on **fast detection + trajectory replanning** (distinct from single-drone controller FTC). Related to recent adaptive fault-tolerant control (FTC) papers for multi-rotor UAVs (e.g. observer-based and sliding-mode FTC, 2024–2026). |

---

## Concise Motivations for Top 5 Ideas

### Rank 1: Hybrid INDI for Highly Aggressive Flight + Wind/Disturbances
**Problem**: Even advanced INDI controllers (and recent learned versions like LINDI/NA-INDI) lose accuracy during very high-speed aggressive maneuvers when facing real wind and disturbances.

**Solution & Novelty**: Hybrid classical INDI + lightweight data-driven residual learning, optimized for higher aggression and dynamic wind with fast-adapting models suitable for Crazyflie.

**Usefulness if Successful**: Enables reliable aggressive flight in real outdoor conditions for inspection, search & rescue, and cinematic applications.

---

### Rank 2: Iterative Learning for Minimum-Snap Trajectory Refinement
**Problem**: Classical minimum-snap trajectories often have repeatable errors on real hardware that are never automatically corrected at the planning level.

**Solution & Novelty**: Use iterative learning (ILC-style) to refine the trajectory generator itself based on real flight data, rather than only improving tracking of a fixed trajectory.

**Usefulness if Successful**: Much easier and faster development of reliable aggressive maneuvers with less manual tuning.

---

### Rank 3: Lightweight Online Adaptive Compensation for Varying Payloads
**Problem**: Unknown or changing payloads during flight cause large tracking errors; most existing methods use offline models or heavy neural networks.

**Solution & Novelty**: Lightweight online adaptation (non-NN where possible) tightly integrated with INDI for real-time compensation of varying payloads.

**Usefulness if Successful**: Enables practical delivery and inspection drones that can handle different or shifting loads without stopping for re-tuning.

---

### Rank 4: Lightweight Learned Correction for Small Multi-Drone Aggressive Planning
**Problem**: Downwash and aerodynamic interference in confined aggressive multi-drone flight causes instability or overly conservative paths.

**Solution & Novelty**: Lightweight learned correction layer on top of classical minimum-snap planning with explicit safety margins (planning-level, not full controller learning).

**Usefulness if Successful**: Reliable coordinated aggressive flight for small teams in motion-capture rooms or similar confined spaces.

---

### Rank 5: Degradation-Aware Replanning for Small Multi-Drone Teams
**Problem**: Actuator degradation (common in real flights) often leads to mission failure because current systems lack fast, graceful real-time replanning.

**Solution & Novelty**: Combine fast degradation detection with lightweight model adaptation and online trajectory replanning for small teams.

**Usefulness if Successful**: Significantly more reliable long-duration swarm missions and safer operation when partial failures occur.

---

## 2. Autonomous Drone Ideas

These three ideas extend the discussion into **autonomous drones** while staying consistent with the constraints (Crazyflie hardware, controls + motion planning focus, lightweight learning, 6-month feasibility).

| # | Idea | Drone Type | Core Focus | Specific Problem Solved | Usefulness / Impact if Successful | Related Recent / Relevant Papers |
|---|------|------------|------------|-------------------------|-----------------------------------|----------------------------------|
| **A** | Vision-based Autonomous Obstacle Avoidance & Reactive Trajectory Replanning for Aggressive Indoor Flight | Single (Crazyflie) | Controls + Motion Planning + Lightweight Vision/Learning | Heavy reliance on mocap; inability to autonomously react to obstacles in cluttered/unknown indoor spaces during aggressive flight. | Enables true autonomous aggressive flight in real cluttered environments (inspection, disaster response) with reduced external infrastructure dependence. | [arXiv:2505.04972 (2025)](https://arxiv.org/abs/2505.04972) "AI and Vision based Autonomous Navigation of Nano-Drones"; EGO-Planner (Zhou et al., 2020); [Landry et al. (2015)](https://arxiv.org/abs/1503.06267) "Aggressive Quadrotor Flight through Cluttered Environments" |
| **B** | Autonomous Adaptive Inspection Path Planning & Execution in Semi-Structured Indoor Spaces | Single | Motion Planning + Light Learning for Adaptation | Inefficient or unsafe fixed inspection paths in partially unknown or changing indoor environments. | Faster, more complete, and safer autonomous inspection missions with minimal human supervision. | Recent vision-based autonomous navigation in constrained environments (e.g. [arXiv:2505.04972](https://arxiv.org/abs/2505.04972), 2025); adaptive inspection planning literature. |
| **C** | Lightweight Learning for Autonomous Safe Recovery & Landing under Disturbances/Failures | Single | Controls + Light Learning + High-level Autonomy | Lack of autonomous decision-making when disturbances or partial failures occur, leading to crashes or mission abort. | Higher reliability and safety for unsupervised autonomous operations; reduces drone loss and enables longer missions. | Recent adaptive fault-tolerant control (FTC) papers for quadrotors (e.g. observer-based and sliding-mode FTC, 2024–2026); Bitcraze reports on persistent autonomous Crazyflie operation (2025) |

---

## Concise Motivations for Autonomous Ideas

### Autonomous Idea A: Vision-based Autonomous Obstacle Avoidance & Reactive Replanning
**Problem**: Strong dependence on external mocap prevents true autonomy in cluttered or partially unknown indoor environments during aggressive flight.

**Solution & Novelty**: Lightweight onboard vision + reactive replanning layer on top of classical control, designed to run feasibly on Crazyflie-class hardware.

**Usefulness if Successful**: True autonomous aggressive flight in cluttered indoor spaces with much less reliance on external positioning systems.

---

### Autonomous Idea B: Autonomous Adaptive Inspection Path Planning & Execution
**Problem**: Fixed or manually designed inspection paths are inefficient and potentially unsafe in partially unknown or changing indoor environments.

**Solution & Novelty**: Combine classical motion planning with lightweight learning that adapts paths based on onboard sensing and previous flight data.

**Usefulness if Successful**: Faster, safer, and more complete autonomous inspection missions with minimal human intervention.

---

### Autonomous Idea C: Lightweight Learning for Autonomous Safe Recovery & Landing
**Problem**: When disturbances or partial actuator issues occur, drones often crash or require manual intervention because they lack autonomous recovery decisions.

**Solution & Novelty**: Add a lightweight high-level decision layer (learning-assisted) on top of robust low-level control to autonomously trigger safe recovery or landing.

**Usefulness if Successful**: Much higher reliability and safety for unsupervised autonomous operations; reduces drone loss rate.

---

## 3. Drone Racing Ideas

Drone racing in indoor rooms is a specialized and extreme form of aggressive flight. It emphasizes time-optimality, precise gate passing, and robustness under high-speed conditions. The two ideas below are designed to fit the Crazyflie + mocap setup while maintaining a strong controls + motion planning core with light learning support.

| # | Idea | Drone Type | Core Focus | Specific Problem Solved | Usefulness / Impact if Successful | Key Related Work / References |
|---|------|------------|------------|-------------------------|-----------------------------------|---------------------------------|
| **R1** | Time-Optimal Trajectory Generation and Tracking for Indoor Drone Racing through Gates | Single (Crazyflie) | Motion Planning (minimum-time optimization) + Light Learning Refinement | Classical minimum-snap trajectories are smooth but not time-optimal. Racing requires flying as fast as possible through gates while remaining accurately trackable on real hardware. | Faster lap times and more reliable gate passing in indoor racing setups. Directly applicable to drone racing competitions and agile flight research. | Minimum-time quadrotor trajectory optimization literature; aggressive flight through gates/cluttered environments ([Landry et al. 2015](https://arxiv.org/abs/1503.06267) and extensions). |
| **R2** | Robust Gate-Passing Control and Reactive Replanning for Indoor Drone Racing under Disturbances | Single (Crazyflie) | Controls (e.g. INDI/hybrid) + Motion Planning + Light Learning | Even good trajectories fail at high speeds when disturbances (wind, ground effect, model mismatch) occur. Pure tracking often leads to missed gates or crashes. | Significantly more reliable high-speed racing in real conditions. Enables robust autonomous racing behavior. | Reactive planning and robust control papers for agile quadrotors; disturbance rejection and reactive replanning in high-speed/aggressive flight. |

---

## 4. Concise Motivations

### Racing Idea R1: Time-Optimal Trajectory Generation and Tracking for Indoor Drone Racing
**Problem**: Classical minimum-snap trajectories are not time-optimal. In drone racing, the goal is to complete laps as fast as possible while accurately passing through gates on real hardware.

**Solution & Novelty**: Develop minimum-time trajectory generation for gate sequences combined with a lightweight learning component that refines trajectories or tracking performance from real laps.

**Usefulness if Successful**: Faster and more reliable indoor drone racing performance. Directly relevant to drone racing research and competitions.

---

### Racing Idea R2: Robust Gate-Passing Control and Reactive Replanning for Indoor Drone Racing under Disturbances
**Problem**: At high racing speeds, even small disturbances (wind, ground effect, model mismatch) cause missed gates or crashes because pure trajectory tracking lacks robustness and reactivity.

**Solution & Novelty**: Robust low-level control (INDI/hybrid) combined with a light reactive replanning module that adjusts trajectories when deviation is detected, enabling reliable gate passing under real conditions.

**Usefulness if Successful**: Much more reliable high-speed racing behavior in realistic indoor environments with disturbances.
- Enables faster, more reliable aggressive flight for real applications such as infrastructure inspection (wind turbines, bridges), search & rescue, and cinematic shots.
- Reduces mission time and improves safety in non-ideal weather.
- Demonstrates practical hybrid classical + learning control with strong real-hardware validation.

---

## 4. Concise Motivations for All 8 Ideas

### Rank 1: Hybrid INDI for Highly Aggressive Flight + Wind/Disturbances
**Problem**: Even advanced INDI controllers (and recent learned versions like LINDI/NA-INDI) lose accuracy during very high-speed aggressive maneuvers when facing real wind and disturbances.

**Solution & Novelty**: Hybrid classical INDI + lightweight data-driven residual learning, optimized for higher aggression and dynamic wind with fast-adapting models suitable for Crazyflie.

**Usefulness if Successful**: Enables reliable aggressive flight in real outdoor conditions for inspection, search & rescue, and cinematic applications.

---

### Rank 2: Iterative Learning for Minimum-Snap Trajectory Refinement
**Problem**: Classical minimum-snap trajectories often have repeatable errors on real hardware that are never automatically corrected at the planning level.

**Solution & Novelty**: Use iterative learning (ILC-style) to refine the trajectory generator itself based on real flight data, rather than only improving tracking of a fixed trajectory.

**Usefulness if Successful**: Much easier and faster development of reliable aggressive maneuvers with less manual tuning.

---

### Rank 3: Lightweight Online Adaptive Compensation for Varying Payloads
**Problem**: Unknown or changing payloads during flight cause large tracking errors; most existing methods use offline models or heavy neural networks.

**Solution & Novelty**: Lightweight online adaptation (non-NN where possible) tightly integrated with INDI for real-time compensation of varying payloads.

**Usefulness if Successful**: Enables practical delivery and inspection drones that can handle different or shifting loads without stopping for re-tuning.

---

### Rank 4: Lightweight Learned Correction for Small Multi-Drone Aggressive Planning
**Problem**: Downwash and aerodynamic interference in confined aggressive multi-drone flight causes instability or overly conservative paths.

**Solution & Novelty**: Lightweight learned correction layer on top of classical minimum-snap planning with explicit safety margins (planning-level, not full controller learning).

**Usefulness if Successful**: Reliable coordinated aggressive flight for small teams in motion-capture rooms or similar confined spaces.

---

### Rank 5: Degradation-Aware Replanning for Small Multi-Drone Teams
**Problem**: Actuator degradation (common in real flights) often leads to mission failure because current systems lack fast, graceful real-time replanning.

**Solution & Novelty**: Combine fast degradation detection with lightweight model adaptation and online trajectory replanning for small teams.

**Usefulness if Successful**: Significantly more reliable long-duration swarm missions and safer operation when partial failures occur.

---

### Autonomous Idea A: Vision-based Autonomous Obstacle Avoidance & Reactive Replanning
**Problem**: Strong dependence on external mocap prevents true autonomy in cluttered or partially unknown indoor environments during aggressive flight.

**Solution & Novelty**: Lightweight onboard vision + reactive replanning layer on top of classical control, designed to run feasibly on Crazyflie-class hardware.

**Usefulness if Successful**: True autonomous aggressive flight in cluttered indoor spaces with much less reliance on external positioning systems.

---

### Autonomous Idea B: Autonomous Adaptive Inspection Path Planning & Execution
**Problem**: Fixed or manually designed inspection paths are inefficient and potentially unsafe in partially unknown or changing indoor environments.

**Solution & Novelty**: Combine classical motion planning with lightweight learning that adapts paths based on onboard sensing and previous flight data.

**Usefulness if Successful**: Faster, safer, and more complete autonomous inspection missions with minimal human intervention.

---

### Autonomous Idea C: Lightweight Learning for Autonomous Safe Recovery & Landing
**Problem**: When disturbances or partial actuator issues occur, drones often crash or require manual intervention because they lack autonomous recovery decisions.

**Solution & Novelty**: Add a lightweight high-level decision layer (learning-assisted) on top of robust low-level control to autonomously trigger safe recovery or landing.

**Usefulness if Successful**: Much higher reliability and safety for unsupervised autonomous operations; reduces drone loss rate.

---

## Notes for Supervisor

- All ideas keep **learning lightweight** and treat it as a supporting tool only.
- Strong emphasis on **real hardware experiments** on Crazyflie (single or small multi).
- Clear differentiation from recent work (e.g., Cobo-Briesewitz, Wahba & Hönig (2026) LINDI/NA-INDI paper).
- Good potential for measurable contributions (tracking error reduction, success rate under disturbance, iteration improvement, etc.).
- All are scoped realistically for a 6-month Master's thesis.

**Recommended next step**: Choose 1–2 favorite ideas for deeper proposal development (objectives, methodology sketch, timeline, risk mitigation).

---

---

## References (Key Related Works)

- [Cobo-Briesewitz et al. (2026) LINDI/NA-INDI](https://arxiv.org/abs/2503.09441) — *Learned Incremental Nonlinear Dynamic Inversion for Quadrotors with and without Slung Payloads*.
- [arXiv:2505.04972 (2025)](https://arxiv.org/abs/2505.04972) — *AI and Vision based Autonomous Navigation of Nano-Drones*.
- [Tal & Karaman (2018)](https://arxiv.org/abs/1809.04048) — Accurate Tracking of Aggressive Quadrotor Trajectories Using INDI.
- [Preiss et al. (2017)](https://arxiv.org/abs/1704.04852) — Downwash-Aware Trajectory Planning for Large Quadrotor Teams.
- [Shi et al. (2020) Neural-Swarm](https://arxiv.org/abs/2003.02992) — Decentralized Close-Proximity Multirotor Control Using Learned Interactions.
- Zhou et al. (2020). EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors.
- Landry et al. (2015). Aggressive Quadrotor Flight through Cluttered Environments.

*Additional references can be added during literature review.*

---

*Document prepared for discussion with supervisor – July 2026*