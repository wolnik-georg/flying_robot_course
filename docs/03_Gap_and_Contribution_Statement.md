# Gap & Contribution Statement

> **Superseded in part, 2026-08-23.** The *contribution* section below is superseded by
> `19_Contribution_Statement.md`, which was written against the
> research questions in `15` and states what the
> thesis does **not** claim. The gap statement here has been corrected (see
> `17` §1) and is retained as the shorter version.

### Current State of the Literature

Two main families of methods address aerodynamic interaction forces (mainly downwash) in close-proximity multirotor flight:

- **Reactive methods** (primarily INDI) estimate residual forces online from inertial measurements and compensate them incrementally. They are computationally light and react quickly, but do not use prior knowledge of the interaction.
- **Predictive methods** learn a neural network model of the residual interaction forces from relative states and inject the prediction as feed-forward into a baseline controller (geometric SE(3), feedback-linearising, or MPC).

A hybrid approach combining a learned residual with INDI has been demonstrated on a **single vehicle**, with and without a slung payload (Cobo-Briesewitz et al., L4DC 2026). There the hybrid (NA-INDI) was best without a payload but **by a small margin** (4.14 cm vs 4.28 cm for INDI on a figure-8), and with a payload it *"remains similar in performance"* to INDI. That paper's headline contribution is a different one: a learned residual (LINDI) that matches INDI **without requiring rotor-speed sensors**. The hybrid has not been studied in the multi-robot case, where the residual is caused by neighbouring vehicles rather than by the vehicle itself and is a function of measurable relative state.

### The Gap

There is no systematic experimental study that:
1. Implements representative controllers from the reactive (INDI), predictive (Geometric+NN and FBL+NN), and hybrid families,
2. Evaluates them under identical conditions on the same multirotor hardware,
3. Quantifies performance differences (tracking accuracy, residual force, control effort, robustness, computational cost),
4. Examines scaling from 2-robot to ≥3-robot teams in tight formation.

### Contribution of This Thesis

This thesis provides the first systematic multi-robot comparison of reactive, predictive, and hybrid control strategies for interaction-force compensation during tight formation flight.

Concretely it will:
- Implement and fairly compare at least three controllers (Pure INDI, Geometric+NN, FBL+NN) on Crazyflie brushless platforms;
- Extend the evaluation from 2 to ≥3 robots;
- Optionally implement a multi-robot hybrid (neural-augmented INDI) and a residual RL controller;
- Quantify the trade-offs using consistent real-flight experiments;
- Derive clear engineering recommendations on when each method is preferable.
