# Unified Residual-Wrench Model

> ⚠️ **Superseded on the sign convention, 2026-08-23.** The equation below is written with a
> `+ m g e_3` gravity term. The implementation and the thesis text use a **z-up (ENU)** frame, in
> which gravity acts along `-e_3`. The authoritative version is
> [`thesis/ch3_system_modelling.tex`](thesis/ch3_system_modelling.tex) §3.1–3.2, which matches
> `firmware_app/src/lib.rs`. This document remains useful as the short statement of how each
> controller family uses the residual.

We model each multirotor as a rigid body on SE(3). Aerodynamic interaction forces and torques (mainly downwash) are collected into an unknown **residual wrench**.

## Continuous-time dynamics

\[
\begin{aligned}
\dot{p} &= v \\
m \dot{v} &= -m g e_3 + f R e_3 + f_{\mathrm{res}} \quad\text{(z-up; see Ch.3)} \\
\dot{R} &= R \hat{\omega} \\
J \dot{\omega} &= -\omega \times J \omega + \tau + \tau_{\mathrm{res}}
\end{aligned}
\]

- \(f_{\mathrm{res}}, \tau_{\mathrm{res}}\) = residual wrench caused by inter-vehicle aerodynamic interactions

## How each controller family uses the residual

**Pure INDI (Reactive)**  
Estimates the residual effect online from measured accelerations and compensates it incrementally. No explicit model needed.

**Geometric + NN / FBL + NN (Predictive)**  
A neural network predicts the residual wrench from relative states of neighbours. The prediction is used as feed-forward compensation:
\[
f_{\mathrm{cmd}} = f_{\mathrm{nominal}} - R^\top \hat{f}_{\mathrm{res}}, \quad
\tau_{\mathrm{cmd}} = \tau_{\mathrm{nominal}} - \hat{\tau}_{\mathrm{res}}
\]

**Hybrid**  
Combines neural prediction with online INDI residual estimation.

## Main Assumptions
- Relative states of neighbours are known with sufficient accuracy
- Residual changes slowly enough or is well predicted by the NN
- Nominal mass and inertia are known
