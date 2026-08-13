# Experimental Protocol – Two-Robot Comparison

## Goal
Fair comparison of:
- Pure INDI
- Geometric + NN
- FBL + NN

on two Crazyflie brushless drones in tight formation. We aim for up to 7 controllers if time allows (see `docs/01_Thesis_Project_Snapshot.md` §2 for the full list and target levels: Minimum = Methods 1–3, Ideal = Methods 1–5, Super perfect = all 7).

## Hardware
- 2× Crazyflie brushless (identical configuration)
- OptiTrack @ 100 Hz + Crazyswarm2
- High-rate logging of states, commands, and motor signals

## Formations (Phase A)

**Primary:**
- Vertical stack
- Horizontal offset stack
- Side-by-side (close)

**Secondary / Multi-robot:**
- 3-drone vertical / I-stack or V-stack
- Triangle / diamond
- Leader-follower line
- Dynamic height exchange / swapping
- Close docking / very tight vertical approach
- Dynamic formation change (e.g. stack → side-by-side)

**Separations to test:** 40 cm → 30 cm → 25 cm → 20 cm (and tighter if stable)

## Trajectories
- Hover
- Horizontal circle / figure-8
- Vertical motion while maintaining formation
- Minimum-snap trajectories

## Primary Metrics
- Position RMSE (total and per axis)
- Maximum vertical deviation
- Attitude RMSE
- Control effort
- Residual force magnitude (when available)

## Procedure
For each controller and each separation:
1. Take off and form the formation at safe distance
2. Reduce to target separation
3. Execute trajectory
4. Log data
5. Repeat ≥ 5 times

Safety: Human pilot always ready. Start with larger separations.
