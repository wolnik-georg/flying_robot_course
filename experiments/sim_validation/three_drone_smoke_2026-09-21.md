# Three-drone sim smoke — 2026-09-21T22:37+02:00

Script: `experiments/analysis/run_3drone_smoke.sh run`

| Scenario | Controller | Verdict | Notes |
|---|---|---|---|
| B1 | geo | **ABORT (pre-takeoff)** | EKF vs mocap gate failed; see `client_geo.log`. No new `B1_*.meta.json`. |
| B2 | geo | **ABORT (pre-takeoff)** | Same as B1. |

Automated verify rows that paired **A8** sidecars with idle `record_states` dirs are **invalid** —
disregard negative coverage percentages from that mismatch.

## Authoritative sim status (unchanged)

See [`docs/33_Three_Drone_Sim_Smoke.md`](../../docs/33_Three_Drone_Sim_Smoke.md) and
[`matrix_results.md`](matrix_results.md):

- **B1 geo:** EXPECTED ~58.7 mm (combined wash)
- **B1 indi:** PASS
- **B2 geo / indi:** PASS in full matrix

## Hardware blockers (unchanged)

- Third airframe + mocap volume / anchor for `crazyflies.yaml` three-robot roster.
- uSD on all three for thesis-grade metrics (current pipeline is 2-drone merge).
- Lab time: B1/B2 are **Priority B** in the flight checklist (`docs/07`), after 2-drone C.1/C.4 core.

## Interpretation

- Sim **PASS** means commanded geometry is realised under the sim physics backend — not that downwash compensation works on hardware.
- B1 under **geometric** may show **EXPECTED** ~59 mm vertical error (combined wash of two neighbours); INDI typically passes tighter — see docs/12 §4.
