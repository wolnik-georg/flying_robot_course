# 33 — Three-drone sim smoke (B1 / B2)

**Purpose.** Ch. 7 / multi-robot scaling needs a **honest sim baseline** before hardware B1–B3.
This doc records what is already validated, how to re-run smoke, and what blocks three-robot lab
flights.

**Last updated:** 21 September 2026

---

## What the formation library already proves (authoritative)

Full matrix write-up: [`12_Sim_Formation_Validation_Report.md`](12_Sim_Formation_Validation_Report.md).  
Raw rows: [`experiments/sim_validation/matrix_results.md`](../experiments/sim_validation/matrix_results.md).

| Scenario | Geometric | INDI | Thesis reading |
|---|---|---|---|
| **B1** I-stack (combined wash of two neighbours) | **EXPECTED** — mean \|e_z\| ≈ **58.7 mm** (exceeds 50 mm tol **by design** of uncorrected geometric tracking under double downwash) | **PASS** — sub-mm vertical error in matrix | Superposition / combined-wash case is sim-validated; geometric miss is documented, not a scenario bug |
| **B2** V-stack (asymmetric overlap) | **PASS** — ~43 mm | **PASS** after full matrix re-run | Asymmetric geometry flies under both controllers in sim |

**Do not re-litigate B1 geometric as a “defect”.** INDI passes the same geometry; that is the
evidence the scenario and sim backend are sound ([`12`](12_Sim_Formation_Validation_Report.md) §4).

---

## Desk smoke script (re-run on a sim-capable machine)

```bash
cd /home/georg/Desktop/flying_robot_course
./experiments/analysis/run_3drone_smoke.sh        # B1 + B2, geo, line path
./experiments/analysis/run_3drone_smoke.sh verify # pair newest B1/B2 meta + record_states only
```

Uses `run_sim_matrix.sh one geo 3 --scenario B1|B2 ...` with `crazyflies_sim3.yaml` (~3 min/run).

**Prerequisites:** sourced ROS Humble + `crazyswarm2/install`, pyenv `flying_robots` for verify,
SIL backend `neuralswarm`, write access to `experiments/logs/` for sidecars.

---

## 2026-09-21 smoke attempt (this workspace)

| Step | Result |
|---|---|
| B1 geo | Client **aborted at EKF vs mocap gate** (300–671 mm \|err\|, limit 150 mm) — **no takeoff**, no new `B1_*.meta.json` |
| B2 geo | Same abort pattern |
| Sidecar pairing | Fallback matcher picked unrelated A8 meta → **ignore** automated FAIL rows in `three_drone_smoke_2026-09-21.md` from that run |

Log excerpt: `experiments/sim_validation/client_geo.log` (`*** ABORT: EKF does not agree with mocap ***`).

**Likely causes to fix before the next smoke:** sim server not fully converged in 15 s presleep;
stale `initial_position` vs spawned mocap layout; or environment-specific path (`Permission denied:
/home/flyingrobots` for log save — different user home in runner config).

**Desk conclusion:** Thesis can cite **August 2026 matrix** for B1/B2 sim. Fresh smoke is for
**regression only** when the sim stack is next touched — not a blocker for current C.1/C.4 work.

---

## Hardware blockers (three-robot lab)

| Blocker | Notes |
|---|---|
| **Third vehicle + roster** | `crazyflies_sim3.yaml` names ≠ current cf5/cf_second pair; need consistent 3-drone yaml + mocap layout |
| **Volume / anchor** | B1/B2 need height and lateral clearance; auto-center moves anchor — document in `.meta.json` |
| **uSD pipeline** | `merge_usd_logs.py` is exercised for **2** drones; three-robot merge is extension work (roles bottom/center/top) |
| **Campaign priority** | [`07`](07_Thesis_Progress_Checklist.md): 2-drone A1→A8 ladder before B1–B3 hardware |
| **Thesis scope** | Minimum Methods 1–4 are **2-drone**; B1–B3 support **advanced / scaling** claims — state as planned or limitation in Ch. 7 |

---

## Ch. 7 wording (suggested)

> Three-robot scenarios B1 and B2 were validated in software-in-the-loop simulation (combined-wash
> and asymmetric V-stack). Hardware experiments focused on the two-robot campaign required for
> residual learning and controller comparison; extending the same protocol to three vehicles is
> left to future work / was not completed within the flight budget ([`32`](32_C4_Flight_Budget.md)).

---

## Related

- Sim commands cheat sheet: [`lab_sessions/sim_scenario_commands_2026-09-08.md`](lab_sessions/sim_scenario_commands_2026-09-08.md)
- Downwash backend 3-drone notes: `flying_drone_stack/firmware_app/host/naindi_reference_build_notes.md` (2026-09-18)
