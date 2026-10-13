# 75 — How the simulation works, end to end (2026-10-13)

One sentence: the simulation runs **the same controller source as on the drone**, compiled for the host, in closed loop with a simple physics model; sensors, estimator, radio and ground are not simulated. This page is a reading of the code (file and function names are given so every statement can be checked), plus what the simulation was validated for. Results: `docs/72`, `docs/sil_study_screening.md`.

## 1. The loop, independent of the controller (one tick = 1 ms)
```
scenario (formation library)            experiments: crazyswarm2/.../formations/scenarios.py, poly4d.py
   │  curve → polynomial pieces
   ▼
firmware high-level commander (host build)  takeoff / uploadTrajectory / startTrajectory / getSetpoint   (crazyflie_sim/crazyflie_sil.py, CrazyflieSIL)
   │  setpoint: position, velocity, acceleration, yaw
   ▼
state handed to the controller  = plant truth, plus noise on position and gyro only   (state_for_controller in ns2_closed_loop_sil_sim.py)
   │  position, velocity, attitude, acceleration (body specific force), measured RPM: truth; noise std from the hardware logs: position 3.6 / 10 / 5.3 mm, gyro 25.6 / 12.5 / 2.3 °/s
   ▼
CONTROLLER = the firmware source, host-compiled, called through the Python bindings (cffirmware):
   geometric / ours INDI (Rust lib.rs, controller 6), Omar C (controller_omar_indi.c), Omar Rust, Tal & Karaman (Rust / C)
   │  output: thrustSi [N] and torque x, y, z [N m], once per millisecond (the command is held over the physics substeps)
   ▼
firmware power distribution + cap (powerDistribution, powerDistributionCap)  →  PWM  →  inverse of the thrust model  →  RPM command per motor
   ▼
plant: crazyflie_sim/backend/np.py, class Quadrotor (numpy, rigid body)
   RPM → first-order motor lag (44 ms, brushless) → force k_t·rpm² (per-motor k_t, mass, inertia, arm length, thrust-to-torque taken from the firmware constants)
   → mixer → rigid-body dynamics (no drag), external force and torque added (see §2)
   │  returns position, velocity, attitude, angular velocity, specific force in the body frame (what an accelerometer would read, gravity excluded) and the measured RPM (after the lag)
   └── back to "state handed to the controller"
```
What is deliberately **not** in the loop: mocap latency and the EKF (the controller sees the truth plus noise), radio, DShot telemetry latency / noise / invalid readings, battery sag and ESC behaviour (PWM is mapped to force with an ideal inverse model), ground contact (the plant only stops at z = 0), thrust-model mismatch between the firmware constants and the real motors (about 1.14 on the hardware), and a real downwash shape.

Because the controllers are the flight source, a change in the Rust or C file is simulated without a second implementation. Host-build detail that matters: the host build renames `rpm_get_all` and the `indi_*_write` log functions for every source file (`-D…_fw_unused`), so a C controller has to `#undef` them (`controller_tk_indi.c`), otherwise it reads RPM 0 silently.

## 2. Example: Neural-Swarm 2 (NS2) end to end
**Training (offline).** Flights of the formation library (26 A-scenario flights) → neighbour-relative position and velocity per sample, gate |dx| < 0.2 m and |dy| < 0.2 m → the network learns the residual specific force `a_res = a_meas − a_model` → 19,297 weights, R² 0.944 → `flying_drone_stack/tools/residual/weights/ns2_v1.npz` (v1 frozen, sha256 `8a43b5b5…`).

**On the drone (flight).** The weights are flash-resident (uploaded once, `rnn.n` verifies the count). The bottom drone receives the other drone's position (peer packets, held at 100 Hz, `rnn.div 10`), forms the relative position and velocity, evaluates the network and gets a predicted residual force. With `rnn.en 1` and `res_sign −1` the controller adds the compensation to the desired force **before** the downwash arrives, because the network predicts it from the geometry. Logged as `rnn_pred_x/y/z`.

**In the simulation.** Two drones, the formation scenario A8 or A1 (`ns2_closed_loop_sil_sim.py`):
- the bottom drone runs the same controller with the same network weights uploaded through the same parameter path;
- the **plant** applies a downwash force to the drone that has the other one above it (`above_only`, `ns2_closed_loop_sil_plant_bank.py`), computed from the relative state by a network. Two plants exist: the original Neural-Swarm-2 reference weights, or the "bank" network trained on our flights (the default for the study screening); a torque term from the lateral force gradient (one scalar fitted on the network-off cohort only);
- the force enters the dynamics **and** the simulated accelerometer, so the controller sees it as a residual, as on the hardware.

**The catch.** In the simulation the controller's network and the plant's network are the same function, so the prediction is perfect and the compensation cancels the wash almost completely (dip −1.0 cm). On the hardware the real force at the crossings is 1.14–1.49× the prediction and the dip is −3.2 cm. The simulated NS2 benefit (−5.5 → −1.0 cm) is an upper bound; the hardware value is −5.9 → −3.2 cm (`docs/72` step 1).

## 3. What the simulation is validated for, and what not
| Question | Status |
|---|---|
| A8 crossing dip, geometric controller, network off / `res_sign +1` / `−1` | reproduced within 1.5 cm for off and `+1` (−5.5 vs −5.9, −10.6 vs −11.0 cm), `−1` overstated by 2.3 cm (above) |
| A1, A2–A7 | unvalidated; `docs/sil_study_screening.md` is a screen, not a prediction |
| Omar C / Omar Rust | run in the harness, but **no dip appears (−0.0 / +0.2 cm) while the hardware shows −9 to −22 cm**: in the simulation the RPM-based residual compensates the wash perfectly, on the hardware it does not (mechanism open, see `docs/76`) |
| Ours INDI | cannot run in the two-drone harness (host gains are process-global, per-drone modes need a per-step re-application that destabilises it), `docs/72` |
| Tal & Karaman | single-drone SIL only (`tk_indi_sil_run.py`, a rotor-level plant with RPM and accelerometer): hover, A1, A8-like paths, figure-8; no downwash, no estimator delay, no ground contact, so it cannot show the takeoff flip seen on the drone (`lab_sessions/2026-10-13.md` §7) |

## 4. Commands
```bash
python3 experiments/analysis/ns2_closed_loop_sil_run.py        # two-drone NS2 closed loop (A8 / A1), results in experiments/analysis/out/ns2_closed_loop_sil/
python3 experiments/analysis/sil_study_screening.py            # study settings screen, docs/sil_study_screening.md
python3 experiments/analysis/tk_indi_sil_run.py --scenario hover --controller oot7 --noise   # single-drone Tal & Karaman closed loop
```
Host bindings are the Python 3.10 extension (`PYTHONPATH=<crazyflie-firmware>/build`, system `python3`, not the 3.12 pyenv); the host library has to be built with `--features residual_nn` or the network upload is rejected.
