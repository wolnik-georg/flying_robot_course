# Controllers

All five run the same firmware function, `controllerOmarIndi`, compiled into the bindings; the controller names only select firmware switches (`frsim/controllers.py`, one place per controller).

| Name | What it is | Firmware switches |
|---|---|---|
| S0 | geometric controller: position PID with feedforward, attitude control on the rotation error; no downwash compensation | attitude mode 0, neural compensation off |
| S1 | incremental nonlinear dynamic inversion (INDI): the same position loop, the attitude loop inverts the measured angular acceleration (uses gyro, accelerometer and rpm telemetry) | attitude mode 3, compensation off |
| S2 | S0 + neural downwash compensation (NS2): a network predicts the downwash force from the other vehicle's position and adds it as a feed-forward | attitude mode 0, compensation on |
| S3 | S1 + NS2 | attitude mode 3, compensation on |
| FBL | feedback linearisation with a learned downwash residual: thrust along the measured body axis, the stock rate loop inside, downwash from a reduced-order model + a 4-neuron network | feedback-linearisation branch on, stock inner loop, network on |

S2 and S3 use the **gains of S0 and S1** (no tuning of their own, decision of 7 Nov 2026): the compensation is the only difference.

## Gains (`gains/defaults.yaml`)

S0 - S3 share 10 gains; x and y are tied.

| Gain | Meaning | Default |
|---|---|---|
| `Pxy`, `Pz` | position proportional gain | 7.0, 7.0 |
| `Dxy`, `Dz` | velocity (damping) gain | 4.0, 4.0 |
| `Iz` | position integral gain in z (removes a height offset) | 1.5 |
| `KRxy`, `KRz` | attitude error gain (roll and pitch, yaw) | 0.007, 0.008 |
| `Kwxy`, `Kwz` | angular-rate error gain | 0.00115, 0.002 |
| `KIxy` | attitude integral gain (roll and pitch) | 0.03 |

FBL has 6 gains: `kp_xy`, `kv_xy`, `ki_xy`, `kp_z`, `kv_z`, `ki_z` (position proportional, velocity, integral gains for x / y and z; defaults 14, 9.5, 12, 11.25, 5.4, 12). The gains of its inner rate loop are fixed (the stock Crazyflie rate PID).

## Tuned gains (`gains/tuned/`)

S0 - S3, one set per scenario (A1, A4, A8). The file header names the pick rule: per scenario the optimisation method with the lowest median selection error over its seeds (CMA-ES, Bayesian optimisation, Sobol sampling, gradient descent compared), then its best seed. Each entry lists method, seed, selection error, crashed episodes and the test error for nominal and 10 / 40 / 60 ms delay (exact vehicle inertia). Tuned gains of FBL are not added yet (retuning with the exact inertia is running; they will be added when its closing table is final).

Use them with `--gains tuned`, or write your own YAML with gain names (`--gains my_gains.yaml`; missing names take the default).
