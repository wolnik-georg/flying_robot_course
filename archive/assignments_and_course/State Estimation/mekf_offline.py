#!/usr/bin/env python3
"""
Multiplicative Extended Kalman Filter (MEKF) — offline validation
Runs against the recorded figure-8 flight log (fr00.csv) and compares
the MEKF orientation estimate to the on-board EKF.

State:  x = [p (3), b (3), delta (3)]  (9-dimensional)
        p     : position in world frame [m]
        b     : velocity in body frame [m/s]
        delta : attitude error angles  [rad]
        qref  : reference quaternion (maintained outside the filter state)

Actions (IMU treated as inputs, not measurements):
        og    : gyroscope  [rad/s]
        oa    : accelerometer [m/s^2]

Measurements:
        height : range.zrange [m]
        flow   : motion.deltaX/Y [pixels]

Reference: Markley (2003), Mueller et al. (2015), lecture slides
"""

import csv
import math
import numpy as np

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
G_MS2 = 9.81  # gravitational acceleration [m/s^2]
NP = 350.0  # flow sensor: nominal pixel count (CF convention)
THETA_P = 3.50  # effective calibration constant [empirical, March 15 2026]
# NP/THETA_P ≈ 100 (old 0.71674 was 4.9× too large, gave 4.9× too little XY motion)
DEG2RAD = math.pi / 180.0
G_TO_MS2 = G_MS2  # 1 G = 9.81 m/s^2

# ---------------------------------------------------------------------------
# Quaternion helpers  (convention: q = [w, x, y, z])
# ---------------------------------------------------------------------------


def quat_mult(q, p):
    """Hamilton product of two quaternions [w,x,y,z]."""
    qw, qx, qy, qz = q
    pw, px, py, pz = p
    return np.array(
        [
            qw * pw - qx * px - qy * py - qz * pz,
            qx * pw + qw * px - qz * py + qy * pz,
            qy * pw + qz * px + qw * py - qx * pz,
            qz * pw - qy * px + qx * py + qw * pz,
        ]
    )


def quat_conj(q):
    """Conjugate (inverse for unit quaternions) of q = [w,x,y,z]."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_norm(q):
    """Normalise quaternion."""
    return q / np.linalg.norm(q)


def quat_rotate(q, v):
    """Rotate vector v by quaternion q: q * [0,v] * q^-1, returns 3-vec."""
    qv = np.array([0.0, v[0], v[1], v[2]])
    return quat_mult(quat_mult(q, qv), quat_conj(q))[1:]


def q_from_delta(delta):
    """
    Approximate small-angle error quaternion from delta [rad] (Markley eq 19).
        q(delta) = [1 - ||delta||^2/8,  delta/2]
    """
    d2 = np.dot(delta, delta)
    return quat_norm(
        np.array([1.0 - d2 / 8.0, delta[0] / 2.0, delta[1] / 2.0, delta[2] / 2.0])
    )


def quat_to_euler(q):
    """
    Convert unit quaternion [w,x,y,z] to roll/pitch/yaw [rad].
    ZYX convention (same as Crazyflie firmware quat2rpy).
    Note: pitch is negated to match firmware convention.
    """
    w, x, y, z = q
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw])


# ---------------------------------------------------------------------------
# MEKF steps
# ---------------------------------------------------------------------------


def mekf_reset(x, qref):
    """
    Reset step: fold delta back into qref, zero out delta.
    qref_new = qref * q(delta)
    delta    = 0
    """
    delta = x[6:9].copy()
    qref_new = quat_norm(quat_mult(qref, q_from_delta(delta)))
    x[6:9] = 0.0
    return x, qref_new


def mekf_predict(x, qref, Sigma, og_rads, oa_ms2, dt, R_proc):
    """
    Prediction step driven by IMU (gyro + accel as actions).

    State layout:
        x[0:3] = p  (world position)
        x[3:6] = b  (velocity in body frame)
        x[6:9] = delta (attitude error)

    Dynamics (simplified, using qref only — delta=0 after reset):
        p_new   = p + (qref * b) * dt
        b_new   = b + (qref^-1 * (-g*e3)) * dt + oa * dt
        delta_new = delta + og * dt

    Returns updated x, Sigma.
    """
    p = x[0:3]
    b = x[3:6]
    delta = x[6:9]  # should be ~0 after reset

    # Rotate body velocity to world frame
    v_world = quat_rotate(qref, b)

    # Gravity in body frame: g_body = qref^-1 * [0, 0, -g]
    g_world = np.array([0.0, 0.0, -G_MS2])
    g_body = quat_rotate(quat_conj(qref), g_world)

    # Propagate mean
    p_new = p + v_world * dt
    b_new = b + (g_body + oa_ms2) * dt
    delta_new = delta + og_rads * dt

    x_new = np.concatenate([p_new, b_new, delta_new])

    # Jacobian G = dg/dx  (9x9, computed numerically)
    G = _compute_G(x, qref, og_rads, oa_ms2, dt)

    # Covariance propagation
    Sigma_new = G @ Sigma @ G.T + R_proc

    return x_new, Sigma_new


def _compute_G(x, qref, og_rads, oa_ms2, dt):
    """Numerical Jacobian of the process model w.r.t. state x."""
    eps = 1e-5
    n = len(x)
    G = np.zeros((n, n))
    f0 = _process_model(x, qref, og_rads, oa_ms2, dt)
    for i in range(n):
        x_plus = x.copy()
        x_plus[i] += eps
        f_plus = _process_model(x_plus, qref, og_rads, oa_ms2, dt)
        G[:, i] = (f_plus - f0) / eps
    return G


def _process_model(x, qref, og_rads, oa_ms2, dt):
    """Evaluate g(x, u): next state from current state and IMU inputs."""
    p = x[0:3]
    b = x[3:6]
    delta = x[6:9]

    # Full quaternion including error
    q_full = quat_norm(quat_mult(qref, q_from_delta(delta)))
    q_full_conj = quat_conj(q_full)

    v_world = quat_rotate(q_full, b)
    g_body = quat_rotate(q_full_conj, np.array([0.0, 0.0, -G_MS2]))

    p_new = p + v_world * dt
    b_new = b + (g_body + oa_ms2) * dt
    delta_new = delta + og_rads * dt

    return np.concatenate([p_new, b_new, delta_new])


def mekf_scalar_update(x, Sigma, h_val, z_meas, H_row, R_scalar):
    """
    Single scalar EKF measurement update.
    h_val   : predicted measurement (scalar)
    z_meas  : actual measurement (scalar)
    H_row   : 1x9 measurement Jacobian row (numpy array)
    R_scalar: measurement noise variance (scalar)
    """
    S = float(H_row @ Sigma @ H_row.T) + R_scalar
    K = (Sigma @ H_row.T) / S  # 9x1
    innovation = z_meas - h_val
    x_new = x + K * innovation
    Sigma_new = (np.eye(9) - np.outer(K, H_row)) @ Sigma
    return x_new, Sigma_new


def mekf_update_height(x, qref, Sigma, oz_m, R_height):
    """
    Height measurement update.
    h(x) = p_z  =>  H = [0,0,1, 0,0,0, 0,0,0]
    """
    H_row = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    h_val = x[2]  # predicted p_z
    return mekf_scalar_update(x, Sigma, h_val, oz_m, H_row, R_height)


def mekf_update_flow(x, qref, Sigma, dnx, dny, dt_flow, R_flow):
    """
    Optical flow measurement update.
    Measurement model (body-frame velocity tracked):
        h(x) = (dt*Np / (pz*theta_p)) * [bx, by]
    Applied as two separate scalar updates.
    """
    pz = x[2]
    bx = x[3]
    by = x[4]

    if abs(pz) < 0.05:
        # avoid division by near-zero height
        return x, Sigma

    scale = dt_flow * NP / (pz * THETA_P)

    # --- update for delta_nx (x component) ---
    h_x = scale * bx
    # d(scale*bx)/dx: partial w.r.t. pz = -scale/pz * bx, w.r.t. bx = scale
    H_x = np.zeros(9)
    H_x[2] = -scale / pz * bx  # d/d(pz)
    H_x[3] = scale  # d/d(bx)
    x, Sigma = mekf_scalar_update(x, Sigma, h_x, dnx, H_x, R_flow)

    # recompute scale after state update
    pz = x[2]
    by = x[4]
    if abs(pz) < 0.05:
        return x, Sigma
    scale = dt_flow * NP / (pz * THETA_P)

    # --- update for delta_ny (y component) ---
    h_y = scale * by
    H_y = np.zeros(9)
    H_y[2] = -scale / pz * by  # d/d(pz)
    H_y[4] = scale  # d/d(by)
    x, Sigma = mekf_scalar_update(x, Sigma, h_y, dny, H_y, R_flow)

    return x, Sigma


# ---------------------------------------------------------------------------
# Noise parameters  (tunable)
# ---------------------------------------------------------------------------


# Default values — can be overridden by passing a NoiseParams to run_mekf()
class NoiseParams:
    def __init__(
        self,
        q_pos=1e-7,  # process noise: position      (tuned)
        q_vel=1e-3,  # process noise: body velocity  (tuned)
        q_att=1e-6,  # process noise: attitude error  (tuned)
        r_height=1e-3,  # measurement noise: height [m^2]  (tuned)
        r_flow=8.0,  # measurement noise: flow [pixels^2] (tuned)
    ):
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.q_att = q_att
        self.r_height = r_height
        self.r_flow = r_flow

    def __repr__(self):
        return (
            f"NoiseParams(q_pos={self.q_pos:.2e}, q_vel={self.q_vel:.2e}, "
            f"q_att={self.q_att:.2e}, r_height={self.r_height:.2e}, "
            f"r_flow={self.r_flow:.2e})"
        )

    def make_process_noise(self):
        R = np.zeros((9, 9))
        R[0, 0] = R[1, 1] = R[2, 2] = self.q_pos
        R[3, 3] = R[4, 4] = R[5, 5] = self.q_vel
        R[6, 6] = R[7, 7] = R[8, 8] = self.q_att
        return R

    @staticmethod
    def make_initial_covariance():
        P = np.zeros((9, 9))
        P[0, 0] = P[1, 1] = P[2, 2] = 0.1
        P[3, 3] = P[4, 4] = P[5, 5] = 0.1
        P[6, 6] = P[7, 7] = P[8, 8] = 0.01
        return P


DEFAULT_PARAMS = NoiseParams()

# ---------------------------------------------------------------------------
# CSV loading
# ---------------------------------------------------------------------------


def load_csv(path):
    """
    Load fr00.csv and return list of row dicts with float values.
    NaN entries are represented as float('nan').
    Strips the leading '# ' from the timestamp column name.
    """
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            parsed = {}
            for k, v in row.items():
                key = k.strip().lstrip("#").strip()
                try:
                    parsed[key] = float(v)
                except ValueError:
                    parsed[key] = float("nan")
            rows.append(parsed)
    return rows


def is_valid(val):
    return not math.isnan(val)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------


def run_mekf(csv_path, params: NoiseParams = None):
    if params is None:
        params = DEFAULT_PARAMS
    rows = load_csv(csv_path)

    # ---- Seed initial qref from the first on-board EKF quaternion so that
    #      both filters share the same absolute heading reference.
    #      Yaw is unobservable from accel/height/flow, so without this the
    #      MEKF would always start at yaw=0 regardless of the drone's actual
    #      heading at power-on.
    qref = np.array([1.0, 0.0, 0.0, 0.0])  # fallback; overwritten below
    _qref_initialised = False
    for _row in rows:
        _qw = _row["stateEstimate.qw"]
        if is_valid(_qw):
            qref = np.array(
                [
                    _qw,
                    _row["stateEstimate.qx"],
                    _row["stateEstimate.qy"],
                    _row["stateEstimate.qz"],
                ]
            )
            qref = qref / np.linalg.norm(qref)
            _qref_initialised = True
            break

    x = np.zeros(9)
    x[2] = 1.0  # initial height guess ~1 m
    Sigma = NoiseParams.make_initial_covariance()
    R_proc = params.make_process_noise()

    # Buffers for paired IMU data (gyro and accel arrive on separate rows)
    last_gyro = None  # (timestamp, [gx,gy,gz])
    last_accel = None  # (timestamp, [ax,ay,az])
    last_flow_t = None  # timestamp of previous flow measurement

    # Output storage
    out_times = []
    out_euler = []  # MEKF roll/pitch/yaw [rad]
    out_pos = []  # MEKF position [m]

    # On-board EKF baseline (read directly from CSV)
    ekf_times = []
    ekf_euler = []
    ekf_pos = []  # on-board EKF x/y/z [m]

    prev_t = None

    for row in rows:
        t = row["timestamp"]

        # ------------------------------------------------------------------
        # On-board EKF baseline (just collect, not used to drive filter)
        # ------------------------------------------------------------------
        qw = row["stateEstimate.qw"]
        if is_valid(qw):
            q_ekf = np.array(
                [
                    qw,
                    row["stateEstimate.qx"],
                    row["stateEstimate.qy"],
                    row["stateEstimate.qz"],
                ]
            )
            ekf_times.append(t)
            ekf_euler.append(quat_to_euler(q_ekf))
            ekf_pos.append(
                [
                    row["stateEstimate.x"],
                    row["stateEstimate.y"],
                    row["stateEstimate.z"],
                ]
            )

        # ------------------------------------------------------------------
        # Gyroscope row  (deg/s -> rad/s)
        # ------------------------------------------------------------------
        gx = row["gyro.x"]
        if is_valid(gx):
            og_rads = np.array([gx, row["gyro.y"], row["gyro.z"]]) * DEG2RAD
            last_gyro = (t, og_rads)

        # ------------------------------------------------------------------
        # Accelerometer row  (G -> m/s^2)
        # ------------------------------------------------------------------
        ax = row["acc.x"]
        if is_valid(ax):
            oa_ms2 = np.array([ax, row["acc.y"], row["acc.z"]]) * G_TO_MS2
            last_accel = (t, oa_ms2)

        # ------------------------------------------------------------------
        # When we have a new gyro AND accel pair: do predict
        # (gyro/accel alternate on consecutive rows — pair them by time proximity)
        # ------------------------------------------------------------------
        if last_gyro is not None and last_accel is not None:
            tg, og = last_gyro
            ta, oa = last_accel

            # Use the more recent timestamp as the step time
            t_imu = max(tg, ta)

            if prev_t is not None and t_imu > prev_t:
                dt = t_imu - prev_t

                # 1. Reset: fold delta into qref
                x, qref = mekf_reset(x, qref)

                # 2. Predict
                x, Sigma = mekf_predict(x, qref, Sigma, og, oa, dt, R_proc)

                # Record output
                out_times.append(t_imu)
                out_euler.append(quat_to_euler(qref))
                out_pos.append(x[0:3].copy())

            prev_t = t_imu
            # Consume both so we wait for the next pair
            last_gyro = None
            last_accel = None

        # ------------------------------------------------------------------
        # Height measurement update
        # ------------------------------------------------------------------
        oz = row["range.zrange"]
        if is_valid(oz):
            oz_m = oz / 1000.0  # mm -> m
            x, Sigma = mekf_update_height(x, qref, Sigma, oz_m, params.r_height)

        # ------------------------------------------------------------------
        # Optical flow measurement update
        # ------------------------------------------------------------------
        dnx = row["motion.deltaX"]
        if is_valid(dnx):
            dny = row["motion.deltaY"]
            # dt for flow: time since last flow measurement
            if last_flow_t is not None and t > last_flow_t:
                dt_flow = t - last_flow_t
                x, Sigma = mekf_update_flow(
                    x, qref, Sigma, dnx, dny, dt_flow, params.r_flow
                )
            last_flow_t = t

    return (
        np.array(out_times),
        np.array(out_euler),
        np.array(out_pos),
        np.array(ekf_times),
        np.array(ekf_euler),
        np.array(ekf_pos),
    )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import os

    csv_path = os.path.join(
        os.path.dirname(__file__), "logging_ekf", "logging", "fr00.csv"
    )

    print("Running MEKF offline on:", csv_path)
    mekf_t, mekf_euler, mekf_pos, ekf_t, ekf_euler, ekf_pos = run_mekf(csv_path)

    print(f"MEKF produced {len(mekf_t)} estimates over {mekf_t[-1]:.2f} s")
    print(f"On-board EKF has {len(ekf_t)} samples")

    # Save results for plotting
    out_path = os.path.join(os.path.dirname(__file__), "mekf_results.npz")
    np.savez(
        out_path,
        mekf_t=mekf_t,
        mekf_roll=mekf_euler[:, 0],
        mekf_pitch=mekf_euler[:, 1],
        mekf_yaw=mekf_euler[:, 2],
        mekf_x=mekf_pos[:, 0],
        mekf_y=mekf_pos[:, 1],
        mekf_z=mekf_pos[:, 2],
        ekf_t=ekf_t,
        ekf_roll=ekf_euler[:, 0],
        ekf_pitch=ekf_euler[:, 1],
        ekf_yaw=ekf_euler[:, 2],
        ekf_x=ekf_pos[:, 0],
        ekf_y=ekf_pos[:, 1],
        ekf_z=ekf_pos[:, 2],
    )
    print("Results saved to:", out_path)
