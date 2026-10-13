#!/usr/bin/env python3
"""Literal NumPy reference of the Tal & Karaman (2020, arXiv 1809.04048v2) INDI controller.

Everything inside `TKReference.step` is written in the paper's own frames: world NED, body FRD,
quaternion xi = body -> world, specific thrust tau < 0.  Only the inputs and the outputs are
converted from / to the firmware frames (world ENU, body FLU).  Equation numbers are the paper's.

Used (1) to verify the flatness feed-forward (14)/(15) against a true trajectory and (2) as the
bit-level cross-check of the Rust (`tk_indi.rs`) and C (`controller_tk_indi.c`) controllers.

Not in the law (so not here): clamps, notch, velocity low-pass, integral action.  The only
additions are division-by-zero guards (marked GUARD) and the filter priming at the first sample.
"""
import math
import numpy as np

G = 9.81
IZ = np.array([0.0, 0.0, 1.0])
SKEW_IZ = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]], float)

# Table III (platform independent) -- NED, so z is the vertical axis
KX = np.diag([18.0, 18.0, 13.5])
KV = np.diag([7.8, 7.8, 5.9])
KA = np.diag([0.5, 0.5, 0.3])
KXI = np.diag([175.0, 175.0, 82.0])
KOM = np.diag([19.5, 19.5, 19.2])

C_W = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)   # ENU <-> NED (involution)
C_B = np.diag([1.0, -1.0, -1.0])                             # FLU <-> FRD (involution)

# firmware mixer sign pattern (FLU): tau_x = arm(-t1-t2+t3+t4), tau_y = arm(-t1+t2+t3-t4),
# tau_z = t2t(-t1+t2-t3+t4)
MIX_X = np.array([-1, -1, 1, 1.0])
MIX_Y = np.array([-1, 1, 1, -1.0])
MIX_Z = np.array([-1, 1, -1, 1.0])


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], float)


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw*bw - ax*bx - ay*by - az*bz,
                     aw*bx + ax*bw + ay*bz - az*by,
                     aw*by - ax*bz + ay*bw + az*bx,
                     aw*bz + ax*by - ay*bx + az*bw])


def quat_to_rot(q):
    w, x, y, z = q
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
                     [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
                     [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]])


class Biquad:
    """2nd-order Butterworth, same design as the Omar/Bitcraze lpf2p (q = 0.7071)."""

    def __init__(self, fc, dt, n):
        tau = 1.0 / (2 * math.pi * fc)
        k = math.tan(dt / (2 * tau))
        q = 0.7071
        poly = k*k + k/q + 1.0
        self.a0 = 2*(k*k - 1)/poly
        self.a1 = (k*k - k/q + 1)/poly
        self.b0 = k*k/poly
        self.b1 = 2*self.b0
        self.n = n
        self.primed = False
        self.i = np.zeros((2, n))
        self.o = np.zeros((2, n))

    def update(self, x):
        x = np.asarray(x, float)
        if not self.primed:
            self.i[:] = x
            self.o[:] = x
            self.primed = True
        y = self.b0*x + self.b1*self.i[0] + self.b0*self.i[1] - self.a0*self.o[0] - self.a1*self.o[1]
        self.i[1] = self.i[0]; self.i[0] = x
        self.o[1] = self.o[0]; self.o[0] = y
        return y


def flatness_matrix(R, tau):
    """4x4 matrix of (14)/(15): [[tau R [iz]x^T, bz], [S, 0]] and S, with b* = columns of R."""
    bx, by, bz = R[:, 0], R[:, 1], R[:, 2]
    rpsi = np.array([bx[0], bx[1]])
    d = float(rpsi @ rpsi)
    n = np.array([-bx[1], bx[0]])
    P = np.array([[0.0, -bz[0], by[0]], [0.0, -bz[1], by[1]]])
    S = (n @ P) / d
    M = np.zeros((4, 4))
    M[:3, :3] = tau * R @ SKEW_IZ.T
    M[:3, 3] = bz
    M[3, :3] = S
    return M, S, (n, P, d)


def s_dot(R, Om, parts):
    """dS/dt for body rate Om (needed in (15)); columns of R rotate with R[Om]x."""
    bx, by, bz = R[:, 0], R[:, 1], R[:, 2]
    Rdot = R @ skew(Om)
    dbx, dby, dbz = Rdot[:, 0], Rdot[:, 1], Rdot[:, 2]
    n, P, d = parts
    ndot = np.array([-dbx[1], dbx[0]])
    Pdot = np.array([[0.0, -dbz[0], dby[0]], [0.0, -dbz[1], dby[1]]])
    ddot = 2*(bx[0]*dbx[0] + bx[1]*dbx[1])
    num = n @ P
    return (ndot @ P + n @ Pdot) / d - num * ddot / d**2


def feedforward(R, tau, taudot, Om, j_ref, s_ref, psid, psidd):
    """Eqs. (14), (15): Omega_ref, Omegadot_ref (body FRD). GUARD: singular matrix -> zeros."""
    M, S, parts = flatness_matrix(R, tau)
    if abs(tau) < 0.5 or abs(np.linalg.det(M)) < 1e-9:   # GUARD (tau -> 0 on the ground)
        return np.zeros(3), np.zeros(3), M
    sol = np.linalg.solve(M, np.append(j_ref, psid))
    Om_ref = sol[:3]
    bz = R[:, 2]
    A = R @ ((2*taudot*np.eye(3) + tau*skew(Om)) @ SKEW_IZ.T @ Om)
    rhs = np.append(s_ref - A, psidd - float(sd_term(R, Om, parts)))
    sol2 = np.linalg.solve(M, rhs)
    return Om_ref, sol2[:3], M


def sd_term(R, Om, parts):
    """S-dot * Omega as in (15) (bottom row)."""
    return s_dot(R, Om, parts) @ Om


class TKReference:
    def __init__(self, mass=0.041, kt=(4.2e-10,)*4, arm=0.0353553, t2t=0.0056928,
                 J=(23.951e-6, 23.951e-6, 32.347e-6), jrz=1.0e-7, fc=30.0, dt=1.0/1000.0,
                 yaw_literal=True):
        self.m, self.kt, self.arm, self.t2t = mass, np.array(kt), arm, t2t
        self.J = np.diag(J)
        self.jrz = jrz
        self.dt = dt
        self.yaw_literal = yaw_literal
        mk = lambda n: Biquad(fc, dt, n)
        self.f_a, self.f_s = mk(3), mk(3)         # a_f, (tau bz)_f  (inertial NED)
        self.f_w2, self.f_w = mk(4), mk(4)        # (omega^2)_f, omega_f (-> omegadot_f)
        self.f_Om = mk(3)                          # Omega_f (-> Omegadot_f)
        self.prev = None                           # previous omega_f, Omega_f, tau_f

    def step(self, pos, vel, quat, gyro, acc_b_g, rpm, ref, rpm_ok=True):
        """pos, vel ENU; quat (w,x,y,z) FLU->ENU; gyro rad/s FLU; acc_b_g specific force in g,
        FLU; rpm[4]; ref = dict(pos, vel, acc, jerk, snap [ENU], yaw, yaw_rate, yaw_acc [rad])."""
        dt, m = self.dt, self.m
        R_enu = quat_to_rot(quat)
        R = C_W @ R_enu @ C_B                               # body FRD -> world NED
        x, v = C_W @ np.asarray(pos), C_W @ np.asarray(vel)
        Om = C_B @ np.asarray(gyro)
        ab = C_B @ (np.asarray(acc_b_g) * G)                # a^b, FRD (hover: z = -g)

        # --- filtered measurements (Fig. 4) -------------------------------------------------
        a_f = self.f_a.update(R @ ab + G * IZ)              # (16)
        Om_f = self.f_Om.update(Om)
        om = np.asarray(rpm, float) * 2*math.pi/60.0
        if rpm_ok:
            T_i = self.kt * np.asarray(rpm, float)**2
            tau_raw = -float(T_i.sum()) / m
            bz = R[:, 2]
            s_f = self.f_s.update(tau_raw * bz)
            w2_f = self.f_w2.update(T_i)                    # filtered per-motor thrust (k w^2)
            w_f = self.f_w.update(om)
            tau_f = -float(w2_f.sum()) / m
            mu_f_flu = np.array([self.arm*float(MIX_X @ w2_f), self.arm*float(MIX_Y @ w2_f),
                                 self.t2t*float(MIX_Z @ w2_f)])
        else:                                               # IMU-only fallback
            s_f = self.f_s.update(a_f - G*IZ)
            w2_f = self.f_w2.update(np.zeros(4)); w_f = self.f_w.update(np.zeros(4))
            tau_f = -float(np.linalg.norm(s_f))
            mu_f_flu = None
        if self.prev is None:
            Omd_f, wd_f, taud_f = np.zeros(3), np.zeros(4), 0.0
        else:
            Omd_f = (Om_f - self.prev[0]) / dt
            wd_f = (w_f - self.prev[1]) / dt
            taud_f = (tau_f - self.prev[2]) / dt
        self.prev = (Om_f.copy(), w_f.copy(), tau_f)
        if mu_f_flu is not None:
            mu_f_flu = mu_f_flu + np.array([0.0, 0.0, self.jrz * float(MIX_Z @ wd_f)])   # G2 omegadot
            mu_f = C_B @ mu_f_flu
        else:
            mu_f = self.J @ Omd_f

        # --- position / acceleration (17), (20), (21) --------------------------------------
        xr, vr = C_W @ np.asarray(ref['pos']), C_W @ np.asarray(ref['vel'])
        ar, jr, sr = (C_W @ np.asarray(ref[k]) for k in ('acc', 'jerk', 'snap'))
        a_c = KX @ (xr - x) + KV @ (vr - v) + KA @ (ar - a_f) + ar
        s_c = s_f + a_c - a_f                               # (20)
        ns = float(np.linalg.norm(s_c))
        T_c = m * ns                                        # (21) (magnitude, N)

        # --- incremental attitude (22)-(27) -------------------------------------------------
        v_b = R.T @ (s_c / ns) if ns > 1e-6 else -IZ        # (22) GUARD ns -> 0
        qbar = np.array([1.0 - IZ @ v_b, *(-np.cross(IZ, v_b))])   # (23)
        nq = np.linalg.norm(qbar)
        qbar = qbar / nq if nq > 1e-9 else np.array([0.0, 1.0, 0.0, 0.0])
        Rbar = quat_to_rot(qbar)
        psi = math.pi/2 - ref['yaw']                        # ENU yaw (CCW from +x) -> NED heading
        nbar = (R @ Rbar).T @ np.array([math.sin(psi), -math.cos(psi), 0.0])   # (24)
        if self.yaw_literal:                                                   # (25) as printed
            qpsi = np.array([1.0, 0.0, 0.0, -nbar[0]/nbar[1]]) if abs(nbar[1]) > 1e-9 else np.array([1.0, 0, 0, 0])
        else:                                                                  # half-angle form
            phi = math.atan(-nbar[0]/nbar[1]) if abs(nbar[1]) > 1e-9 else 0.0
            qpsi = np.array([math.cos(phi/2), 0, 0, math.sin(phi/2)])
        qpsi = qpsi / np.linalg.norm(qpsi)
        qc = qmul(qbar, qpsi)                                                  # (26)
        w = min(1.0, max(-1.0, qc[0]))
        sq = math.sqrt(max(0.0, 1.0 - w*w))
        k = 2*math.acos(w)/sq if sq > 1e-6 else 2.0
        xi_e = k * qc[1:]                                                      # (27)

        # --- feed-forward (14), (15) with measured tau_f, xi, Omega_f -----------------------
        Om_ref, Omd_ref, _ = feedforward(R, tau_f, taud_f, Om_f, jr, sr,
                                         -ref['yaw_rate'], -ref['yaw_acc'])
        Omd_c = KXI @ xi_e + KOM @ (Om_ref - Om_f) + Omd_ref                   # (28)
        mu_c = mu_f + self.J @ (Omd_c - Omd_f)                                 # (31)
        mu_flu = C_B @ mu_c
        return dict(thrust=T_c, torque=mu_flu, a_f=C_W @ a_f, s_f=C_W @ s_f, s_c=C_W @ s_c,
                    xi_e=xi_e, Om_ref=Om_ref, Omd_ref=Omd_ref, Omd_c=Omd_c, mu_f=mu_f)
