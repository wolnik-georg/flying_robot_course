#!/usr/bin/env python3
"""Verify the flatness feed-forward (14)/(15) of tk_indi_reference against a true trajectory
(Omega from R^T Rdot, Omegadot / taudot by central differences of the true signals)."""
import math
import numpy as np
from tk_indi_reference import feedforward, G, IZ, skew

A_, B_, C_, D_ = 0.8, 0.6, 1.0, 0.25
w1, w2, w3 = 1.3, 0.9, 1.7
def xyz_ned(t):
    # derivatives 0..4 of an NED reference (z down)
    out = []
    for d in range(5):
        f = lambda w, c, ph: c * w**d * math.sin(w*t + ph + d*math.pi/2)
        out.append(np.array([f(w1, A_, 0), f(w2, B_, math.pi/2), -(C_ if d == 0 else 0) - f(w3, D_, 0)]))
    return out
psi = lambda t: 0.4*math.sin(0.7*t)
psid = lambda t: 0.4*0.7*math.cos(0.7*t)
psidd = lambda t: -0.4*0.7**2*math.sin(0.7*t)

def true_state(t):
    d = xyz_ned(t)
    tb = d[2] - G*IZ                        # tau * bz  (a = g iz + tau bz)
    tau = -np.linalg.norm(tb)
    bz = tb / tau
    xc = np.array([math.cos(psi(t)), math.sin(psi(t)), 0.0])
    # paper yaw (11): heading of the horizontal projection of b_x is psi
    lam = -(xc @ bz) / (IZ @ bz)
    bx = xc + lam*IZ; bx /= np.linalg.norm(bx)
    by = np.cross(bz, bx)
    return np.column_stack([bx, by, bz]), tau

def omega(t, h=1e-5):
    R, _ = true_state(t); Rp, _ = true_state(t+h); Rm, _ = true_state(t-h)
    W = R.T @ (Rp - Rm) / (2*h)
    return np.array([W[2, 1], W[0, 2], W[1, 0]])

worst = 0.0
for t in np.linspace(0.3, 6.0, 12):
    h = 1e-4
    R, tau = true_state(t)
    Om = omega(t)
    taud = (true_state(t+h)[1] - true_state(t-h)[1]) / (2*h)
    Omd = (omega(t+h) - omega(t-h)) / (2*h)
    d = xyz_ned(t)
    Om_ref, Omd_ref, _ = feedforward(R, tau, taud, Om, d[3], d[4], psid(t), psidd(t))
    e1 = np.abs(Om_ref - Om).max(); e2 = np.abs(Omd_ref - Omd).max()
    worst = max(worst, e1, e2)
    print(f"t={t:4.1f}  |Om_ref-Om|={e1:.2e}  |Omd_ref-Omd|={e2:.2e}   (Om~{np.abs(Om).max():.2f}, Omd~{np.abs(Omd).max():.2f})")
assert worst < 5e-3, worst
print("OK worst", worst)
