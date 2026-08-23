#!/usr/bin/env python3
"""Numerical check of the onboard residual network against an independent reference.

The point of this test is that the *flight* code is the thing under test. It links the same
`residual_nn.rs` that is cross-compiled for the drone, drives the real CRTP-style weight-upload
protocol one parameter at a time, and compares the firmware's prediction against a NumPy
implementation written from the architecture description rather than from the Rust source.

Two failure modes it is specifically built to catch:

* **Weight-layout drift.** The training pipeline flattens PyTorch tensors into a single vector,
  and the firmware indexes that vector by hand. Nothing in either language checks that the two
  agree, so an off-by-one in a bias block would silently produce a plausible-looking but wrong
  prediction. Random weights make any such disagreement show up immediately.
* **A partially-arrived upload being accepted.** A dropped parameter packet must leave the
  network inert, not half-loaded -- a half-loaded network is indistinguishable from a badly
  trained one.

Run:  python3 host/test_residual_nn.py
Needs the SIL bindings built:  cd ~/Desktop/crazyflie-firmware && make bindings_python
"""

import sys

import numpy as np

sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/build")
import cffirmware as fw  # noqa: E402

# ── Architecture, mirrored from residual_nn.rs ──────────────────────────────
PHI = [(6, 16), (16, 16), (16, 8)]          # (n_in, n_out) per layer, all ReLU
RHO = [(8, 16), (16, 16), (16, 3)]          # last layer linear
LATENT = 8
MAX_NEIGHBOURS = 3
OUT_CLAMP = 8.0
MIN_DIST, MAX_DIST = 0.04, 2.0

N_WEIGHTS = sum(i * o + o for i, o in PHI + RHO)


def unpack(w):
    """Split the flat vector into (W, b) per layer, in upload order."""
    out, off = [], 0
    for n_in, n_out in PHI + RHO:
        W = w[off:off + n_out * n_in].reshape(n_out, n_in)
        off += n_out * n_in
        b = w[off:off + n_out]
        off += n_out
        out.append((W, b))
    assert off == N_WEIGHTS
    return out


def reference(w, rel):
    """a_res = rho(sum_j phi(rel_j)), computed independently of the Rust code."""
    layers = unpack(w)
    latent, any_used = np.zeros(LATENT, dtype=np.float32), False

    for dp, dv in rel[:MAX_NEIGHBOURS]:
        d = float(np.linalg.norm(dp))
        if d > MAX_DIST:
            continue
        scale = MIN_DIST / d if (d < MIN_DIST and d > 1e-6) else 1.0
        x = np.concatenate([np.asarray(dp, np.float32) * scale,
                            np.asarray(dv, np.float32)]).astype(np.float32)
        for W, b in layers[:3]:
            x = np.maximum(W @ x + b, 0.0).astype(np.float32)
        latent += x
        any_used = True

    if not any_used:
        return np.zeros(3, np.float32), False

    x = latent
    for W, b in layers[3:5]:
        x = np.maximum(W @ x + b, 0.0).astype(np.float32)
    W, b = layers[5]
    y = (W @ x + b).astype(np.float32)

    mag = float(np.linalg.norm(y))
    if mag > OUT_CLAMP:
        return (y * (OUT_CLAMP / mag)).astype(np.float32), True
    return y, False


# ── Firmware harness ────────────────────────────────────────────────────────
class Fw:
    """Drives controllerOutOfTree the way the simulator does, one millisecond at a time."""

    def __init__(self, own_pos=(0.0, 0.0, 1.0), own_vel=(0.0, 0.0, 0.0)):
        fw.controllerOutOfTreeInit()
        fw.oot_select_drone(0)
        self.tick = 0

        self.sp = fw.setpoint_t()
        # Above the 0.05 m arming threshold, otherwise the controller returns before predicting.
        self.sp.position.x, self.sp.position.y, self.sp.position.z = own_pos
        self.sp.mode.x = self.sp.mode.y = self.sp.mode.z = fw.modeAbs

        self.st = fw.state_t()
        self.st.position.x, self.st.position.y, self.st.position.z = own_pos
        self.st.velocity.x, self.st.velocity.y, self.st.velocity.z = own_vel
        q = self.st.attitudeQuaternion
        q.x, q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0

        self.sens = fw.sensorData_t()
        self.sens.acc.z = 1.0  # 1 g, level and stationary
        self.ctl = fw.control_t()

    def step(self, n=1):
        for _ in range(n):
            self.tick += 1
            fw.controllerOutOfTree(self.ctl, self.sp, self.sens, self.st, self.tick)

    def pred(self):
        return np.array([fw.cvar.g_rnn_pred_x, fw.cvar.g_rnn_pred_y,
                         fw.cvar.g_rnn_pred_z], np.float32)

    def upload(self, w, drop=None):
        """Push weights through the real protocol. `drop` omits one index, simulating loss."""
        fw.cvar.g_rnn_n = len(w)
        fw.cvar.g_rnn_begin = 1
        self.step()
        for i, v in enumerate(w):
            if i == drop:
                continue
            fw.cvar.g_rnn_wi = i
            fw.cvar.g_rnn_wv = float(v)
            fw.cvar.g_rnn_wc = 1
            self.step()
        fw.cvar.g_rnn_end = 1
        self.step()
        return fw.cvar.g_rnn_ready

    def peers(self, pts, t_ms):
        for i, p in enumerate(pts):
            fw.oot_set_peer(i, float(p[0]), float(p[1]), float(p[2]), int(t_ms))
        fw.oot_set_peer_count(len(pts))


def main():
    rng = np.random.default_rng(0xC0FFEE)
    # Xavier-ish scaling: large enough to exercise the arithmetic, small enough that the output
    # lands below OUT_CLAMP so the comparison tests the network and not the limiter.
    w = (rng.standard_normal(N_WEIGHTS) * 0.15).astype(np.float32)

    fails = []

    def check(name, ok, detail=""):
        print(f"  {'PASS' if ok else 'FAIL'}  {name}{('  ' + detail) if detail else ''}")
        if not ok:
            fails.append(name)

    print(f"N_WEIGHTS = {N_WEIGHTS}")

    # 1. An un-uploaded network must be inert.
    h = Fw()
    h.peers([(0.0, 0.0, 1.3)], 1000)
    h.step(2)
    check("inert before upload", np.allclose(h.pred(), 0.0), f"pred={h.pred()}")

    # 2. A dropped weight must be refused outright, not half-accepted.
    ready = h.upload(w, drop=N_WEIGHTS // 2)
    check("incomplete upload refused", ready == 0, f"ready={ready}")
    h.peers([(0.0, 0.0, 1.3)], 2000)
    h.step(2)
    check("inert after refused upload", np.allclose(h.pred(), 0.0), f"pred={h.pred()}")

    # 3. Complete upload, then numerical agreement.
    h = Fw(own_pos=(0.1, -0.2, 1.0), own_vel=(0.3, 0.0, -0.1))
    ready = h.upload(w)
    check("complete upload accepted", ready == 1, f"ready={ready}")

    own_p = np.array([0.1, -0.2, 1.0], np.float32)
    own_v = np.array([0.3, 0.0, -0.1], np.float32)

    # First sample: no previous timestamp, so relative velocity must be exactly zero.
    p1 = [np.array([0.15, -0.20, 1.32], np.float32),
          np.array([-0.30, 0.25, 0.72], np.float32)]
    h.peers(p1, 5000)
    h.step()
    ref, _ = reference(w, [(p - own_p, -own_v) for p in p1])
    got = h.pred()
    check("2 neighbours, first sample (zero peer velocity)",
          np.allclose(got, ref, atol=2e-4), f"fw={got} ref={ref}")

    # Second sample 100 ms later: peer velocity is differenced from the timestamps.
    p2 = [p + np.array([0.02, 0.00, -0.01], np.float32) for p in p1]
    h.peers(p2, 5100)
    h.step()
    peer_v = [(b - a) * 10.0 for a, b in zip(p1, p2)]  # 100 ms -> x10
    ref, _ = reference(w, [(p - own_p, v - own_v) for p, v in zip(p2, peer_v)])
    got = h.pred()
    check("2 neighbours, differenced peer velocity",
          np.allclose(got, ref, atol=2e-4), f"fw={got} ref={ref}")

    # 4. Permutation invariance -- the property deep sets exist for.
    h.peers(list(reversed(p2)), 6000)
    h.step()
    a = h.pred()
    h.peers(p2, 7000)
    h.step()
    b = h.pred()
    check("permutation invariant", np.allclose(a, b, atol=2e-4), f"{a} vs {b}")

    # 5. Three neighbours, and one beyond MAX_DIST that must contribute nothing.
    h = Fw()
    h.upload(w)
    near = [np.array([0.10, 0.05, 1.30], np.float32),
            np.array([-0.15, 0.10, 0.70], np.float32)]
    far = np.array([0.0, 3.0, 1.0], np.float32)
    own_p = np.array([0.0, 0.0, 1.0], np.float32)
    h.peers(near + [far], 5000)
    h.step()
    with_far = h.pred()
    h = Fw()
    h.upload(w)
    h.peers(near, 5000)
    h.step()
    without_far = h.pred()
    check("distant neighbour ignored", np.allclose(with_far, without_far, atol=2e-4),
          f"{with_far} vs {without_far}")

    # 6. The clamp must engage rather than let an absurd network command an absurd acceleration.
    big = (w * 60.0).astype(np.float32)
    h = Fw()
    h.upload(big)
    h.peers([np.array([0.05, 0.0, 1.15], np.float32)], 5000)
    h.step()
    mag = float(np.linalg.norm(h.pred()))
    check("output clamped", fw.cvar.g_rnn_clamped == 1 and mag <= OUT_CLAMP + 1e-3,
          f"|pred|={mag:.3f} clamped={fw.cvar.g_rnn_clamped}")

    # 7. No neighbours at all -- the single-drone case, which must stay exactly zero.
    h.peers([], 5100)
    h.step()
    check("zero with no neighbours", np.allclose(h.pred(), 0.0), f"pred={h.pred()}")

    print()
    if fails:
        print(f"{len(fails)} FAILED: {', '.join(fails)}")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
