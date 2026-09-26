#!/usr/bin/env python3
"""Numerical check of the onboard residual network against an independent reference.

The point of this test is that the *flight* code is the thing under test. It links the same
`residual_nn.rs` that is cross-compiled for the drone, drives the real CRTP-style weight-upload
protocol one parameter at a time, and compares the firmware's prediction against a NumPy
implementation written from the architecture description rather than from the Rust source.

2026-09-14: rewritten for the Neural-Swarm2 architecture port. The previous version validated
the older, smaller custom network (phi 6->16->16->8, rho 8->16->16->3, 987 weights, 3-vector
output, distance-based neighbour cutoff with near-field rescaling). None of that survives, so
every numerical assertion here is new. Three behavioural changes drive most of the rewrite:

* **The output is scalar and Z-only.** `compute_Fa` returns `(0, 0, rho(...))`; there is no
  x/y prediction to compare any more, and the clamp acts on a magnitude that is now just
  `|z|`. A test asserting a non-zero x or y would have been asserting a bug.
* **The ground-effect term is unconditional.** `phi_G` is evaluated every tick for every
  vehicle, with no neighbour to gate it on, so a lone drone with weights loaded does **not**
  predict zero. The old suite asserted exactly that ("zero with no neighbours"), which is now
  the wrong assertion -- it is replaced by a check that the lone-drone prediction equals the
  pure-ground-effect reference.
* **Neighbour inclusion is a three-term gate, not a distance cutoff.** `|dx| < 0.2 and
  |dy| < 0.2 and |dvx| < 1.5` -- asymmetric, and deliberately replicated from the reference
  rather than normalised into something tidier. Each of the three terms gets its own test,
  since a plausible "cleanup" of any one of them would silently change what the network sees.

Two failure modes carried over from the old suite, still worth catching:

* **Weight-layout drift.** The training pipeline flattens PyTorch tensors into one vector and
  the firmware indexes that vector by hand. Nothing in either language checks the two agree,
  so an off-by-one in a bias block would produce a plausible-looking but wrong prediction.
  Random weights make any such disagreement show up immediately. With five weight sets rather
  than two, and 19297 weights rather than 987, there is considerably more to get wrong.
* **A partially-arrived upload being accepted.** A dropped parameter packet must leave the
  network inert, not half-loaded -- a half-loaded network is indistinguishable from a badly
  trained one.

Run:  python3 host/test_residual_nn.py

Flash-resident parity (upload vs `residual_nn_flash`): `../tools/residual/test_flash_vs_upload.py`.

Needs the SIL bindings built *with the residual_nn feature*, which is OFF by default (it costs
~77 KB of .bss and overflows real firmware RAM):

    cd firmware_app && DRONE_PLATFORM=bl RUSTFLAGS="-C panic=abort" \
        cargo build --release --target x86_64-unknown-linux-gnu --features residual_nn
    cd ~/Desktop/crazyflie-firmware && make bindings_python

Without the feature, `rnn_service` is a no-op and `rnn_predict` returns zero. This script
detects that and fails loudly rather than reporting a pass on a network that was never there.
"""

import sys

import numpy as np

sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/build")
import cffirmware as fw  # noqa: E402

# ── Architecture, mirrored from residual_nn.rs / neuralswarm.py ──────────────
# phi_Net(input_dim): input_dim -> 25 -> 40 -> 40 -> H,  ReLU/ReLU/ReLU/LINEAR
# rho_Net:            H         -> 40 -> 40 -> 40 -> 1,  ReLU/ReLU/ReLU/LINEAR
HIDDEN = 20
PHI_IN_SL = 6          # neighbour relative [dx,dy,dz,dvx,dvy,dvz]
PHI_IN_G = 4           # ground effect      [0-z, -vx, -vy, -vz]


def phi_layers(input_dim):
    return [(input_dim, 25), (25, 40), (40, 40), (40, HIDDEN)]


RHO_LAYERS = [(HIDDEN, 40), (40, 40), (40, 40), (40, 1)]

N_PHI_SL = sum(i * o + o for i, o in phi_layers(PHI_IN_SL))   # 3675
N_PHI_G = sum(i * o + o for i, o in phi_layers(PHI_IN_G))     # 3625
N_RHO = sum(i * o + o for i, o in RHO_LAYERS)                 # 4161

# Contiguous layout: phi_S | phi_L | phi_G | rho_S | rho_L
OFF_PHI_S = 0
OFF_PHI_L = OFF_PHI_S + N_PHI_SL
OFF_PHI_G = OFF_PHI_L + N_PHI_SL
OFF_RHO_S = OFF_PHI_G + N_PHI_G
OFF_RHO_L = OFF_RHO_S + N_RHO
N_WEIGHTS = OFF_RHO_L + N_RHO                                 # 19297

MAX_NEIGHBOURS = 3
OUT_CLAMP = 8.0
GATE_DXY = 0.2
GATE_DVX = 1.5
GRAMS_TO_NEWTONS = 9.81 / 1000.0


def forward(w, off, layers, x, last_linear=True):
    """Run a phi_Net/rho_Net stack from the flat weight vector.

    Layout per layer, matching `layer_relu`/`layer_linear` in residual_nn.rs: `n_out` rows of
    `n_in` weights (PyTorch nn.Linear `W` is `[n_out, n_in]`), then the `n_out` biases.
    """
    x = np.asarray(x, np.float32)
    for k, (n_in, n_out) in enumerate(layers):
        W = w[off:off + n_out * n_in].reshape(n_out, n_in)
        off += n_out * n_in
        b = w[off:off + n_out]
        off += n_out
        x = (W @ x + b).astype(np.float32)
        if not (last_linear and k == len(layers) - 1):
            x = np.maximum(x, 0.0).astype(np.float32)
    return x


def reference(w, rel, own_z, own_vel, mass):
    """Fa = (0, 0, rho_S(phi_G(ground) + sum_j gated phi_S(x_12))), computed independently.

    Returns (vec3_acceleration, clamped_flag).
    """
    ground = [0.0 - own_z, -own_vel[0], -own_vel[1], -own_vel[2]]
    rho_input = forward(w, OFF_PHI_G, phi_layers(PHI_IN_G), ground)

    for dp, dv in rel[:MAX_NEIGHBOURS]:
        if not (abs(dp[0]) < GATE_DXY and abs(dp[1]) < GATE_DXY and abs(dv[0]) < GATE_DVX):
            continue
        x12 = [dp[0], dp[1], dp[2], dv[0], dv[1], dv[2]]
        rho_input = rho_input + forward(w, OFF_PHI_S, phi_layers(PHI_IN_SL), x12)

    faz_grams = float(forward(w, OFF_RHO_S, RHO_LAYERS, rho_input)[0])
    faz = faz_grams * GRAMS_TO_NEWTONS / mass

    if not np.isfinite(faz):
        return np.zeros(3, np.float32), True
    if abs(faz) > OUT_CLAMP:
        return np.array([0.0, 0.0, np.sign(faz) * OUT_CLAMP], np.float32), True
    return np.array([0.0, 0.0, faz], np.float32), False


# ── Firmware harness ────────────────────────────────────────────────────────
class Fw:
    """Drives controllerOutOfTree the way the simulator does, one millisecond at a time."""

    def __init__(self, own_pos=(0.0, 0.0, 1.0), own_vel=(0.0, 0.0, 0.0)):
        fw.controllerOutOfTreeInit()
        fw.oot_select_drone(0)
        self.tick = 0
        self.own_pos = np.array(own_pos, np.float32)
        self.own_vel = np.array(own_vel, np.float32)

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

    def unload(self):
        """Return the network to its never-uploaded state.

        `RNN` is a process-global static and `controllerOutOfTreeInit()` does NOT reset it, so
        constructing a fresh `Fw` does not undo a previous upload -- weights loaded by any
        earlier test stay loaded for the rest of the run. `begin_upload` clears `loaded` and
        zeroes the buffer, so starting an upload and never finishing it is the way back to
        inert. Any test that asserts inertness must call this first, or it is really asserting
        that no earlier test uploaded anything.
        """
        fw.cvar.g_rnn_n = N_WEIGHTS
        fw.cvar.g_rnn_begin = 1
        self.step()

    def ref(self, w, peer_pos, peer_vel=None, mass=None):
        """Reference prediction for peers at `peer_pos`, relative to this harness's own state."""
        if peer_vel is None:
            peer_vel = [np.zeros(3, np.float32)] * len(peer_pos)
        rel = [(np.asarray(p, np.float32) - self.own_pos,
                np.asarray(v, np.float32) - self.own_vel)
               for p, v in zip(peer_pos, peer_vel)]
        return reference(w, rel, float(self.own_pos[2]), self.own_vel,
                         mass if mass is not None else fw.cvar.g_indi_mass)


def main():
    rng = np.random.default_rng(0xC0FFEE)
    # Scale chosen empirically: large enough that a single gated-in neighbour moves the output
    # by ~3e-2 m/s^2 (so the gate tests below have real discriminating power -- at 0.02 the
    # neighbour term moved the prediction by ~1e-6 and any gate assertion would have passed
    # whether the gate worked or not), small enough that the output stays far below OUT_CLAMP
    # so the numerical comparisons test the network rather than the limiter.
    w = (rng.standard_normal(N_WEIGHTS) * 0.2).astype(np.float32)
    mass = float(fw.cvar.g_indi_mass)

    fails = []

    def check(name, ok, detail=""):
        print(f"  {'PASS' if ok else 'FAIL'}  {name}{('  ' + detail) if detail else ''}")
        if not ok:
            fails.append(name)

    print(f"N_WEIGHTS = {N_WEIGHTS}  (phi_S/phi_L {N_PHI_SL} each, phi_G {N_PHI_G}, "
          f"rho_S/rho_L {N_RHO} each)")
    print(f"g_indi_mass = {mass:.6f} kg")
    print()

    # 0. The feature must actually be compiled in. Without it rnn_service is a no-op, every
    #    prediction is zero, and several checks below would "pass" on a network that does not
    #    exist. Fail loudly here instead of reporting a meaningless green run.
    probe = Fw()
    if probe.upload(w) != 1:
        print("FATAL: weight upload did not complete -- the bindings were almost certainly "
              "built without the `residual_nn` Cargo feature.\n"
              "       Rebuild with --features residual_nn (see this file's docstring), then "
              "re-run `make bindings_python`.")
        return 2
    print("  PASS  residual_nn feature present (upload completes)")

    # 1. An un-uploaded network must be inert -- including the ground-effect term, which is
    #    otherwise unconditional.
    #
    #    Inertness is checked by poisoning the log variables with a sentinel and showing the
    #    controller does not overwrite it, NOT by asserting the log reads zero. The two are not
    #    the same thing, and the difference is a real property of the firmware worth stating:
    #    `rnn_pred_write` is called only from inside `rnn_predict`, which runs only while
    #    `g_rnn_ready != 0`. When the network is not ready the control path uses zero, but the
    #    LOGGED `rnn.pred_*` keeps whatever it last held. From a cold boot that is 0.0, so the
    #    distinction never shows up in a normal flight -- but it does if `ready` ever drops back
    #    to 0 mid-session (starting a fresh weight upload does exactly that), and it shows up
    #    here because the feature probe above already ran a prediction. A sentinel makes the
    #    test assert the property that actually matters: the predict path did not execute.
    #
    #    `unload()` is also required, because the probe left a complete weight set in the
    #    process-global RNN; see Fw.unload's docstring.
    SENTINEL = -12345.0
    h = Fw()
    h.unload()
    fw.cvar.g_rnn_pred_x = fw.cvar.g_rnn_pred_y = fw.cvar.g_rnn_pred_z = SENTINEL
    h.peers([(0.0, 0.0, 1.3)], 1000)
    h.step(2)
    check("inert before upload (predict path does not run)",
          np.allclose(h.pred(), SENTINEL), f"pred={h.pred()}")

    # 2. A dropped weight must be refused outright, not half-accepted.
    ready = h.upload(w, drop=N_WEIGHTS // 2)
    check("incomplete upload refused", ready == 0, f"ready={ready}")
    fw.cvar.g_rnn_pred_x = fw.cvar.g_rnn_pred_y = fw.cvar.g_rnn_pred_z = SENTINEL
    h.peers([(0.0, 0.0, 1.3)], 2000)
    h.step(2)
    check("inert after refused upload", np.allclose(h.pred(), SENTINEL), f"pred={h.pred()}")

    # 2b. Record the staleness behaviour explicitly rather than leaving it implicit in the two
    #     checks above: after a refused upload the log holds the sentinel, not 0.0. If this ever
    #     starts failing because the firmware learned to zero the log when not ready, that is an
    #     improvement -- update this check, do not restore the old behaviour.
    check("rnn.pred_* is stale (not zeroed) while not ready",
          abs(float(fw.cvar.g_rnn_pred_z) - SENTINEL) < 1e-6,
          f"pred_z={fw.cvar.g_rnn_pred_z}")

    # 3. Ground effect alone: no neighbours, weights loaded. Under the previous architecture
    #    this was exactly zero; under Neural-Swarm2 phi_G runs unconditionally, so the correct
    #    assertion is agreement with the pure-ground-effect reference AND a non-zero value.
    h = Fw(own_pos=(0.1, -0.2, 0.35), own_vel=(0.3, 0.0, -0.1))
    check("complete upload accepted", h.upload(w) == 1, f"ready={fw.cvar.g_rnn_ready}")
    h.peers([], 5000)
    h.step()
    ref, _ = h.ref(w, [])
    got = h.pred()
    check("ground effect only (no neighbours)", np.allclose(got, ref, atol=2e-4),
          f"fw={got[2]:+.6f} ref={ref[2]:+.6f}")
    check("ground effect is non-zero", abs(ref[2]) > 1e-6, f"ref_z={ref[2]:+.6f}")

    # 4. Output is Z-only: compute_Fa returns (0, 0, faz) by construction.
    check("prediction is z-only", got[0] == 0.0 and got[1] == 0.0,
          f"x={got[0]} y={got[1]}")

    # 5. Two neighbours inside the gate. First sample has no previous timestamp, so the
    #    relative velocity is exactly -own_vel.
    h = Fw(own_pos=(0.1, -0.2, 1.0), own_vel=(0.3, 0.0, -0.1))
    h.upload(w)
    p1 = [np.array([0.15, -0.20, 1.32], np.float32),
          np.array([0.05, -0.25, 0.72], np.float32)]
    h.peers(p1, 5000)
    h.step()
    ref, _ = h.ref(w, p1)
    got = h.pred()
    check("2 neighbours in gate, first sample", np.allclose(got, ref, atol=2e-4),
          f"fw={got[2]:+.6f} ref={ref[2]:+.6f}")

    # 6. Second sample 100 ms later: peer velocity is differenced from the timestamps.
    p2 = [p + np.array([0.02, 0.00, -0.01], np.float32) for p in p1]
    h.peers(p2, 5100)
    h.step()
    peer_v = [(b - a) * 10.0 for a, b in zip(p1, p2)]  # 100 ms -> x10
    ref, _ = h.ref(w, p2, peer_v)
    got = h.pred()
    check("differenced peer velocity", np.allclose(got, ref, atol=2e-4),
          f"fw={got[2]:+.6f} ref={ref[2]:+.6f}")

    # 7. Permutation invariance -- the property deep sets exist for.
    h.peers(list(reversed(p2)), 6000)
    h.step()
    a = h.pred()
    h.peers(p2, 7000)
    h.step()
    b = h.pred()
    check("permutation invariant", np.allclose(a, b, atol=2e-4), f"{a[2]:+.6f} vs {b[2]:+.6f}")

    # 8-10. The proximity gate, one term at a time. Each neighbour below sits just outside one
    #       of the three bounds and inside the other two, so it must contribute nothing -- the
    #       prediction has to equal the ground-effect-only value. These are separate checks
    #       because the gate is asymmetric (dx, dy, dvx -- not dz, dvy, dvz) and any attempt to
    #       "regularise" it would break exactly one of them.
    def gate_case(name, peer_offset, own_vel=(0.0, 0.0, 0.0), peer_vel=None, t0=5000):
        g = Fw(own_pos=(0.0, 0.0, 1.0), own_vel=own_vel)
        g.upload(w)
        g.peers([], t0)
        g.step()
        alone = g.pred()
        pos = np.array([0.0, 0.0, 1.0], np.float32) + np.asarray(peer_offset, np.float32)
        if peer_vel is None:
            g.peers([pos], t0 + 100)
            g.step()
        else:
            # Two samples 100 ms apart to give the peer the intended velocity.
            prev = pos - np.asarray(peer_vel, np.float32) * 0.1
            g.peers([prev], t0 + 100)
            g.step()
            g.peers([pos], t0 + 200)
            g.step()
        check(name, np.allclose(g.pred(), alone, atol=2e-4),
              f"with={g.pred()[2]:+.6f} alone={alone[2]:+.6f}")

    gate_case("gate excludes |dx| >= 0.2", (0.25, 0.0, 0.30))
    gate_case("gate excludes |dy| >= 0.2", (0.0, 0.25, 0.30))
    gate_case("gate excludes |dvx| >= 1.5", (0.05, 0.05, 0.30), peer_vel=(2.0, 0.0, 0.0))

    # 11. A neighbour just inside every bound must change the prediction -- otherwise the three
    #     checks above would also pass with the neighbour term accidentally disabled entirely.
    g = Fw(own_pos=(0.0, 0.0, 1.0))
    g.upload(w)
    g.peers([], 5000)
    g.step()
    alone = g.pred()
    g.peers([np.array([0.10, 0.10, 1.30], np.float32)], 5100)
    g.step()
    near = g.pred()
    check("neighbour inside gate changes prediction",
          not np.allclose(near, alone, atol=1e-5),
          f"with={near[2]:+.6f} alone={alone[2]:+.6f}")

    # 12. dz is deliberately NOT gated: a neighbour far above, but aligned in x/y, still counts.
    g.peers([np.array([0.05, 0.05, 2.60], np.float32)], 6000)
    g.step()
    check("dz is not gated", not np.allclose(g.pred(), alone, atol=1e-5),
          f"with={g.pred()[2]:+.6f} alone={alone[2]:+.6f}")

    # 13. Unit conversion: the network's raw output is in the reference's equivalent-grams unit,
    #     converted grams -> N -> /mass exactly once. Recomputing that chain by hand from the
    #     firmware's own mass parameter catches a dropped or doubled conversion, which a
    #     self-consistent reference would not.
    h = Fw(own_pos=(0.0, 0.0, 0.40))
    h.upload(w)
    h.peers([], 5000)
    h.step()
    ground = [0.0 - 0.40, 0.0, 0.0, 0.0]
    rho_in = forward(w, OFF_PHI_G, phi_layers(PHI_IN_G), ground)
    grams = float(forward(w, OFF_RHO_S, RHO_LAYERS, rho_in)[0])
    expect = grams * GRAMS_TO_NEWTONS / mass
    check("grams -> N -> /mass conversion applied once",
          abs(h.pred()[2] - expect) < 2e-4,
          f"fw={h.pred()[2]:+.6f} hand={expect:+.6f} (raw {grams:+.3f} g)")

    # 14. The clamp must engage rather than let an absurd network command an absurd acceleration.
    big = (w * 80.0).astype(np.float32)
    h = Fw(own_pos=(0.0, 0.0, 0.30))
    h.upload(big)
    h.peers([np.array([0.05, 0.0, 0.45], np.float32)], 5000)
    h.step()
    mag = abs(float(h.pred()[2]))
    check("output clamped", fw.cvar.g_rnn_clamped == 1 and mag <= OUT_CLAMP + 1e-3,
          f"|pred_z|={mag:.3f} clamped={fw.cvar.g_rnn_clamped}")

    # 15. More neighbours than MAX_NEIGHBOURS: the extras are dropped by the firmware's fixed
    #     peer buffer, which is a systems limit rather than part of the architecture. The
    #     reference is given the same truncation, so this checks the two agree on which three.
    h = Fw(own_pos=(0.0, 0.0, 1.0))
    h.upload(w)
    many = [np.array([0.05, 0.05, 1.20], np.float32),
            np.array([-0.05, 0.05, 0.80], np.float32),
            np.array([0.05, -0.05, 1.40], np.float32),
            np.array([-0.05, -0.05, 0.60], np.float32)]
    h.peers(many, 5000)
    h.step()
    ref, _ = h.ref(w, many)          # reference truncates at MAX_NEIGHBOURS too
    check("respects MAX_NEIGHBOURS", np.allclose(h.pred(), ref, atol=2e-4),
          f"fw={h.pred()[2]:+.6f} ref={ref[2]:+.6f}")

    print()
    if fails:
        print(f"{len(fails)} FAILED: {', '.join(fails)}")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
