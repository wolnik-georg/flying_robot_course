#!/usr/bin/env python3
"""End-to-end check of the training pipeline against the COMPILED firmware.

What this proves: a model trained in PyTorch, normalised, folded, flattened and uploaded through
the real weight protocol computes the same thing on the drone as it did in training. Every step
in that chain is a place where a silent disagreement can hide -- a transposed weight block, a
fold applied to the wrong layer, a gate implemented differently in two languages -- and none of
them crashes when it is wrong. They produce a plausible number.

2026-09-14: rewritten for the Neural-Swarm2 architecture port. The previous version tested the
older custom network (987 weights, 3-vector output, distance cutoff) and, through `dataset.py`,
the merged-CSV loader.

SCOPE, stated plainly: this file no longer exercises `dataset.py`. That module still produces
the old shapes -- a 3-vector target and no ground-effect input -- and rewriting it needs
decisions that are not the test's to make (see README). Everything here is therefore the
model <-> firmware contract, driven by synthetic tensors generated in this file. The loader
tests that used to live here (peer-minus-own convention, both drones as ego, dropping
`a_res == 0` samples) are NOT covered anywhere right now. That is a real gap, not an omission
by choice.

Run:  python3 test_pipeline.py

Flash vs upload parity (separate script): `test_flash_vs_upload.py` (rebuilds bindings twice).

Needs torch, and the SIL bindings built *with the residual_nn feature* (off by default):

    cd firmware_app && DRONE_PLATFORM=bl RUSTFLAGS="-C panic=abort" \\
        cargo build --release --target x86_64-unknown-linux-gnu --features residual_nn
    cd ~/Desktop/crazyflie-firmware && make bindings_python

Use system python3 -- it has torch and the bindings; the pyenv `flying_robots` env has neither.
"""

import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/build")

import cffirmware as fw  # noqa: E402
import model as M  # noqa: E402

FAILS = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{('  ' + detail) if detail else ''}")
    if not ok:
        FAILS.append(name)


# ── Synthetic problem ───────────────────────────────────────────────────────
def synthetic(n=4000, k=M.MAX_NEIGHBOURS, seed=0):
    """A downwash-shaped target, for exercising the pipeline with no flight data.

    Deliberately crude: a neighbour directly above pushes down, the effect decays with vertical
    separation, and being near the ground pushes up. The point is a learnable signal with the
    right *signs*, not a physical model -- nothing here is a result.

    Returns (rel, present, ground, y_grams).
    """
    rng = np.random.default_rng(seed)
    rel = np.zeros((n, k, 6), np.float32)
    present = np.zeros((n, k), np.float32)
    n_nb = rng.integers(0, k + 1, size=n)

    own_z = rng.uniform(0.15, 1.6, size=n).astype(np.float32)
    own_v = rng.normal(0.0, 0.25, size=(n, 3)).astype(np.float32)

    for i in range(n):
        for j in range(n_nb[i]):
            # Biased towards the gate so a useful fraction of samples actually reach the network.
            dp = np.array([rng.normal(0.0, 0.15), rng.normal(0.0, 0.15),
                           rng.uniform(-0.8, 0.8)], np.float32)
            dv = rng.normal(0.0, 0.4, size=3).astype(np.float32)
            rel[i, j] = np.concatenate([dp, dv])
            present[i, j] = 1.0

    ground = np.stack([0.0 - own_z, -own_v[:, 0], -own_v[:, 1], -own_v[:, 2]], axis=1)

    mask = M.build_mask(rel, present)
    a = np.zeros(n, np.float64)
    for i in range(n):
        for j in range(k):
            if mask[i, j] == 0:
                continue
            dz = rel[i, j, 2]
            if dz > 0:                       # neighbour above -> pushed down
                a[i] -= 3.0 * np.exp(-abs(dz) / 0.35)
        a[i] += 1.5 * np.exp(-own_z[i] / 0.20)    # ground effect pushes up
    a += rng.normal(0.0, 0.02, size=n)

    return (rel, present, ground.astype(np.float32),
            M.accel_to_grams(a).astype(np.float32))


class FwHarness:
    """Drives controllerOutOfTree the way the simulator does, one millisecond at a time."""

    def __init__(self, own_pos=(0.0, 0.0, 1.0), own_vel=(0.0, 0.0, 0.0)):
        fw.controllerOutOfTreeInit()
        fw.oot_select_drone(0)
        self.tick = 0
        self.own_pos = np.array(own_pos, np.float32)
        self.own_vel = np.array(own_vel, np.float32)

        self.sp = fw.setpoint_t()
        self.sp.position.x, self.sp.position.y, self.sp.position.z = own_pos
        self.sp.mode.x = self.sp.mode.y = self.sp.mode.z = fw.modeAbs

        self.st = fw.state_t()
        self.st.position.x, self.st.position.y, self.st.position.z = own_pos
        self.st.velocity.x, self.st.velocity.y, self.st.velocity.z = own_vel
        q = self.st.attitudeQuaternion
        q.x, q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0

        self.sens = fw.sensorData_t()
        self.sens.acc.z = 1.0
        self.ctl = fw.control_t()

    def step(self, n=1):
        for _ in range(n):
            self.tick += 1
            fw.controllerOutOfTree(self.ctl, self.sp, self.sens, self.st, self.tick)

    def pred_z(self):
        """The last logged prediction.

        This reads a process-global that the controller overwrites on every tick, from ANY
        harness -- the firmware has one log variable, not one per Python object. Capture the
        value immediately after the step that produced it; comparing `a.pred_z()` against
        `b.pred_z()` after both have run reads the same number twice.
        """
        return float(fw.cvar.g_rnn_pred_z)

    def upload(self, w):
        fw.cvar.g_rnn_n = len(w)
        fw.cvar.g_rnn_begin = 1
        self.step()
        for i, v in enumerate(w):
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
    torch.manual_seed(0)
    np.random.seed(0)
    mass = float(fw.cvar.g_indi_mass)

    print(f"N_WEIGHTS = {M.N_WEIGHTS}  (phi_S/phi_L {M.N_PHI_SL}, phi_G {M.N_PHI_G}, "
          f"rho_S/rho_L {M.N_RHO})")
    print(f"g_indi_mass = {mass:.6f} kg")
    print()

    rel, present, ground, y = synthetic()
    mask = M.build_mask(rel, present)
    print(f"synthetic: {len(y)} samples, {int(mask.sum())} gated-in neighbour terms, "
          f"target {M.grams_to_accel(y).std():.3f} m/s^2 rms")

    # ── Train briefly. The fit quality is not under test; the plumbing is. ──
    net = M.NeuralSwarm2()
    opt = torch.optim.Adam(net.parameters(), lr=1e-3)
    # Normalisation is applied here and folded into the first layers at export, so the trained
    # network and the shipped network are arithmetically the same function.
    flat = rel.reshape(-1, M.PHI_IN_SL)[mask.reshape(-1) > 0]
    mu_r, sd_r = flat.mean(0), np.maximum(flat.std(0), 1e-6)
    mu_g, sd_g = ground.mean(0), np.maximum(ground.std(0), 1e-6)

    rel_n = ((rel - mu_r) / sd_r).astype(np.float32) * mask[..., None]
    g_n = ((ground - mu_g) / sd_g).astype(np.float32)

    X = torch.from_numpy(rel_n)
    Mk = torch.from_numpy(mask)
    G = torch.from_numpy(g_n)
    Y = torch.from_numpy(y)
    for ep in range(400):
        opt.zero_grad()
        loss = torch.nn.functional.smooth_l1_loss(net(X, Mk, G), Y, beta=2.0)
        loss.backward()
        opt.step()
    print(f"trained: final loss {float(loss):.4f} (plumbing test, not a result)")
    print()

    # 1. Layout: the flat vector must be the length the firmware indexes.
    exported = M.fold_normalisation(net, mu_r, sd_r, mu_g, sd_g)
    w = exported.flatten()
    check("flatten produces the expected length", w.size == M.N_WEIGHTS, f"{w.size}")

    # 2. flatten/load_flat round trip -- the layout is its own inverse or it is wrong.
    rt = M.NeuralSwarm2()
    rt.load_flat(w)
    check("flatten -> load_flat is lossless",
          np.abs(rt.flatten() - w).max() == 0.0,
          f"max |diff| = {np.abs(rt.flatten() - w).max():.2e}")

    # 3. The untrained large-vehicle path ships as zeros, and says so rather than looking trained.
    check("phi_L / rho_L exported as zeros",
          np.all(w[M.OFF_PHI_L:M.OFF_PHI_G] == 0) and np.all(w[M.OFF_RHO_L:] == 0))

    # 4. The fold is exact: the trained model on normalised inputs must equal the exported model
    #    on raw inputs. This is where a mis-derived fold shows up.
    with torch.no_grad():
        ref = net(X[:256], Mk[:256], G[:256]).numpy()
        got = exported(torch.from_numpy(rel[:256]) * torch.from_numpy(mask[:256]).unsqueeze(-1),
                       Mk[:256], torch.from_numpy(ground[:256])).numpy()
    check("normalisation fold is exact", np.allclose(ref, got, atol=2e-2),
          f"max |diff| = {np.abs(ref - got).max():.3e} grams")

    # 5. torch == the NumPy firmware_forward on the exported vector, gate and units included.
    worst = 0.0
    for i in range(64):
        peers = [(rel[i, j, :3], rel[i, j, 3:]) for j in range(M.MAX_NEIGHBOURS)
                 if present[i, j] > 0]
        own_z = -float(ground[i, 0])
        own_v = -ground[i, 1:]
        a_np, _ = M.firmware_forward(w, peers, own_z, own_v, mass=mass, apply_clamp=False)
        with torch.no_grad():
            grams = float(exported(torch.from_numpy(rel[i:i + 1]),
                                   torch.from_numpy(mask[i:i + 1]),
                                   torch.from_numpy(ground[i:i + 1]))[0])
        worst = max(worst, abs(a_np - M.grams_to_accel(grams, mass)))
    check("torch == firmware_forward on the exported vector", worst < 1e-4,
          f"max |diff| = {worst:.2e} m/s^2")

    # 6. The one that matters: upload into the COMPILED controller and compare. Zero relative
    #    velocity, because a first peer sample has no previous timestamp -- which is also
    #    exactly what the firmware does on the first tick after takeoff.
    own_pos = (0.0, 0.0, 0.8)
    h = FwHarness(own_pos=own_pos, own_vel=(0.0, 0.0, 0.0))
    check("firmware accepted the exported weights", h.upload(w) == 1,
          f"ready={fw.cvar.g_rnn_ready}")

    worst, t = 0.0, 5000
    for i in range(24):
        peers_rel = [rel[i, j, :3] for j in range(M.MAX_NEIGHBOURS) if present[i, j] > 0]
        pts = [np.asarray(own_pos, np.float32) + d for d in peers_rel]
        t += 1000
        h.peers(pts, t)
        h.step()
        ref_a, _ = M.firmware_forward(w, [(d, np.zeros(3, np.float32)) for d in peers_rel],
                                      own_pos[2], np.zeros(3, np.float32), mass=mass)
        worst = max(worst, abs(h.pred_z() - ref_a))
    check("compiled firmware == reference, zero peer velocity", worst < 2e-4,
          f"max |diff| = {worst:.2e} m/s^2")

    # 7. With a differenced peer velocity, which is the flight case: two peer samples 100 ms
    #    apart, so the firmware reconstructs a known relative velocity itself.
    h = FwHarness(own_pos=own_pos, own_vel=(0.1, -0.05, 0.0))
    h.upload(w)
    worst, t = 0.0, 20000
    for i in range(24):
        peers_rel = [rel[i, j, :3] for j in range(M.MAX_NEIGHBOURS) if present[i, j] > 0]
        if not peers_rel:
            continue
        vel = [np.array([0.2, -0.1, 0.05], np.float32)] * len(peers_rel)
        first = [np.asarray(own_pos, np.float32) + d for d in peers_rel]
        second = [p + v * 0.1 for p, v in zip(first, vel)]
        t += 1000
        h.peers(first, t)
        h.step()
        h.peers(second, t + 100)
        h.step()
        own_v = np.array([0.1, -0.05, 0.0], np.float32)
        ref_a, _ = M.firmware_forward(
            w, [(p - np.asarray(own_pos, np.float32), v - own_v)
                for p, v in zip(second, vel)],
            own_pos[2], own_v, mass=mass)
        worst = max(worst, abs(h.pred_z() - ref_a))
    check("compiled firmware == reference, differenced peer velocity", worst < 2e-4,
          f"max |diff| = {worst:.2e} m/s^2")

    # 8. Sign convention, checked against the firmware rather than asserted. A neighbour ABOVE
    #    the ego vehicle must produce downward acceleration, because that is what the synthetic
    #    ground truth encodes and what downwash does. Sign is what is under test, not magnitude:
    #    this model is fitted in 400 steps on 4000 made-up samples.
    h = FwHarness(own_pos=(0.0, 0.0, 1.0))
    h.upload(w)
    h.peers([np.array([0.02, 0.02, 1.25], np.float32)], 40000)
    h.step()
    above = h.pred_z()
    h.peers([np.array([0.02, 0.02, 0.75], np.float32)], 41000)
    h.step()
    below = h.pred_z()
    check("neighbour above pushes the ego vehicle down", above < 0.0, f"a_z={above:+.3f}")
    check("neighbour below pushes less than one above", below > above,
          f"below={below:+.3f} above={above:+.3f}")

    # 9. Ground effect: the reference models it, so low altitude must differ from high, with no
    #    neighbours present at all. Under the previous architecture this was identically zero.
    #     `pred_z()` reads a process-global log variable, so each value must be captured
    #     immediately after its own step -- reading both at the end returns the last one twice.
    h = FwHarness(own_pos=(0.0, 0.0, 1.0))
    h.upload(w)
    h.peers([], 50000)

    def at_height(z):
        h.st.position.z = z
        h.sp.position.z = z
        h.step(2)
        return h.pred_z()

    lo = at_height(0.18)
    hi = at_height(1.50)
    check("ground effect present with no neighbours", abs(lo - hi) > 1e-3,
          f"z=0.18 -> {lo:+.4f}, z=1.50 -> {hi:+.4f}")
    check("ground effect pushes up near the floor", lo > hi,
          f"low={lo:+.4f} high={hi:+.4f}")

    # 10. The clamp survives the pipeline: absurd weights must not command absurd acceleration.
    h = FwHarness(own_pos=(0.0, 0.0, 0.30))
    h.upload((w * 400.0).astype(np.float32))
    h.peers([np.array([0.02, 0.02, 0.45], np.float32)], 60000)
    h.step()
    check("output clamp still engages after a pipeline upload",
          fw.cvar.g_rnn_clamped == 1 and abs(h.pred_z()) <= M.OUT_CLAMP + 1e-3,
          f"|a_z|={abs(h.pred_z()):.3f} clamped={fw.cvar.g_rnn_clamped}")

    print()
    if FAILS:
        print(f"{len(FAILS)} FAILED: {', '.join(FAILS)}")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
