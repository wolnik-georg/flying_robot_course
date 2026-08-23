#!/usr/bin/env python3
"""End-to-end check of the training pipeline against the compiled firmware.

Step 1's test (`firmware_app/host/test_residual_nn.py`) proved the firmware evaluates a given
weight vector correctly. It said nothing about whether *this pipeline produces that vector*.
That is the gap this file closes, and it is the gap where the expensive bug lives: PyTorch stores
its parameters in its own layout, the firmware indexes a flat array by hand, and normalisation is
folded in between. Every one of those steps can be wrong in a way that still yields smooth,
believable numbers.

So the check runs a real trained model through the real export and the real upload protocol into
the real compiled controller, and compares against PyTorch's own forward pass:

    torch model  ->  fold normalisation  ->  flatten  ->  rnn.* param upload  ->  firmware
                                                                                     |
    torch forward on normalised inputs  ==============================================

Run:  python3 test_pipeline.py
Needs torch and the SIL bindings (both on system python3).
"""

import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/build")

import cffirmware as fw  # noqa: E402

import dataset  # noqa: E402
from model import (DeepSets, N_WEIGHTS, MAX_NEIGHBOURS, OUT_CLAMP,  # noqa: E402
                   fold_normalisation, firmware_forward)

FAILS = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{('  ' + detail) if detail else ''}")
    if not ok:
        FAILS.append(name)


class Sil:
    """The compiled controller, driven a millisecond at a time, as the simulator drives it."""

    def __init__(self, own_pos=(0.0, 0.0, 1.0), own_vel=(0.0, 0.0, 0.0)):
        fw.controllerOutOfTreeInit()
        fw.oot_select_drone(0)
        self.tick = 0
        self.own_pos = np.asarray(own_pos, np.float32)
        self.own_vel = np.asarray(own_vel, np.float32)

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

    def upload(self, w):
        """The real rnn.* protocol, one weight per tick, exactly as the ROS uploader does."""
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

    def predict(self, rel_row, mask_row, t_ms):
        """Present one sample's neighbours as absolute peer positions and read the prediction.

        Peer POSITIONS are what the firmware receives, so the relative state has to be turned
        back into absolutes here. Relative velocity cannot be injected at all -- the firmware
        differences it from consecutive peer samples -- so callers that need a specific relative
        velocity must present two samples; see `test_with_velocity`.
        """
        n = 0
        for k in range(MAX_NEIGHBOURS):
            if mask_row[k] <= 0:
                continue
            p = self.own_pos + rel_row[k, :3]
            fw.oot_set_peer(n, float(p[0]), float(p[1]), float(p[2]), int(t_ms))
            n += 1
        fw.oot_set_peer_count(n)
        self.step()
        return np.array([fw.cvar.g_rnn_pred_x, fw.cvar.g_rnn_pred_y,
                         fw.cvar.g_rnn_pred_z], np.float32)


def main():
    rng = np.random.default_rng(7)
    torch.manual_seed(7)

    print("training a small model on synthetic data (mechanism test, not a result)")
    rel, mask, y = dataset.synthetic(n=4000, seed=1)
    mu, sigma = dataset.normalisation(rel, mask)
    rel_n = ((rel - mu) / sigma).astype(np.float32) * mask[..., None]

    model = DeepSets()
    opt = torch.optim.Adam(model.parameters(), lr=2e-3)
    X, M, Y = (torch.from_numpy(rel_n), torch.from_numpy(mask), torch.from_numpy(y))
    for _ in range(600):
        opt.zero_grad()
        loss = torch.nn.functional.smooth_l1_loss(model(X, M), Y)
        loss.backward()
        opt.step()
    print(f"  final train loss {float(loss):.5f}\n")

    exported = fold_normalisation(model, mu, sigma)
    w = exported.flatten()
    check("flatten produces the expected length", w.size == N_WEIGHTS, f"{w.size}")

    # 1. flatten/load_flat round trip -- the layout is its own inverse or it is wrong.
    rt = DeepSets()
    rt.load_flat(w)
    check("flatten -> load_flat is lossless",
          np.array_equal(rt.flatten(), w))

    # 2. The fold is exact: the trained model on normalised inputs must equal the exported
    #    model on raw inputs. This is where a mis-derived fold shows up.
    with torch.no_grad():
        ref = model(torch.from_numpy(rel_n[:500]), torch.from_numpy(mask[:500])).numpy()
        fold = exported(torch.from_numpy(rel[:500] * mask[:500][..., None]),
                        torch.from_numpy(mask[:500])).numpy()
    check("normalisation fold is exact", np.allclose(ref, fold, atol=1e-4),
          f"max err {np.abs(ref - fold).max():.2e}")

    # 3. The exported vector, run through the firmware's own NumPy evaluation.
    idx = rng.choice(len(y), 300, replace=False)
    npy = np.array([firmware_forward(
        w, [(rel[i, k, :3], rel[i, k, 3:]) for k in range(MAX_NEIGHBOURS) if mask[i, k] > 0],
        apply_clamp=False)[0] for i in idx])
    with torch.no_grad():
        ref = model(torch.from_numpy(rel_n[idx]), torch.from_numpy(mask[idx])).numpy()
    check("torch == firmware_forward on the exported vector",
          np.allclose(ref, npy, atol=2e-4), f"max err {np.abs(ref - npy).max():.2e}")

    # 4. The one that matters: upload into the COMPILED controller and compare.
    #    Zero relative velocity, because a first peer sample has no previous timestamp --
    #    which is also exactly what the firmware does on the first tick after takeoff.
    print("\nuploading to the compiled firmware (987 params, one per tick)")
    sil = Sil(own_pos=(0.0, 0.0, 1.0))
    ready = sil.upload(w)
    check("firmware accepted the exported weights", ready == 1, f"ready={ready}")

    # The synthetic set has non-zero relative velocity everywhere, so the comparison zeroes it
    # rather than hunting for samples that happen to fit.
    errs = []
    for j, i in enumerate(idx[:80]):
        r = rel[i].copy()
        r[:, 3:] = 0.0
        got = sil.predict(r, mask[i], t_ms=10_000 + j * 1000)   # >500 ms apart: no differencing
        want, _ = firmware_forward(
            w, [(r[k, :3], r[k, 3:]) for k in range(MAX_NEIGHBOURS) if mask[i, k] > 0])
        errs.append(np.abs(got - want).max())
    check("compiled firmware == reference, zero peer velocity",
          max(errs) < 2e-4, f"max err {max(errs):.2e} over {len(errs)} samples")

    # 5. With a differenced peer velocity, which is the flight case: two peer samples 100 ms
    #    apart, so the firmware reconstructs a known relative velocity itself.
    print("\nwith peer velocity differenced onboard")
    sil = Sil(own_pos=(0.0, 0.0, 1.0), own_vel=(0.2, -0.1, 0.05))
    sil.upload(w)
    errs = []
    for j in range(40):
        i = int(idx[j])
        dp = rel[i, 0, :3].astype(np.float64)
        v_peer = rng.normal(0, 0.3, 3)
        t0 = 20_000 + j * 2000
        p0 = sil.own_pos + dp
        p1 = p0 + v_peer * 0.1                        # 100 ms later

        fw.oot_set_peer(0, *[float(x) for x in p0], int(t0))
        fw.oot_set_peer_count(1)
        sil.step()                                     # first sample: relative velocity is 0
        fw.oot_set_peer(0, *[float(x) for x in p1], int(t0 + 100))
        sil.step()
        got = np.array([fw.cvar.g_rnn_pred_x, fw.cvar.g_rnn_pred_y,
                        fw.cvar.g_rnn_pred_z], np.float32)

        dp1 = np.asarray(p1, np.float32) - sil.own_pos
        dv1 = (v_peer - sil.own_vel).astype(np.float32)
        want, _ = firmware_forward(w, [(dp1, dv1)])
        errs.append(float(np.abs(got - want).max()))
    check("compiled firmware == reference, differenced peer velocity",
          max(errs) < 2e-4, f"max err {max(errs):.2e}")

    # 6. The dataset's sign convention, checked against the firmware rather than asserted.
    #    A neighbour ABOVE the ego vehicle must produce downward acceleration, because that is
    #    what the synthetic ground truth encodes and what downwash does. If dataset.build
    #    flipped the sign, the trained model would predict upward here.
    print("\nsign convention")
    r = np.zeros((MAX_NEIGHBOURS, 6), np.float32)
    r[0, :3] = (0.03, 0.0, 0.35)           # peer 0.35 m ABOVE, slightly offset
    m = np.zeros(MAX_NEIGHBOURS, np.float32)
    m[0] = 1.0
    sil = Sil(own_pos=(0.0, 0.0, 1.0))
    sil.upload(w)
    above = sil.predict(r, m, t_ms=90_000)
    r[0, :3] = (0.03, 0.0, -0.35)          # peer BELOW
    below = sil.predict(r, m, t_ms=95_000)
    # Sign is what is under test here, not magnitude -- this model is fitted in 600 steps on
    # 4000 made-up samples and is not expected to hit the ground truth closely.
    check("neighbour above pushes the ego vehicle down", above[2] < -0.2,
          f"a_z={above[2]:.3f}")
    check("neighbour below does not", below[2] > above[2],
          f"above {above[2]:.3f} vs below {below[2]:.3f}")

    # 7. A merged-CSV round trip, so the loader is exercised on the format it will really see.
    print("\ndataset loader on a merged-CSV layout")
    n = 600
    t = np.arange(n) / 500.0
    cols = {"t": t}
    cols.update({f"cfA.{k}": v for k, v in zip(
        "x y z vx vy vz".split(),
        [np.zeros(n), np.zeros(n), np.full(n, 1.0), np.zeros(n), np.zeros(n), np.zeros(n)])})
    cols.update({f"cfB.{k}": v for k, v in zip(
        "x y z vx vy vz".split(),
        [np.zeros(n), np.zeros(n), np.full(n, 1.4), np.zeros(n), np.zeros(n), np.zeros(n)])})
    for d, az in (("cfA", -1.7), ("cfB", -0.05)):
        for ax, val in zip("xyz", (0.0, 0.0, az)):
            cols[f"{d}.a_res_{ax}"] = np.full(n, val)
    d_rel, d_mask, d_y, st = dataset.build([cols], verbose=False)
    ego_a = d_rel[0]                    # cfA is first: its neighbour cfB sits 0.4 m above
    check("loader uses the firmware's peer-minus-own convention",
          abs(ego_a[0, 2] - 0.4) < 1e-5, f"dz={ego_a[0, 2]:.3f} (expect +0.4)")
    check("loader keeps both drones as ego", len(d_y) == 2 * n, f"{len(d_y)}")

    # 8. Samples with a_res identically zero -- no RPM source -- must be dropped, not learned.
    for ax in "xyz":
        cols[f"cfA.a_res_{ax}"] = np.zeros(n)
    _, _, d_y2, st2 = dataset.build([cols], verbose=False)
    check("a_res == 0 samples dropped (no RPM source is not 'no interaction')",
          len(d_y2) == n and st2["dropped_zero_a_res"] == n,
          f"kept {len(d_y2)}, dropped {st2['dropped_zero_a_res']}")

    # 9. The clamp survives the pipeline: absurd weights must not command absurd acceleration.
    sil = Sil(own_pos=(0.0, 0.0, 1.0))
    sil.upload((w * 80.0).astype(np.float32))
    p = sil.predict(r * 0 + np.array([0.0, 0.0, 0.3, 0, 0, 0], np.float32), m, t_ms=99_000)
    check("output clamp still engages after a pipeline upload",
          float(np.linalg.norm(p)) <= OUT_CLAMP + 1e-3 and fw.cvar.g_rnn_clamped == 1,
          f"|pred|={np.linalg.norm(p):.3f}")

    print()
    if FAILS:
        print(f"{len(FAILS)} FAILED: {', '.join(FAILS)}")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
