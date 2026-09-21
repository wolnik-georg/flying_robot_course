#!/usr/bin/env python3
"""Stage C: real merged CSV -> dataset.build -> NumPy vs compiled firmware (docs/40).

Run: python3 test_real_data_pipeline.py [--weights path.npz] [--csv merged.csv]

Needs torch + cffirmware built with residual_nn (see test_pipeline.py header).

Scope (read before citing "closes the loader↔firmware gap"):
  - Compares `dataset.build()` tensors through `model.firmware_forward` vs compiled
    `residual_nn.rs` via FwHarness + `oot_set_peer`.
  - Path A: peer relative *positions* from the loader; NumPy uses peer dv = 0 (same as the
    original Stage C run); firmware sees zero peer velocity unless differenced.
  - Path B: measured peer `rel[...,3:6]` — two `oot_set_peer` calls 100 ms apart so the
    firmware reconstructs the same relative velocity as the loader (see `test_pipeline.py`
    check 7). NumPy ref uses the real (dp, dv) pairs.
  - Ego velocity: loader ground block → own_vel = −ground[1:4]; applied to both paths.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/build")

import cffirmware as fw  # noqa: E402
import dataset  # noqa: E402
import model as M  # noqa: E402
from test_pipeline import FwHarness  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CSV = (
    ROOT / "experiments/logs/c1_2026-09-21_merged/"
    "A3_2026-09-21_13-00-57/A3_2026-09-21_13-00-57_merged_usd.csv"
)
DEFAULT_W = ROOT / "experiments/analysis/out/c2_e2e_2026-09-21/loo_weights/train_without_A3_13-00-57.npz"
PEER_DT_S = 0.1


def sign_sanity(rel, mask, ground, y, n_show=3):
    idx = np.where((mask.sum(axis=1) > 0) & (y != 0))[0]
    if len(idx) == 0:
        return {"ok": False, "reason": "no gated rows with nonzero y"}
    samples = []
    ok = True
    for i in idx[:n_show]:
        for k in range(rel.shape[1]):
            if mask[i, k] <= 0:
                continue
            dz = float(rel[i, k, 2])
            samples.append({"row": int(i), "rel_z": dz, "y": float(y[i])})
            if dz <= 0:
                ok = False
            break
    return {"ok": ok, "samples": samples}


def set_own_state(h, own_z, own_v):
    h.st.position.z = own_z
    h.sp.position.z = own_z
    h.st.velocity.x, h.st.velocity.y, h.st.velocity.z = (
        float(own_v[0]), float(own_v[1]), float(own_v[2]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", type=Path, default=DEFAULT_W)
    ap.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    ap.add_argument("--n", type=int, default=200, help="rows to compare per path")
    args = ap.parse_args()

    if not args.weights.is_file():
        print(f"Missing weights {args.weights} — run c2_e2e_validation.py Stage B first",
              file=sys.stderr)
        return 1
    if not args.csv.is_file():
        print(f"Missing csv {args.csv}", file=sys.stderr)
        return 1

    rel, mask, ground, y, _stats = dataset.build([str(args.csv)], verbose=True)
    print(f"loaded {len(y)} rows from {args.csv.name}")
    sign = sign_sanity(rel, mask, ground, y)
    print("sign sanity (peer above -> rel_z > 0):", sign)

    w = np.load(args.weights)["weights"].astype(np.float64)
    mass = float(fw.cvar.g_indi_mass)
    rng = np.random.default_rng(0)
    idx = rng.choice(len(y), size=min(args.n, len(y)), replace=False)

    tol = 2e-4
    worst_zero = 0.0
    t_ms = 10000
    h0 = None
    for i in idx:
        peers_rel = []
        for k in range(M.MAX_NEIGHBOURS):
            if mask[i, k] <= 0:
                continue
            peers_rel.append(rel[i, k, :3].astype(np.float32))
        if not peers_rel:
            continue
        own_z = -float(ground[i, 0])
        own_v = np.zeros(3, np.float32)
        own_pos = (0.0, 0.0, own_z)
        ref_a, _ = M.firmware_forward(
            w, [(d, np.zeros(3, np.float32)) for d in peers_rel],
            own_z, own_v, mass=mass, apply_clamp=False)
        if h0 is None:
            h0 = FwHarness(own_pos=own_pos, own_vel=(0.0, 0.0, 0.0))
            if h0.upload(w) != 1:
                print("FAIL: firmware rejected weights")
                return 1
        set_own_state(h0, own_z, own_v)  # ego vel 0 — matches original Stage C path
        peers_full = [
            np.array([own_pos[0] + d[0], own_pos[1] + d[1], own_pos[2] + d[2]], np.float32)
            for d in peers_rel
        ]
        t_ms += 1000
        h0.peers(peers_full, t_ms)
        h0.step()
        worst_zero = max(worst_zero, abs(h0.pred_z() - ref_a))

    worst_diff = 0.0
    t_ms = 20000
    h1 = None
    for i in idx:
        peers_rel, peers_dv = [], []
        for k in range(M.MAX_NEIGHBOURS):
            if mask[i, k] <= 0:
                continue
            peers_rel.append(rel[i, k, :3].astype(np.float32))
            peers_dv.append(rel[i, k, 3:].astype(np.float32))
        if not peers_rel:
            continue
        own_z = -float(ground[i, 0])
        own_v = (-ground[i, 1:]).astype(np.float32)
        own_pos = (0.0, 0.0, own_z)
        ref_pairs = [(d, dv.astype(np.float32)) for d, dv in zip(peers_rel, peers_dv)]
        ref_a, _ = M.firmware_forward(w, ref_pairs, own_z, own_v, mass=mass, apply_clamp=False)
        if h1 is None:
            h1 = FwHarness(own_pos=own_pos, own_vel=tuple(float(x) for x in own_v))
            if h1.upload(w) != 1:
                print("FAIL: firmware rejected weights")
                return 1
        set_own_state(h1, own_z, own_v)
        peers_full = [
            np.array([own_pos[0] + d[0], own_pos[1] + d[1], own_pos[2] + d[2]], np.float32)
            for d in peers_rel
        ]
        # Firmware: peer world vel = Δpos/dt; network input dv = peer_world_vel − own_vel.
        # Loader rel[...,3:6] is already peer − own, so peer_world_vel = own_v + rel_dv.
        first = [
            p - (own_v + dv.astype(np.float32)) * PEER_DT_S
            for p, dv in zip(peers_full, peers_dv)
        ]
        t_ms += 1000
        h1.peers(first, t_ms)
        h1.step()
        h1.peers(peers_full, t_ms + int(PEER_DT_S * 1000))
        h1.step()
        worst_diff = max(worst_diff, abs(h1.pred_z() - ref_a))

    ok_zero = worst_zero < tol
    ok_diff = worst_diff < tol
    ok = ok_zero and ok_diff and sign.get("ok", False)
    print(f"max |NumPy firmware_forward - compiled| = {worst_zero:.2e} m/s^2  "
          f"({'PASS' if ok_zero else 'FAIL'} tol {tol})  [peer dv forced zero in NumPy]")
    print(f"max |NumPy firmware_forward - compiled| = {worst_diff:.2e} m/s^2  "
          f"({'PASS' if ok_diff else 'FAIL'} tol {tol})  "
          f"[differenced peer velocity, dt={PEER_DT_S}s]")
    print(f"overall: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
