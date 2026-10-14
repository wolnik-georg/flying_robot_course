#!/usr/bin/env python3
"""Phase 4 (docs/plan_omar_tree_ns2_2026-10-14.md, docs/ns2_c_module_spec.md §7): the C implementation of NS2
(src/modules/src/ns2_residual.c, flash weights ns2_weights.c) against
  (a) an independent NumPy reference (written from the architecture, from test_residual_nn.py),
  (b) the Rust flight implementation (ResidualNet via controller 6, weights uploaded through the real protocol).
All with the frozen v1 weights. Run with the system python3 (3.10 bindings):
  python3 flying_drone_stack/firmware_app/host/test_ns2_c_equivalence.py [--tol 1e-5]
"""
from __future__ import annotations

import argparse
import ctypes
import hashlib
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))
sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/build")
import cffirmware as fw  # noqa: E402
import test_residual_nn as T  # noqa: E402  (reference(), Fw harness, constants)

NPZ = ROOT / "flying_drone_stack/tools/residual/weights/ns2_v1.npz"
WC = ROOT / "flying_drone_stack/firmware_app/ns2_weights.c"
SO = "/home/georg/Desktop/crazyflie-firmware/build/_cffirmware.cpython-310-x86_64-linux-gnu.so"
MASS = 0.0427

lib = ctypes.CDLL(SO)
F32P = ctypes.c_void_p


class PeerPrev(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float), ("ts", ctypes.c_uint32)]


class Ns2State(ctypes.Structure):
    _fields_ = [("prev", PeerPrev * 3), ("vel", (ctypes.c_float * 3) * 3), ("hold", ctypes.c_float * 3),
                ("hold_clamped", ctypes.c_uint8), ("started", ctypes.c_uint8)]


lib.ns2_eval_rel.argtypes = [F32P, F32P, ctypes.c_int, ctypes.c_float, F32P, ctypes.c_float, F32P, F32P]
lib.ns2_eval.argtypes = [ctypes.POINTER(Ns2State), F32P, F32P, ctypes.c_float, F32P, F32P]
lib.ns2_update.argtypes = [ctypes.POINTER(Ns2State), F32P, F32P, ctypes.c_float, ctypes.c_uint32]
lib.ns2_reset.argtypes = [ctypes.POINTER(Ns2State)]
lib.ns2_weights_present.restype = ctypes.c_bool


def c_eval_rel(rel, own_z, own_vel, mass):
    n = len(rel)
    P = np.array([r[0] for r in rel] or [[0, 0, 0]], np.float32).reshape(-1, 3).copy()
    V = np.array([r[1] for r in rel] or [[0, 0, 0]], np.float32).reshape(-1, 3).copy()
    ov = np.asarray(own_vel, np.float32).copy()
    out = np.zeros(3, np.float32)
    cl = ctypes.c_uint8(0)
    lib.ns2_eval_rel(P.ctypes.data, V.ctypes.data, n, float(own_z), ov.ctypes.data, float(mass), out.ctypes.data,
                     ctypes.byref(cl))
    return out, bool(cl.value)


def c_eval(state, pos, vel, mass):
    p = np.asarray(pos, np.float32).copy(); v = np.asarray(vel, np.float32).copy()
    out = np.zeros(3, np.float32); cl = ctypes.c_uint8(0)
    lib.ns2_eval(ctypes.byref(state), p.ctypes.data, v.ctypes.data, float(mass), out.ctypes.data, ctypes.byref(cl))
    return out, bool(cl.value)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tol", type=float, default=1e-5, help="relative tolerance (abs floor 1e-6 m/s^2)")
    a = ap.parse_args()
    fails: list[str] = []

    def check(name, ok, detail=""):
        print(f"  {'PASS' if ok else 'FAIL'}  {name}{('  ' + detail) if detail else ''}")
        if not ok:
            fails.append(name)

    def close(x, y):
        return bool(np.all(np.abs(x - y) <= a.tol * np.maximum(np.abs(y), 1.0) + 1e-6))

    w = np.load(NPZ)["weights"].astype(np.float32)
    check("weights present in the linked build", lib.ns2_weights_present())

    # 5. generator: re-parse ns2_weights.c and compare bit for bit with the npz slices
    txt = WC.read_text()
    sha = re.search(r'ns2_weights_sha256\[\] = "([0-9a-f]{64})"', txt).group(1)
    check("embedded sha256 equals the npz file", sha == hashlib.sha256(NPZ.read_bytes()).hexdigest())
    want = {"ns2_w_phi_s": w[0:3675], "ns2_w_phi_g": w[7350:10975], "ns2_w_rho_s": w[10975:15136]}
    for name, ref in want.items():
        body = re.search(rf"const float {name}\[\d+\] = \{{(.*?)\}};", txt, re.S).group(1)
        vals = np.array([float(x.strip().rstrip("f")) for x in body.replace("\n", " ").split(",") if x.strip()], np.float32)
        check(f"{name}: {len(vals)} floats equal the npz slice bit-for-bit", vals.shape == ref.shape and np.array_equal(vals.view(np.uint32), ref.view(np.uint32)))

    # 2a. C vs NumPy reference, random relative states
    rng = np.random.default_rng(20261014)
    worst_np = 0.0
    nbad = 0
    N = 3000
    for it in range(N):
        n = int(rng.integers(0, 4))
        rel = []
        for _ in range(n):
            dp = np.array([rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), rng.uniform(-1.0, 1.5)], np.float32)
            dv = np.array([rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-2, 2)], np.float32)
            rel.append((dp, dv))
        own_z = float(rng.uniform(0.0, 2.0)); own_vel = rng.uniform(-1.5, 1.5, 3).astype(np.float32)
        ref, rcl = T.reference(w, rel, own_z, own_vel, MASS)
        out, ccl = c_eval_rel(rel, own_z, own_vel, MASS)
        d = float(np.max(np.abs(out - ref)) / max(1.0, float(np.max(np.abs(ref)))))
        worst_np = max(worst_np, d)
        if not (close(out, ref) and rcl == ccl):
            nbad += 1
    check(f"C vs NumPy, {N} random cases (0-3 neighbours, ground term)", nbad == 0, f"worst relative diff {worst_np:.2e}, mismatches {nbad}")

    # gate boundaries, one at a time (dx, dy, dvx), each side
    gate_ok = True; gate_note = []
    for label, dp, dv in [("dx 0.199", [0.199, 0, 0.5], [0, 0, 0]), ("dx 0.201", [0.201, 0, 0.5], [0, 0, 0]),
                          ("dy 0.199", [0, 0.199, 0.5], [0, 0, 0]), ("dy 0.201", [0, 0.201, 0.5], [0, 0, 0]),
                          ("dvx 1.49", [0, 0, 0.5], [1.49, 0, 0]), ("dvx 1.51", [0, 0, 0.5], [1.51, 0, 0]),
                          ("dz large (not gated)", [0, 0, 5.0], [0, 0.0, 0.0]), ("dvy,dvz large (not gated)", [0, 0, 0.5], [0, 3.0, -3.0])]:
        rel = [(np.array(dp, np.float32), np.array(dv, np.float32))]
        ref, _ = T.reference(w, rel, 0.5, np.zeros(3, np.float32), MASS)
        out, _ = c_eval_rel(rel, 0.5, np.zeros(3), MASS)
        ok = close(out, ref); gate_ok &= ok
        gate_note.append(f"{label}:{out[2]:+.3f}")
    check("gate boundaries match the reference", gate_ok, "  ".join(gate_note))
    g_in, _ = c_eval_rel([(np.array([0.199, 0, 0.5], np.float32), np.zeros(3, np.float32))], 0.5, np.zeros(3), MASS)
    g_out, _ = c_eval_rel([(np.array([0.201, 0, 0.5], np.float32), np.zeros(3, np.float32))], 0.5, np.zeros(3), MASS)
    g_none, _ = c_eval_rel([], 0.5, np.zeros(3), MASS)
    check("gate has an effect (inside differs from outside, outside equals ground only)", abs(g_in[2] - g_none[2]) > 1e-3 and abs(g_out[2] - g_none[2]) < 1e-9)

    # clamp and non-finite
    out, cl = c_eval_rel([(np.array([0, 0, 0.3], np.float32), np.zeros(3, np.float32))], 0.5, np.zeros(3), 1e-4)
    ref, rcl = T.reference(w, [(np.array([0, 0, 0.3], np.float32), np.zeros(3, np.float32))], 0.5, np.zeros(3, np.float32), 1e-4)
    check("output clamp at 8 m/s^2 (tiny mass)", cl and rcl and abs(abs(out[2]) - 8.0) < 1e-4 and close(out, ref), f"c={out[2]:.4f} ref={ref[2]:.4f}")
    # non-finite handling: mass 0 makes the acceleration infinite -> zero output with the fault flag (C and Rust below)
    out0, cl0 = c_eval_rel([(np.array([0, 0, 0.3], np.float32), np.zeros(3, np.float32))], 0.5, np.zeros(3), 0.0)
    check("non-finite acceleration (mass 0) -> zero output with the fault flag", cl0 and np.all(out0 == 0.0), f"out={out0.tolist()} clamped={cl0}")

    # 2b. C vs Rust through the real flight path
    fw.cvar.g_rnn_div = 1
    fw.cvar.g_indi_mass = MASS
    h = T.Fw(own_pos=(0.0, 0.0, 1.0), own_vel=(0.0, 0.0, 0.0))
    ready = h.upload(w)
    if ready != 1:
        print("  SKIP  C vs Rust: the Rust residual_nn upload path is not compiled into this build (feature residual_nn)")
        rust = False
    else:
        rust = True
        worst_rs = 0.0; nbad = 0; M = 400
        st_r = Ns2State(); lib.ns2_reset(ctypes.byref(st_r))   # C peer history, evolving exactly like the Rust State
        for it in range(M):
            n = int(rng.integers(0, 4))
            own = np.array([0.0, 0.0, float(rng.uniform(0.3, 1.5))], np.float32)
            h.own_pos = own; h.st.position.z = float(own[2]); h.sp.position.z = float(own[2])
            pts = [(own + np.array([rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4), rng.uniform(-0.8, 1.0)], np.float32))
                   for _ in range(n)]
            h.peers([p.tolist() for p in pts], T.next_peer_ts())
            h.step(2)
            rs = h.pred()
            out, ccl = c_eval(st_r, own, np.zeros(3), MASS)   # same peer injection, own velocity 0, same history rules
            d = float(np.max(np.abs(out - rs)) / max(1.0, float(np.max(np.abs(rs)))))
            worst_rs = max(worst_rs, d)
            if not close(out, rs):
                nbad += 1
        check(f"C vs Rust flight path (stateful, same peer history), {M} random cases", nbad == 0, f"worst relative diff {worst_rs:.2e}, mismatches {nbad}")

        # NaN / non-finite own state: both paths must agree (ReLU swallows NaN identically)
        for label, oz, ov in [("NaN own z", float("nan"), (0, 0, 0)), ("overflow own velocity", 1.0, (1e30, 1e30, 1e30))]:
            h.own_pos = np.array([0, 0, oz], np.float32); h.st.position.z = oz; h.sp.position.z = 1.0
            h.st.velocity.x, h.st.velocity.y, h.st.velocity.z = ov
            h.peers([], T.next_peer_ts()); h.step(2)
            rs = h.pred()
            stn = Ns2State(); lib.ns2_reset(ctypes.byref(stn))
            out, ccl = c_eval(stn, [0, 0, oz], list(ov), MASS)
            check(f"non-finite state ({label}): C equals Rust", np.array_equal(np.nan_to_num(out), np.nan_to_num(rs)) or close(out, rs), f"c={out.tolist()} rust={rs.tolist()}")
        fw.cvar.g_indi_mass = 0.0
        h.peers([], T.next_peer_ts()); h.step(2)
        rs0 = h.pred()
        fw.cvar.g_indi_mass = MASS
        check("non-finite acceleration (mass 0): Rust also outputs zero (same fault path)", np.all(rs0 == 0.0), f"rust={rs0.tolist()}")
        h.st.velocity.x = h.st.velocity.y = h.st.velocity.z = 0.0
        h.st.position.z = 1.0; h.sp.position.z = 1.0; h.own_pos = np.array([0, 0, 1.0], np.float32)

        # 3. peer-history rules on a scripted timestamp sequence (Rust State vs C ns2_state_t)
        fw.cvar.g_rnn_div = 1
        hh = T.Fw(own_pos=(0.0, 0.0, 1.0), own_vel=(0.0, 0.0, 0.0))
        hh.upload(w)
        st = Ns2State(); lib.ns2_reset(ctypes.byref(st))
        t0 = 50_000
        seq = [  # (peer position, timestamp ms) -- first seen, same ts, normal, normal, gap 600, backwards, normal
            ((0.05, 0.0, 1.5), t0), ((0.05, 0.0, 1.5), t0), ((0.10, 0.0, 1.5), t0 + 100), ((0.15, 0.0, 1.5), t0 + 200),
            ((0.18, 0.0, 1.5), t0 + 800), ((0.10, 0.0, 1.5), t0 + 700), ((0.12, 0.0, 1.5), t0 + 800), ((0.14, 0.0, 1.5), t0 + 900)]
        names = ["first seen", "same ts", "normal +100", "normal +100", "gap 600 ms", "ts backwards", "after backwards", "normal +100"]
        ok_all = True; notes = []
        for (pp, ts), nm in zip(seq, names):
            hh.peers([pp], ts)
            hh.step(2)
            rs = hh.pred()
            out, _ = c_eval(st, [0, 0, 1.0], [0, 0, 0], MASS)
            ok = close(out, rs); ok_all &= ok
            notes.append(f"{nm}:{'ok' if ok else 'DIFF ' + str(out[2]) + ' vs ' + str(rs[2])}")
        check("peer-history rules match the Rust implementation", ok_all, "; ".join(notes))

    # 4. decimation: ns2_update evaluates on the first call and on tick % div == 0, republishes in between
    fw.cvar.g_rnn_div = 10
    fw.cvar.g_rnn_ready = 0
    st2 = Ns2State(); lib.ns2_reset(ctypes.byref(st2))
    evals = []
    prev = None
    for tick in range(0, 100, 2):  # Omar's controller executes on even main-loop ticks only
        z = 0.5 + 0.01 * tick  # ground term changes every call, so an evaluation shows up as a new value
        p = np.array([0, 0, z], np.float32); v = np.zeros(3, np.float32)
        lib.ns2_update(ctypes.byref(st2), p.ctypes.data, v.ctypes.data, ctypes.c_float(MASS), tick)
        cur = float(st2.hold[2])
        if prev is None or cur != prev:
            evals.append(tick)
        prev = cur
    expect = [0] + [t for t in range(10, 100, 10)]
    check("decimation: evaluations at the first call and every 10th main tick (100 Hz)", evals == expect, f"got {evals}")
    check("host build: the hook leaves rnn.ready untouched (firmware sets it; host globals are shared between drones)", fw.cvar.g_rnn_ready == 0)

    print()
    print("RESULT:", "ALL PASS" if not fails else f"{len(fails)} FAIL: {fails}", "(Rust path " + ("tested" if rust else "NOT available") + ")")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
