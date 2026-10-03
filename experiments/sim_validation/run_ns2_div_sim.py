#!/usr/bin/env python3
"""Two-drone SIL: rnn.div=1 vs 10 (subprocess-isolated; optional 100 Hz peer packets).

Custom A8-like pass (bottom z=0.5 m, top z=1.0 m, top y sine) — **not** formation library A8.
`np` plant has **no** downwash; pred-vs-a_res is **not interpretable**.

Each configuration runs in a **fresh subprocess** so controller `State` (peer_prev, rnn_tick) is clean.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

WORKER = r'''
import json, sys
from pathlib import Path
import numpy as np

import os
ROOT = Path(os.environ["NS2_SIM_ROOT"])
HOST = ROOT / "flying_drone_stack/firmware_app/host"
sys.path.insert(0, str(HOST))
sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/build")
sys.path.insert(0, "/home/georg/Desktop/crazyswarm2/crazyflie_sim")

import cffirmware as firm
from crazyflie_sim.crazyflie_sil import CrazyflieSIL
from crazyflie_sim import sim_data_types
from crazyflie_sim.backend.np import Quadrotor
from test_residual_nn import N_WEIGHTS

cfg = json.loads(sys.argv[1])

GAINS = dict(
    kr=2400.0, kw=170.0, kr_z=2400.0, kw_z=170.0, fc_bw=60.0, mass=0.041,
    kt1=4.1623e-10, kt2=4.0592e-10, kt3=4.1116e-10, kt4=4.0631e-10, ff_free=0,
    filt_order=1, filt_tau=1, j_scale=1.0, clamp_en=11, tau_xy_max=0.045,
    tau_z_max=0.0025, tilt_max_deg=30.0, thrust_max=0.8, notch_en=0,
    notch_f0=6.9, notch_bw=3.0,
)

_peer_real = firm.oot_set_peer
_peer_state = {"last_ms": {}, "held": {}}


def _install_peer_patch(peer_hz: float):
    period_ms = max(1, int(round(1000.0 / peer_hz)))

    def patched(k, x, y, z, tick):
        tick = int(tick)
        st = _peer_state
        if k not in st["last_ms"]:
            st["last_ms"][k] = -period_ms
            st["held"][k] = (float(x), float(y), float(z), tick)
        if tick - st["last_ms"][k] >= period_ms:
            st["held"][k] = (float(x), float(y), float(z), tick)
            st["last_ms"][k] = tick
        hx, hy, hz, ht = st["held"][k]
        _peer_real(k, hx, hy, hz, ht)

    firm.oot_set_peer = patched


def apply_geometric_gains(c):
    for k, v in GAINS.items():
        setattr(c, "g_indi_" + k, v)
    c.g_kp_xy, c.g_kp_z, c.g_kv_xy, c.g_kv_z = 64.0, 48.0, 8.0, 7.0
    c.g_ki_z, c.g_ki_z_limit = 16.0, 1.5
    c.g_controller_mode = 0


def upload_weights(w):
    c = firm.cvar
    c.g_rnn_n = int(w.size)
    c.g_rnn_begin = 1
    firm.oot_rnn_service()
    for i, v in enumerate(w):
        c.g_rnn_wi = int(i)
        c.g_rnn_wv = float(v)
        c.g_rnn_wc = 1
        firm.oot_rnn_service()
    c.g_rnn_end = 1
    firm.oot_rnn_service()
    if c.g_rnn_ready != 1:
        raise RuntimeError("RNN upload rejected")


def top_xy(t, phase):
    return 0.0, float(0.15 * np.sin(2.0 * np.pi * t / 10.0 + phase))


def run(cfg):
    w = np.load(cfg["weights"])["weights"].astype(np.float32)
    if w.size != N_WEIGHTS:
        raise SystemExit(f"bad weight count {w.size}")

    peer_hz = float(cfg.get("peer_hz", 1000.0))
    if peer_hz < 999.0:
        _install_peer_patch(peer_hz)

    firm.controllerOutOfTreeInit()
    firm.oot_select_drone(0)
    c = firm.cvar
    apply_geometric_gains(c)
    c.g_rnn_div = int(cfg["div"])
    c.g_rnn_en = 1 if cfg.get("rnn_en") else 0
    upload_weights(w)

    CrazyflieSIL._oot_count = 0
    J = [firm.oot_inertia(i) for i in range(3)]
    ph = dict(
        mass=0.041,
        kt=[c.g_indi_kt1, c.g_indi_kt2, c.g_indi_kt3, c.g_indi_kt4],
        arm_length=firm.oot_arm_length(),
        t2t=firm.oot_thrust2torque(),
        inertia=J,
        motor_tau=0.044,
    )
    duration = float(cfg.get("duration_s", 14.0))
    dt = 1e-3
    z_bot = float(cfg.get("z_bottom", 0.5))
    dz = float(cfg.get("dz", 0.5))
    phase = float(cfg.get("phase", 0.0))
    seed_tag = cfg.get("label", "")

    t_box = [0.0]
    cfs, quads = [], []
    for i, z0 in enumerate((z_bot, z_bot + dz)):
        p0 = np.array([0.0, 0.0, 0.0])
        cfs.append(CrazyflieSIL(f"cf{i}", p0, "oot", lambda: t_box[0]))
        quads.append(Quadrotor(sim_data_types.State(pos=p0.copy()), ph))
        cfs[-1].takeoff(z0, 4.0)

    n_steps = int(duration / dt)
    pred_z, clamp_n = [], 0
    pos_bot, tilt_bot = [], []

    for k in range(1, n_steps + 1):
        t_box[0] = k * dt
        positions = [q.state.pos.copy() for q in quads]
        actions = []
        for i, cf in enumerate(cfs):
            firm.oot_select_drone(i)
            if i == 1:
                tx, ty, tz = top_xy(t_box[0], phase)[0], top_xy(t_box[0], phase)[1], z_bot + dz
                cf.cmdFullState((tx, ty, tz), (0, 0, 0), (0, 0, 0), 0.0, (0, 0, 0))
            else:
                cf.getSetpoint()
            cf.peers = [
                (float(positions[j][0]), float(positions[j][1]), float(positions[j][2]))
                for j in range(len(cfs)) if j != i
            ]
            cf.setState(quads[i].state)
            act = cf.executeController()
            actions.append(act)
            if i == 0 and k % 10 == 0:
                pred_z.append(float(c.g_rnn_pred_z))
                if c.g_rnn_clamped:
                    clamp_n += 1
        for i, (q, act) in enumerate(zip(quads, actions)):
            q.step(act, dt, np.zeros(3))
        if k % 10 == 0 and quads[0].state.pos[2] > z_bot * 0.8:
            p = quads[0].state.pos
            pos_bot.append(p.copy())
            tilt_bot.append(float(np.linalg.norm(quads[0].state.vel[:2])))

    pos_bot = np.asarray(pos_bot) if pos_bot else np.zeros((0, 3))
    sp = np.array([0.0, 0.0, z_bot])
    track = {}
    if len(pos_bot):
        err = pos_bot - sp
        track = {
            "rms_3d_m": float(np.sqrt(np.mean(np.sum(err**2, axis=1)))),
            "rms_z_m": float(np.sqrt(np.mean(err[:, 2] ** 2))),
            "max_tilt_proxy": float(np.max(tilt_bot)) if tilt_bot else 0.0,
        }
    pred_z = np.asarray(pred_z, np.float64)
    return {
        "label": seed_tag,
        "div": cfg["div"],
        "rnn_en": int(cfg.get("rnn_en", 0)),
        "peer_hz": peer_hz,
        "n_pred_samples": int(len(pred_z)),
        "pred_z_mean": float(np.mean(pred_z)) if len(pred_z) else float("nan"),
        "pred_z_std": float(np.std(pred_z)) if len(pred_z) else float("nan"),
        "clamp_events_logged": int(clamp_n),
        "pred_z_series": pred_z.tolist(),
        "tracking_bottom": track,
    }


try:
    print(json.dumps(run(cfg)))
except Exception as e:
    print(json.dumps({"error": str(e), "label": cfg.get("label", "")}))
'''


def compare_series(a: list[float], b: list[float], thr: float = 0.1) -> dict:
    n = min(len(a), len(b))
    if n == 0:
        return {"n": 0}
    da = np.asarray(a[:n], np.float64)
    db = np.asarray(b[:n], np.float64)
    diff = db - da
    return {
        "n": n,
        "rms_diff_mps2": float(np.sqrt(np.mean(diff**2))),
        "max_abs_diff_mps2": float(np.max(np.abs(diff))),
        "frac_abs_diff_gt_0p1": float(np.mean(np.abs(diff) > thr)),
    }


def lag_by_min_error(a: list[float], b: list[float], max_ms: int = 20, step_ms: int = 10) -> dict:
    """Shift b relative to a over ±max_ms (sample step_ms) minimizing RMS diff."""
    n = min(len(a), len(b))
    if n < 20:
        return {"best_lag_ms": 0, "rms_at_best": float("nan")}
    aa = np.asarray(a[:n], np.float64)
    bb = np.asarray(b[:n], np.float64)
    best_lag, best_rms = 0, 1e9
    for lag_ms in range(-max_ms, max_ms + 1, step_ms):
        shift = lag_ms // step_ms
        if shift >= 0:
            x, y = aa[shift:], bb[: n - shift]
        else:
            x, y = aa[: n + shift], bb[-shift:]
        if len(x) < 10:
            continue
        rms = float(np.sqrt(np.mean((y - x) ** 2)))
        if rms < best_rms:
            best_rms, best_lag = rms, lag_ms
    return {"best_lag_ms": best_lag, "rms_at_best_mps2": best_rms}


def run_worker(cfg: dict) -> dict:
    script = Path(__file__).read_text()
    # Extract worker body only for -c
    code = WORKER
    env = {**os.environ, "NS2_SIM_ROOT": str(ROOT)}
    p = subprocess.run(
        [sys.executable, "-c", code, json.dumps(cfg)],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        env=env,
    )
    out = p.stdout.strip()
    if p.returncode != 0 and not out:
        raise RuntimeError(p.stderr or p.stdout)
    data = json.loads(out.splitlines()[-1])
    if "error" in data:
        raise RuntimeError(data["error"])
    return data


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--weights",
        type=Path,
        default=ROOT / "experiments/analysis/out/c2_e2e_2026-10-01/full_bank_c1_complete.npz",
    )
    ap.add_argument("--duration", type=float, default=14.0)
    ap.add_argument("-o", type=Path, default=ROOT / "experiments/sim_validation/ns2_div_sil_results.json")
    args = ap.parse_args()

    base = {
        "weights": str(args.weights),
        "duration_s": args.duration,
        "z_bottom": 0.5,
        "dz": 0.5,
    }
    phases = [0.0, 1.1, 2.4]
    report = {
        "scenario": "custom_A8_like_pass_not_library_A8",
        "plant": "np (no downwash; pred-vs-a_res not interpretable)",
        "weights": str(args.weights),
        "isolation": "fresh subprocess per configuration",
        "comparisons": {},
    }

    for peer_hz, peer_label in ((100.0, "peer_100hz_packets"), (1000.0, "peer_1khz_sil_default")):
        block = {"predict_only": {}, "closed_loop": {}}
        for rnn_en, mode in ((0, "predict_only"), (1, "closed_loop")):
            pairs = []
            for ph in phases:
                try:
                    r1 = run_worker({**base, "div": 1, "rnn_en": rnn_en, "peer_hz": peer_hz,
                                     "phase": ph, "label": f"div1_ph{ph}"})
                    r10 = run_worker({**base, "div": 10, "rnn_en": rnn_en, "peer_hz": peer_hz,
                                      "phase": ph, "label": f"div10_ph{ph}"})
                except RuntimeError as exc:
                    pairs.append({"phase": ph, "error": str(exc)})
                    continue
                pairs.append(
                    {
                        "phase": ph,
                        "div1": {k: r1[k] for k in r1 if k != "pred_z_series"},
                        "div10": {k: r10[k] for k in r10 if k != "pred_z_series"},
                        "div1_vs_div10": compare_series(r1["pred_z_series"], r10["pred_z_series"]),
                        "lag_search_pm20ms": lag_by_min_error(r1["pred_z_series"], r10["pred_z_series"]),
                    }
                )
            block[mode] = {"by_phase": pairs}
        report["comparisons"][peer_label] = block

    # Headline: 100 Hz packets, predict-only, aggregate div1 vs div10 across phases
    po100 = report["comparisons"]["peer_100hz_packets"]["predict_only"]["by_phase"]
    rms_list = [p["div1_vs_div10"]["rms_diff_mps2"] for p in po100 if p["div1_vs_div10"].get("n")]
    frac_list = [p["div1_vs_div10"]["frac_abs_diff_gt_0p1"] for p in po100 if p["div1_vs_div10"].get("n")]
    report["summary"] = {
        "realistic_100hz_peer_predict_only": {
            "mean_rms_div1_vs_div10_mps2": float(np.mean(rms_list)) if rms_list else float("nan"),
            "max_rms_mps2": float(np.max(rms_list)) if rms_list else float("nan"),
            "mean_frac_gt_0p1": float(np.mean(frac_list)) if frac_list else float("nan"),
        },
        "verdict_100hz_hold": (
            "In 100 Hz peer-packet mode (subprocess-isolated), div=10 vs div=1 predict-only "
            f"mean RMS diff ≈ {float(np.mean(rms_list)):.3f} m/s², "
            f"mean fraction |Δ|>0.1 ≈ {float(np.mean(frac_list)):.1%}. "
            "1 kHz SIL peer stamping inflates differences (noisy 1 ms differencing, clamp spikes)."
        ),
    }

    text = json.dumps(report, indent=2) + "\n"
    print(text)
    args.o.parent.mkdir(parents=True, exist_ok=True)
    args.o.write_text(text)
    return 0


if __name__ == "__main__":
    import numpy as np  # noqa: E402 — main-only aggregate

    sys.exit(main())
