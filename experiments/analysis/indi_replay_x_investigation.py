#!/usr/bin/env python3
"""Time-boxed a_res_x gap analysis (hand kinematics, no controller)."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
ANALYSIS = Path(__file__).resolve().parent
sys.path.insert(0, str(ANALYSIS))

from indi_replay_ladder import OUT, RUNG0_FLIGHTS, steady_indices  # noqa: E402
from indi_replay_worker import WARMUP_TICKS  # noqa: E402

G = 9.81
MASS = 0.041


class BW2:
    """Match lib.rs Butterworth2 (prewarp=1)."""

    def __init__(self) -> None:
        self.b = self.a1 = self.a2 = 0.0
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0

    def init(self, fc: float, dt: float) -> None:
        tau = 1.0 / (2.0 * math.pi * fc)
        q = 0.7071
        k = math.tan(dt / (2.0 * tau))
        poly = k * k + k / q + 1.0
        self.b = k * k / poly
        self.a1 = 2.0 * (k * k - 1.0) / poly
        self.a2 = (k * k - k / q + 1.0) / poly
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0

    def seed(self, v: float) -> None:
        self.x1 = self.x2 = self.y1 = self.y2 = v

    def update(self, x: float) -> float:
        y = self.b * x + 2.0 * self.b * self.x1 + self.b * self.x2 - self.a1 * self.y1 - self.a2 * self.y2
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        return y


def quat_to_rot_lib(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    return np.array(
        [
            [qw * qw + qx * qx - qy * qy - qz * qz, 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy)],
            [2.0 * (qx * qy + qw * qz), qw * qw - qx * qx + qy * qy - qz * qz, 2.0 * (qy * qz - qw * qx)],
            [2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), qw * qw - qx * qx - qy * qy + qz * qz],
        ]
    )


def clamp_norm(v: np.ndarray, lim: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n > lim and n > 1e-9:
        return v * (lim / n)
    return v


def dshot_rpm_stream(rpm4: np.ndarray) -> np.ndarray:
    """traj_iface.c slew hold on 4-vector rows."""
    prev = np.zeros(4, dtype=np.uint16)
    out = np.zeros_like(rpm4, dtype=float)
    dshot_rpm_abs_max = 28000
    dshot_slew_max_rpm = 10000
    for row in range(len(rpm4)):
        for i in range(4):
            vi = int(rpm4[row, i])
            if vi == 0xFFFF:
                vi = 0
            reject = False
            if vi > 0 and vi > dshot_rpm_abs_max:
                reject = True
            elif vi > 0 and prev[i] > 500:
                lo, hi = min(vi, prev[i]), max(vi, prev[i])
                if hi - lo > dshot_slew_max_rpm:
                    reject = True
            if reject and prev[i] > 0:
                vi = int(prev[i])
            elif not reject and vi > 0:
                prev[i] = vi
            out[row, i] = vi
    return out


def hand_a_res(
    quat: np.ndarray,
    acc_g: np.ndarray,
    rpm: np.ndarray,
    kt: np.ndarray,
    *,
    acc_fc: float | None,
    acc_dt: float,
    res_fc: float,
    res_clamp: float,
) -> np.ndarray:
    n = len(quat)
    a_res = np.zeros((n, 3))
    bw_ax, bw_ay, bw_az = BW2(), BW2(), BW2()
    bw_mx, bw_my, bw_mz = BW2(), BW2(), BW2()
    bw_dx, bw_dy, bw_dz = BW2(), BW2(), BW2()
    if acc_fc:
        for b in (bw_ax, bw_ay, bw_az):
            b.init(acc_fc, acc_dt)
    if res_fc > 0:
        for b in (bw_mx, bw_my, bw_mz, bw_dx, bw_dy, bw_dz):
            b.init(res_fc, 0.001)
        seeded = False
    for i in range(n):
        qw, qx, qy, qz = map(float, quat[i])
        r = quat_to_rot_lib(qw, qx, qy, qz)
        acc_body = acc_g[i].astype(float)
        if acc_fc:
            acc_body = np.array(
                [bw_ax.update(acc_body[0]), bw_ay.update(acc_body[1]), bw_az.update(acc_body[2])]
            )
        f_total = float(np.sum(kt * (rpm[i].astype(float) ** 2)))
        body_z_w = r[:, 2]
        a_model = body_z_w * (f_total / MASS) + np.array([0.0, 0.0, -G])
        a_meas = r @ (acc_body * G) + np.array([0.0, 0.0, -G])
        if res_clamp > 0:
            a_meas = clamp_norm(a_meas, res_clamp)
            a_model = clamp_norm(a_model, res_clamp)
        if res_fc > 0:
            if not seeded:
                for b, v in zip((bw_mx, bw_my, bw_mz), a_meas):
                    b.seed(float(v))
                for b, v in zip((bw_dx, bw_dy, bw_dz), a_model):
                    b.seed(float(v))
                seeded = True
            a_meas = np.array([bw_mx.update(a_meas[0]), bw_my.update(a_meas[1]), bw_mz.update(a_meas[2])])
            a_model = np.array([bw_dx.update(a_model[0]), bw_dy.update(a_model[1]), bw_dz.update(a_model[2])])
        a_res[i] = a_meas - a_model
    return a_res


def corr_on_steady(z, hand_x: np.ndarray, log_x: np.ndarray) -> dict:
    n = len(z["pos"])
    start = int(z["warmup_start"]) + WARMUP_TICKS
    ticks = np.arange(start, n)
    steady = steady_indices(z, ticks)
    steady = steady[steady % 2 == 0]
    hx = hand_x[steady]
    lx = log_x[steady]
    return {
        "n": int(len(steady)),
        "corr": float(np.corrcoef(hx, lx)[0, 1]),
        "mean_abs_err": float(np.mean(np.abs(hx - lx))),
        "rms_err": float(np.sqrt(np.mean((hx - lx) ** 2))),
        "std_log": float(np.std(lx)),
        "std_hand": float(np.std(hx)),
    }


def investigate(label: str) -> dict:
    spec = next(s for s in RUNG0_FLIGHTS if s["label"] == label)
    npz_path = OUT / "rung0" / label / "inputs.npz"
    if not npz_path.exists():
        raise FileNotFoundError(npz_path)
    meta = json.loads(spec["meta"].read_text())
    kt = np.array([meta["per_drone"]["cf5"]["indi"][f"kt{i}"] for i in range(1, 5)])
    z = np.load(npz_path)
    log_a = z["a_res_log"]

    sys.path.insert(0, str(REPO / "flying_drone_stack/tools"))
    from decode_usd_log import load as load_usd  # noqa: E402

    d = load_usd(str(spec["usd"]))
    t = np.asarray(d["t"], float)
    lag = float(z["lag_s"])
    total = float(meta["duration"])
    margin = 3.0
    mask = (t >= lag - margin) & (t <= lag + total + margin)
    rpm_raw = np.stack(
        [d["motor_m1_rpm"][mask], d["motor_m2_rpm"][mask], d["motor_m3_rpm"][mask], d["motor_m4_rpm"][mask]],
        axis=1,
    )
    rpm_raw = np.repeat(rpm_raw, 2, axis=0)
    rpm_slew = dshot_rpm_stream(rpm_raw)

    variants = {}
    variants["raw_acc_npz_rpm"] = hand_a_res(
        z["quat"], z["acc_g"], z["rpm"], kt, acc_fc=None, acc_dt=0.001, res_fc=0.0, res_clamp=0.0
    )
    variants["raw_acc_res80"] = hand_a_res(
        z["quat"], z["acc_g"], z["rpm"], kt, acc_fc=None, acc_dt=0.001, res_fc=80.0, res_clamp=10.0
    )
    variants["acc206_res80"] = hand_a_res(
        z["quat"], z["acc_g"], z["rpm"], kt, acc_fc=206.0, acc_dt=0.001, res_fc=80.0, res_clamp=10.0
    )
    variants["acc206_res80_slew_rpm"] = hand_a_res(
        z["quat"], z["acc_g"], rpm_slew, kt, acc_fc=206.0, acc_dt=0.001, res_fc=80.0, res_clamp=10.0
    )

    x_stats = {name: corr_on_steady(z, v[:, 0], log_a[:, 0]) for name, v in variants.items()}

    steady = steady_indices(z, np.arange(int(z["warmup_start"]) + WARMUP_TICKS, len(z["pos"])))
    steady = steady[steady % 2 == 0]
    rep = np.load(OUT / "rung0" / label / "replay_cache.npz", allow_pickle=False) if (
        OUT / "rung0" / label / "replay_cache.npz"
    ).exists() else None
    replay_x_corr = None
    if rep is not None:
        idx_map = {int(t): i for i, t in enumerate(rep["ticks"])}
        sel = [idx_map[t] for t in steady if int(t) in idx_map]
        replay_x_corr = float(np.corrcoef(log_a[steady, 0], rep["a_res"][sel, 0])[0, 1])

    best = max(x_stats.items(), key=lambda kv: kv[1]["corr"])
    cause_found = best[1]["corr"] > 0.95 and best[0] != "raw_acc_npz_rpm"

    rms_raw = x_stats["raw_acc_npz_rpm"]["rms_err"]
    thrust_impact_mN = MASS * rms_raw * 1000.0

    return {
        "label": label,
        "lib_rs_a_res_inputs": {
            "rpm": "rpm_get_all (DShot+motor_* when rpm_source=1; slew hold in traj_iface.c)",
            "a_model": "sum(kt_i*rpm_i^2)/m along R[:,2] minus g",
            "a_meas": "R @ (acc_body * g) - g; acc_body from sensors.acc with optional bw_acc_* @ fc_bw",
            "ENABLE_ACC_PREFILTER": True,
            "acc_prefilter_fc": "g_indi_fc_bw (206 Hz flight), filt_dt_us=1000 -> dt=1ms at init",
            "res_clamp": "g_indi_res_clamp (10 m/s^2), per-side before subtract",
            "res_fc": "g_indi_res_fc (80 Hz), paired filters on meas and model, seed on first use",
        },
        "x_axis_stats": x_stats,
        "replay_x_corr": replay_x_corr,
        "cause_found": cause_found,
        "best_variant": best[0] if cause_found else None,
        "unresolved_residual": {
            "mean_abs_err_mps2": x_stats["raw_acc_npz_rpm"]["mean_abs_err"],
            "rms_err_mps2": rms_raw,
            "thrust_impact_estimate_mN": thrust_impact_mN,
            "note": "m·RMS(Δa_res_x) upper bound if x residual entered thrust via res_sign (geometric mode: a_indi=0)",
        },
    }


def main() -> None:
    label = sys.argv[1] if len(sys.argv) > 1 else RUNG0_FLIGHTS[0]["label"]
    out = OUT / "rung0" / "x_investigation.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    rep = investigate(label)
    out.write_text(json.dumps(rep, indent=2))
    print(json.dumps(rep, indent=2))


if __name__ == "__main__":
    main()
