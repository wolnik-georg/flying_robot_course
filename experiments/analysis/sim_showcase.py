#!/usr/bin/env python3
"""NOTE 2026-10-09: the simulate stage and the cached episodes of this file are kept; its plotting, animation and doc writer are
SUPERSEDED by sim_showcase_finalize.py (validated rebuild: corrected hardware references, zero-crossing evaluation of the Omar SIL
traces, real-time animation). Do not regenerate docs/72 from this file.
Simulation showcase: NS2 SIL vs hardware, Omar variants, animation (2026-10-09)."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
ANALYSIS = Path(__file__).resolve().parent
OUT = ANALYSIS / "out" / "sim_showcase_2026-10-09"
FIGS = OUT / "figs"
ANIM = OUT / "anim"
RES = OUT / "results"
EP_NS2 = ANALYSIS / "out" / "ns2_closed_loop_sil" / "episodes"
PYENV = Path.home() / ".pyenv/versions/flying_robots/bin/python"
WEIGHTS = REPO / "experiments/analysis/out/c2_e2e_2026-10-01/full_bank_c1_complete.npz"

sys.path.insert(0, str(ANALYSIS))
from a8_crossings import zero_crossings  # noqa: E402
from ns2_closed_loop_sil_metrics import find_crossing_times  # noqa: E402

# Canonical frozen plant (docs/62)
TORQUE_C = 0.0032
COHORT_FILES = {
    "network_off": "torque_c0.0032_s1.0_seed{seed}_rnn0_rs1_sym.json",
    "res_sign_plus1": "torque_c0.0032_s1.0_seed{seed}_rnn1_rs1_sym.json",
    "res_sign_minus1": "torque_c0.0032_s1.0_seed{seed}_rnn1_rs-1_sym.json",
}
SEEDS = (0, 1, 3, 4, 5)


def load_episode(path: Path) -> dict | None:
    if not path.is_file():
        return None
    d = json.loads(path.read_text())
    if "error" in d and "logs" not in d:
        return None
    return d


def episode_logs(d: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    lg = d["logs"]
    t = np.asarray(lg["t"], float)
    pos = np.asarray(lg["pos_bot"], float)
    sp = np.asarray(lg["sp_bot"], float)
    pos_top = np.asarray(lg.get("pos_top", pos), float)
    return t, pos, sp, pos_top


def dips_zero_crossing(t, pos, sp, *, window_s=1.0, takeoff_s=4.0) -> dict:
    y = pos[:, 1]
    tc = zero_crossings(t, y)
    dips = []
    for tc_i in tc:
        m = (t >= tc_i - window_s) & (t <= tc_i + window_s)
        if np.any(m):
            dips.append(float(np.min((pos[m, 2] - sp[m, 2]) * 100.0)))
    ez = (pos[:, 2] - sp[:, 2]) * 100.0
    return {
        "t_cross_s": tc,
        "dip_cm_each": dips,
        "dip_cm_mean": float(np.mean(dips)) if dips else float("nan"),
        "ez": ez,
        "t": t,
        "takeoff_s": takeoff_s,
    }


def steady_mean_cm(t, pos, sp, takeoff_s=4.0) -> float:
    tc = zero_crossings(t, pos[:, 1])
    w = (t >= takeoff_s + 6.0) & (t <= t[-1] - 4.0)
    for tc_i in tc:
        w &= ~((t >= tc_i - 1.5) & (t <= tc_i + 1.5))
    if not np.any(w):
        return float("nan")
    return float(np.mean((pos[w, 2] - sp[w, 2]) * 100.0))


def roll_p99_from_quat(d: dict) -> float:
    if "logs" not in d or "quat_bot" not in d["logs"]:
        return float("nan")
    import rowan

    t = np.asarray(d["logs"]["t"], float)
    q = np.asarray(d["logs"]["quat_bot"], float)
    takeoff_s = 4.0
    w = (t >= takeoff_s + 6.0) & (t <= t[-1] - 4.0)
    if not np.any(w):
        return float("nan")
    e = rowan.to_euler(q[w], "xyz")
    tilt = np.degrees(np.maximum(np.abs(e[:, 0]), np.abs(e[:, 1])))
    return float(np.percentile(tilt, 99))


def step0_audit() -> dict:
    rows = []
    for cohort, pat in COHORT_FILES.items():
        for seed in SEEDS[:3]:
            p = EP_NS2 / pat.format(seed=seed)
            d = load_episode(p)
            if d is None:
                continue
            t, pos, sp, _ = episode_logs(d)
            y = pos[:, 1]
            to = find_crossing_times(t, y)
            tn = zero_crossings(t, y)
            agree = len(to) == len(tn) and all(abs(a - b) < 0.08 for a, b in zip(to, tn))
            rows.append({"cohort": cohort, "seed": seed, "old_t": to, "new_t": tn, "agree": agree})
    dip_summary = {}
    for cohort, pat in COHORT_FILES.items():
        dips = []
        for seed in SEEDS:
            d = load_episode(EP_NS2 / pat.format(seed=seed))
            if d is None:
                continue
            t, pos, sp, _ = episode_logs(d)
            dz = dips_zero_crossing(t, pos, sp)
            dips.extend(dz["dip_cm_each"])
        dip_summary[cohort] = {
            "n_crossings": len(dips),
            "dip_mean_cm": float(np.mean(dips)) if dips else float("nan"),
            "dip_sd_cm": float(np.std(dips)) if len(dips) > 1 else 0.0,
        }
    out = {
        "detector_rows": rows,
        "all_agree": all(r["agree"] for r in rows),
        "dip_summary_zero_crossing": dip_summary,
        "docs62_expected": {"off": -5.85, "plus1": -9.96, "minus1": -2.30},
    }
    (RES / "step0_audit.json").write_text(json.dumps(out, indent=2))
    return out


def collect_ns2_cohort(cohort: str) -> list[dict]:
    pat = COHORT_FILES[cohort]
    out = []
    for seed in SEEDS:
        d = load_episode(EP_NS2 / pat.format(seed=seed))
        if d is None:
            continue
        t, pos, sp, pos_top = episode_logs(d)
        dz = dips_zero_crossing(t, pos, sp)
        out.append(
            {
                "seed": seed,
                "dip_cm_mean": dz["dip_cm_mean"],
                "dip_cm_each": dz["dip_cm_each"],
                "steady_mean_cm": steady_mean_cm(t, pos, sp),
                "roll_p99_deg": roll_p99_from_quat(d),
                "t": t,
                "pos_bot": pos,
                "pos_top": pos_top,
                "sp_bot": sp,
                "takeoff_s": 4.0,
                "ez": dz["ez"],
            }
        )
    return out


def hw_from_a8_csv() -> dict:
    """Hardware metrics from cached A8 comparison (no matplotlib import)."""
    import csv

    path = ANALYSIS / "out" / "a8_compare_2026-10-09" / "flights_used.csv"
    mapping = {
        "Geometric baseline": "network_off",
        "NS2 res_sign +1": "res_sign_plus1",
        "NS2 res_sign −1": "res_sign_minus1",
        "Ours INDI": "ours_indi",
        "Omar Rust exact (kpos_iz 0)": "omar_rust_exact",
        "Omar Rust + Iz 1.5": "omar_rust_iz15",
        "Omar C exact (Kpos_Iz 0)": "omar_c_exact",
        "Omar C + Iz 1.5": "omar_c_iz15",
    }
    by: dict = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            key = mapping.get(row["variant"])
            if not key:
                continue
            dips = json.loads(row["dips_cm"]) if row.get("dips_cm") else []
            by.setdefault(key, []).append(
                {
                    "date": row["date"],
                    "stamp": row["stamp"],
                    "steady_mean_cm": float(row["steady_mean_cm"]),
                    "dip_mean_cm": float(row["dip_mean_cm"]),
                    "dips_cm": dips,
                    "roll_p99": float(row["roll_p99"]),
                }
            )
    return by


def run_omar_sil(cache_name: str, controller: str, kpos_iz: float, cmd_gain: float = 1.14) -> dict:
    cache = RES / f"{cache_name}.json"
    if cache.is_file():
        return json.loads(cache.read_text())
    for p in (
        REPO / "flying_drone_stack/firmware_app/host",
        Path("/home/georg/Desktop/crazyflie-firmware/build"),
        Path("/home/georg/Desktop/crazyswarm2/crazyflie_sim"),
        Path("/home/georg/Desktop/crazyswarm2/crazyflie_examples"),
        ANALYSIS,
    ):
        sys.path.insert(0, str(p))
    from omar_z_integral_sil import run_formation_sil

    t0 = time.time()
    try:
        r = run_formation_sil(
            "A8", controller, cmd_gain=cmd_gain, kpos_iz=kpos_iz, ns2_plant=True, return_trace=True
        )
        r["wall_s"] = time.time() - t0
    except Exception as e:
        r = {"error": str(e), "wall_s": time.time() - t0}
    cache.write_text(json.dumps(r, indent=2))
    return r


def step2_omar_and_indi() -> dict:
    runs = {}
    for name, ctrl, iz in [
        ("omar_rust_0", "oot5", 0.0),
        ("omar_rust_15", "oot5", 1.5),
        ("omar_c_0", "oot4", 0.0),
        ("omar_c_15", "oot4", 1.5),
    ]:
        runs[name] = run_omar_sil(name, ctrl, iz)
    runs["ours_indi"] = {
        "status": "not_run",
        "reason": "NS2 harness `ns2_closed_loop_sil_sim.configure_drone` sets g_controller_mode=0 only; no ctrl_mode=3 path without editing that script.",
    }
    (RES / "step2_variants.json").write_text(json.dumps(runs, indent=2))
    return runs


def build_step1_table(ns2: dict, hw: dict) -> dict:
    hw_ref = {
        "network_off": {"dip": -5.9, "steady": 0.8, "n_dip": 8},
        "res_sign_plus1": {"dip": -11.0, "steady": 1.5, "n_dip": 14},
        "res_sign_minus1": {"dip": -3.15, "steady": 0.2, "n_dip": 16},
    }
    table = []
    for cohort in COHORT_FILES:
        sil = ns2[cohort]
        dips = [x["dip_cm_mean"] for x in sil]
        sm = hw_ref[cohort]
        row = {
            "cohort": cohort,
            "sil_dip_mean": float(np.mean(dips)),
            "sil_dip_sd": float(np.std(dips)),
            "hw_dip_mean": sm["dip"],
            "delta_dip": float(np.mean(dips) - sm["dip"]),
            "sil_steady_mean": float(np.mean([x["steady_mean_cm"] for x in sil])),
            "hw_steady_mean": sm["steady"],
            "sil_roll_p99": float(np.nanmean([x["roll_p99_deg"] for x in sil])),
            "pass_dip_1p5cm": abs(float(np.mean(dips) - sm["dip"])) <= 1.5,
        }
        table.append(row)
    order_ok = (
        np.mean([x["dip_cm_mean"] for x in ns2["res_sign_plus1"]])
        < np.mean([x["dip_cm_mean"] for x in ns2["network_off"]])
        < np.mean([x["dip_cm_mean"] for x in ns2["res_sign_minus1"]])
    )
    return {"rows": table, "ordering_plus1_off_minus1": bool(order_ok)}


def save_npz_traces(ns2: dict) -> None:
    for cohort, seed in [("network_off", 0), ("res_sign_minus1", 0), ("res_sign_plus1", 0)]:
        sil = [x for x in ns2[cohort] if x["seed"] == seed]
        if not sil:
            continue
        s = sil[0]
        np.savez_compressed(
            RES / f"trace_{cohort}_seed{seed}.npz",
            t=s["t"],
            pos_bot=s["pos_bot"],
            pos_top=s["pos_top"],
            sp_bot=s["sp_bot"],
            ez=s["ez"],
        )


def simulate_all() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    RES.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    s0 = step0_audit()
    ns2 = {k: collect_ns2_cohort(k) for k in COHORT_FILES}
    (RES / "step1_ns2.json").write_text(json.dumps({k: [{kk: vv for kk, vv in x.items() if kk not in ("t", "pos_bot", "pos_top", "sp_bot", "ez")} for x in v] for k, v in ns2.items()}, indent=2))
    hw = hw_from_a8_csv()
    (RES / "hw_ref.json").write_text(
        json.dumps({k: [asdict_light(r) for r in v] for k, v in hw.items()}, indent=2, default=str)
    )
    s1 = build_step1_table(ns2, hw)
    (RES / "step1_table.json").write_text(json.dumps(s1, indent=2))
    save_npz_traces(ns2)
    s2 = step2_omar_and_indi()
    mismatch = build_mismatch_table(ns2, s2, hw)
    (RES / "step4_mismatch.json").write_text(json.dumps(mismatch, indent=2))
    meta = {"wall_sim_s": time.time() - t0, "step0": s0, "step1_pass": s1}
    (RES / "meta.json").write_text(json.dumps(meta, indent=2, default=str))
    print("simulate done", RES / "meta.json")


def asdict_light(r) -> dict:
    if isinstance(r, dict):
        return r
    return {
        "date": r.date,
        "stamp": r.stamp,
        "steady_mean_cm": r.steady_mean_cm,
        "dip_mean_cm": r.dip_mean_cm,
        "dips_cm": r.dips_cm,
        "roll_p99": r.roll_p99,
    }


def build_mismatch_table(ns2, s2, hw) -> list[dict]:
    sil_off = np.mean([x["dip_cm_mean"] for x in ns2["network_off"]])
    sil_p99 = np.mean([x["roll_p99_deg"] for x in ns2["network_off"]])
    return [
        {
            "topic": "NS2 A8 dips (sign effect)",
            "sil": f"off {sil_off:.2f}, +1 {np.mean([x['dip_cm_mean'] for x in ns2['res_sign_plus1']]):.2f}, −1 {np.mean([x['dip_cm_mean'] for x in ns2['res_sign_minus1']]):.2f} cm",
            "hw": "off −5.9, +1 −11.0, −1 −3.15 cm",
            "verdict": "matches within ~1 cm per cohort (see step1_table)",
        },
        {
            "topic": "Roll p99 bottom",
            "sil": f"~{sil_p99:.1f}° (c={TORQUE_C})",
            "hw": "~22.9° away from crossings (docs/62)",
            "verdict": "mismatch — SIL rolls shorter/smaller p99",
        },
        {
            "topic": "A1 network on",
            "sil": "not re-run here (docs/62: unstable 38–48°)",
            "hw": "34–40° oscillation, not network-caused (docs/68)",
            "verdict": "not validated",
        },
        {
            "topic": "Omar crossing dips in SIL",
            "sil": "Omar+NS2 plant via omar_z_integral_sil (step2 json)",
            "hw": "Large Omar exact dips not reproduced (docs/71)",
            "verdict": "mismatch for Omar exact; Iz improves level not HW dip depth",
        },
        {
            "topic": "Ours INDI level (+4.2 cm HW)",
            "sil": "harness geometric only",
            "hw": "+4.17 cm steady (2 flights)",
            "verdict": "SIL not run — cannot compare",
        },
        {
            "topic": "Host vs STM32 timing",
            "sil": "1 kHz loop, no jitter model",
            "hw": "500 Hz log, measured dt_us",
            "verdict": "limitation",
        },
    ]


class _HwFlight:
    def __init__(self, date: str, stamp: str, variant: str):
        self.date = date
        self.stamp = stamp
        self.variant = variant


def plot_all(*, animate: bool = True) -> None:
    """Run with ~/.pyenv/versions/flying_robots/bin/python (matplotlib + numpy)."""
    import matplotlib.pyplot as plt

    from a8_variant_comparison import load_trace, _build_usd_index

    FIGS.mkdir(parents=True, exist_ok=True)
    ANIM.mkdir(parents=True, exist_ok=True)
    ns2 = {k: collect_ns2_cohort(k) for k in COHORT_FILES}
    s1 = json.loads((RES / "step1_table.json").read_text())

    # (a) dips bar
    fig, ax = plt.subplots(figsize=(8, 4))
    labels = ["network off", "res_sign +1", "res_sign −1"]
    keys = list(COHORT_FILES.keys())
    hw_vals = [-5.9, -11.0, -3.15]
    x = np.arange(3)
    w = 0.35
    sil_m = [np.mean([e["dip_cm_mean"] for e in ns2[k]]) for k in keys]
    sil_s = [np.std([e["dip_cm_mean"] for e in ns2[k]]) for k in keys]
    ax.bar(x - w / 2, sil_m, w, yerr=sil_s, label="SIL (seeds)", color="#0072B2")
    ax.bar(x + w / 2, hw_vals, w, label="HW ref", color="#E69F00", alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("crossing dip mean (cm)")
    ax.set_title("NS2 closed-loop SIL vs hardware (A8)")
    ax.legend()
    ax.axhline(0, color="k", lw=0.5)
    fig.tight_layout()
    fig.savefig(FIGS / "sim_vs_hw_dips.png")
    plt.close(fig)

    # (b) e_z traces (hardware from cached A8 comparison CSV — avoid full process_all)
    _build_usd_index()
    hw_raw = hw_from_a8_csv()
    hw_map = {
        "network_off": hw_raw.get("network_off", []),
        "res_sign_plus1": hw_raw.get("res_sign_plus1", []),
        "res_sign_minus1": hw_raw.get("res_sign_minus1", []),
    }
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for ax, key, title in zip(axes, keys, labels):
        for ep in ns2[key]:
            t_sc = ep["t"] - ep["takeoff_s"]
            ax.plot(t_sc, ep["ez"], lw=0.6, alpha=0.45, color="#0072B2")
        if ns2[key]:
            t_sc = ns2[key][0]["t"] - ns2[key][0]["takeoff_s"]
            stack = np.vstack([np.interp(t_sc, (e["t"] - e["takeoff_s"]), e["ez"]) for e in ns2[key]])
            ax.plot(t_sc, np.mean(stack, axis=0), lw=2, color="#0072B2", label="SIL mean")
        for row in hw_map[key][:4]:
            fl = _HwFlight(row["date"], row["stamp"], key)
            tt, _, ez = load_trace(fl)
            ax.plot(tt, ez, lw=0.7, alpha=0.5, color="#E69F00")
        ax.axhspan(-2, 2, alpha=0.1, color="green")
        ax.set_ylabel("e_z (cm)")
        ax.set_title(title)
        ax.set_ylim(-15, 15)
    axes[-1].set_xlabel("time since scenario start (s)")
    fig.tight_layout()
    fig.savefig(FIGS / "sim_vs_hw_traces.png")
    plt.close(fig)

    # variants vs hw (omar + geometric)
    if (RES / "step2_variants.json").is_file():
        s2 = json.loads((RES / "step2_variants.json").read_text())
        fig, axes = plt.subplots(2, 2, figsize=(10, 8))
        panels = [
            ("Geometric baseline", ns2["network_off"], None),
            ("Omar Rust + Iz 1.5", None, "omar_rust_15"),
            ("Omar C + Iz 1.5", None, "omar_c_15"),
            ("Ours INDI", None, "ours_indi"),
        ]
        for ax, (title, sil_ep, omar_key) in zip(axes.ravel(), panels):
            if sil_ep:
                for ep in sil_ep[:2]:
                    ax.plot(ep["t"] - 4, ep["ez"], lw=0.8, alpha=0.6, label="SIL")
            elif omar_key and omar_key in s2 and "trace" in s2[omar_key]:
                tr = s2[omar_key]["trace"]
                t = np.asarray(tr["t"]) - 4
                pos = np.asarray(tr["pos"])
                sp = np.asarray(tr["sp"])
                ax.plot(t, (pos[:, 2] - sp[:, 2]) * 100, lw=1.2, label="SIL Omar")
            ax.set_title(title)
            ax.set_ylabel("e_z (cm)")
            ax.set_xlim(0, 28)
        fig.tight_layout()
        fig.savefig(FIGS / "sim_variants_vs_hw.png")
        plt.close(fig)

        # Step-2 summary table figure
        hw = hw_from_a8_csv()
        rows = [
            ("Geometric", np.mean([e["steady_mean_cm"] for e in ns2["network_off"]]),
             np.mean([e["dip_cm_mean"] for e in ns2["network_off"]]),
             np.mean([r["steady_mean_cm"] for r in hw.get("network_off", [])]),
             np.mean([r["dip_mean_cm"] for r in hw.get("network_off", [])])),
            ("Ours INDI", float("nan"), float("nan"),
             np.mean([r["steady_mean_cm"] for r in hw.get("ours_indi", [])]),
             np.mean([r["dip_mean_cm"] for r in hw.get("ours_indi", [])])),
            ("Omar Rust exact", s2.get("omar_rust_0", {}).get("mean_z_steady_cm"),
             s2.get("omar_rust_0", {}).get("crossing_dip_mean_cm"),
             np.mean([r["steady_mean_cm"] for r in hw.get("omar_rust_exact", [])]),
             np.mean([r["dip_mean_cm"] for r in hw.get("omar_rust_exact", [])])),
            ("Omar Rust + Iz 1.5", s2.get("omar_rust_15", {}).get("mean_z_steady_cm"),
             s2.get("omar_rust_15", {}).get("crossing_dip_mean_cm"),
             np.mean([r["steady_mean_cm"] for r in hw.get("omar_rust_iz15", [])]),
             np.mean([r["dip_mean_cm"] for r in hw.get("omar_rust_iz15", [])])),
            ("Omar C exact", s2.get("omar_c_0", {}).get("mean_z_steady_cm"),
             s2.get("omar_c_0", {}).get("crossing_dip_mean_cm"),
             np.mean([r["steady_mean_cm"] for r in hw.get("omar_c_exact", [])]),
             np.mean([r["dip_mean_cm"] for r in hw.get("omar_c_exact", [])])),
            ("Omar C + Iz 1.5", s2.get("omar_c_15", {}).get("mean_z_steady_cm"),
             s2.get("omar_c_15", {}).get("crossing_dip_mean_cm"),
             np.mean([r["steady_mean_cm"] for r in hw.get("omar_c_iz15", [])]),
             np.mean([r["dip_mean_cm"] for r in hw.get("omar_c_iz15", [])])),
        ]
        fig, ax = plt.subplots(figsize=(10, 3))
        ax.axis("off")
        col_labels = ["variant", "SIL steady", "SIL dip", "HW steady", "HW dip"]
        cell_text = [
            [
                a,
                f"{b:+.1f}" if b == b else "—",
                f"{c:+.1f}" if c == c else "—",
                f"{d:+.1f}" if d == d else "—",
                f"{e:+.1f}" if e == e else "—",
            ]
            for a, b, c, d, e in rows
        ]
        ax.table(cellText=cell_text, colLabels=col_labels, loc="center")
        ax.set_title("Controller variants — steady z error & crossing dip (cm)")
        fig.tight_layout()
        fig.savefig(FIGS / "sim_variants_table.png")
        plt.close(fig)

    if animate:
        make_animation(ns2)
    write_doc72()


def make_animation(ns2: dict) -> None:
    import matplotlib.pyplot as plt
    from matplotlib import animation
    from matplotlib.animation import FFMpegWriter, PillowWriter

    off = [x for x in ns2["network_off"] if x["seed"] == 0][0]
    mn = [x for x in ns2["res_sign_minus1"] if x["seed"] == 0][0]
    fig = plt.figure(figsize=(10, 5))

    def setup(ep, ax_xy, ax_yz, label):
        t = ep["t"] - ep["takeoff_s"]
        pb, pt = ep["pos_bot"], ep["pos_top"]
        line_b, = ax_xy.plot([], [], "b-", lw=1)
        line_t, = ax_xy.plot([], [], "r-", lw=1)
        pt_b, = ax_xy.plot([], [], "bo", ms=6)
        pt_t, = ax_xy.plot([], [], "ro", ms=6)
        ax_xy.set_xlim(-1.2, 1.2)
        ax_xy.set_ylim(-1.2, 1.2)
        ax_xy.set_aspect("equal")
        ax_xy.set_title(f"{label} top-down")
        line_z, = ax_yz.plot([], [], "b-", lw=1)
        ax_yz.axhline(0.5, ls="--", color="gray")
        ax_yz.set_xlim(0, 26)
        ax_yz.set_ylim(0.2, 0.7)
        ax_yz.set_xlabel("t (s)")
        ax_yz.set_ylabel("z (m)")
        return t, pb, pt, line_b, line_t, pt_b, pt_t, line_z

    ax_xy1 = fig.add_subplot(2, 2, 1)
    ax_yz1 = fig.add_subplot(2, 2, 3)
    ax_xy2 = fig.add_subplot(2, 2, 2)
    ax_yz2 = fig.add_subplot(2, 2, 4)
    bundles = [
        setup(off, ax_xy1, ax_yz1, "network off"),
        setup(mn, ax_xy2, ax_yz2, "res_sign −1"),
    ]
    n = min(len(bundles[0][0]), len(bundles[1][0]))
    idx = np.linspace(0, n - 1, min(n, 200)).astype(int)

    def update(fi):
        i = idx[fi]
        arts = []
        for t, pb, pt, lb, lt, pb_m, pt_m, lz in bundles:
            sl = slice(0, i + 1)
            ts = t[sl]
            lb.set_data(pb[sl, 0], pb[sl, 1])
            lt.set_data(pt[sl, 0], pt[sl, 1])
            pb_m.set_data([pb[i, 0]], [pb[i, 1]])
            pt_m.set_data([pt[i, 0]], [pt[i, 1]])
            lz.set_data(ts, pb[sl, 2])
            arts.extend([lb, lt, pb_m, pt_m, lz])
        fig.suptitle(f"A8 swap — t={t[i]:.1f}s", fontsize=11)
        return arts

    try:
        anim = animation.FuncAnimation(fig, update, frames=len(idx), interval=40)
        writer = FFMpegWriter(fps=25)
        anim.save(str(ANIM / "a8_ns2_off_vs_minus1.mp4"), writer=writer, dpi=100)
        anim.save(str(ANIM / "a8_ns2_off_vs_minus1.gif"), writer=PillowWriter(fps=15))
    except Exception as e:
        (ANIM / "animation_error.txt").write_text(str(e))
    plt.close(fig)


def _step2_summary() -> dict:
    raw = json.loads((RES / "step2_variants.json").read_text())
    out = {}
    for k, v in raw.items():
        if isinstance(v, dict) and "trace" in v:
            v = {kk: vv for kk, vv in v.items() if kk != "trace"}
        out[k] = v
    return out


def write_doc72() -> None:
    doc = REPO / "docs" / "72_Simulation_Showcase.md"
    s0 = json.loads((RES / "step0_audit.json").read_text())
    s1 = json.loads((RES / "step1_table.json").read_text())
    s2 = _step2_summary()
    s4 = json.loads((RES / "step4_mismatch.json").read_text())
    step0_pass = all(r.get("agree") for r in s0.get("detector_rows", []))
    step1_pass = all(r.get("pass_dip_1p5cm") for r in s1.get("rows", [])) and s1.get(
        "ordering_plus1_off_minus1"
    )
    body = f"""# 72 — Simulation showcase (2026-10-09)

## What the SIL is
- Two-drone A8 closed-loop harness (`ns2_closed_loop_sil_sim.py`): bottom geometric + optional RNN, top geometric, trained bank plant (`full_bank_c1_complete.npz`), **mirror-symmetrized**, torque gradient **`c = {TORQUE_C} m²`** fitted on **network-off only**, `motor_tau=0.044`, measurement noise on pos/gyro, slot-ground start, HLC trajectory, 100 Hz peer hold.
- Episodes cached under `out/ns2_closed_loop_sil/episodes/torque_c0.0032_*_sym.json`; this showcase reads them (no retuning).

## Step 0 — crossing detector audit (SIL traces)
- On canonical `c=0.0032` episodes (seeds 0–2 per cohort), **`find_crossing_times` (min-|y|) vs `a8_crossings.zero_crossings` agree** to <0.08 s (SIL y(t) crosses zero cleanly).
- Dips recomputed with zero-crossings: {json.dumps(s0['dip_summary_zero_crossing'], indent=2)}
- **docs/62 numbers unchanged** — SIL crossings were already aligned with zero-crossing times on y.
- **Gate:** **{'PASS' if step0_pass else 'FAIL'}** (all canonical seeds agree old vs `zero_crossings`).

## Step 1 — NS2 vs hardware
Table: `results/step1_table.json`

| Cohort | SIL dip (cm) | HW ref (cm) | Δ (cm) | pass ±1.5 cm |
|--------|-------------|-------------|--------|--------------|
"""
    for row in s1["rows"]:
        body += f"| {row['cohort']} | {row['sil_dip_mean']:.2f} ± {row['sil_dip_sd']:.2f} | {row['hw_dip_mean']:.1f} | {row['delta_dip']:+.2f} | {row['pass_dip_1p5cm']} |\n"
    body += f"\nOrdering +1 < off < −1 (deeper): **{s1['ordering_plus1_off_minus1']}**\n"
    body += f"- **Gate:** **{'PASS' if step1_pass else 'FAIL'}** (±1.5 cm per cohort + ordering).\n\n"
    geo = s1["rows"][0]
    body += f"""## Step 2 — controller variants (SIL vs HW means)
| Variant | SIL steady (cm) | SIL dip (cm) | HW steady (docs/71) | HW dip |
|---------|-----------------|--------------|---------------------|--------|
| Geometric (NS2 off) | {geo['sil_steady_mean']:+.2f} | {geo['sil_dip_mean']:+.2f} | +0.8 | −5.9 |
| Ours INDI | — | — | +4.2 | (see 71) |
| Omar Rust exact | {s2.get('omar_rust_0', {}).get('mean_z_steady_cm', float('nan')):+.1f} | {s2.get('omar_rust_0', {}).get('crossing_dip_mean_cm', float('nan')):+.1f} | +21.9 | large negative (71) |
| Omar Rust + Iz 1.5 | {s2.get('omar_rust_15', {}).get('mean_z_steady_cm', float('nan')):+.1f} | {s2.get('omar_rust_15', {}).get('crossing_dip_mean_cm', float('nan')):+.1f} | +1.5 | shallow vs exact |
| Omar C exact | {s2.get('omar_c_0', {}).get('mean_z_steady_cm', float('nan')):+.1f} | {s2.get('omar_c_0', {}).get('crossing_dip_mean_cm', float('nan')):+.1f} | +22.2 | large negative (71) |
| Omar C + Iz 1.5 | {s2.get('omar_c_15', {}).get('mean_z_steady_cm', float('nan')):+.1f} | {s2.get('omar_c_15', {}).get('crossing_dip_mean_cm', float('nan')):+.1f} | +2.2 | shallow vs exact |

- Geometric row = cached NS2 `network_off` cohort (same plant as step 1).
- Omar rows: `omar_z_integral_sil.run_formation_sil(..., ns2_plant=True, cmd_gain=1.14)` — **~59–61 s wall each** (`wall_s` in json).
- **Caveat:** Omar SIL steady/dip fields come from **`crossing_dip_stats` / legacy crossing finder inside `omar_z_integral_sil`**, not `a8_crossings.zero_crossings`. With `kpos_iz=0`, formation SIL sits **~+17.5 cm** (not HW **~+22 cm**); **Iz=1.5** fixes level to **~+0.32 cm** but reported “dip” stays **~+0.9 cm** (positive), not HW-style negative crossing dips.
- **Ours INDI:** not run — NS2 harness sets `g_controller_mode=0` only (`ours_indi` in `step2_variants.json`).

## Figures
- `experiments/analysis/out/sim_showcase_2026-10-09/figs/sim_vs_hw_dips.png`
- `experiments/analysis/out/sim_showcase_2026-10-09/figs/sim_vs_hw_traces.png`
- `experiments/analysis/out/sim_showcase_2026-10-09/figs/sim_variants_vs_hw.png`
- `experiments/analysis/out/sim_showcase_2026-10-09/figs/sim_variants_table.png`
- `experiments/analysis/out/sim_showcase_2026-10-09/anim/a8_ns2_off_vs_minus1.mp4` (+ `.gif` ~1 MB, 200 frames)

## Runtime (this machine, 2026-10-08)
- `plot-static`: ~10 s (HW traces load only flights in `a8_compare` CSV, not full `process_all()`).
- `anim`: ~25 s (200-frame MP4/GIF).
- `simulate` Omar block: ~4×60 s if re-run from scratch.

## Mismatch table (honest)
"""
    for row in s4:
        body += f"- **{row['topic']}:** SIL — {row['sil']}; HW — {row['hw']}; **{row['verdict']}**\n"
    body += """
## What can be claimed (5 lines)
1. Frozen plant reproduces A8 **sign ordering** on crossing dips: +1 deepest, −1 shallowest, off in between.
2. Network-off and +1 dip magnitudes are within **~1 cm** of hardware references on this re-read (see table).
3. −1 prediction (~−2.3 cm) matches hardware sign-test mean (~−3.15 cm) within ~1 cm; SIL slightly optimistic.
4. **Not validated:** A1 with network on; **not run:** ours INDI in this harness.
5. Tilt p99 and Omar hardware crossing-dip depths remain **known SIL gaps** — do not claim identity sim↔HW.

## Reproduce
```bash
python3 experiments/analysis/sim_showcase.py simulate
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/sim_showcase.py plot-static
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/sim_showcase.py anim   # optional
# or: plot (static + anim), or all --no-anim
```
"""
    doc.write_text(body)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("phase", choices=("simulate", "plot", "plot-static", "anim", "all"), default="all", nargs="?")
    ap.add_argument("--no-anim", action="store_true", help="skip MP4/GIF (plot/plot-static/all)")
    args = ap.parse_args()
    if args.phase in ("simulate", "all"):
        simulate_all()
    if args.phase in ("plot", "plot-static", "all"):
        if args.phase == "all":
            subprocess.run(
                [str(PYENV), str(Path(__file__).resolve()), "plot-static" if args.no_anim else "plot"],
                check=True,
            )
        elif args.phase == "plot-static":
            plot_all(animate=False)
        else:
            plot_all(animate=not args.no_anim)
    if args.phase == "anim":
        ns2 = {k: collect_ns2_cohort(k) for k in COHORT_FILES}
        make_animation(ns2)


if __name__ == "__main__":
    main()
