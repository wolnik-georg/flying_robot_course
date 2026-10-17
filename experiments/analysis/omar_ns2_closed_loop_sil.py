#!/usr/bin/env python3
"""Phase 5 (docs/plan_omar_tree_ns2_2026-10-14.md): closed-loop two-drone SIL of the Omar C tree with NS2 on top.

Bottom drone cf5 = Omar C (controller 'oot4', indi 0 = geometric, 3 = full INDI, Kpos_Iz 1.5, NS2 in C via the
ns2_residual.c hook, rnn.en 0/1); top drone cf_second = our geometric (as in the existing formation harness, the
top drone only generates the downwash). Plant = the downwash bank network (same function as the controller's network,
so the prediction is perfect: an UPPER BOUND, `scale` makes the real force larger/smaller than predicted; a negative
scale reverses the force and is the negative control). Scenario A8 (4 crossings).

  python3 experiments/analysis/omar_ns2_closed_loop_sil.py [--jobs 4]      (system python3, 3.10 bindings)
Output: experiments/analysis/out/omar_ns2_closed_loop_sil/results.json + summary.md
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / "out" / "omar_ns2_closed_loop_sil"


def run_one(cfg: dict) -> dict:
    sys.path.insert(0, str(HERE))
    import omar_z_integral_sil as S  # sets up paths, imports cffirmware etc.
    from omar_z_integral_formation_run import rel_dips

    fw, FSC = S.fw, S.FSC
    CrazyflieSIL, State, Quadrotor = S.CrazyflieSIL, S.State, S.Quadrotor
    sc = FSC.A8(dz=0.5, span=1.0, duration=6.0, settle=2.0, passes=4)
    anchor = np.array([0.0, 0.0, 0.5])
    slot_bot = anchor + sc.robots[0].slot
    slot_top = anchor + sc.robots[1].slot
    takeoff_s = 4.0
    ph = dict(mass=S.MASS, inertia=S.J, kt=[S.KT] * 4, arm_length=S.ARM, t2t=S.T2T, motor_tau=0.044)
    bank = S.BankPlantDisturbance(S.WEIGHTS)
    bank.symmetrize = True
    torque_c = 0.0032
    indi, en, scale, cg, kiz = int(cfg["indi"]), int(cfg["ns2"]), float(cfg["scale"]), float(cfg.get("cg", 1.14)), float(cfg.get("kiz", 1.5))

    devnull = os.open(os.devnull, os.O_WRONLY)
    o1, o2 = os.dup(1), os.dup(2)
    os.dup2(devnull, 1); os.dup2(devnull, 2)
    try:
        S.CrazyflieSIL._oot_count = 0
        t_box = [0.0]
        p0_bot = np.array([slot_bot[0], slot_bot[1], 0.0])
        p0_top = np.array([slot_top[0], slot_top[1], 0.0])
        cfs = [CrazyflieSIL("cf5", p0_bot, "oot4", lambda: t_box[0]), CrazyflieSIL("cf_second", p0_top, "oot", lambda: t_box[0])]
        cfs[0].omar_indi_control.indi = indi
        if "ns2_mode" in cfg:   # 2026-10-17: 0 = Version A (plain sum, default), 1 = Version B (INDI on what NS2 left over)
            cfs[0].omar_indi_control.ns2_mode = int(cfg["ns2_mode"])
        S.set_omar_kpos_iz("oot4", cfs[0], kiz)
        fw.controllerOutOfTreeInit()
        fw.oot_select_drone(1)
        fw.cvar.g_kp_xy, fw.cvar.g_kp_z = 64.0, 48.0
        fw.cvar.g_kv_xy, fw.cvar.g_kv_z = 8.0, 7.0
        fw.cvar.g_controller_mode = 0
        fw.cvar.g_rnn_div = 10
        for i, cf in enumerate(cfs):
            cf.takeoff(float(slot_bot[2] if i == 0 else slot_top[2]), takeoff_s)
        quads = [Quadrotor(State(pos=p0_bot.copy()), ph), Quadrotor(State(pos=p0_top.copy()), ph)]
        hlc = False
        duration = float(sc.duration) + takeoff_s + 3.0
        t_l, pos_b, sp_b, pred_z = [], [], [], []
        for k in range(1, int(duration / 1e-3) + 1):
            t_now = k * 1e-3
            t_box[0] = t_now
            positions = [q.state.pos.copy() for q in quads]
            vels = [q.state.vel.copy() for q in quads]
            if not hlc and t_now >= takeoff_s:
                for j, cfj in enumerate(cfs):
                    cfj.uploadTrajectory(0, 0, S.curve_to_pieces(sc.robots[j].curve))
                for cfj in cfs:
                    cfj.startTrajectory(0, timescale=1.0, relative=True)
                hlc = True
            actions = []
            for i, cf in enumerate(cfs):
                if i == 1:
                    fw.cvar.g_rnn_en = 0
                    fw.cvar.g_rnn_ready = 0   # shared host global: set by the bottom drone's C hook, must not reach the top (Rust) controller
                    fw.oot_select_drone(1)
                    cf.cmdFullState((float(slot_top[0]), float(slot_top[1]), float(slot_top[2])), (0, 0, 0), (0, 0, 0), 0.0, (0, 0, 0))
                else:
                    fw.cvar.g_rnn_en = en
                    fw.oot_select_drone(0)
                    # the 'oot4' branch of CrazyflieSIL does not inject peers; the NS2 hook reads peer_get_all()
                    p = positions[1]
                    fw.oot_set_peer(0, float(p[0]), float(p[1]), float(p[2]), int(k))
                    fw.oot_set_peer_count(1)
                    cf.getSetpoint()
                cf.peers = [tuple(positions[j].tolist()) for j in range(2) if j != i]
                cf.setState(quads[i].state)
                cf.motors_rpm_meas = [int(x) for x in quads[i].state.rpm]
                actions.append(cf.executeController())
                if i == 0:
                    pz = float(fw.cvar.g_rnn_pred_z)
            for i, (q, act) in enumerate(zip(quads, actions)):
                act_p = S.scale_action_rpm(act, cg)
                f_a = bank.compute_fa_newtons(i, positions, vels, S.MASS, scale=scale)
                tau_a = bank.compute_tau_nm(i, positions, vels, S.MASS, c_lever2=torque_c) if i == 0 else None
                q.step(act_p, 1e-3, f_a)
                if tau_a is not None:
                    q.state.omega = q.state.omega + q.inv_J * np.asarray(tau_a, float) * 1e-3
            if k % 2 == 0:
                t_l.append(t_now); pos_b.append(quads[0].state.pos.copy()); pred_z.append(pz)
                tt = min(max(0.0, t_now - takeoff_s), sc.robots[0].curve.duration)
                sp_b.append(anchor + sc.robots[0].slot + np.asarray(sc.robots[0].curve.at(tt), float)[:3])
        t, pos, sp = np.asarray(t_l), np.asarray(pos_b), np.asarray(sp_b)
        tr = {"t": t[::5].tolist(), "pos": pos[::5].tolist(), "sp": sp[::5].tolist()}
        off, dips = rel_dips({"trace": tr})
        # upward excursions as well (the negative control pushes the drone UP, which a min-based dip cannot see)
        from ns2_closed_loop_sil_metrics import find_crossing_times
        e_cm = (pos[:, 2] - sp[:, 2]) * 100.0
        tcs = find_crossing_times(t, pos[:, 1], n_expect=4)
        bumps = [float(np.max(e_cm[(t >= x - 1.0) & (t <= x + 1.0)]) - off) for x in tcs]
        steady = t >= takeoff_s + 8.0
        return {"cfg": cfg, "offset_cm": off, "dips_rel_cm": dips, "dip_mean_cm": float(np.mean(dips)) if dips else None, "bumps_rel_cm": bumps, "bump_mean_cm": float(np.mean(bumps)) if bumps else None,
                "mean_z_steady_cm": float(np.mean((pos[steady, 2] - sp[steady, 2]) * 100)),
                "max_z_err_cm": float(np.max(np.abs(pos[:, 2] - sp[:, 2])) * 100),
                "pred_z_min": float(np.min(pred_z)), "pred_z_max": float(np.max(pred_z)),
                "crashed": bool(np.min(pos[:, 2]) < 0.05 and t[-1] > 6.0 and np.any(pos[t > takeoff_s + 1, 2] < 0.15))}
    finally:
        os.dup2(o1, 1); os.dup2(o2, 2); os.close(devnull)


def child() -> None:
    cfg = json.loads(sys.argv[2])
    r = run_one(cfg)
    sys.stdout.write("RESULT " + json.dumps(r) + "\n")


def main() -> None:
    if len(sys.argv) > 2 and sys.argv[1] == "--child":
        child(); return
    ap = argparse.ArgumentParser(); ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--scales", default="", help="comma list: run only indi 0/3 x NS2 off/on for these plant scales (extension matrix)")
    ap.add_argument("--indis", default="0,3"); ap.add_argument("--tag", default="")
    ap.add_argument("--mode-matrix", default="", help="2026-10-17: comma list of plant scales: indi 3 + NS2 on with ns2_mode 0 (A) and 1 (B), the INDI-only and NS2-off references, and indi 0 + NS2 on with both modes (must be identical)")
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    jobs = []
    if a.mode_matrix:
        for scale in [float(x) for x in a.mode_matrix.split(",")]:
            jobs.append(dict(indi=3, ns2=0, scale=scale, ns2_mode=0))
            for mode in (0, 1):
                jobs.append(dict(indi=3, ns2=1, scale=scale, ns2_mode=mode))
                jobs.append(dict(indi=0, ns2=1, scale=scale, ns2_mode=mode))
    elif a.scales:
        for scale in [float(x) for x in a.scales.split(",")]:
            for indi in [int(x) for x in a.indis.split(",")]:
                for ns2 in (0, 1):
                    jobs.append(dict(indi=indi, ns2=ns2, scale=scale))
    for scale in (() if (a.scales or a.mode_matrix) else (1.0, 1.3)):
        for indi in (0, 3):
            for ns2 in (0, 1):
                jobs.append(dict(indi=indi, ns2=ns2, scale=scale))
    for indi in (() if (a.scales or a.mode_matrix) else (0, 3)):  # negative control: the real force is the reverse of what NS2 predicts
        for ns2 in (0, 1):
            jobs.append(dict(indi=indi, ns2=ns2, scale=-1.0))

    def one(cfg):
        p = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--child", json.dumps(cfg)], capture_output=True, text=True, cwd=str(HERE))
        for ln in p.stdout.splitlines():
            if ln.startswith("RESULT "):
                return json.loads(ln[7:])
        return {"cfg": cfg, "error": (p.stderr or p.stdout)[-400:]}

    with ThreadPoolExecutor(a.jobs) as ex:
        res = list(ex.map(one, jobs))
    (OUT / f"results{a.tag}.json").write_text(json.dumps(res, indent=1))
    rows = ["| indi | NS2 | plant force scale | dip mean [cm] | bump mean [cm] | offset [cm] | steady z [cm] | max |ez| [cm] | pred_z range [m/s²] |", "|---|---|---|---|---|---|---|---|---|"]
    for r in res:
        c = r["cfg"]
        if "error" in r:
            rows.append(f"| {c['indi']} | {c['ns2']} | {c['scale']} | ERROR {r['error'][:80]} | | | | | |"); continue
        rows.append(f"| {c['indi']} | {'on' if c['ns2'] else 'off'}{' mode ' + str(c['ns2_mode']) if 'ns2_mode' in c else ''} | {c['scale']:+.1f} | {r['dip_mean_cm']:.2f} | {r['bump_mean_cm']:+.2f} | {r['offset_cm']:.2f} | {r['mean_z_steady_cm']:.2f} | {r['max_z_err_cm']:.1f} | {r['pred_z_min']:.2f} … {r['pred_z_max']:.2f} |")
    (OUT / f"summary{a.tag}.md").write_text("\n".join(rows) + "\n")
    print("\n".join(rows))


if __name__ == "__main__":
    main()
