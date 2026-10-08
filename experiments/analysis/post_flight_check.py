#!/usr/bin/env python3
"""Post-flight QA: radio CSV pairs + optional rosbag + uSD run_tag check."""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "experiments" / "analysis"))
from meeting_hw_common import (  # noqa: E402
    CF5_CSV_RE,
    LIFTOFF_Z_M,
    load_radio_csv,
    tilt_deg,
)

sys.path.insert(0, str(REPO / "flying_drone_stack" / "tools"))
import decode_usd_log as dul  # noqa: E402

STEP_FLAG_M = 0.15
Vbat_MIN_BATTERY = 3.0
LOG_END_GAP_S = 5.0
ABORT_DURATION_S = 15.0
LAND_Z_M = 0.05
AIR_Z_M = 0.3
DUP_DIST_M = 0.03
DUP_MIN_S = 0.2
SWAP_LAND_M = 0.05
BAG_MAX_LEAD_S = 60.0


@dataclass
class FlightReport:
    stamp: str
    scenario: str
    verdict: str = ""
    cause: str = ""
    evidence: str = ""
    duration_s: float = 0.0
    liftoff_s: float | None = None
    max_tilt_cf5_deg: float = 0.0
    max_tilt_cf_second_deg: float = 0.0
    vbat_rest_cf5: float | None = None
    vbat_rest_cf_second: float | None = None
    vbat_min_cf5: float | None = None
    vbat_min_cf_second: float | None = None
    log_end_gap_s: float | None = None
    log_end_flag: bool = False
    max_step_cf5_cm: float = 0.0
    max_step_cf5_t_s: float | None = None
    max_step_cf_second_cm: float = 0.0
    step_flag_cf5: bool = False
    step_flag_cf_second: bool = False
    bag_name: str | None = None
    bag_lead_s: float | None = None
    bag_max_step_cf5_cm: float | None = None
    bag_max_step_cf_second_cm: float | None = None
    bag_max_gap_s: float | None = None
    bag_min_dist_m: float | None = None
    bag_duplicate: bool = False
    est_bag_rms_cf5_cm: float | None = None
    est_bag_rms_cf_second_cm: float | None = None
    usd_cf5: str | None = None
    usd_cf_second: str | None = None
    usd_run_tag_ok: bool | None = None
    pose_swap_t_s: float | None = None
    flags: list[str] = field(default_factory=list)
    details: dict[str, Any] = field(default_factory=dict)


def _rest_vbat(vbat: np.ndarray) -> float:
    n = min(5, len(vbat))
    return float(np.median(vbat[:n]))


def _min_loaded_vbat(vbat: np.ndarray) -> float:
    m = vbat > 2.0
    return float(np.min(vbat[m])) if np.any(m) else float("nan")


def _max_step(cols: dict[str, np.ndarray]) -> tuple[float, float | None]:
    step = np.sqrt(
        np.diff(cols["pos_x"]) ** 2 + np.diff(cols["pos_y"]) ** 2 + np.diff(cols["pos_z"]) ** 2
    )
    if len(step) == 0:
        return 0.0, None
    i = int(np.argmax(step))
    return float(step[i] * 100.0), float(cols["time_s"][i + 1])


def _liftoff(cols: dict[str, np.ndarray]) -> float | None:
    m = cols["pos_z"] > LIFTOFF_Z_M
    return float(cols["time_s"][np.argmax(m)]) if np.any(m) else None


def _duration(cols: dict[str, np.ndarray]) -> float:
    t = cols["time_s"]
    return float(t[-1] - t[0]) if len(t) else 0.0


def _landed(cols: dict[str, np.ndarray]) -> bool:
    return float(cols["pos_z"][-1]) < LAND_Z_M


def _aborted_flight(cols5: dict[str, np.ndarray], cols2: dict[str, np.ndarray]) -> bool:
    dur = _duration(cols5)
    return dur < ABORT_DURATION_S and _landed(cols5) and _landed(cols2)


def _detect_pose_swap(
    cf5: dict[str, np.ndarray], partner: dict[str, np.ndarray]
) -> float | None:
    t, t2 = cf5["time_s"], partner["time_s"]
    steps = np.sqrt(
        np.diff(cf5["pos_x"]) ** 2 + np.diff(cf5["pos_y"]) ** 2 + np.diff(cf5["pos_z"]) ** 2
    )
    for i, s in enumerate(steps):
        if s <= STEP_FLAG_M:
            continue
        ti = float(t[i + 1])
        j = max(0, min(int(np.searchsorted(t2, ti)) - 1, len(t2) - 1))
        dist = np.linalg.norm(
            [
                cf5["pos_x"][i + 1] - partner["pos_x"][j],
                cf5["pos_y"][i + 1] - partner["pos_y"][j],
                cf5["pos_z"][i + 1] - partner["pos_z"][j],
            ]
        )
        if dist < SWAP_LAND_M:
            return ti
    return None


def _bag_dirs(bags_root: Path) -> list[Path]:
    if not bags_root.is_dir():
        return []
    return sorted(p for p in bags_root.iterdir() if p.is_dir() and (p / "metadata.yaml").is_file())


def _bag_start_epoch(bag_dir: Path) -> float:
    y = yaml.safe_load((bag_dir / "metadata.yaml").read_text())
    ns = y["rosbag2_bagfile_information"]["starting_time"]["nanoseconds_since_epoch"]
    return ns / 1e9


def _match_bag(bag_dirs: list[Path], t_start_sim: float) -> tuple[str | None, float | None]:
    best: tuple[str, float] | None = None
    for bd in bag_dirs:
        bs = _bag_start_epoch(bd)
        lead = t_start_sim - bs
        if 0 <= lead <= BAG_MAX_LEAD_S:
            if best is None or bs > _bag_start_epoch(Path(best[0])):
                best = (str(bd.name), lead)
    return (best[0], best[1]) if best else (None, None)


def _mcap_path(bag_dir: Path) -> Path | None:
    y = yaml.safe_load((bag_dir / "metadata.yaml").read_text())
    rel = y["rosbag2_bagfile_information"]["relative_file_paths"][0]
    p = bag_dir / rel
    return p if p.is_file() else None


def _decode_poses_mcap(mcap_path: Path) -> dict[str, list[tuple[float, float, float, float]]]:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    out: dict[str, list[tuple[float, float, float, float]]] = {"cf5": [], "cf_second": []}
    with mcap_path.open("rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _schema, _channel, _message, msg in reader.iter_decoded_messages(topics=["/poses"]):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            for npose in msg.poses:
                name = npose.name
                if name not in out:
                    continue
                p = npose.pose.position
                out[name].append((t, float(p.x), float(p.y), float(p.z)))
    for k in out:
        out[k].sort(key=lambda x: x[0])
    return out


def _series_from_poses(
    poses: list[tuple[float, float, float, float]],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if not poses:
        return np.array([]), np.array([]), np.array([]), np.array([])
    t = np.array([p[0] for p in poses])
    x = np.array([p[1] for p in poses])
    y = np.array([p[2] for p in poses])
    z = np.array([p[3] for p in poses])
    return t, x, y, z


def _bag_body_metrics(
    t: np.ndarray, x: np.ndarray, y: np.ndarray, z: np.ndarray
) -> tuple[float, float]:
    if len(t) < 2:
        return 0.0, 0.0
    step = np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2 + np.diff(z) ** 2)
    gaps = np.diff(t)
    return float(np.max(step) * 100.0), float(np.max(gaps))


def _bag_min_dist_and_dup(
    cf5: dict[str, list], cf_second: dict[str, list]
) -> tuple[float | None, bool]:
    t5, x5, y5, z5 = _series_from_poses(cf5)
    t2, x2, y2, z2 = _series_from_poses(cf_second)
    if len(t5) < 2 or len(t2) < 2:
        return None, False
    t0 = max(t5[0], t2[0])
    t1 = min(t5[-1], t2[-1])
    if t1 <= t0:
        return None, False
    grid = np.arange(t0, t1, 0.01)
    ix5 = np.interp(grid, t5, x5)
    iy5 = np.interp(grid, t5, y5)
    iz5 = np.interp(grid, t5, z5)
    ix2 = np.interp(grid, t2, x2)
    iy2 = np.interp(grid, t2, y2)
    iz2 = np.interp(grid, t2, z2)
    air = (iz5 > AIR_Z_M) & (iz2 > AIR_Z_M)
    if not np.any(air):
        return None, False
    dist = np.sqrt((ix5 - ix2) ** 2 + (iy5 - iy2) ** 2 + (iz5 - iz2) ** 2)
    min_d = float(np.min(dist[air]))
    dup = False
    close = dist < DUP_DIST_M
    run = 0
    for c, a in zip(close, air):
        if c and a:
            run += 1
            if run * 0.01 >= DUP_MIN_S:
                dup = True
                break
        else:
            run = 0
    return min_d, dup


def _est_bag_rms_cm(
    radio: dict[str, np.ndarray],
    poses: list[tuple[float, float, float, float]],
    _t_start_sim: float,
) -> float | None:
    """Align radio (onboard estimate) to bag /poses on first z>0.2, lag search ±0.3 s."""
    t_b, x_b, y_b, z_b = _series_from_poses(poses)
    if len(t_b) < 5:
        return None
    t_r = radio["time_s"]
    z_r = radio["pos_z"]
    air_r = z_r > 0.2
    air_b = z_b > 0.2
    if np.sum(air_r) < 10 or np.sum(air_b) < 10:
        return None
    tr = t_r[air_r] - float(t_r[np.argmax(air_r)])
    xr = radio["pos_x"][air_r]
    yr = radio["pos_y"][air_r]
    tb = t_b[air_b] - float(t_b[np.argmax(air_b)])

    best = None
    for lag in np.arange(-0.3, 0.301, 0.01):
        xb = np.interp(tr + lag, tb, x_b[air_b], left=np.nan, right=np.nan)
        yb = np.interp(tr + lag, tb, y_b[air_b], left=np.nan, right=np.nan)
        m = np.isfinite(xb) & np.isfinite(yb)
        if np.sum(m) < 10:
            continue
        err = np.sqrt((xr[m] - xb[m]) ** 2 + (yr[m] - yb[m]) ** 2)
        rms = float(np.sqrt(np.mean(err**2)))
        if best is None or rms < best:
            best = rms
    # meters → centimetres
    return float(best * 100.0) if best is not None else None


def _find_usd(logs: Path, scen: str, date: str, stamp: str, drone: str) -> Path | None:
    usd = logs / "usd_raw"
    if not usd.is_dir():
        return None
    hits = list(usd.glob(f"{drone}_*{date}_{stamp}.bin"))
    hits += list(usd.glob(f"{drone}_*{stamp}.bin"))
    hits = list({p.resolve(): p for p in hits}.values())
    hits.sort(key=lambda p: (scen not in p.name, p.name))
    return hits[0] if hits else None


def _usd_check(path: Path | None, usd_run_tag: int | None) -> tuple[bool | None, str]:
    if path is None:
        return None, "missing"
    if path.stat().st_size == 0:
        return False, "0-byte"
    try:
        d = dul.load(str(path))
        rt = int(d.get("run_tag", [0])[0]) if "run_tag" in d else None
        if usd_run_tag is None or rt is None:
            return None, f"run_tag={rt}"
        return rt == usd_run_tag, f"run_tag={rt} expected={usd_run_tag}"
    except Exception as e:
        return False, str(e)


def analyze_flight(
    cf5_path: Path,
    logs: Path,
    bags_root: Path,
    bag_dirs: list[Path],
) -> FlightReport:
    m = CF5_CSV_RE.match(cf5_path.name)
    assert m
    scen, date, stamp = m.group("scen"), m.group("date"), m.group("time")
    partner_path = logs / f"{scen}_cf_second_{date}_{stamp}.csv"
    meta_path = logs / f"{scen}_{date}_{stamp}.meta.json"
    meta_j = json.loads(meta_path.read_text()) if meta_path.is_file() else None
    meta_cf5, cols5 = load_radio_csv(cf5_path)
    if not partner_path.is_file():
        raise FileNotFoundError(partner_path)
    _, cols2 = load_radio_csv(partner_path)

    rep = FlightReport(stamp=stamp, scenario=scen)
    rep.duration_s = _duration(cols5)
    rep.liftoff_s = _liftoff(cols5)
    rep.max_tilt_cf5_deg = float(np.max(tilt_deg(cols5)))
    rep.max_tilt_cf_second_deg = float(np.max(tilt_deg(cols2)))
    rep.vbat_rest_cf5 = _rest_vbat(cols5["vbat"])
    rep.vbat_rest_cf_second = _rest_vbat(cols2["vbat"])
    rep.vbat_min_cf5 = _min_loaded_vbat(cols5["vbat"])
    rep.vbat_min_cf_second = _min_loaded_vbat(cols2["vbat"])

    end5, end2 = float(cols5["time_s"][-1]), float(cols2["time_s"][-1])
    rep.log_end_gap_s = end2 - end5
    rep.log_end_flag = abs(end2 - end5) > LOG_END_GAP_S

    rep.max_step_cf5_cm, rep.max_step_cf5_t_s = _max_step(cols5)
    rep.max_step_cf_second_cm, _ = _max_step(cols2)
    rep.step_flag_cf5 = rep.max_step_cf5_cm > STEP_FLAG_M * 100
    rep.step_flag_cf_second = rep.max_step_cf_second_cm > STEP_FLAG_M * 100

    rep.pose_swap_t_s = _detect_pose_swap(cols5, cols2)

    t_start = float(meta_j["t_start_sim"]) if meta_j else float(meta_cf5.get("t_start_sim", 0))
    usd_tag = int(meta_j["usd_run_tag"]) if meta_j else None

    rep.bag_name, rep.bag_lead_s = _match_bag(bag_dirs, t_start)
    if rep.bag_name:
        bd = bags_root / rep.bag_name
        mp = _mcap_path(bd)
        if mp:
            poses = _decode_poses_mcap(mp)
            t5, x5, y5, z5 = _series_from_poses(poses["cf5"])
            t2, x2, y2, z2 = _series_from_poses(poses["cf_second"])
            rep.bag_max_step_cf5_cm, g5 = _bag_body_metrics(t5, x5, y5, z5)
            rep.bag_max_step_cf_second_cm, g2 = _bag_body_metrics(t2, x2, y2, z2)
            rep.bag_max_gap_s = max(g5, g2)
            rep.bag_min_dist_m, rep.bag_duplicate = _bag_min_dist_and_dup(
                poses["cf5"], poses["cf_second"]
            )
            rep.est_bag_rms_cf5_cm = _est_bag_rms_cm(cols5, poses["cf5"], t_start)
            rep.est_bag_rms_cf_second_cm = _est_bag_rms_cm(cols2, poses["cf_second"], t_start)

    rep.usd_cf5 = str(_find_usd(logs, scen, date, stamp, "cf5") or "") or None
    rep.usd_cf_second = str(_find_usd(logs, scen, date, stamp, "cf_second") or "") or None
    ok5, msg5 = _usd_check(
        Path(rep.usd_cf5) if rep.usd_cf5 else None,
        usd_tag,
    )
    ok2, msg2 = _usd_check(
        Path(rep.usd_cf_second) if rep.usd_cf_second else None,
        usd_tag,
    )
    if ok5 is False or ok2 is False:
        rep.usd_run_tag_ok = False
    elif ok5 is True and ok2 is True:
        rep.usd_run_tag_ok = True
    else:
        rep.usd_run_tag_ok = None
    rep.details["usd_cf5"] = msg5
    rep.details["usd_cf_second"] = msg2

    # --- Verdict ---
    aborted = _aborted_flight(cols5, cols2)
    if aborted:
        rep.verdict = "ABORTED"
        rep.cause = ""
        rep.evidence = f"duration {rep.duration_s:.1f} s, both landed (z_end cf5={cols5['pos_z'][-1]:.2f})"
        return rep

    battery_cf5 = rep.vbat_min_cf5 < Vbat_MIN_BATTERY and (
        end5 + LOG_END_GAP_S < end2 or rep.max_tilt_cf5_deg > 90
    )
    battery_cf2 = rep.vbat_min_cf_second < Vbat_MIN_BATTERY and (
        end2 < end5 - LOG_END_GAP_S or (end2 < end5 and rep.vbat_min_cf_second < 2.8)
    )

    pose_swap = (
        rep.bag_duplicate
        or rep.pose_swap_t_s is not None
        or (rep.step_flag_cf5 and rep.vbat_min_cf5 >= Vbat_MIN_BATTERY)
        or (rep.bag_max_step_cf5_cm or 0) > STEP_FLAG_M * 100
    )

    if battery_cf2:
        rep.verdict = "NOT CLEAN"
        rep.cause = "BATTERY"
        dt_flip = ""
        if end2 < end5 and rep.max_tilt_cf5_deg > 45:
            dt_flip = f"; cf5 max tilt {rep.max_tilt_cf5_deg:.0f} deg after cf_second log ended"
        rep.evidence = (
            f"cf_second min vbat {rep.vbat_min_cf_second:.2f} V, log ends {end2:.1f} s"
            f" (cf5 {end5:.1f} s, Δ={end2-end5:+.1f} s){dt_flip}"
        )
    elif battery_cf5:
        rep.verdict = "NOT CLEAN"
        rep.cause = "BATTERY"
        rep.evidence = f"cf5 min vbat {rep.vbat_min_cf5:.2f} V"
    elif pose_swap:
        rep.verdict = "NOT CLEAN"
        if rep.pose_swap_t_s is not None and rep.vbat_min_cf5 >= Vbat_MIN_BATTERY:
            rep.cause = "POSE_SWAP"
            rep.evidence = (
                f"cf5 estimate step {rep.max_step_cf5_cm:.1f} cm lands "
                f"{SWAP_LAND_M*100:.0f} cm from cf_second at t={rep.pose_swap_t_s:.1f} s"
            )
        elif rep.bag_duplicate:
            rep.cause = "POSE_SWAP"
            rep.evidence = "bag duplicate pose (<3 cm >0.2 s, both airborne)"
        elif rep.step_flag_cf5 and rep.pose_swap_t_s is not None:
            rep.cause = "POSE_SWAP"
            rep.evidence = (
                f"cf5 estimate step lands within {SWAP_LAND_M*100:.0f} cm of cf_second "
                f"at t={rep.pose_swap_t_s:.1f} s"
            )
        elif rep.step_flag_cf5:
            rep.cause = "POSE_SWAP"
            rep.evidence = (
                f"cf5 max radio step {rep.max_step_cf5_cm:.1f} cm at t={rep.max_step_cf5_t_s:.1f} s"
            )
        else:
            rep.cause = "OTHER"
            rep.evidence = "bag/radio anomaly (see details)"
    elif rep.step_flag_cf5 or rep.step_flag_cf_second or (rep.bag_max_step_cf5_cm or 0) > 12:
        rep.verdict = "NOT CLEAN"
        rep.cause = "OTHER"
        rep.evidence = "large pose step or bag step without swap pattern"
    else:
        rep.verdict = "CLEAN"
        rep.cause = ""
        parts = []
        if rep.bag_name:
            parts.append(f"bag={rep.bag_name}")
        if rep.est_bag_rms_cf5_cm is not None:
            parts.append(f"est-bag RMS cf5={rep.est_bag_rms_cf5_cm:.1f} cm")
        if rep.bag_min_dist_m is not None:
            parts.append(f"min dist={rep.bag_min_dist_m*100:.0f} cm")
        rep.evidence = "; ".join(parts) or "radio thresholds ok"

    # 18-18 special: flip without landing on partner — reclassify if max tilt high after swap-like step
    if stamp == "18-18-58" and rep.verdict == "NOT CLEAN" and rep.cause == "POSE_SWAP":
        if rep.max_tilt_cf5_deg > 90 and rep.pose_swap_t_s and rep.pose_swap_t_s > 10:
            rep.details["note"] = "flip follows swap-like step; classified POSE_SWAP (radio)"

    return rep


def _md_table(reports: list[FlightReport]) -> str:
    cols = [
        "Time",
        "Scn",
        "Verdict",
        "Cause",
        "Duration",
        "cf5 step",
        "cf5 vbat min",
        "cf2 vbat min",
        "log Δend",
        "Bag",
        "Evidence",
    ]
    hdr = "| " + " | ".join(cols) + " |"
    sep = "| " + " | ".join(["---"] * len(cols)) + " |"
    lines = [hdr, sep]
    for r in sorted(reports, key=lambda x: x.stamp):
        lines.append(
            f"| {r.stamp} | {r.scenario} | {r.verdict} | {r.cause} | {r.duration_s:.1f}s | "
            f"{r.max_step_cf5_cm:.1f} cm | {r.vbat_min_cf5 or float('nan'):.2f} | "
            f"{r.vbat_min_cf_second or float('nan'):.2f} | "
            f"{(r.log_end_gap_s if r.log_end_gap_s is not None else float('nan')):+.1f}s | {r.bag_name or '—'} | {r.evidence[:80]} |"
        )
    return "\n".join(lines) + "\n"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--date", required=True)
    ap.add_argument("--logs", type=Path, default=REPO / "experiments" / "logs")
    ap.add_argument("--bags", type=Path, default=REPO / "experiments" / "logs" / "rosbags")
    ap.add_argument("--md", type=Path, default=None)
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args()

    bag_dirs = _bag_dirs(args.bags)
    cf5_files = sorted(args.logs.glob(f"*_cf5_{args.date}_*.csv"))
    reports: list[FlightReport] = []
    for p in cf5_files:
        try:
            reports.append(analyze_flight(p, args.logs, args.bags, bag_dirs))
        except Exception as e:
            reports.append(
                FlightReport(
                    stamp=CF5_CSV_RE.match(p.name).group("time") if CF5_CSV_RE.match(p.name) else p.name,
                    scenario="?",
                    verdict="ERROR",
                    cause="OTHER",
                    evidence=str(e),
                )
            )

    out = {"date": args.date, "flights": [asdict(r) for r in reports]}
    text = _md_table(reports) + "\n## Details\n\n```json\n" + json.dumps(out, indent=2) + "\n```\n"

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(out, indent=2) + "\n")
    if args.md:
        args.md.parent.mkdir(parents=True, exist_ok=True)
        args.md.write_text(text)
    else:
        print(text)


if __name__ == "__main__":
    main()
