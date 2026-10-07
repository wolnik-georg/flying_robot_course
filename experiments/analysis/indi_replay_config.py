"""Flight-accurate controller parameters for 2026-10-02 cf5 replay (yaml + meta)."""

from __future__ import annotations

import json
import subprocess
from copy import deepcopy
from pathlib import Path
from typing import Any

FLIGHT_YAML_REV = "1a43568"
CS2 = Path("/home/georg/Desktop/crazyswarm2")

# Keys exposed as g_indi_* on the host .so (cffirmware.i)
INDI_G_KEYS = (
    "kr", "kw", "kr_z", "kw_z", "fc_bw", "fc_bw_yaw", "mass",
    "kt1", "kt2", "kt3", "kt4", "j_scale", "ff_free", "filt_order", "filt_tau",
    "clamp_en", "tau_xy_max", "tau_z_max", "tilt_max_deg", "thrust_max",
    "notch_en", "notch_f0", "notch_bw", "filt_dt_us", "filt_prewarp",
    "res_fc", "res_clamp", "tau_clamp", "act_tau", "frame_conv", "omega_src",
    "n1_fix", "dt_usec",
)

POS_G_KEYS = ("kp_xy", "kp_z", "kv_xy", "kv_z", "ki_z", "ki_z_limit")


def _flatten_params(block: dict[str, Any], prefix: str = "") -> dict[str, Any]:
    out: dict[str, Any] = {}
    for k, v in block.items():
        key = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            out.update(_flatten_params(v, key))
        else:
            out[key] = v
    return out


def load_yaml_flight_params() -> dict[str, Any]:
    """Read-only: crazyflies.yaml at FLIGHT_YAML_REV."""
    try:
        import yaml  # type: ignore
    except ImportError:
        yaml = None
    raw = subprocess.check_output(
        ["git", "-C", str(CS2), "show", f"{FLIGHT_YAML_REV}:crazyflie/config/crazyflies.yaml"],
        text=True,
    )
    if yaml is None:
        raise RuntimeError("PyYAML required to parse flight crazyflies.yaml")
    doc = yaml.safe_load(raw)
    all_p = _flatten_params(doc.get("all", {}).get("firmware_params", {}))
    cf5_p = _flatten_params(doc.get("robots", {}).get("cf5", {}).get("firmware_params", {}))
    merged = {**all_p, **cf5_p}
    return merged


def flight_ours_config(meta: dict, yaml_cache: dict[str, Any] | None = None) -> dict[str, Any]:
    """Resolved indi + pos for cf5 as flown (meta is authoritative for overrides)."""
    yaml_p = yaml_cache if yaml_cache is not None else load_yaml_flight_params()
    cf5 = meta["per_drone"]["cf5"]
    indi = deepcopy(cf5["indi"])
    pos = deepcopy(cf5["pos"])

    def y(key: str, default=None):
        return yaml_p.get(key, default)

    # Meta records flight values; fill host-only fields from yaml at flight rev.
    for k in INDI_G_KEYS:
        yk = f"indi_gains.{k}"
        if k not in indi and yk in yaml_p:
            indi[k] = yaml_p[yk]
    if "rpm_source" not in indi and "indi_gains.rpm_source" in yaml_p:
        indi["rpm_source"] = yaml_p["indi_gains.rpm_source"]
    if float(indi.get("notch_f0") or 0) > 0:
        indi["notch_en"] = 1
    for k in POS_G_KEYS:
        pk = f"pos_gains.{k}"
        if k not in pos and pk in yaml_p:
            pos[k] = yaml_p[pk]

    return {
        "ctrl_mode": int(cf5.get("ctrl_mode", y("indi_gains.ctrl_mode", 3))),
        "indi": indi,
        "pos": pos,
        "res_sign": int(indi.get("res_sign", yaml_p.get("indi_gains.res_sign", 1))),
        "rnn_en": int(yaml_p.get("rnn.en", 0) or 0),
        "yaml_rev": FLIGHT_YAML_REV,
        "yaml_snapshot_keys": sorted(yaml_p.keys()),
    }


def _safe_set(cv, name: str, val: Any) -> bool:
    try:
        setattr(cv, name, val)
        return True
    except Exception:
        return False


def apply_ours_globals(fw, cfg: dict, level: str) -> dict[str, Any]:
    """Apply globals BEFORE controllerOutOfTreeInit(). Returns snapshot of values set."""
    cv = fw.cvar
    indi = cfg["indi"]
    pos = cfg["pos"]
    snap: dict[str, Any] = {"level": level, "side": "ours"}

    for k in INDI_G_KEYS:
        if k not in indi:
            continue
        v = indi[k]
        if k in ("notch_en", "ff_free", "filt_order", "filt_tau", "clamp_en", "filt_prewarp", "n1_fix"):
            v = int(v)
        elif k == "filt_dt_us":
            v = int(v)
        elif k in ("frame_conv", "omega_src", "dt_usec"):
            v = int(v)
        name = f"g_indi_{k}"
        if _safe_set(cv, name, v):
            snap[name] = v

    cv.g_controller_mode = int(cfg["ctrl_mode"])
    snap["g_controller_mode"] = int(cfg["ctrl_mode"])
    cv.g_kp_xy = float(pos["kp_xy"])
    cv.g_kp_z = float(pos["kp_z"])
    cv.g_kv_xy = float(pos["kv_xy"])
    cv.g_kv_z = float(pos["kv_z"])
    cv.g_ki_z = float(pos.get("ki_z", 0.0))
    cv.g_ki_z_limit = float(pos.get("ki_z_limit", 1.5))
    snap.update(
        {
            "g_kp_xy": cv.g_kp_xy,
            "g_kp_z": cv.g_kp_z,
            "g_kv_xy": cv.g_kv_xy,
            "g_kv_z": cv.g_kv_z,
            "g_ki_z": cv.g_ki_z,
            "g_ki_z_limit": cv.g_ki_z_limit,
        }
    )
    _safe_set(cv, "g_indi_res_sign", int(cfg.get("res_sign", cfg["indi"].get("res_sign", 1))))
    snap["g_indi_res_sign"] = int(getattr(cv, "g_indi_res_sign", 1))

    # L1+: constants already in indi block; L2/L3 same for ours (flight-native full config).
    if hasattr(cv, "g_rnn_en"):
        cv.g_rnn_en = 0
        snap["g_rnn_en"] = 0

    fw.oot_select_drone(0)
    fw.oot_set_peer_count(0)
    return snap


def omar_native_snapshot(fw) -> dict[str, Any]:
    return {
        "side": "omar",
        "compile_time": {
            "oot_omar_mass_kg": float(fw.oot_omar_mass()),
            "oot_omar_kt_equiv": float(fw.oot_omar_kt_equiv()),
            "oot_thrust_max": float(fw.oot_thrust_max()),
        },
        "note": "Omar thrust uses MOTORRPM2FORCE/CF_MASS at compile time; not all PARAMs exposed on host .so",
    }


def harmonize_omar_constants(meta: dict) -> dict[str, Any]:
    """L1: document target constants (host cannot set MOTORRPM2FORCE)."""
    indi = meta["per_drone"]["cf5"]["indi"]
    return {
        "target_mass_kg": float(indi["mass"]),
        "target_kt1..4": [float(indi[f"kt{i}"]) for i in range(1, 5)],
        "host_mappable": False,
    }
