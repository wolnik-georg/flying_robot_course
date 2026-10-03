#!/usr/bin/env python3
"""Read-only pre-flight checker for crazyswarm2 crazyflies.yaml (effective per-drone params)."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    print("lab_preflight.py requires PyYAML (python3-yaml)", file=sys.stderr)
    sys.exit(2)


def deep_merge(base: dict, override: dict) -> dict:
    out = dict(base)
    for k, v in override.items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def effective_robot(cfg: dict, name: str) -> dict:
    robots = cfg.get("robots") or {}
    if name not in robots:
        raise KeyError(f"robot {name!r} not in yaml")
    all_block = cfg.get("all") or {}
    robot = robots[name]
    eff = {"enabled": robot.get("enabled", True), "uri": robot.get("uri"), "type": robot.get("type")}
    fp_all = (all_block.get("firmware_params") or {}).copy()
    fp_robot = (robot.get("firmware_params") or {}).copy()
    eff["firmware_params"] = deep_merge(fp_all, fp_robot)
    return eff


def dig(d: dict, *keys, default=None):
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def fmt_robot(name: str, eff: dict) -> str:
    fp = eff.get("firmware_params") or {}
    ctrl = dig(fp, "stabilizer", "controller", default="(inherit)")
    mode = dig(fp, "indi_gains", "ctrl_mode", default="(inherit)")
    kiz = dig(fp, "pos_gains", "ki_z", default=dig(fp, "indi_gains", "pos_gains", "ki_z", default="(inherit)"))
    rnn_en = dig(fp, "rnn", "en", default="(unset)")
    rnn_div = dig(fp, "rnn", "div", default="(unset, firmware default 10)")
    lines = [
        f"=== {name} ===",
        f"  enabled={eff.get('enabled')}  type={eff.get('type')}  uri={eff.get('uri')}",
        f"  stabilizer.controller={ctrl}  indi_gains.ctrl_mode={mode}  ki_z={kiz}",
        f"  rnn.en={rnn_en}  rnn.div={rnn_div}",
    ]
    return "\n".join(lines)


def check_robot(name: str, eff: dict, *, expect_geometric: bool, allow_rnn_en: bool) -> list[str]:
    errs: list[str] = []
    fp = eff.get("firmware_params") or {}
    mode = dig(fp, "indi_gains", "ctrl_mode")
    kiz = dig(fp, "pos_gains", "ki_z") or dig(fp, "indi_gains", "pos_gains", "ki_z")
    rnn_en = dig(fp, "rnn", "en")

    if mode == 3 and kiz is not None and float(kiz) >= 15.9:
        errs.append(f"{name}: ki_z={kiz} with ctrl_mode=3 (full INDI) — forbidden (10-02 tumble)")
    if rnn_en == 1 and not allow_rnn_en:
        errs.append(f"{name}: rnn.en=1 set in yaml — use --allow-rnn-en for Checklist G")
    if expect_geometric and name == "cf5" and mode not in (0, "0", None):
        if mode != 0:
            errs.append(f"{name}: ctrl_mode={mode} but --expect-geometric-cf5 requires ctrl_mode=0")
    return errs


def yaml_freshness(yaml_path: Path, repo_root: Path | None) -> str | None:
    if repo_root is None or not (repo_root / ".git").is_dir():
        return "could not verify yaml age (no git repo)"
    try:
        rel = yaml_path.resolve().relative_to(repo_root.resolve())
    except ValueError:
        return "yaml outside crazyswarm2 git root — could not compare to HEAD"
    r = subprocess.run(
        ["git", "-C", str(repo_root), "log", "-1", "--format=%ci", "--", str(rel)],
        capture_output=True,
        text=True,
        check=False,
    )
    if r.returncode != 0 or not r.stdout.strip():
        return None
    head_mtime = r.stdout.strip()
    if yaml_path.stat().st_mtime < Path(repo_root / ".git").stat().st_mtime:
        pass
    mtime = yaml_path.stat().st_mtime
    log = subprocess.run(
        ["git", "-C", str(repo_root), "log", "-1", "--format=%ct", "--", str(rel)],
        capture_output=True,
        text=True,
        check=False,
    )
    if log.returncode == 0 and log.stdout.strip():
        head_ts = float(log.stdout.strip())
        if mtime + 2 < head_ts:
            return f"WARN: {yaml_path.name} mtime older than last git commit on that file ({head_mtime}) — pull?"
    return None


def scan_launch_log(path: Path) -> list[str]:
    notes: list[str] = []
    text = path.read_text(errors="replace")
    if "CS2_CONNECT_PARAM_PACE_V1" not in text:
        notes.append("FAIL: launch log missing CS2_CONNECT_PARAM_PACE_V1 pacing marker")
    if re.search(r"Assert failed", text, re.I):
        notes.append("FAIL: launch log contains Assert failed")
    if not notes:
        notes.append("OK: pacing marker present, no Assert failed in log")
    return notes


def main() -> int:
    ap = argparse.ArgumentParser(description="Pre-flight crazflies.yaml checker (read-only)")
    ap.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("lab_preflight.yaml"),
        help="Tool config (default crazyflies path)",
    )
    ap.add_argument("--yaml", type=Path, default=None, help="Override crazyflies.yaml path")
    ap.add_argument("--expect-geometric-cf5", action="store_true", help="Fail if cf5 ctrl_mode != 0")
    ap.add_argument("--allow-rnn-en", action="store_true", help="Allow rnn.en=1 in yaml")
    ap.add_argument("--launch-log", type=Path, default=None, help="Optional launch stdout log to scan")
    args = ap.parse_args()

    tool_cfg = load_config(args.config.expanduser())
    yaml_path = (args.yaml or Path(tool_cfg["crazyflies_yaml"]).expanduser()).resolve()
    if not yaml_path.is_file():
        print(f"ERROR: crazyflies.yaml not found: {yaml_path}", file=sys.stderr)
        return 2

    cfg = load_config(yaml_path)
    robots = tool_cfg.get("robots") or ["cf5", "cf_second"]
    repo_root = yaml_path.parent.parent.parent  # .../crazyflie/config -> src/crazyswarm2

    print(f"crazyflies.yaml: {yaml_path}\n")
    warn = yaml_freshness(yaml_path, repo_root if (repo_root / ".git").is_dir() else None)
    if warn:
        print(warn + "\n")

    all_errs: list[str] = []
    for name in robots:
        try:
            eff = effective_robot(cfg, name)
        except KeyError as e:
            print(e)
            all_errs.append(str(e))
            continue
        print(fmt_robot(name, eff))
        all_errs.extend(
            check_robot(
                name,
                eff,
                expect_geometric=args.expect_geometric_cf5,
                allow_rnn_en=args.allow_rnn_en,
            )
        )
        print()

    if args.launch_log:
        print(f"Launch log: {args.launch_log}")
        for line in scan_launch_log(args.launch_log):
            print(f"  {line}")
        print()

    if all_errs:
        print("ERRORS:")
        for e in all_errs:
            print(f"  - {e}")
        return 1
    print("PASS: no blocking issues detected.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
