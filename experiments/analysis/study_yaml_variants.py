#!/usr/bin/env python3
"""Study yaml variants (2026-10-11): the cf5 block of crazyflies.yaml per strategy, as small patches, verified against the frozen blocks.

The current default yaml is S0 (controller 6, ctrl_mode 0, ki_z 16, rnn.en 0, res_sign -1, dormant Omar block with Kpos_Iz 1.5).
  S1    : stabilizer.controller 9                       (Omar C + Kpos_Iz 1.5; deck RPM)
  S2    : rnn.en 1                                      (NS2, res_sign -1, 100 Hz)
  ours  : ctrl_mode 3, ki_z 0, res_sign 1               (supplementary; never ki_z 16 with ctrl_mode 3)
Writes crazyswarm2/crazyflie/config/study/{S1,S2,ours}.patch (git-style, applied from the crazyswarm2 root with `git apply`), then applies each
patch to a scratch copy and runs the study preflight check (lab_preflight.study_checks) on it.
  python3 experiments/analysis/study_yaml_variants.py"""
import json, re, subprocess, sys, tempfile
from pathlib import Path

CS2 = Path.home() / "Desktop" / "crazyswarm2"
YAML = CS2 / "crazyflie" / "config" / "crazyflies.yaml"
OUT = CS2 / "crazyflie" / "config" / "study"
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "flying_drone_stack" / "tools"))
import lab_preflight as lp  # noqa: E402

EDITS = {
    "S1": [(r"controller: 6(?=\s)", "controller: 9")],
    "S2": [(r"(?<=\n      rnn:\n        en: )0", "1")],
    "ours": [(r"ctrl_mode: 0(?=\s)", "ctrl_mode: 3"), (r"ki_z: 16\.0(?=\s)", "ki_z: 0.0"), (r"res_sign: -1(?=\s)", "res_sign: 1")],
}


def variant_text(text: str, strat: str) -> str:
    head, sep, cf5 = text.partition("    uri: radio://0/80/2M/E7E7E7BB02")      # only the cf5 block is touched
    assert sep, "cf5 block not found"
    end = cf5.index("    firmware_logging:")                                      # end of cf5 firmware_params
    body, tail = cf5[:end], cf5[end:]
    for pat, rep in EDITS[strat]:
        new, n = re.subn(pat, rep, body, count=1)
        assert n == 1, f"{strat}: pattern {pat!r} not found exactly once in the cf5 block"
        body = new
    return head + sep + body + tail


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    text = YAML.read_text()
    ok = True
    for strat in ("S1", "S2", "ours"):
        new = variant_text(text, strat)
        with tempfile.TemporaryDirectory() as d:
            d = Path(d); (d / "a").mkdir(); (d / "b").mkdir()
            (d / "a" / "crazyflies.yaml").write_text(text); (d / "b" / "crazyflies.yaml").write_text(new)
            r = subprocess.run(["git", "diff", "--no-index", "a/crazyflies.yaml", "b/crazyflies.yaml"], cwd=d, capture_output=True, text=True)
            # keep only the hunks and write the header by hand: a/ and b/ prefixes so that plain `git apply` (-p1) resolves the path
            lines = r.stdout.splitlines()
            hunks = lines[next(i for i, l in enumerate(lines) if l.startswith("@@")):]
            patch = ("diff --git a/crazyflie/config/crazyflies.yaml b/crazyflie/config/crazyflies.yaml\n"
                     "--- a/crazyflie/config/crazyflies.yaml\n+++ b/crazyflie/config/crazyflies.yaml\n" + "\n".join(hunks) + "\n")
        (OUT / f"{strat}.patch").write_text(patch)
        # 1. the patch applies to the real file in a scratch repo, 2. the result passes the frozen-block check for this strategy
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            subprocess.run(["git", "init", "-q", str(d)], check=True)
            (d / "crazyflie" / "config").mkdir(parents=True)
            for f in ("crazyflies.yaml", "study_strategies.json", "study_manifest.json"):
                (d / "crazyflie" / "config" / f).write_text((CS2 / "crazyflie" / "config" / f).read_text())
            chk = subprocess.run(["git", "apply", "--check", str(OUT / f"{strat}.patch")], cwd=d, capture_output=True, text=True)
            subprocess.run(["git", "apply", str(OUT / f"{strat}.patch")], cwd=d, check=True)
            cfg = lp.load_config(d / "crazyflie" / "config" / "crazyflies.yaml")
            errs, warns = lp.study_checks(strat, cfg, d / "crazyflie" / "config" / "crazyflies.yaml", d, None)
            frozen = [e for e in errs if e.startswith("cf5 ")]
            print(f"{strat}: apply --check rc={chk.returncode}, frozen-block errors: {frozen or 'none'}")
            ok &= chk.returncode == 0 and not frozen
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
