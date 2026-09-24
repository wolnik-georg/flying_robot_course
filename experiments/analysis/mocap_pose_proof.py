#!/usr/bin/env python3
"""Quick proof: bad mocap vs sane controller setpoints (uSD + optional radio CSV).

uSD logs stateEstimate.* (EKF state, mocap-fed in this lab) and ctrltarget.* (what the
position loop commands). If ctrltarget stays in the room but stateEstimate teleports/jumps,
the failure is pose/tracking — not formation scenario generation.

Usage:
  python3 experiments/analysis/mocap_pose_proof.py \\
      experiments/logs/usd_raw/.../cf5_....bin \\
      --radio experiments/logs/A1_cf5_2026-09-24_17-33-49.csv
"""
from __future__ import annotations

import argparse
import sys
from io import StringIO
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "flying_drone_stack" / "tools"))
import decode_usd_log as dul  # noqa: E402


def _load_radio_csv(path: Path) -> dict[str, np.ndarray]:
    lines = [ln for ln in path.read_text().splitlines() if not ln.startswith("#")]
    import pandas as pd

    df = pd.read_csv(StringIO("\n".join(lines)))
    return {c: df[c].to_numpy(dtype=float) for c in df.columns}


def _summarize(label: str, x, y, z, tgt_x=None, tgt_y=None, tgt_z=None) -> None:
    step = np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2 + np.diff(z) ** 2)
    print(f"\n{label}")
    print(f"  estimate x,y,z  [{x.min():+.2f},{x.max():+.2f}]  "
          f"[{y.min():+.2f},{y.max():+.2f}]  [{z.min():+.2f},{z.max():+.2f}]")
    print(f"  max |Δestimate| step  {step.max():.3f} m")
    if tgt_x is not None:
        err = np.sqrt((x - tgt_x) ** 2 + (y - tgt_y) ** 2 + (z - tgt_z) ** 2)
        print(f"  ctrltarget z range [{tgt_z.min():+.2f},{tgt_z.max():+.2f}]")
        print(f"  |estimate - target|  max {err.max():.3f} m  mean {err.mean():.3f} m")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("usd_bin", type=Path, help="decoded .bin from uSD")
    p.add_argument("--radio", type=Path, default=None, help="matching radio CSV (pos_* columns)")
    args = p.parse_args()

    d = dul.load(str(args.usd_bin))
    _summarize(
        f"uSD {args.usd_bin.name}",
        d["x"],
        d["y"],
        d["z"],
        d.get("ctrltarget_x"),
        d.get("ctrltarget_y"),
        d.get("ctrltarget_z"),
    )

    if args.radio and args.radio.is_file():
        r = _load_radio_csv(args.radio)
        _summarize(f"radio {args.radio.name}", r["pos_x"], r["pos_y"], r["pos_z"])
        n = min(len(d["x"]), len(r["pos_x"]))
        if n > 10:
            rmse = np.sqrt(
                np.mean((d["x"][:n] - r["pos_x"][:n]) ** 2 + (d["y"][:n] - r["pos_y"][:n]) ** 2)
            )
            print(f"\n  uSD vs radio xy RMSE (first {n} samples, unsynced clocks): {rmse:.3f} m")
            print("  (same order of magnitude ⇒ radio /state is the same EKF estimate as uSD)")


if __name__ == "__main__":
    main()
