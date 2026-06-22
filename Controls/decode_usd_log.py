#!/usr/bin/env python3
"""
Decode uSD deck binary log(s) into the same CSV format as the radio/CS2 log.

How it works:
    flight.py starts usd.logging=1 before each trajectory and stops it after.
    This creates one segment per flight inside the binary (separated by gaps).
    decode_usd_log.py splits the binary at those gaps and pairs each segment
    with the matching radio CSV by chronological order.

Single flight (explicit):
    python decode_usd_log.py log00.bin --radio logs/figure8_..._14-30-00.csv

Single flight (auto — latest bin + latest radio CSV):
    python decode_usd_log.py --auto

Multiple flights (batch — all segments paired with all radio CSVs by order):
    python decode_usd_log.py --batch
    python decode_usd_log.py log00.bin --batch

All modes write logs/<radio_name>_usd500hz.csv next to the original radio CSV.
"""

import sys
import argparse
import csv
from pathlib import Path
import numpy as np

sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/tools/usdlog")
import cfusdlog

LOGS_DIR = Path(__file__).parent / "logs"

_USD_TO_COL = {
    "gyro.x": "gyro_x", "gyro.y": "gyro_y", "gyro.z": "gyro_z",
    "acc.x":  "acc_x",  "acc.y":  "acc_y",  "acc.z":  "acc_z",
    "indi.alp_raw_x": "alp_raw_x", "indi.alp_raw_y": "alp_raw_y", "indi.alp_raw_z": "alp_raw_z",
    "indi.alp_x": "alp_x", "indi.alp_y": "alp_y", "indi.alp_z": "alp_z",
    "indi.tau_x": "tau_x", "indi.tau_y": "tau_y", "indi.tau_z": "tau_z",
    "rpm.m1": "rpm_m1", "rpm.m2": "rpm_m2", "rpm.m3": "rpm_m3", "rpm.m4": "rpm_m4",
    "pm.vbat": "vbat",
}

_RADIO_COLS = ["x", "y", "z", "vx", "vy", "vz",
               "roll_deg", "pitch_deg", "yaw_deg", "thrust"]

_FIELDNAMES = [
    "time_s",
    "x", "y", "z", "vx", "vy", "vz",
    "roll_deg", "pitch_deg", "yaw_deg", "thrust", "vbat",
    "gyro_x", "gyro_y", "gyro_z",
    "acc_x",  "acc_y",  "acc_z",
    "rpm_m1", "rpm_m2", "rpm_m3", "rpm_m4",
    "tau_x",  "tau_y",  "tau_z",
    "alp_x",  "alp_y",  "alp_z",
    "alp_raw_x", "alp_raw_y", "alp_raw_z",
]


# ── Helpers ────────────────────────────────────────────────────────────────────

def _latest_csv():
    files = [p for p in sorted(LOGS_DIR.glob("*.csv"), key=lambda p: p.stat().st_mtime)
             if "_usd500hz" not in p.name]
    if not files:
        print(f"[error] no radio CSVs found in {LOGS_DIR}")
        sys.exit(1)
    return files[-1]


def _latest_bin(directory: Path):
    # uSD firmware writes files without extension (log00, log01, ...)
    # Accept both extensionless log files and .bin files (if user renamed them)
    files = sorted(
        [p for p in directory.iterdir()
         if p.is_file() and (p.suffix == ".bin" or p.stem.startswith("log"))],
        key=lambda p: p.stat().st_mtime,
    )
    if not files:
        print(f"[error] no uSD log files found in {directory}")
        sys.exit(1)
    return files[-1]


def _load_radio_csv(path: Path):
    meta_lines, rows = [], []
    with open(path) as f:
        for line in f:
            (meta_lines if line.startswith("#") else rows).append(line.rstrip("\n"))
    if not rows:
        return {}, meta_lines
    reader = csv.DictReader(rows)
    data = {col: [] for col in reader.fieldnames}
    for row in reader:
        for col in reader.fieldnames:
            try:
                data[col].append(float(row[col]))
            except (ValueError, TypeError):
                data[col].append(float("nan"))
    return {k: np.array(v) for k, v in data.items()}, meta_lines


def _split_segments(block: dict, min_gap_s: float = 3.0):
    """Split a uSD block into per-flight segments at timestamp gaps > min_gap_s."""
    ts = block["timestamp"]  # milliseconds
    gap_idx = np.where(np.diff(ts) > min_gap_s * 1000)[0]
    cuts = [0] + list(gap_idx + 1) + [len(ts)]
    segments = []
    for i in range(len(cuts) - 1):
        sl = slice(cuts[i], cuts[i + 1])
        seg = {k: v[sl] for k, v in block.items() if isinstance(v, np.ndarray)}
        if len(seg["timestamp"]) > 50:   # skip tiny noise fragments
            segments.append(seg)
    return segments


def _find_time_offset(seg: dict, radio: dict) -> float:
    """Cross-correlate gyro_x to find t_offset (seconds) aligning seg to radio."""
    if "gyro.x" not in seg or "gyro_x" not in radio or "time_s" not in radio:
        return 0.0
    ts = seg["timestamp"]
    usd_t = (ts - ts[0]) / 1000.0
    radio_t = radio["time_s"]
    radio_gyro = radio["gyro_x"]

    dt = float(np.median(np.diff(radio_t)))
    if dt <= 0 or usd_t[-1] < (radio_t[-1] - radio_t[0]):
        return 0.0

    t_grid = np.arange(usd_t[0], usd_t[-1], dt)
    gyro_rs = np.interp(t_grid, usd_t, seg["gyro.x"])

    r = radio_gyro - radio_gyro.mean()
    u = gyro_rs    - gyro_rs.mean()
    if r.std() < 1e-6 or u.std() < 1e-6 or len(u) < len(r):
        return 0.0

    corr = np.correlate(u, r, mode="valid")
    lag = int(np.argmax(np.abs(corr)))
    t_offset = radio_t[0] - t_grid[lag]
    return t_offset


def _write_csv(seg: dict, t_offset: float, radio: dict,
               meta_lines: list, bin_name: str, out_path: Path):
    ts = seg["timestamp"]
    usd_t = (ts - ts[0]) / 1000.0 + t_offset
    n = len(usd_t)
    rate = n / ((usd_t[-1] - usd_t[0]) + 1e-9)

    # Interpolate radio columns onto uSD time axis
    radio_interp = {}
    if radio and "time_s" in radio:
        t_r = radio["time_s"]
        for col in _RADIO_COLS:
            if col not in radio:
                radio_interp[col] = np.full(n, float("nan"))
                continue
            idx = np.clip(np.searchsorted(t_r, usd_t), 0, len(t_r) - 1)
            idx_l = np.maximum(idx - 1, 0)
            pick_l = np.abs(t_r[idx_l] - usd_t) < np.abs(t_r[idx] - usd_t)
            radio_interp[col] = radio[col][np.where(pick_l, idx_l, idx)]
    else:
        for col in _RADIO_COLS:
            radio_interp[col] = np.full(n, float("nan"))

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="") as f:
        for line in meta_lines:
            f.write(line + "\n")
        f.write(f"# meta:usd_source={bin_name}\n")
        f.write(f"# meta:usd_n_samples={n}\n")
        f.write(f"# meta:usd_rate_hz={rate:.1f}\n")
        f.write(f"# meta:usd_time_offset_s={t_offset:.6f}\n")
        writer = csv.DictWriter(f, fieldnames=_FIELDNAMES)
        writer.writeheader()
        for i in range(n):
            row = {"time_s": round(float(usd_t[i]), 6)}
            for usd_name, col_name in _USD_TO_COL.items():
                arr = seg.get(usd_name)
                row[col_name] = float(arr[i]) if arr is not None else float("nan")
            for col in _RADIO_COLS:
                row[col] = float(radio_interp[col][i])
            writer.writerow(row)

    print(f"  → {out_path.name}  ({n} rows, {rate:.0f} Hz, offset={t_offset:+.2f}s)")


# ── Main modes ─────────────────────────────────────────────────────────────────

def run_single(bin_path: Path, radio_path: Path, out_path: Path | None):
    data = cfusdlog.decode(str(bin_path))
    block = data and data.get("fixedFrequency")
    if not block:
        print(f"[error] no fixedFrequency block in {bin_path.name}")
        sys.exit(1)

    segments = _split_segments(block)
    seg = segments[0] if segments else block  # use first segment or full block
    n = len(seg["timestamp"])
    print(f"[usd] {n} samples from {bin_path.name}")

    radio, meta_lines = {}, []
    if radio_path:
        radio, meta_lines = _load_radio_csv(radio_path)
        print(f"[radio] {len(radio.get('time_s', []))} rows from {radio_path.name}")

    t_offset = _find_time_offset(seg, radio)
    print(f"[align] t_offset = {t_offset:+.3f} s")

    if out_path is None:
        if radio_path:
            out_path = radio_path.parent / (radio_path.stem + "_usd500hz.csv")
        else:
            out_path = LOGS_DIR / (bin_path.stem + "_usd500hz.csv")

    _write_csv(seg, t_offset, radio, meta_lines, bin_path.name, out_path)


def run_batch(bin_path: Path):
    data = cfusdlog.decode(str(bin_path))
    block = data and data.get("fixedFrequency")
    if not block:
        print(f"[error] no fixedFrequency block in {bin_path.name}")
        sys.exit(1)

    segments = _split_segments(block)
    radio_csvs = sorted(
        [p for p in LOGS_DIR.glob("*.csv") if "_usd500hz" not in p.name],
        key=lambda p: p.stat().st_mtime,
    )

    print(f"[batch] {len(segments)} segment(s) in {bin_path.name}, "
          f"{len(radio_csvs)} radio CSV(s)")

    if len(segments) != len(radio_csvs):
        print(f"[warn] count mismatch — pairing {min(len(segments), len(radio_csvs))} pair(s)")

    for i, (seg, csv_path) in enumerate(zip(segments, radio_csvs)):
        n = len(seg["timestamp"])
        dur = (seg["timestamp"][-1] - seg["timestamp"][0]) / 1000.0
        print(f"[pair {i+1}] segment {n} samples / {dur:.1f}s  ←→  {csv_path.name}")
        radio, meta_lines = _load_radio_csv(csv_path)
        t_offset = _find_time_offset(seg, radio)
        out_path = csv_path.parent / (csv_path.stem + "_usd500hz.csv")
        _write_csv(seg, t_offset, radio, meta_lines, bin_path.name, out_path)


# ── CLI ────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Decode uSD log → CSV")
    parser.add_argument("bin", nargs="?", type=Path, help="uSD .bin file")
    parser.add_argument("--radio", type=Path, default=None,
                        help="Radio CSV (single mode); 'auto' for latest")
    parser.add_argument("--auto", action="store_true",
                        help="Latest .bin in cwd + latest radio CSV")
    parser.add_argument("--batch", action="store_true",
                        help="Split binary by flight segments, pair with all radio CSVs in order")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    bin_path = args.bin
    if args.auto or bin_path is None:
        bin_path = _latest_bin(Path("."))
        print(f"[auto] bin: {bin_path.name}")

    if not bin_path.exists():
        print(f"[error] not found: {bin_path}")
        sys.exit(1)

    if args.batch:
        run_batch(bin_path)
        return

    # Single mode
    radio_path = args.radio
    if args.auto or str(radio_path) == "auto":
        radio_path = _latest_csv()
        print(f"[auto] radio: {radio_path.name}")

    run_single(bin_path, radio_path, args.out)


if __name__ == "__main__":
    main()
