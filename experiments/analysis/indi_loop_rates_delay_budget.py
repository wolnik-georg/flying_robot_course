#!/usr/bin/env python3
"""Delay-budget / phase-loss calculator for INDI loop-rate comparison (read-only).

Outputs JSON + markdown snippet under experiments/analysis/out/indi_loop_rates/.
Assumptions are explicit; measured motor lag from bench ID (τ≈44 ms, fc≈3.6 Hz).
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
from scipy import signal

OUT_DIR = Path(__file__).resolve().parent / "out" / "indi_loop_rates"
F_OSC = 6.3  # Hz — observed limit-cycle band (flight + prior investigation)
TAU_MOTOR_S = 0.044  # bench first-order lag, investigation §16
MOTOR_FC_HZ = 1.0 / (2.0 * math.pi * TAU_MOTOR_S)


@dataclass
class DelayTerm:
    name: str
    time_ms: float
    phase_deg: float
    source: str  # measured | assumption | calculation
    confidence: str
    note: str


def phase_first_order_lag(f_hz: float, tau_s: float) -> tuple[float, float]:
    """Phase lag (deg) and equivalent delay (ms) at frequency f for 1/(1+sτ)."""
    w = 2.0 * math.pi * f_hz
    phi_rad = math.atan(w * tau_s)
    phi_deg = math.degrees(phi_rad)
    delay_s = phi_rad / w if w > 0 else 0.0
    return phi_deg, delay_s * 1000.0


def phase_zoh(f_hz: float, period_s: float) -> tuple[float, float]:
    """Zero-order hold: exp(-jωT/2) approximation."""
    w = 2.0 * math.pi * f_hz
    delay_s = period_s / 2.0
    phi_deg = math.degrees(w * delay_s)
    return phi_deg, delay_s * 1000.0


def butterworth2_phase_at_f(fc_hz: float, fs_hz: float, f_hz: float) -> tuple[float, float]:
    """2nd-order Butterworth LPF group delay approx via scipy bode at f."""
    # Bitcraze init: tau = 1/(2π fc), bilinear 2nd-order — match with standard BW design
    wn = 2.0 * math.pi * fc_hz
    # Normalized for fs: use analog prototype → digital via bilinear
    b, a = signal.butter(2, fc_hz, btype="low", fs=fs_hz)
    w, h = signal.freqz(b, a, worN=8192, fs=fs_hz)
    idx = int(np.argmin(np.abs(w - f_hz)))
    phi_deg = -math.degrees(np.angle(h[idx]))
    # equivalent delay from phase slope approx at f
    delay_ms = (phi_deg / 360.0) / f_hz * 1000.0 if f_hz > 0 else 0.0
    return phi_deg, delay_ms


def lpf2p_phase_at_f(fc_hz: float, fs_hz: float, f_hz: float) -> tuple[float, float]:
    """1st-order lpf2p (Bitcraze IMU): same as first-order at fc."""
    return phase_first_order_lag(f_hz, 1.0 / (2.0 * math.pi * fc_hz))


def variant_budget(variant: str, att_hz: float, filt_fc: float, filt_fs: float, filt_mismatch: bool) -> list[DelayTerm]:
    terms: list[DelayTerm] = []

    p, t = phase_first_order_lag(F_OSC, TAU_MOTOR_S)
    terms.append(
        DelayTerm(
            "Actuator (1st-order lag, bench τ=44 ms)",
            t,
            p,
            "measured",
            "HIGH",
            f"fc≈{MOTOR_FC_HZ:.2f} Hz; cited investigation §16 / handoff",
        )
    )

    if att_hz < 1000:
        pz, tz = phase_zoh(F_OSC, 1.0 / att_hz)
        terms.append(
            DelayTerm(
                f"Attitude INDI ZOH ({att_hz:.0f} Hz effective update)",
                tz,
                pz,
                "calculation",
                "HIGH",
                "Controller law / torque increment updated on decimated ticks",
            )
        )
    else:
        pz, tz = phase_zoh(F_OSC, 1.0 / att_hz)
        terms.append(
            DelayTerm(
                f"Control output sample period ({att_hz:.0f} Hz stabilizer)",
                tz,
                pz,
                "calculation",
                "MEDIUM",
                "Law runs every tick; still one sample actuator hold",
            )
        )

    pf, tf = butterworth2_phase_at_f(filt_fc, filt_fs, F_OSC)
    terms.append(
        DelayTerm(
            f"INDI Butterworth chain (fc={filt_fc} Hz, design fs={filt_fs:.0f} Hz)",
            tf,
            pf,
            "calculation",
            "MEDIUM" if not filt_mismatch else "LOW",
            "2× BW on α/τ paths (Omar/NA/stock pos-INDI); ours uses custom biquad",
        )
    )

    if filt_mismatch:
        pf2, tf2 = butterworth2_phase_at_f(filt_fc, filt_fs * 2.0, F_OSC)
        terms.append(
            DelayTerm(
                "Filter rate mismatch (coefficients for 500 Hz, called at 1000 Hz)",
                tf2 - tf,
                pf2 - pf,
                "assumption",
                "MEDIUM",
                "docs/22 §2f: effective cutoff ~1.9–3.4× nominal; phase at 6.3 Hz may shrink",
            )
        )

    pg, tg = lpf2p_phase_at_f(80.0, 1000.0, F_OSC)
    terms.append(
        DelayTerm(
            "IMU gyro onboard LPF (80 Hz @ 1 kHz)",
            tg,
            pg,
            "calculation",
            "MEDIUM",
            "sensors_bmi088_bmp3xx.c GYRO_LPF_CUTOFF_FREQ=80",
        )
    )

    pe, te = phase_zoh(F_OSC, 1.0 / 100.0)
    terms.append(
        DelayTerm(
            "EKF predict / mocap-heavy state (100 Hz predict)",
            te,
            pe,
            "assumption",
            "LOW",
            "Upper bound ZOH; attitude quaternion still updated every 1 ms read",
        )
    )

    ps, ts = phase_zoh(F_OSC, 1.0 / 100.0)
    terms.append(
        DelayTerm(
            "HLC / external setpoint (100 Hz)",
            ts,
            ps,
            "calculation",
            "HIGH",
            "RATE_HL_COMMANDER=100 Hz; hover uses cmdFullState stream from CS2",
        )
    )

    # Optical RPM ~ one blade event per rev; at ~12k RPM → ~800 Hz blade pass; DShot telem slower
    pr, tr = phase_zoh(F_OSC, 1.0 / 50.0)
    terms.append(
        DelayTerm(
            "RPM→force path (conservative 50 Hz hold, DShot telem)",
            tr,
            pr,
            "assumption",
            "LOW",
            "Oct-02 flights meta indi_rpm_source=1; exact telem rate not read in this pass",
        )
    )

    return terms


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    variants = {
        "Stock Bitcraze INDI (ctrl=3)": variant_budget("stock", 500, 70, 500, False),
        "Ours OOT (ctrl=6, ctrl_mode=3)": variant_budget("ours", 1000, 206, 1000, False),  # flown yaml: filt_dt_us=1000, filt_prewarp=1 (crazyflies.yaml stage 2b)
        "Omar C (ctrl=9)": variant_budget("omar_c", 500, 30, 500, False),
        "Omar Rust (ctrl=10)": variant_budget("omar_rust", 500, 30, 500, False),
        "NA-INDI port (ctrl=7/8, ref ctrl=7/8)": variant_budget("naindi", 500, 40, 500, False),
    }

    summary = {}
    for name, terms in variants.items():
        total_phase = sum(t.phase_deg for t in terms)
        total_time = sum(t.time_ms for t in terms)
        summary[name] = {
            "f_hz": F_OSC,
            "terms": [asdict(t) for t in terms],
            "sum_phase_deg": total_phase,
            "sum_time_ms_equiv": total_time,
            "note": "Sum of listed terms is NOT a rigorous loop Nyquist margin — terms overlap.",
        }

    payload = {
        "f_osc_hz": F_OSC,
        "motor_tau_s": TAU_MOTOR_S,
        "motor_fc_hz": MOTOR_FC_HZ,
        "variants": summary,
    }

    json_path = OUT_DIR / "delay_budget_6p3hz.json"
    json_path.write_text(json.dumps(payload, indent=2))

    md_lines = [
        "# Delay budget snapshot @ 6.3 Hz",
        "",
        f"Motor lag: **τ={TAU_MOTOR_S*1000:.0f} ms** (bench, investigation §16).",
        "",
        "| Variant | Σ phase (deg) | Σ equiv. delay (ms) | Confidence |",
        "|---|---:|---:|---|",
    ]
    for name, data in summary.items():
        md_lines.append(
            f"| {name} | {data['sum_phase_deg']:.1f} | {data['sum_time_ms_equiv']:.2f} | MEDIUM (overlapping terms) |"
        )
    md_lines.append("")
    md_lines.append("_Full term breakdown: `delay_budget_6p3hz.json`_")

    (OUT_DIR / "delay_budget_summary.md").write_text("\n".join(md_lines) + "\n")
    print(f"Wrote {json_path}")
    print(f"Wrote {OUT_DIR / 'delay_budget_summary.md'}")


if __name__ == "__main__":
    main()
