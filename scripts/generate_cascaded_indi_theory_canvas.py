#!/usr/bin/env python3
"""Generate cascaded-indi-theory.canvas.tsx — paper-aligned cascade INDI only."""

from __future__ import annotations

import re
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import generate_indi_canvas as gen  # noqa: E402

MD_PATH = ROOT / "flying_drone_stack/docs/INDI_STUDY.md"
MD_OUT_PATH = ROOT / "flying_drone_stack/docs/CASCADED_INDI_THEORY.md"
OUT_PATH = (
    Path.home()
    / ".cursor/projects/home-georg-Desktop-flying-robot-course/canvases/cascaded-indi-theory.canvas.tsx"
)
CANVAS_DIR = OUT_PATH.parent.as_posix()

DROP_LINE = re.compile(
    r"firmware_app|indi\.rs|controller_indi|CONTROLLER_MODE|"
    r"analyze_flight|ros2 run|make cload|Crazyflie|Bitcraze|"
    r"Mode [0-9]|KR_INDI|KW_INDI|FC_GYRO_HZ\s*=\s*60|"
    r"our firmware|Our default|our implementation|Our Mode|"
    r"Förster|16\.5717|16\.6556|29\.2617",
    re.I,
)

SKIP_SECTION = re.compile(
    r"^### (3\.3|3\.7|3\.8|5\.5|6\.|11\.|12\.|13\.7|13\.8|13\.9)",
)


def between(md: str, start: str, end: str | None = None) -> str:
    s = md.find(start)
    if s < 0:
        raise ValueError(f"Marker not found: {start!r}")
    if end:
        e = md.find(end, s + len(start))
        if e < 0:
            raise ValueError(f"End marker not found: {end!r}")
        return md[s:e].strip()
    return md[s:].strip()


def filter_theory(text: str) -> str:
    lines: list[str] = []
    skip_section = False
    in_crazyflie_block = False

    for line in text.splitlines():
        if line.startswith("### "):
            skip_section = bool(SKIP_SECTION.match(line))
            in_crazyflie_block = False

        if skip_section:
            continue

        if "**Crazyflie" in line or "Crazyflie 2.x values" in line:
            in_crazyflie_block = True
            continue
        if in_crazyflie_block:
            if line.strip().startswith("```"):
                in_crazyflie_block = False
            continue

        if DROP_LINE.search(line):
            continue

        # Rewrite firmware-centric phrasing to paper-neutral
        line = line.replace("KR_INDI", "KR").replace("KW_INDI", "KW")
        line = line.replace("KR_cas", "KR_cas").replace("KW_cas", "KW_cas")
        line = re.sub(
            r"This matches `firmware_app/src/lib\.rs` exactly.*",
            "This is the standard INDI output structure in Smeur 2016.",
            line,
        )
        line = line.replace(
            "INDI only replaces the inner attitude loop that computes torque from (eR, eω).",
            "In the cascaded architecture, INDI replaces the inner rate loop; the outer attitude loop is also INDI (Smeur 2018).",
        )
        line = line.replace(
            "The inner loop is identical to Mode 1 with one substitution:",
            "The inner loop is the standard single-loop INDI rate controller with one substitution:",
        )
        line = line.replace(
            "ω_ref from the outer loop replaces the old fixed ω_des.",
            "ω_ref from the outer cascade replaces a trajectory feedforward rate ω_ff.",
        )
        line = line.replace("KW_cas is the only tuning gain in the inner loop. FC_GYRO_HZ is the IIR filter cutoff, same as Mode 1.", "KW_cas is the inner-loop rate gain; fc is the IIR cutoff on α_meas.")
        line = re.sub(r"analyze_flight\.py[^.]*\.", "", line)
        # Drop legacy section numbers from embedded extracts
        if re.match(r"^### (8\.|13\.)\d", line):
            continue
        if re.match(r"^## (8\.|13\.|14\.|Glossary)", line):
            continue
        # Drop legacy subsection numbers from pasted INDI_STUDY extracts
        if re.match(r"^### \d+\.\d+", line):
            continue

        lines.append(line)

    text = re.sub(r"\n{3,}", "\n\n", "\n".join(lines)).strip()
    return sanitize_code_fences(text)


def sanitize_code_fences(text: str) -> str:
    """Remove orphan ``` lines after aggressive line filtering."""
    lines = text.splitlines()
    out: list[str] = []
    open_fence = False
    for line in lines:
        if line.strip() == "```":
            if open_fence:
                out.append(line)
                open_fence = False
            else:
                out.append(line)
                open_fence = True
            continue
        out.append(line)
    if open_fence and out and out[-1].strip() == "```":
        out.pop()
    return "\n".join(out)


def polish_theory_prose(text: str) -> str:
    """Theory-neutral wording and cross-reference fixes."""
    replacements = [
        ("In single-loop INDI (§§3–12),", "In single-loop INDI,"),
        ("single-loop INDI (§§3–12)", "single-loop INDI"),
        ("the same 3rd-order characteristic polynomial (§8.2)", "the same 3rd-order characteristic polynomial (§5.1)"),
        ("FC_GYRO_HZ", "fc"),
        (
            "Tune the inner loop first (bench rate steps, hover), then close the outer loop "
            "with the inner already validated.",
            "Tune the inner loop first (rate-hold experiments), then close the outer loop "
            "with the inner already validated.",
        ),
        (
            "At 500 Hz sample rate, 60 Hz cutoff:",
            "Example at sample rate fs = 500 Hz, cutoff fc = 60 Hz:",
        ),
        ("→ lower FC_GYRO_HZ (try 40 Hz)", "→ lower fc (e.g. try 40 Hz)"),
        ("→ raise FC_GYRO_HZ (try 80 Hz)", "→ raise fc (e.g. try 80 Hz)"),
        ("| FC_GYRO_HZ too HIGH | FC_GYRO_HZ too LOW |", "| fc too HIGH | fc too LOW |"),
        ("how high FC can go", "how high fc can go"),
        ("At **500 Hz** (our firmware rate): Δt = 2 ms", "At high rate (e.g. fs = 500 Hz): Δt = 2 ms"),
        (
            "Note: because α_meas already contains the gyroscopic term f(ω), the increment `δτ` "
            "implicitly cancels it without any explicit model of ω×(Jω). This is why INDI adds "
            "`gyro_comp = ω×(Jω)` back at the *output* stage (§4.3) — not to cancel it in the "
            "dynamics, but to account for it in the torque command that goes to the motor mixer, "
            "which operates on net torque. The gyroscopic term is measured and subtracted in the "
            "increment, then added back at the output — net effect: it passes through transparently.",
            "**Gyroscopic term (see also §4.1):** Because `α_meas` already contains the "
            "*actual physical* gyroscopic acceleration, the INDI increment does not need to "
            "cancel it explicitly (unlike NDI). The explicit `gyro_comp = ω × (Jω)` is added "
            "only at the mixer output because the motor mixer expects *net torque* commands. "
            "Net effect: the gyroscopic term passes through transparently.",
        ),
        (
            "This makes the outer loop structurally simpler than the inner loop.",
            "This makes the outer loop structurally simpler than the inner loop.\n\n"
            "The outer loop applies the incremental principle directly to the (approximately) "
            "integrator plant η̇ ≈ ω, with control effectiveness G1_outer ≈ I.",
        ),
        (
            "The gyroscopic term `f(ω) = −ω×(Jω)` is embedded in the measurement. NDI must cancel "
            "f(ω) explicitly using a model (and fails if J is wrong). INDI does not cancel it — it "
            "is already subtracted out when we compute `α_ref − α_meas`, because both sides share "
            "the same f(ω). The gyroscopic nonlinearity disappears from the increment automatically, "
            "regardless of J accuracy.",
            "The gyroscopic term `f(ω) = −ω×(Jω)` is embedded in `α_meas`. NDI must cancel f(ω) "
            "explicitly using a model (and fails if J is wrong). INDI does not model-cancel it: "
            "`α_meas` reflects the true physical acceleration (including gyroscopic effects), so "
            "the increment `J·(α_ref − α_meas)` only corrects the acceleration *error*; the "
            "explicit `gyro_comp` at the mixer output is described in §4.1.",
        ),
        (
            "│ α_meas = IIR(dω/dt)           │  δτ → τ_prev → τ_out + ω×Jω",
            "│ α_meas = IIR(dω/dt)           │  τ_current+δτ → τ_out+ω×Jω (RPM)",
        ),
        (
            " │  τ_base  = clamp(τ_prev + δτ, ±TAU_CLAMP)                    │\n"
            " │  τ_prev  ← τ_base                        [inner integrator]   │\n"
            " │  τ_out   = τ_base + ω×(Jω)              [gyro feed-forward]  │",
            " │  τ_cmd   = τ_current(RPM) + δτ   [recommended] or τ_prev+δτ│\n"
            " │  τ_out   = τ_cmd + ω×(Jω)                [gyro feed-forward]  │",
        ),
        (
            "Two separate integrators: `ω_des_prev` (outer, accumulates attitude error "
            "into rate command) and `τ_prev` (inner, accumulates rate error into torque). "
            "Each loop applies the incremental principle to its own plant.",
            "Two separate integrators: `ω_des_prev` (outer) and inner torque state via "
            "`τ_prev` **or** measured `τ_current` from the RPM deck (recommended). "
            "Each loop applies the incremental principle to its own plant.",
        ),
        (
            "Tal & Karaman also study RPM-measured torque",
            "Tal & Karaman also use RPM-measured torque",
        ),
        (
            "That is an **actuator-sensing extension**, separate from the cascaded two-loop "
            "structure documented here.",
            "the **RPM-deck / Mode 2** approach recommended for this project’s inner loop (§7.2).",
        ),
        (
            "⑤ Accumulate:        τ_base = clamp(τ_prev + δτ, ±CLAMP)\n"
            "                     τ_prev ← τ_base                       [store, NO gyro_comp]\n"
            "⑥ Gyro feed-fwd:     τ_out  = τ_base + ω×(Jω)\n"
            "⑦ Update:            ω_prev ← ω",
            "⑤ Torque command:    τ_cmd = τ_current(RPM) + δτ            [recommended]\n"
            "                     or τ_base = clamp(τ_prev + δτ, ±CLAMP) [classic Smeur]\n"
            "                     τ_prev ← τ_base                       [classic only, NO gyro_comp]\n"
            "⑥ Gyro feed-fwd:     τ_out  = τ_cmd + ω×(Jω)                [or τ_base + ω×(Jω) classic]\n"
            "⑦ Update:            ω_prev ← ω",
        ),
        (
            "τ_base = τ_prev + δτ          [state: pure INDI increment, no gyroscopic term]\n"
            "τ_out  = τ_base + gyro_comp   [output: add gyroscopic feed-forward]\n"
            "τ_prev ← τ_base               [store state without gyro_comp]",
            "τ_base = τ_prev + δτ          [classic accumulator; RPM: τ_cmd = τ_current + δτ — §7.2]\n"
            "τ_out  = τ_base + gyro_comp   [output: add gyroscopic feed-forward; use τ_cmd with RPM]\n"
            "τ_prev ← τ_base               [store state without gyro_comp; skip when using RPM]",
        ),
    ]
    for old, new in replacements:
        text = text.replace(old, new)
    return text


def tal_flatness_section() -> str:
    return polish_theory_prose(
        """### 2.5 Tal & Karaman — flatness feedforward (optional)

Tal & Karaman (2020) extend Smeur 2016 by feeding desired angular acceleration from **differential flatness** into the inner-loop virtual control.

**Acceleration domain (Smeur notation):**

```
α_ref = α_des − KR_std·eR − KW_std·eω          [rad/s²]
δτ    = J·(α_ref − α_meas)
```

**Equivalent form with torque virtual control (J in gains):**

```
v_des = J·α_des − KR·eR − KW·eω                [Nm]
δτ    = v_des − J·α_meas
      = J·(α_des − α_meas) − KR·eR − KW·eω
```

The term `J·(α_des − α_meas)` drives angular-acceleration tracking **before** large attitude/rate errors build up. Setting `α_des = 0` recovers plain Smeur 2016.

**Flatness chain (feeds the cascade):**

| Derivative | Maps to | Used in |
|------------|---------|---------|
| jerk `x⃛` | `ω_ff` [rad/s] | outer: `ω_ref = ω_des + ω_ff` |
| snap `x⁽⁴⁾` | `α_des` [rad/s²] | inner: via `α_ref` or `v_des` |

> Tal & Karaman also use RPM-measured torque (no `τ_prev` accumulator) — the **RPM-deck / Mode 2** approach recommended for this project’s inner loop (§7.2)."""
    )


FIG8_LOG_STEM = "figure8_mode0_2026-05-17_18-47-06"
FIG8_LOGS_SRC = ROOT / "Controls/logs"
# Short names under docs/fig8/ — doc-relative paths for Markdown preview
FIG8_ASSETS_DIR = ROOT / "flying_drone_stack/docs/fig8"
FIG8_ASSETS_URL = "fig8"

# (short filename in docs/fig8/, source filename in Controls/logs/, heading, caption)
FIG8_PLOTS: list[tuple[str, str, str, str]] = [
    (
        "fig8_analysis.png",
        f"{FIG8_LOG_STEM}_analysis.png",
        "0.1 Flight overview",
        "Planned vs flown pose, thrust, and attitudes.",
    ),
    (
        "fig8_3d_orientation.png",
        f"{FIG8_LOG_STEM}_3d_orientation.png",
        "0.2 3D path and orientation",
        "Planned vs flown figure-8 with orientation triads.",
    ),
    (
        "fig8_analysis_axes.png",
        f"{FIG8_LOG_STEM}_analysis_axes.png",
        "0.3 Per-axis tracking",
        "Position and velocity errors in x, y, and z.",
    ),
    (
        "fig8_analysis_kinematics.png",
        f"{FIG8_LOG_STEM}_analysis_kinematics.png",
        "0.4 Kinematics",
        "Speed, acceleration, and flatness-related quantities along the path.",
    ),
]


def sync_fig8_plot_assets() -> None:
    """Copy analysis PNGs into docs/fig8/ for CASCADED_INDI_THEORY.md preview."""
    FIG8_ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    for short_fn, src_fn, _, _ in FIG8_PLOTS:
        src = FIG8_LOGS_SRC / src_fn
        if not src.is_file():
            raise FileNotFoundError(f"Missing plot (run analyze_flight.py first): {src}")
        shutil.copy2(src, FIG8_ASSETS_DIR / short_fn)


FIG8_PLOTS_MD = ROOT / "flying_drone_stack/docs/CASCADED_INDI_FIGURE8_PLOTS.md"


def _fig8_img_ref(short_fn: str, alt: str) -> str:
    return f"![{alt}]({FIG8_ASSETS_URL}/{short_fn})"


def build_figure8_plots_markdown() -> str:
    blocks = [
        "# Figure-8 baseline plots (geometric controller + mocap)\n",
        f"Log: `Controls/logs/{FIG8_LOG_STEM}.csv` · "
        f"from `analyze_flight.py --type figure8`\n",
        "Open this file in **Markdown Preview** (`Ctrl+Shift+V`) to view figures inline.\n",
    ]
    for short_fn, _src_fn, heading, caption in FIG8_PLOTS:
        blocks.append(f"## {heading}\n\n{caption}\n\n{_fig8_img_ref(short_fn, heading)}\n")
    return "\n".join(blocks) + "\n"


def geom_mocap_markdown_section() -> str:
    rows = "\n".join(
        f"| {heading} | {caption} | "
        f"[{short_fn}]({FIG8_ASSETS_URL}/{short_fn}) |"
        for short_fn, _src_fn, heading, caption in FIG8_PLOTS
    )
    figures = "\n".join(
        f"### {heading}\n\n{caption}\n\n{_fig8_img_ref(short_fn, heading)}\n"
        for short_fn, _src_fn, heading, caption in FIG8_PLOTS
    )
    return f"""## 0. Geometric controller and mocap baseline (not INDI)

Flights below the cascaded INDI stack use the **SE(3) geometric position controller** (Lee et al. 2010) with **motion-capture pose feedback** (Lighthouse / CS2). This layer is **not INDI**:

- **Reference:** Poly4D figure-8 (`export_poly4d` → `flight.py`, mode 0).
- **Outer loop:** geometric position control → thrust, desired attitude `Rd`, feedforward.
- **Inner loop:** standard onboard attitude tracking (geometric / stock), not cascaded INDI.
- **Log:** `Controls/logs/{FIG8_LOG_STEM}.csv` (`run_trajectory=figure8`, `run_mode=0`).

**Flight plots** (mode 0) — also in [CASCADED_INDI_FIGURE8_PLOTS.md](CASCADED_INDI_FIGURE8_PLOTS.md). If preview shows broken images, **Ctrl+click** a PNG in the table or open that gallery file.

| § | Description | PNG |
|---|-------------|-----|
{rows}

{figures}

> **Role in this document:** baseline figure-8 tracking context. §1–§10 replace the attitude/torque path with cascaded INDI; position reference and mocap stay as here.
"""


def geom_mocap_canvas_tsx() -> str:
    blocks: list[str] = []
    for short_fn, _src_fn, heading, caption in FIG8_PLOTS:
        abs_path = (FIG8_ASSETS_DIR / short_fn).resolve()
        cap = gen.ts_str(caption)
        blocks.append(
            f"""          <Stack gap={{6}}>
            <Text size="small" weight="semibold">{heading}</Text>
            <Text size="small" tone="secondary">{caption}</Text>
            <img
              src="file://{abs_path.as_posix()}"
              alt={cap}
              style={{{{
                maxWidth: '100%',
                borderRadius: 6,
              }}}}
            />
          </Stack>"""
        )
    return f"""
      <Card>
        <CardHeader>§0 Geometric controller &amp; mocap baseline</CardHeader>
        <CardBody>
          <Stack gap={{16}}>
            <Text size="small">
              SE(3) geometric position loop (Lee 2010) + mocap pose feedback; figure-8 Poly4D mode 0.
              Not INDI — context for the cascade below. Log: {FIG8_LOG_STEM}.csv — regenerate plots with
              analyze_flight.py.
            </Text>
            <Stack gap={{20}}>
{chr(10).join(blocks)}
            </Stack>
          </Stack>
        </CardBody>
      </Card>
"""


RPM_DECK_INNER_SECTION = """
**With RPM deck (recommended for this project):** Instead of using the `τ_prev` accumulator alone, compute the *actual* torque being produced each cycle from measured motor RPMs:

```
τ_current = G(Ω) · [Ω₁², Ω₂², Ω₃², Ω₄²]    [Nm from measured RPM²]
τ_cmd     = τ_current + δτ                   [no accumulator drift]
τ_out     = τ_cmd + ω×(Jω)
```

This eliminates actuator lag drift between estimated and true motor torque and matches Tal & Karaman (2020). The `τ_prev` accumulator path in the per-cycle algorithm remains valid Smeur 2016 theory when RPM feedback is unavailable.
"""


def single_vs_cascade_table() -> str:
    return """### 8.4 Single-loop vs cascaded INDI (conceptual)

| Property | Single-loop INDI | Cascaded INDI (Smeur 2018) |
|----------|------------------|----------------------------|
| **Loops** | 1 (attitude + rate combined) | 2 (attitude outer + rate inner) |
| **Outer virtual control** | `−KR·eR − KW·eω` [Nm] | `−KR_cas·eR` [rad/s] |
| **Inner virtual control** | (same combined law) | `−KW_cas·eω` [Nm] |
| **KR units** | Nm/rad | (rad/s)/rad = 1/s |
| **KW units** | Nm/(rad/s) | Nm/(rad/s) |
| **Integrators** | `τ_prev` (torque) | `ω_des_prev` (rate) + `τ_prev` or RPM-based `τ_current` |
| **Outer measurement** | N/A | `η̇_meas ≈ ω` (gyro, no filter) |
| **Inner measurement** | `α_meas` = filtered dω/dt | `α_meas` + `τ_current` from RPM (recommended) |
| **Bandwidth tuning** | Coupled (KR, KW in one polynomial) | Independent (inner first, then outer) |
| **Disturbance path** | Through combined loop | Absorbed in inner before attitude |
| **Separation ratio** | N/A | inner BW / outer BW should be ≫ 1 |"""


def pseudo_block(title: str, note: str, rust: str, *, style: str = "callout") -> str:
    fence = f"```rust\n{rust.strip()}\n```\n"
    if style == "md":
        return f"\n\n#### {title}\n\n*{note}*\n\n{fence}"
    return f"\n> **{title}** — {note}\n\n{fence}"


# --- Theoretical Rust pseudo-code (paper notation, not project firmware) ---

PSEUDO_SO3_ERROR = """
/// SO(3) attitude error eR = ½(Rd^T R − R^T Rd)^∨  (Lee 2010 / Smeur 2018 outer loop)
fn attitude_error_vee(rd: Mat3, r: Mat3) -> Vec3 {
    let s = rd.transpose() * r - r.transpose() * rd;
    0.5 * vee(s) // extract [s32, s23, s13] from skew-symmetric s
}

fn vee(s: Mat3) -> Vec3 {
    Vec3::new(s.m32, s.m23, s.m13)
}
"""

PSEUDO_INDI_CORE = """
/// Generic INDI torque increment (Smeur 2016, acceleration domain)
/// τ = τ_prev + J · (α_ref − α_meas)
fn indi_torque_increment(
    alpha_ref: Vec3,
    alpha_meas: Vec3,
    tau_prev: Vec3,
    j_diag: Vec3, // diag(Jxx, Jyy, Jzz)
) -> Vec3 {
    let delta = vec3_mul(j_diag, alpha_ref - alpha_meas);
    tau_prev + delta
}

/// Virtual control (standard PD on attitude + rate), optional Tal & Karaman feedforward:
/// α_ref = α_des − KR_std·eR − KW_std·eω   [rad/s²]
fn alpha_ref_from_errors(
    e_r: Vec3,
    e_omega: Vec3,
    alpha_des: Vec3,
    kr_std: f32,
    kw_std: f32,
) -> Vec3 {
    alpha_des - kr_std * e_r - kw_std * e_omega
}
"""

PSEUDO_IIR = """
/// First-order IIR low-pass on α_meas (inner loop sensing)
/// α_meas[k] = k·α_raw[k] + (1−k)·α_meas[k−1]
fn iir_first_order_alpha(
    alpha_raw: Vec3,
    alpha_meas_prev: Vec3,
    dt: f32,
    fc_hz: f32,
) -> Vec3 {
    let rc = 1.0 / (2.0 * PI * fc_hz);
    let k = dt / (dt + rc);
    k * alpha_raw + (1.0 - k) * alpha_meas_prev
}

fn alpha_raw_from_gyro(omega: Vec3, omega_prev: Vec3, dt: f32) -> Vec3 {
    (omega - omega_prev) / dt
}
"""

PSEUDO_BUTTERWORTH = """
/// 2nd-order Butterworth on α_meas — bilinear transform (inner loop)
struct Butterworth2State {
    x1: Vec3,
    x2: Vec3,
    y1: Vec3,
    y2: Vec3,
}

fn butterworth2_coeffs(dt: f32, fc_hz: f32) -> (f32, f32, f32) {
    let tau = 1.0 / (2.0 * PI * fc_hz);
    let denom = tau * tau + SQRT_2 * tau * dt + dt * dt;
    let b0 = dt * dt / denom;
    let a1 = 2.0 * (dt * dt - tau * tau) / denom;
    let a2 = (tau * tau - SQRT_2 * tau * dt + dt * dt) / denom;
    (b0, a1, a2)
}

fn butterworth2_alpha(
    alpha_raw: Vec3,
    st: &mut Butterworth2State,
    dt: f32,
    fc_hz: f32,
) -> Vec3 {
    let (b0, a1, a2) = butterworth2_coeffs(dt, fc_hz);
    let y = b0 * alpha_raw
        + 2.0 * b0 * st.x1
        + b0 * st.x2
        - a1 * st.y1
        - a2 * st.y2;
    st.x2 = st.x1;
    st.x1 = alpha_raw;
    st.y2 = st.y1;
    st.y1 = y;
    y
}
"""

PSEUDO_OUTER = """
/// Outer cascaded INDI — attitude → rate reference (Smeur 2018)
/// Plant: η̇ ≈ ω. Measurement: η̇_meas = ω (no differentiation).
struct OuterIndiState {
    omega_des_prev: Vec3,
}

fn outer_indi_attitude_step(
    e_r: Vec3,
    omega: Vec3,
    omega_ff: Vec3,
    kr_cas: f32,
    omega_max: f32,
    st: &mut OuterIndiState,
) -> Vec3 {
    // ② Virtual control [rad/s]
    let nu_att = -kr_cas * e_r;
    // ③ Measurement
    let eta_dot_meas = omega;
    // ④ INDI increment on rate
    let delta_omega = nu_att - eta_dot_meas;
    // ⑤ Accumulate + clamp
    let omega_des = clamp_vec3(st.omega_des_prev + delta_omega, -omega_max, omega_max);
    st.omega_des_prev = omega_des;
    // ⑥ Flatness feedforward to inner loop
    omega_des + omega_ff
}
"""

PSEUDO_INNER = """
/// Inner cascaded INDI — rate → torque (Smeur 2016 applied to rate loop)
struct InnerIndiState {
    tau_prev: Vec3,
    omega_prev: Vec3,
    alpha_meas: Vec3, // or Butterworth2State per axis filter bank
}

fn inner_indi_rate_step(
    omega: Vec3,
    omega_ref: Vec3,
    j_diag: Vec3,
    kw_cas: f32,
    tau_clamp: f32,
    dt: f32,
    fc_hz: f32,
    st: &mut InnerIndiState,
) -> Vec3 {
    // ① Rate error
    let e_omega = omega - omega_ref;
    // ② Virtual control [Nm] (J absorbed into KW_cas in Nm-domain variant)
    let nu_rate = -kw_cas * e_omega;
    // ③–④ Measure α and INDI increment
    let alpha_raw = (omega - st.omega_prev) / dt;
    let alpha_meas = iir_first_order_alpha(alpha_raw, st.alpha_meas, dt, fc_hz);
    st.alpha_meas = alpha_meas;
    let delta_tau = nu_rate - vec3_mul(j_diag, alpha_meas);
    // With RPM deck (recommended for this project):
    // let tau_current = compute_tau_from_rpm(measured_rpms); // G(Ω)·[Ω₁²,…]
    // let tau_cmd = tau_current + delta_tau;                 // no τ_prev accumulator
    // ⑤ Accumulate (NO gyro_comp in state) — textbook path when no RPM deck:
    let tau_base = clamp_vec3(st.tau_prev + delta_tau, -tau_clamp, tau_clamp);
    st.tau_prev = tau_base;
    let tau_cmd = tau_base;
    // ⑥ Gyro feed-forward at output only
    let gyro_comp = omega.cross(vec3_mul(j_diag, omega));
    let tau_out = tau_cmd + gyro_comp;
    // ⑦
    st.omega_prev = omega;
    tau_out
}
"""

PSEUDO_CASCADE = """
/// Full cascaded INDI step @ rate fs (theoretical stack below position loop)
struct CascadedIndiState {
    outer: OuterIndiState,
    inner: InnerIndiState,
}

fn cascaded_indi_step(
    rd: Mat3,
    r: Mat3,
    omega: Vec3,
    omega_ff: Vec3,
    j_diag: Vec3,
    kr_cas: f32,
    kw_cas: f32,
    omega_max: f32,
    tau_clamp: f32,
    fc_hz: f32,
    dt: f32,
    st: &mut CascadedIndiState,
) -> Vec3 {
    let e_r = attitude_error_vee(rd, r);
    let omega_ref = outer_indi_attitude_step(
        e_r, omega, omega_ff, kr_cas, omega_max, &mut st.outer,
    );
    inner_indi_rate_step(
        omega, omega_ref, j_diag, kw_cas, tau_clamp, dt, fc_hz, &mut st.inner,
    )
}
"""


def filter_sections(*, pseudo_style: str = "callout") -> str:
    def pb(title: str, note: str, rust: str) -> str:
        return pseudo_block(title, note, rust, style=pseudo_style)

    iir = """### 3.3 First-order IIR on α_meas

Many embedded implementations use a 1st-order RC low-pass on the finite difference:

```
RC = 1 / (2π · fc)           [time constant at cutoff fc]
k  = Δt / (Δt + RC)          [blend coefficient]

α_meas[k] = k · α_raw[k] + (1 − k) · α_meas[k−1]
```

Rolloff: −20 dB/decade. Low state cost (one history sample per axis). Typical for hardware inner loops at high sample rate.
"""
    iir += pb(
        "Theoretical Rust — 1st-order IIR",
        "Pseudo-code for §3.3; not project firmware.",
        PSEUDO_IIR,
    )
    butter = """### 3.4 Second-order Butterworth (bilinear transform)

A 2nd-order Butterworth low-pass has a **maximally flat passband** and **−40 dB/decade** rolloff — stronger noise rejection than 1st-order IIR, at the cost of **more group delay** (two states per axis).

Discretised via bilinear transform (pre-warped at cutoff fc):

```
τ     = 1 / (2π·fc)                        [time constant]
denom = τ² + √2·τ·Δt + Δt²

b0 = Δt² / denom   (b1 = 2b0, b2 = b0)     [feed-forward]
a1 = 2·(Δt² − τ²) / denom                  [IIR feedback 1]
a2 = (τ² − √2·τ·Δt + Δt²) / denom          [IIR feedback 2]

α_meas[n] = b0·α_raw[n] + 2b0·α_raw[n−1] + b0·α_raw[n−2]
            − a1·α_meas[n−1] − a2·α_meas[n−2]
```

**When to use which (inner loop only — outer loop does not differentiate ω):**

| Filter | Rolloff | States / axis | Phase lag | Typical use |
|--------|---------|---------------|-----------|-------------|
| 1st-order IIR | −20 dB/dec | 1 | Lower | Simple embedded, 500 Hz+ |
| 2nd-order Butterworth | −40 dB/dec | 2 | Higher | Simulation, aggressive tracking, offline analysis |

For the cascaded **inner** INDI loop, filter choice on α_meas is the main sensing design knob; the **outer** loop reads η̇_meas ≈ ω directly and needs no differentiation filter."""
    butter += pb(
        "Theoretical Rust — 2nd-order Butterworth",
        "Pseudo-code for §3.4 bilinear coefficients; not project firmware.",
        PSEUDO_BUTTERWORTH,
    )
    return iir + "\n\n" + butter


def build_theory_markdown(md: str, *, pseudo_style: str = "callout") -> str:
    def pb(title: str, note: str, rust: str) -> str:
        return pseudo_block(title, note, rust, style=pseudo_style)

    parts = [
        """## 1. Physical model and attitude error

### 1.1 Rotational dynamics""",
        filter_theory(between(md, "### 2.1 Quadrotor", "### 2.2 Attitude")),
        "### 1.2 Attitude on SO(3)",
        filter_theory(between(md, "### 2.2 Attitude", "### 2.3 The geometric"))
        + pb(
            "Theoretical Rust — SO(3) attitude error",
            "Pseudo-code for eR vee map used by the outer loop.",
            PSEUDO_SO3_ERROR,
        ),
        """### 1.3 Position reference loop (not INDI — context only)

The trajectory outer loop (Lee et al. 2010) maps position/velocity errors to desired thrust, desired attitude Rd, and flatness feedforward ω_ff:

```
F_d = m · (ẍ_des + KP·ep + KV·ev + KI·∫ep + g·ẑ)
thrust = F_d · (R·ẑ)
Rd = f(F_d, yaw_des)
ω_ff = flatness(jerk, snap, …)     [optional feedforward into cascade]
```

Cascaded INDI sits **below** this layer: it takes Rd and produces torque via outer + inner INDI loops.""",
        """## 2. Incremental INDI — the principle (Smeur 2016)

### 2.1 Taylor expansion""",
        filter_theory(between(md, "### 3.1 The Taylor", "### 3.2 The INDI")),
        "### 2.2 INDI control law",
        filter_theory(between(md, "### 3.2 The INDI", "### 3.3 Two"))
        + pb(
            "Theoretical Rust — core INDI increment",
            "Smeur 2016 τ = τ_prev + J·(α_ref − α_meas); optional Tal α_des in α_ref.",
            PSEUDO_INDI_CORE,
        ),
        "### 2.3 NDI vs INDI",
        filter_theory(between(md, "### 3.5 NDI vs INDI", "### 3.6 The incremental")),
        "### 2.4 When the incremental assumption holds",
        polish_theory_prose(
            filter_theory(between(md, "### 3.6 The incremental", "### 3.7 The"))
        ),
        tal_flatness_section(),
        """## 3. Inner loop — measuring angular acceleration

### 3.1 Why differentiation is hard""",
        polish_theory_prose(
            filter_theory(between(md, "### 5.1 Why this is hard", "### 5.2 Low-pass"))
        ),
        "### 3.2 Low-pass filtering before differentiation",
        polish_theory_prose(
            filter_theory(between(md, "### 5.2 Low-pass", "### 5.3 Two filter"))
        ),
        filter_sections(pseudo_style=pseudo_style),
        "### 3.5 Filter cutoff trade-off",
        polish_theory_prose(
            filter_theory(between(md, "### 5.4 Filter cutoff", "### 5.5 Filter seeding"))
        ),
        """## 4. Gyroscopic feed-forward at the output

### 4.1 Why gyro_comp is not stored in the integrator""",
        filter_theory(between(md, "### 4.3 Critical subtlety", "### 4.4 Why measure"))
        + "\n\n"
        + (
            "> **Summary:** `α_meas` already includes physical gyroscopic acceleration, so the "
            "INDI increment does not explicitly cancel ω×(Jω) (unlike NDI). `gyro_comp` is added "
            "only at the output for net torque at the mixer — see §2.2.\n"
        )
        + pb(
            "Theoretical Rust — gyro_comp output structure",
            "τ_base accumulates INDI only; ω×(Jω) added at output (§4.1).",
            """
fn apply_gyro_feedforward(tau_base: Vec3, omega: Vec3, j_diag: Vec3) -> Vec3 {
    let j_omega = vec3_mul(j_diag, omega);
    tau_base + omega.cross(j_omega)
}

fn inner_accumulate_tau(tau_prev: Vec3, delta_tau: Vec3, clamp: f32) -> Vec3 {
    clamp_vec3(tau_prev + delta_tau, -clamp, clamp) // store this as tau_prev — NOT tau_out
}
""",
        ),
        filter_theory(between(md, "### 4.4 Why measure", "---\n\n## 5. Angular")),
        """## 5. Inner-loop stability (single-loop rate INDI)

### 5.1 τ integrator and 3rd-order dynamics

INDI outputs a torque **increment** and accumulates:

```
τ[k] = τ[k−1] + δτ[k]     where    δτ = ν_rate − J·α_meas
```

With rigid-body dynamics `J·ω̇ = τ`, the inner loop is **3rd-order** (integrator + double integrator from rate → attitude kinematics when viewed in cascade context).

Characteristic polynomial (single-axis, small-angle proxy):

```
J·s³ + KW·s² + KR·s = 0     →     s · (J·s² + KW·s + KR) = 0
```

- Pole at `s = 0` from the τ integrator (structural in all INDI implementations)
- Remaining roots from `J·s² + KW·s + KR = 0` — stable for J > 0, KW > 0, KR > 0 (Routh)

**Design note:** inner gains KW_cas and KR (in single-loop form) must yield well-damped rate tracking before closing the outer cascade.""",
        """## 6. Why cascade — limit of single-loop INDI""",
        polish_theory_prose(
            filter_theory(between(md, "### 13.1 Why cascade", "### 13.2 Architecture"))
        ),
        """## 7. Target architecture — cascaded INDI (Smeur 2018)

### 7.1 Full stack diagram""",
        filter_theory(between(md, "### 13.2 Architecture", "### 13.3 Inner")),
        """### 7.2 Inner INDI loop — angular rate""",
        polish_theory_prose(
            filter_theory(between(md, "### 13.3 Inner", "### 13.4 Outer"))
        )
        + RPM_DECK_INNER_SECTION
        + pb(
            "Theoretical Rust — inner INDI loop (rate → torque)",
            "Steps ①–⑦ from §7.2; uses 1st-order IIR on α_meas.",
            PSEUDO_INNER,
        ),
        """### 7.3 Outer INDI loop — INDI on angular rate""",
        filter_theory(between(md, "### 13.4 Outer", "### 13.5 Why the cascade"))
        + pb(
            "Theoretical Rust — outer INDI loop (attitude → ω_ref)",
            "Steps ①–⑥ from §7.3; no gyro differentiation.",
            PSEUDO_OUTER,
        ),
        pb(
            "Theoretical Rust — full cascaded step",
            "Composes outer + inner for one control period (below Lee position loop).",
            PSEUDO_CASCADE,
        ),
        """## 8. Cascade performance and stability

### 8.1 Why the cascade outperforms single-loop""",
        polish_theory_prose(
            filter_theory(between(md, "### 13.5 Why the cascade", "### 13.6 Stability"))
        ),
        """### 8.2 Stability and bandwidth separation

**Inner loop:** same 3rd-order structure as §5.1 — stable for KW_cas > 0, J > 0.

**Outer loop:** with inner tracking idealized, attitude error dynamics are 1st-order:

```
ė_R + KR_cas · e_R = 0     →     pole at s = −KR_cas
```

**Cascade separation (Khalil):** tune inner bandwidth first; require inner BW / outer BW ≥ 5.

```
ω_inner  ≈ KW_cas / J     [rad/s]
ω_outer  ≈ KR_cas         [rad/s]
```

When ω_inner ≫ ω_outer, the loops decouple and can be designed independently (Smeur 2018).""",
        """### 8.3 Tuning concept (paper procedure, generic)

| Phase | Loop | Fix | Tune |
|-------|------|-----|------|
| 1 | Inner (rate) | Hold ω_ref constant (rate hold) | KW_cas, fc until rate tracks cleanly |
| 2 | Outer (attitude) | Close outer with inner validated | KR_cas; require inner BW >> outer BW |

| Gain | Role | Typical units |
|------|------|----------------|
| KW_cas | Inner rate damping | Nm/(rad/s) |
| KR_cas | Outer attitude → rate command | (rad/s)/rad = 1/s |
| fc | IIR cutoff on α_meas | Hz |
| ω_max | Outer rate command clamp | rad/s |

**Separation principle:** design inner bandwidth first; outer bandwidth ≪ inner (ratio ≥ 5 in classical cascade theory).""",
        single_vs_cascade_table(),
        """## 9. Key references

1. Smeur et al. (2016) — incremental NDI / INDI law, Taylor expansion, adaptive G.
2. Smeur et al. (2018) — cascaded INDI: outer on attitude rate, inner on torque.
3. Lee et al. (2010) — geometric position loop (Rd, thrust); unchanged in cascade stack.
4. Tal & Karaman (2020) — α_des and ω_ff from differential flatness (optional extension).""",
        """## 10. Symbol glossary (cascade)""",
        """| Term | Meaning |
|------|---------|
| INDI | Incremental NDI — local linearization + measured increment |
| eR | SO(3) attitude error between Rd and R |
| eω | ω − ω_ref (inner rate error) |
| ν_att | Outer virtual control [rad/s] = −KR_cas·eR |
| ν_rate | Inner virtual control [Nm] = −KW_cas·eω |
| δω_des | Outer INDI increment [rad/s] = ν_att − η̇_meas |
| δτ | Inner INDI increment [Nm] = ν_rate − J·α_meas |
| ω_des_prev | Outer integrator state [rad/s] |
| τ_prev | Inner integrator state [Nm] (excludes gyro_comp); optional if using RPM deck |
| τ_current | Torque from measured RPM² via G(Ω) (RPM deck, recommended) |
| G(Ω) | Maps motor RPM² vector to applied torque [Nm] |
| η̇_meas | Measured attitude rate ≈ ω (gyro, no differentiation) |
| α_meas | Filtered angular acceleration from gyro [rad/s²] |
| α_des | Flatness feedforward angular acceleration (Tal 2020) |
| ω_ref | ω_des + ω_ff passed to inner loop |
| gyro_comp | ω×(Jω) added at output only |
| G1_inner | ∂ω̇/∂τ = J⁻¹ |
| G1_outer | ∂η̇/∂ω_des ≈ I |
| Separation ratio | (inner BW) / (outer BW) — should be ≫ 1 |""",
    ]
    return polish_theory_prose("\n\n".join(parts))


STUDY_PATH_ASCII = """```text
Position (Lee 2010) — not INDI
        │  Rd, ω_ff
        ▼
┌───────────────────────────────┐
│ OUTER INDI — attitude (2018)  │  ν_att = −KR_cas·eR
│ η̇_meas ≈ ω  (no filter)       │  δω → ω_des_prev → ω_ref
└───────────────┬───────────────┘
                │ ω_ref
                ▼
┌───────────────────────────────┐
│ INNER INDI — rate (2016)      │  ν_rate = −KW_cas·eω
│ α_meas = IIR(dω/dt)           │  τ_current+δτ → τ_out+ω×Jω (RPM)
└───────────────┬───────────────┘
                ▼
           Motor mixer
```"""


def build_md_document(source_md: str) -> str:
    header = """# Cascaded INDI — Theory & Target Architecture

**Inner + outer INDI loops (Smeur 2018)** — paper notation, no firmware.

| Field | Content |
|-------|---------|
| **Scope** | Theory, math, ideal cascaded architecture, theoretical Rust pseudo-code |
| **Papers** | Smeur 2016 → Smeur 2018 cascade; Lee 2010 position loop (context); Tal 2020 flatness (optional) |
| **Companion** | Full course doc: [INDI_STUDY.md](INDI_STUDY.md) |

## Suggested reading order

1. **§0** Geometric controller and mocap baseline (figure-8 plots)
2. **§1** Physical model and SO(3) error
3. **§2** Incremental INDI principle (Smeur 2016); optional §2.5 Tal flatness
4. **§3** Inner-loop α_meas filtering (IIR, Butterworth, cutoff)
5. **§4** Gyroscopic output structure
6. **§5** Inner-loop stability
7. **§6–§8** Cascaded architecture, both loops, stability; §8.4 single vs cascade
8. **§9–§10** References and symbol glossary

"""
    body = build_theory_markdown(source_md, pseudo_style="md")
    footer = """

---

*Curated from [INDI_STUDY.md](INDI_STUDY.md). Regenerate theory + plot gallery:*

```bash
python3 scripts/generate_cascaded_indi_theory_canvas.py
```
"""
    study_path = "## Study path at a glance\n\n" + STUDY_PATH_ASCII.strip() + "\n"
    return (
        header
        + geom_mocap_markdown_section()
        + "\n"
        + study_path
        + "\n---\n\n"
        + body
        + footer
    )


HEADER = f"""import {{
  Callout,
  Card,
  CardBody,
  CardHeader,
  Code,
  Divider,
  H1,
  H2,
  H3,
  Link,
  Pill,
  Row,
  Stack,
  Table,
  Text,
  useHostTheme,
}} from 'cursor/canvas';

const CANVAS_DIR = '{CANVAS_DIR}';

function StudyBoardLink({{ file, children }}: {{ file: string; children: string }}) {{
  return <Link href={{`${{CANVAS_DIR}}/${{file}}`}}>{{children}}</Link>;
}}

type LegendEntry = {{ sym: string; def: string }};

function Pre({{ children }}: {{ children: string }}) {{
  return (
    <Text
      size="small"
      style={{{{
        whiteSpace: 'pre-wrap',
        display: 'block',
        fontFamily: 'monospace',
        fontSize: 11,
        lineHeight: 1.4,
      }}}}
    >
      {{children}}
    </Text>
  );
}}

function Formula({{ formula, legend }}: {{ formula: string; legend: LegendEntry[] }}) {{
  return (
    <Stack gap={{6}}>
      <Pre>{{formula}}</Pre>
      {{legend.length > 0 ? (
        <Stack gap={{4}}>
          <Text size="small" tone="tertiary" weight="medium">where</Text>
          <Row gap={{10}} wrap>
            {{legend.map(({{ sym, def }}, i) => (
              <Text key={{`${{sym}}-${{i}}`}} size="small" tone="secondary">
                <Text weight="semibold">{{sym}}</Text> — {{def}}
              </Text>
            ))}}
          </Row>
        </Stack>
      ) : null}}
    </Stack>
  );
}}

function Bullets({{ items }}: {{ items: string[] }}) {{
  return (
    <Stack gap={{6}}>
      {{items.map((item) => (
        <Text key={{item}} size="small">• {{item}}</Text>
      ))}}
    </Stack>
  );
}}

function AsciiDiagram({{ children }}: {{ children: string }}) {{
  const theme = useHostTheme();
  return (
    <div
      style={{{{
        whiteSpace: 'pre',
        overflowX: 'auto',
        fontFamily: 'monospace',
        fontSize: 10,
        lineHeight: 1.3,
        padding: 12,
        borderRadius: 6,
        background: theme.fill.tertiary,
        color: theme.text.primary,
        border: `1px solid ${{theme.stroke.secondary}}`,
      }}}}
    >
      {{children}}
    </div>
  );
}}
"""


def main() -> None:
    sync_fig8_plot_assets()
    md = MD_PATH.read_text(encoding="utf-8")

    plots_doc = build_figure8_plots_markdown()
    FIG8_PLOTS_MD.write_text(plots_doc, encoding="utf-8")
    print(
        f"Wrote {FIG8_PLOTS_MD} ({len(plots_doc)} bytes, {plots_doc.count(chr(10)) + 1} lines)"
    )

    md_doc = build_md_document(md)
    MD_OUT_PATH.write_text(md_doc, encoding="utf-8")
    print(f"Wrote {MD_OUT_PATH} ({len(md_doc)} bytes, {md_doc.count(chr(10)) + 1} lines)")

    body = build_theory_markdown(md, pseudo_style="callout")
    gen.CALLOUTS_AFTER_H2 = {}
    content = gen.md_body_to_tsx(body)

    full = (
        HEADER
        + f"""
export default function CascadedIndiTheoryBoard() {{
  return (
    <Stack gap={{24}}>
      <Stack gap={{8}}>
        <H1>Cascaded INDI — Theory &amp; Target Architecture</H1>
        <Text weight="semibold">Inner + outer INDI loops (Smeur 2018) — paper notation, no firmware</Text>
        <Row gap={{8}} wrap>
          <Pill tone="info">Smeur 2016 → Smeur 2018 cascade</Pill>
          <Pill tone="neutral">Lee 2010 position loop (context)</Pill>
          <Pill tone="neutral">Tal 2020 flatness (optional)</Pill>
        </Row>
      </Stack>

      <Callout tone="info" title="How this board relates to the full study doc">
        <Stack gap={{6}}>
          <Text size="small">
            Focused companion to{' '}
            <StudyBoardLink file="indi-study-board.canvas.tsx">indi-study-board</StudyBoardLink>
            {' '}(complete INDI_STUDY.md). Theory, math, ideal cascaded architecture, and{' '}
            <Text weight="semibold">theoretical Rust pseudo-code</Text> after each algorithm (not project firmware).
          </Text>
          <Text size="small" weight="semibold">Suggested reading order</Text>
          <Bullets
            items={{[
              '§0 Geometric controller & mocap baseline (figure-8 plots)',
              '§1 Physical model & SO(3) error',
              '§2 Incremental INDI principle (Smeur 2016)',
              '§3 Inner-loop α_meas filtering',
              '§4 Gyroscopic output structure',
              '§5 Inner-loop stability',
              '§6–§8 Cascaded architecture, both loops, stability',
              '§9–§10 References & symbols',
            ]}}
          />
        </Stack>
      </Callout>

{geom_mocap_canvas_tsx()}

      <Card>
        <CardHeader>Study path at a glance</CardHeader>
        <CardBody>
          <AsciiDiagram>{{`Position (Lee 2010) — not INDI
        │  Rd, ω_ff
        ▼
┌───────────────────────────────┐
│ OUTER INDI — attitude (2018)  │  ν_att = −KR_cas·eR
│ η̇_meas ≈ ω  (no filter)       │  δω → ω_des_prev → ω_ref
└───────────────┬───────────────┘
                │ ω_ref
                ▼
┌───────────────────────────────┐
│ INNER INDI — rate (2016)      │  ν_rate = −KW_cas·eω
│ α_meas = IIR(dω/dt)           │  τ_current+δτ → τ_out+ω×Jω (RPM)
└───────────────┬───────────────┘
                ▼
           Motor mixer`}}</AsciiDiagram>
        </CardBody>
      </Card>

{content}

      <Text tone="secondary" size="small">
        Curated from flying_drone_stack/docs/INDI_STUDY.md — regenerate with
        scripts/generate_cascaded_indi_theory_canvas.py
      </Text>
    </Stack>
  );
}}
"""
    )
    OUT_PATH.write_text(full, encoding="utf-8")
    print(f"Wrote {OUT_PATH} ({len(full)} bytes, {full.count(chr(10)) + 1} lines)")


if __name__ == "__main__":
    main()
