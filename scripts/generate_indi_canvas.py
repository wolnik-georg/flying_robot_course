#!/usr/bin/env python3
"""Generate indi-study-board.canvas.tsx from INDI_STUDY.md (full parity)."""

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MD_PATH = ROOT / "flying_drone_stack/docs/INDI_STUDY.md"
OUT_PATH = Path.home() / ".cursor/projects/home-georg-Desktop-flying-robot-course/canvases/indi-study-board.canvas.tsx"
CANVAS_DIR = OUT_PATH.parent.as_posix()

SYMBOL_DEFS: dict[str, str] = {
    "JXX": "roll inertia [kg·m²]",
    "JYY": "pitch inertia [kg·m²]",
    "JZZ": "yaw inertia [kg·m²]",
    "J⁻¹": "inverse inertia",
    "J·ω̇": "Euler rotational dynamics",
    "KR_INDI": "attitude gain [Nm/rad]",
    "KW_INDI": "rate gain [Nm/(rad/s)]",
    "KR_std": "Smeur attitude gain [rad/s²/rad]",
    "KW_std": "Smeur rate gain [rad/s²/(rad/s)]",
    "KR_cas": "cascade outer gain [1/s]",
    "KW_cas": "cascade inner gain [Nm/(rad/s)]",
    "KR_geometric": "geometric KR",
    "FC_GYRO_HZ": "IIR cutoff on α_meas [Hz]",
    "TAU_INDI_CLAMP": "max |τ_prev| per axis [Nm]",
    "OMEGA_DES_MAX": "outer rate clamp [rad/s]",
    "α_meas": "filtered measured angular acceleration [rad/s²]",
    "α_ref": "desired angular acceleration [rad/s²]",
    "α_des": "flatness feedforward α [rad/s²]",
    "α_raw": "raw gyro finite difference [rad/s²]",
    "v_des": "virtual control J·α_ref [Nm]",
    "δτ": "torque increment [Nm]",
    "δτ_z": "yaw torque increment",
    "δτ_z_prev": "previous yaw increment (du_prev_z)",
    "δω_des": "outer rate increment [rad/s]",
    "τ_prev": "accumulated torque (no gyro_comp)",
    "τ_base": "INDI accumulated state [Nm]",
    "τ_out": "τ_base + gyro_comp [Nm]",
    "τ_current": "torque from measured RPM²",
    "τ_act": "actuator torque state",
    "τ_cmd": "commanded torque",
    "gyro_comp": "ω×(J·ω) feed-forward [Nm]",
    "ω_des_prev": "outer cascade integrator [rad/s]",
    "ω_ref": "inner loop rate reference [rad/s]",
    "η̇_meas": "measured attitude rate ≈ ω [rad/s]",
    "ν_att": "outer virtual control [rad/s]",
    "ν_rate": "inner virtual control [Nm]",
    "eR": "SO(3) attitude error [rad]",
    "eω": "ω − ω_ref or ω − ω_des [rad/s]",
    "ω_des": "desired rate from flatness [rad/s]",
    "ω_prev": "previous gyro sample",
    "ω̇₀": "α at previous operating point",
    "ω̇_des": "desired angular acceleration",
    "B": "control effectiveness = J⁻¹",
    "G1": "primary effectiveness ≈ J⁻¹",
    "G2": "propeller momentum yaw coupling",
    "G_est": "adaptive control effectiveness estimate",
    "Γ": "RLS learning rate",
    "δu": "actuator input increment",
    "ep": "position error",
    "ev": "velocity error",
    "F_d": "desired force vector",
    "Rd": "desired rotation",
    "R": "current rotation",
    "m": "mass [kg]",
    "Δt": "sample period [s]",
    "dt": "control period [s]",
    "k": "IIR blend coefficient",
    "RC": "filter time constant",
    "ζ": "damping ratio",
    "ωn": "natural frequency [rad/s]",
    "τ_dom": "dominant time constant",
    "θ": "attitude angle [rad]",
    "θ̇": "attitude rate [rad/s]",
    "θ̈": "attitude acceleration [rad/s²]",
    "θ⃛": "third derivative of attitude",
    "s": "Laplace variable",
    "σ_gyro": "gyro noise std [rad/s]",
    "σ_α": "differentiated noise [rad/s²]",
    "KT": "thrust coeff [N/RPM²]",
    "KQ_KT": "drag-to-thrust ratio",
    "ARM_LEN": "motor arm length [m]",
    "lh": "effective arm × cos(45°) [m]",
    "Ωᵢ": "motor i speed [RPM]",
    "act_dyn": "actuator bandwidth coefficient",
    "ω_predicted": "predicted gyro (delay comp)",
    "ω_measured": "raw gyro measurement",
    "η̇": "attitude rate",
    "φ̇": "roll Euler rate",
    "ψ̇": "yaw Euler rate",
    "p": "roll body rate",
    "q": "pitch body rate",
    "r": "yaw body rate",
    "J": "inertia tensor [kg·m²]",
    "ω": "body angular velocity [rad/s]",
    "τ": "torque [Nm]",
    "τ_ff": "feedforward torque",
    "τ_fb": "feedback torque",
    "τ_gyro": "gyroscopic torque ω×(Jω)",
    "u": "actuator input",
    "u_prev": "previous actuator input",
    "e": "tracking error",
    "Kp": "proportional gain",
    "Kd": "derivative gain",
    "Ki": "integral gain",
}


def esc_html(s: str) -> str:
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def clean_md_inline(s: str) -> str:
    s = re.sub(r"\*\*([^*]+)\*\*", r"\1", s)
    s = re.sub(r"`([^`]+)`", r"\1", s)
    return s.strip()


def ts_str(s: str) -> str:
    return (
        s.replace("\\", "\\\\")
        .replace('"', '\\"')
        .replace("\n", "\\n")
        .replace("\r", "")
    )


def jsx_backtick(s: str) -> str:
    inner = s.replace("\\", "\\\\").replace("`", "\\`").replace("${", "\\${")
    return "`" + inner + "`"


def guess_legend(formula: str) -> list[tuple[str, str]]:
    found: list[tuple[str, str]] = []
    for sym, defn in sorted(SYMBOL_DEFS.items(), key=lambda x: -len(x[0])):
        if sym in formula and sym not in [f[0] for f in found]:
            found.append((sym, defn))
    return found[:14]


def is_diagram(block: str) -> bool:
    if "┌" in block or ("│" in block and "─" in block):
        return True
    if "▼" in block and "Position reference" in block:
        return True
    if "┌" in block or "└" in block:
        return True
    return False


def is_rust_or_shell(block: str) -> bool:
    markers = (
        "fn ",
        "const CONTROLLER",
        "cd firmware",
        "ros2 run",
        "struct State",
        "let omega_ref",
        "cargo test",
        "python3 ",
        "#!/bin/",
        "impl ",
        "pub fn",
        "use ",
    )
    return any(m in block for m in markers)


def emit_formula_block(block: str, indent: str = "      ") -> str:
    block = block.strip("\n")
    lit = jsx_backtick(block)
    if is_diagram(block):
        return indent + "<AsciiDiagram>{" + lit + "}</AsciiDiagram>"
    legend = guess_legend(block)
    if not legend:
        return indent + "<Pre>{" + lit + "}</Pre>"
    legs = ",\n".join(
        indent + '    { sym: "' + ts_str(sym) + '", def: "' + ts_str(defn) + '" }'
        for sym, defn in legend
    )
    return (
        indent + "<Formula\n"
        + indent + "  formula={" + lit + "}\n"
        + indent + "  legend={[\n" + legs + "\n" + indent + "  ]}\n"
        + indent + "/>"
    )


def parse_table(lines: list[str]) -> tuple[list[str], list[list[str]]]:
    rows = []
    for line in lines:
        if re.match(r"^\s*\|[-| :]+\|\s*$", line):
            continue
        cells = [clean_md_inline(c.strip()) for c in line.strip().strip("|").split("|")]
        if cells and any(cells):
            rows.append(cells)
    if len(rows) < 1:
        return [], []
    return rows[0], rows[1:]


def emit_table(table_lines: list[str], indent: str = "      ") -> str:
    headers, rows = parse_table(table_lines)
    if not headers:
        return ""
    h = ", ".join(f'"{ts_str(esc_html(c))}"' for c in headers)
    row_js = []
    for r in rows:
        cells = ", ".join(f'"{ts_str(esc_html(c))}"' for c in r)
        row_js.append(f"{indent}  [{cells}]")
    rows_part = ",\n".join(row_js)
    return (
        indent
        + "<Table striped stickyHeader headers={[" + h + "]} rows={[\n"
        + rows_part
        + "\n"
        + indent
        + "]} />"
    )


def md_body_to_tsx(body: str) -> str:
    lines = body.splitlines()
    out: list[str] = []
    i = 0
    in_code = False
    code_buf: list[str] = []
    table_buf: list[str] = []
    bullet_buf: list[str] = []

    def flush_bullets():
        nonlocal bullet_buf
        if not bullet_buf:
            return
        items = ",\n".join(
            f'          "{ts_str(esc_html(clean_md_inline(b[2:].strip() if b.startswith(("- ", "* ")) else b.strip())))}"'
            for b in bullet_buf
        )
        out.append("      <Bullets\n        items={[\n" + items + "\n        ]}\n      />")
        bullet_buf = []

    def flush_table():
        nonlocal table_buf
        if table_buf:
            t = emit_table(table_buf)
            if t:
                out.append(t)
            table_buf = []

    while i < len(lines):
        line = lines[i]
        if line.startswith("```"):
            if not in_code:
                flush_bullets()
                flush_table()
                in_code = True
                code_buf = []
            else:
                in_code = False
                block = "\n".join(code_buf)
                if is_rust_or_shell(block):
                    lit = jsx_backtick(block)
                    out.append("      <Pre>{" + lit + "}</Pre>")
                else:
                    out.append(emit_formula_block(block))
                code_buf = []
            i += 1
            continue
        if in_code:
            code_buf.append(line)
            i += 1
            continue
        if line.strip().startswith("|"):
            flush_bullets()
            table_buf.append(line)
            i += 1
            continue
        flush_table()
        if re.match(r"^[\-\*] ", line):
            bullet_buf.append(line)
            i += 1
            continue
        if re.match(r"^\d+\. ", line):
            num = line.split(".", 1)[0].strip()
            item = line.strip()
            j = i + 1
            while j < len(lines):
                nxt = lines[j]
                if (
                    not nxt.strip()
                    or nxt.startswith("#")
                    or re.match(r"^\d+\. ", nxt)
                    or re.match(r"^[\-\*] ", nxt)
                    or nxt.strip().startswith("|")
                    or nxt.startswith("```")
                    or nxt.startswith(">")
                ):
                    if not nxt.strip():
                        j += 1
                        if j < len(lines) and lines[j].strip() and not re.match(
                            r"^\d+\. ", lines[j]
                        ):
                            continue
                    break
                if nxt.strip():
                    item += " " + nxt.strip()
                j += 1
            bullet_buf.append(item)
            i = j
            continue
        flush_bullets()
        if line.startswith("## "):
            t = esc_html(line[3:].strip())
            out.append(f"      <Divider />\n      <H2>{t}</H2>")
            i += 1
            continue
        if line.startswith("### "):
            out.append(f"      <H3>{esc_html(line[4:].strip())}</H3>")
            i += 1
            continue
        if line.startswith("> "):
            t = esc_html(re.sub(r"\*\*([^*]+)\*\*", r"\1", line[2:].strip()))
            tone = "success" if any(x in t.lower() for x in ("step 1", "target", "cascaded indi")) else "info"
            out.append(f'      <Callout tone="{tone}"><Text size="small">{t}</Text></Callout>')
            i += 1
            continue
        if line.strip() in ("---", ""):
            i += 1
            continue
        if line.strip().startswith("**") and line.strip().endswith("**") and line.count("**") == 2:
            t = esc_html(line.strip()[2:-2])
            out.append(f'      <Text size="small" weight="semibold">{t}</Text>')
            i += 1
            continue
        if not line.strip():
            i += 1
            continue
        para = line.strip()
        j = i + 1
        while j < len(lines):
            nxt = lines[j].strip()
            if not nxt or nxt.startswith(("#", "-", "*", "|", ">", "```")) or re.match(r"^\d+\.", nxt):
                break
            if nxt.startswith("**") and nxt.endswith("**"):
                break
            para += " " + nxt
            j += 1
        i = j
        para = esc_html(re.sub(r"\*\*([^*]+)\*\*", r"\1", para))
        para = re.sub(r"`([^`]+)`", r"\1", para)
        if para:
            out.append(f'      <Text size="small">{para}</Text>')
    flush_bullets()
    flush_table()
    return inject_section_callouts("\n".join(out))


CALLOUTS_AFTER_H2: dict[str, list[tuple[str, str, str]]] = {
    "7. Architecture in Our System": [
        ("firmware-geometric-controller-overview.canvas.tsx", "Firmware geometric controller", "Outer loop identical for Mode 0 and Mode 1"),
        ("end-to-end-flight-stack-chain.canvas.tsx", "End-to-end flight stack", "Where mocap, EKF, and inner loop sit in the chain"),
        ("mekf-estimation-deep-dive.canvas.tsx", "MEKF estimation", "Mode E: OptiTrack → EKF → geometric outer"),
    ],
    "11. Our Specific Implementation": [
        ("dynamics-simulator-deep-dive.canvas.tsx", "Dynamics simulator", "indi.rs tests and hover closed-loop"),
        ("firmware-geometric-controller-overview.canvas.tsx", "Firmware geometric controller", "lib.rs CONTROLLER_MODE, geometric_step vs indi_step"),
    ],
    "12. Step-by-Step: Activating Mode 1": [
        ("onboard-script-to-firmware-pipeline.canvas.tsx", "Onboard upload pipeline", "Mode D trajectory upload before flight"),
        ("flight-log-analysis-deep-dive.canvas.tsx", "Flight log analysis", "Post-flight figure-8 validation"),
    ],
    "13. Cascaded INDI — Smeur et al. 2018": [
        ("planner-modes-overview.canvas.tsx", "Planner modes overview", "Flatness chain: jerk → ω_des, snap → α_des"),
        ("firmware-geometric-controller-overview.canvas.tsx", "Firmware geometric controller", "Mode 3 target: outer_indi + inner INDI"),
    ],
}


def inject_section_callouts(content: str) -> str:
    for h2_title, boards in CALLOUTS_AFTER_H2.items():
        marker = f"<H2>{esc_html(h2_title)}</H2>"
        if marker not in content:
            continue
        rows = ",\n".join(
            f'          {{ file: "{f}", title: "{ts_str(t)}", when: "{ts_str(w)}" }}'
            for f, t, w in boards
        )
        callout = (
            f"      <RelatedBoardsCallout\n"
            f"        boards={{[\n{rows}\n        ]}}\n"
            f"      />"
        )
        content = content.replace(marker, marker + "\n" + callout, 1)
    return content


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

const RELATED_BOARDS: {{ file: string; title: string; when: string }}[] = [
  {{ file: 'end-to-end-flight-stack-chain.canvas.tsx', title: 'End-to-end flight stack', when: 'Planning → upload → firmware @ 500 Hz → logs → analysis' }},
  {{ file: 'firmware-geometric-controller-overview.canvas.tsx', title: 'Firmware geometric controller', when: 'Outer loop, CONTROLLER_MODE 0, geometric_step, ω_des' }},
  {{ file: 'onboard-script-to-firmware-pipeline.canvas.tsx', title: 'Onboard upload pipeline', when: 'Mode D: coefficient upload, onboard 500 Hz ref' }},
  {{ file: 'planner-modes-overview.canvas.tsx', title: 'Planner modes overview', when: 'x_des, ẍ_des, jerk, snap from planning' }},
  {{ file: 'dynamics-simulator-deep-dive.canvas.tsx', title: 'Dynamics simulator', when: 'indi.rs unit & closed-loop hover tests' }},
  {{ file: 'flight-log-analysis-deep-dive.canvas.tsx', title: 'Flight log analysis', when: 'analyze_flight.py post-flight' }},
  {{ file: 'mekf-estimation-deep-dive.canvas.tsx', title: 'MEKF estimation', when: 'Mode E mocap → EKF → outer loop' }},
];

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
          <Text size="small" tone="tertiary" weight="medium">
            where
          </Text>
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
        <Text key={{item}} size="small">
          • {{item}}
        </Text>
      ))}}
    </Stack>
  );
}}

function RelatedBoardsCallout({{
  boards,
}}: {{
  boards: {{ file: string; title: string; when: string }}[];
}}) {{
  return (
    <Callout tone="info" title="Related study boards">
      <Stack gap={{6}}>
        {{boards.map((b) => (
          <Text key={{b.file}} size="small">
            <StudyBoardLink file={{b.file}}>{{b.title}}</StudyBoardLink> — {{b.when}}
          </Text>
        ))}}
      </Stack>
    </Callout>
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


def related_boards_table() -> str:
    rows = []
    for b in [
        ("end-to-end-flight-stack-chain.canvas.tsx", "End-to-end flight stack", "Planning → upload → firmware @ 500 Hz → logs → analysis"),
        ("firmware-geometric-controller-overview.canvas.tsx", "Firmware geometric controller", "Outer loop, CONTROLLER_MODE 0, geometric_step, ω_des"),
        ("onboard-script-to-firmware-pipeline.canvas.tsx", "Onboard upload pipeline", "Mode D: coefficient upload, onboard 500 Hz ref"),
        ("planner-modes-overview.canvas.tsx", "Planner modes overview", "x_des, ẍ_des, jerk, snap from planning"),
        ("dynamics-simulator-deep-dive.canvas.tsx", "Dynamics simulator", "indi.rs unit & closed-loop hover tests"),
        ("flight-log-analysis-deep-dive.canvas.tsx", "Flight log analysis", "analyze_flight.py post-flight"),
        ("mekf-estimation-deep-dive.canvas.tsx", "MEKF estimation", "Mode E mocap → EKF → outer loop"),
    ]:
        f, title, when = b
        rows.append(
            f"""          [
            <StudyBoardLink key="{f}" file="{f}">{title}</StudyBoardLink>,
            "{ts_str(when)}",
            <Code key="c-{f}">{f}</Code>,
          ],"""
        )
    return "\n".join(rows)


def main():
    md = MD_PATH.read_text(encoding="utf-8")
    start = md.find("## 1. Motivation")
    if start < 0:
        raise SystemExit("Could not find ## 1. Motivation in INDI_STUDY.md")
    body = md[start:]
    content = md_body_to_tsx(body)

    full = (
        HEADER
        + f"""
export default function IndiStudyBoard() {{
  return (
    <Stack gap={{24}}>
      <Stack gap={{8}}>
        <H1>Incremental Nonlinear Dynamic Inversion (INDI)</H1>
        <Text weight="semibold">Complete Study Document — Quadrotor Attitude Control</Text>
        <Row gap={{8}} wrap>
          <Pill tone="info">theory → math → architecture → implementation → tuning → cascaded INDI</Pill>
        </Row>
      </Stack>
      <Callout tone="info" title="Document metadata (INDI_STUDY.md)">
        <Stack gap={{6}}>
          <Text size="small">
            <Text weight="semibold">Scope:</Text> theory → math → architecture → our exact implementation → tuning → cascaded INDI
          </Text>
          <Text size="small">
            <Text weight="semibold">References:</Text> Smeur et al. 2016, Tal &amp; Karaman 2020, Smeur et al. 2018, Lee et al. 2010
          </Text>
          <Text size="small">
            <Text weight="semibold">Implementation:</Text> <Code>src/controller/indi.rs</Code> (sim),{' '}
            <Code>firmware_app/src/lib.rs</Code> (hardware)
          </Text>
          <Text size="small">
            <Text weight="semibold">Goal:</Text> cascaded INDI (§13) — single-loop INDI (§§3–12) is the stepping stone
          </Text>
        </Stack>
      </Callout>
      <Card>
        <CardHeader>Related study boards — click to open</CardHeader>
        <CardBody>
          <Table
            striped
            headers={{['Board', 'Open when studying…', 'File']}}
            rows={{[
{related_boards_table()}
            ]}}
          />
        </CardBody>
      </Card>
{content}
      <Text tone="secondary" size="small">
        Generated from flying_drone_stack/docs/INDI_STUDY.md — full document parity (§§1–14, glossary, references).
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
