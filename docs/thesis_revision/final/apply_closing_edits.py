#!/usr/bin/env python3
"""Closing edits (2026-10-15): applies D-14 (as written), D-03 (narrowed gap), D-05 (conservative status wording),
D-08 (Abro paragraph dropped), D-10 (corrected bib), and minor fixes to round3/revised -> final/revised."""
import re, shutil, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parent
SRC, DST = ROOT.parent / "round3" / "revised", ROOT / "revised"
if DST.exists(): shutil.rmtree(DST)
shutil.copytree(SRC, DST)
T = {p.name: p.read_text() for p in DST.glob("ch*.tex")}
log = []

def rep(fn, old, new, count=1):
    pat = r"\s+".join(re.escape(w) for w in old.split())
    t, n = re.subn(pat, lambda m: new, T[fn])
    assert n == count, f"{fn}: expected {count} match(es), got {n}: {old[:70]!r}"
    T[fn] = t; log.append((fn, old[:60]))

def drop_block(fn, start_re, count=1):
    """remove a comment line matching start_re plus its continuation lines (start with '%' + >=2 spaces + text)."""
    lines = T[fn].split("\n"); out = []; i = 0; n = 0
    while i < len(lines):
        if re.match(start_re, lines[i]):
            n += 1; i += 1
            while i < len(lines) and re.match(r"^\s*%\s{2,}\S", lines[i]): i += 1
            continue
        out.append(lines[i]); i += 1
    assert n == count, f"{fn}: drop_block {start_re!r} matched {n}, expected {count}"
    T[fn] = "\n".join(out); log.append((fn, "drop " + start_re[:50]))

# ---- D-14 (as written): drop the confirm markers
for fn in ("ch3_system_modelling.tex", "ch5_experimental_setup.tex", "ch6_results_2robot.tex", "ch7_results_multirobot.tex", "ch8_discussion.tex"):
    n = len(re.findall(r"^\s*% DECISION D-14 \(new\): provisional floor[^\n]*\n", T[fn], flags=re.M))
    T[fn] = re.sub(r"^\s*% DECISION D-14 \(new\): provisional floor[^\n]*\n", "", T[fn], flags=re.M)
    log.append((fn, f"D-14 markers removed: {n}"))

# ---- D-03 + D-08: Chapter 1
c1 = "ch1_introduction.tex"
drop_block(c1, r"^% DECISION D-03: the paragraph below lists")
rep(c1, "vehicle~\\cite{gu2025comparative} and fractional-order and classical regulators for close-proximity swarms~\\cite{abro2025fractional}. It is that",
        "vehicle~\\cite{gu2025comparative}. It is that")
drop_block(c1, r"^% DECISION D-03: the italic gap statement")
rep(c1, "\\emph{Representative methods exist for reactive compensation, for predictive compensation and for their hybrid. Each is validated on its own. To the best of our knowledge, they have not been evaluated against one another under identical conditions. The field's reporting conventions make such a comparison impossible to assemble after the fact.}",
        "\\emph{Representative methods exist for reactive compensation, for predictive compensation and for\ntheir hybrid, and each is validated on its own. To the best of our knowledge, no study has compared\nrotor-speed-based reactive compensation, learned per-neighbour predictive compensation and their\ncombination, with the measured and the predicted term added, as switch settings of one onboard\ncontroller on one airframe, across a fixed library of interaction regimes, with one definition of\nthe residual for every flight. The closest study compares an online $\\mathcal{L}_1$ estimate, a\nlearned downwash model and their combination inside one off-board model predictive controller on\nthree vehicles in two stacked formations~\\cite{hsieh2025onlineadaptation}.} The field's reporting\nconventions make such a comparison impossible to assemble after the fact.")
drop_block(c1, r"^\s*% DECISION D-03: \"the first evaluation of reactive and predictive")
rep(c1, "the first evaluation of reactive and predictive compensation on the same airframe, the same trajectories and the same formations,",
        "the first evaluation of reactive, predictive and hybrid compensation as switch settings of one\n    onboard controller, on the same airframe, the same trajectories and the same formations,")
drop_block(c1, r"^% DECISION D-03: the audits recorded below")
rep(c1, "Both found adjacent comparisons, cited in §\\ref{sec:rw-existing-comparisons}, but none spanning these families.",
        "Both found adjacent comparisons, cited in §\\ref{sec:rw-existing-comparisons}; the closest, Hsieh et al.\\ (2025), is named in the gap statement, and none compared the families as switch settings of one onboard controller.")

# ---- D-03 + D-08: Chapter 2
c2 = "ch2_related_work.tex"
drop_block(c2, r"^% DECISION D-03: the next sentence")
rep(c2, "The field does conduct systematic comparisons, and two recent ones bound what remains open.",
        "The field does conduct systematic comparisons, and a few bound what remains open.")
a = T[c2].index("% UNVERIFIED (review P-010): no PDF of Abro")
b = T[c2].index("reactive, predictive and hybrid families remain uncompared against one another.") + len("reactive, predictive and hybrid families remain uncompared against one another.")
T[c2] = T[c2][:a] + ("Hsieh et al.~\\cite{hsieh2025onlineadaptation} come closest: they compare an online estimate, a learned\n"
    "prediction and both on three vehicles inside one off-board model predictive controller\n"
    "(Section~\\ref{sec:rw-inject}). The estimator and the controller are one off-board optimiser, so the\n"
    "families are not compared as settings of one onboard controller.") + T[c2][b:]
log.append((c2, "Abro paragraph replaced by the Hsieh et al. comparison sentence (D-08, D-03)"))
drop_block(c2, r"^% DECISION D-03: the paragraph below states the gap")
rep(c2, "have not been placed under common conditions.",
        "have not been placed under common conditions as switch settings of one onboard controller; the closest study places the three information sources under common conditions inside one off-board model predictive controller~\\cite{hsieh2025onlineadaptation}.")
drop_block(c2, r"^% DECISION D-03: first gap")
rep(c2, "First, and principally, to the best of our knowledge the reactive, predictive and hybrid families have not been evaluated against one another under identical conditions. Systematic comparisons in adjacent territory exist --- of online-learning estimators on a single vehicle, and of classical regulators for close-proximity swarms (Section~\\ref{sec:rw-existing-comparisons}) --- but none spans the families distinguished by how they obtain the interaction force.",
        "First, and principally, to the best of our knowledge no study has compared rotor-speed-based reactive compensation, learned per-neighbour predictive compensation and their combination, with the measured and the predicted term added, as switch settings of one onboard controller on one airframe, across a fixed library of interaction regimes, with one definition of the residual for every flight. Systematic comparisons in adjacent territory exist --- of online-learning estimators on a single vehicle, and one that places an online estimate, a learned prediction and both inside one off-board model predictive controller on three vehicles~\\cite{hsieh2025onlineadaptation} (Section~\\ref{sec:rw-existing-comparisons}) --- but none compares the families as switch settings of one onboard controller.")
drop_block(c2, r"^% DECISION D-03: the provenance note below")
rep(c2, "Both found adjacent comparisons but none spanning these families.",
        "Both found adjacent comparisons; none compared the families as switch settings of one onboard controller, and the closest (Hsieh et al.) is named in the gap statement.")

# ---- D-05
drop_block("ch4_control_architectures.tex", r"^% DECISION D-05 \(H9\): the last column states implementation", 1)
drop_block("ch4_control_architectures.tex", r"^% DECISION D-05 \(H9\): implementation status", 1)
c5 = "ch5_experimental_setup.tex"
a = T[c5].index("% DECISION D-05 (H9, lab status; still open in round 2)")
m = "separation and speed grid is only partly filled."
b = T[c5].index(m) + len(m)
T[c5] = T[c5][:a] + T[c5][b:].lstrip("\n"); log.append((c5, "campaign-status paragraph of the retired controllers deleted (D-05 option i)"))
drop_block(c5, r"^% DECISION D-05 \(H9\): the last sentence of the next paragraph")
rep(c5, "The predicted alignment is a few milliseconds; that figure is to be confirmed against real logs on the first two-vehicle flight.",
        "The expected alignment is a few milliseconds, set by the broadcast jitter and the logging period; the measured value is reported for every two-vehicle session.")
c6 = "ch6_results_2robot.tex"
drop_block(c6, r"^% DECISION D-05 \(H9\): the three paragraphs of this section", 1)
drop_block(c6, r"^% DECISION D-05 \(H5\): the Strategy 4 row", 1)
rep(c6, "Strategy~4 (pred.\\ + FBL) & --- & --- & --- & --- \\\\", "Strategy~4 (FBL; not impl.) & --- & --- & --- & --- \\\\")
c7 = "ch7_results_multirobot.tex"
drop_block(c7, r"^% DECISION D-05 \(H9\): \"the third brushless vehicle", 1)
rep(c7, "Three-robot flights require the third brushless vehicle in the roster and scenarios B1, B2 and/or B3 from the frozen library",
        "Three-robot flights use three vehicles of the platform of Section~\\ref{sec:setup-platform} and scenarios B1, B2 and/or B3 from the frozen library")
drop_block("ch8_discussion.tex", r"^% DECISION D-05: the last clause presupposes", 1)
c9 = "ch9_conclusion.tex"
drop_block(c9, r"^% DECISION D-05 \(H9\): the list below mixes", 1)
drop_block(c9, r"^% DECISION D-05 \(round 2 note\)", 1)
rep(c9, "Several extensions are already scoped in project documentation and do not depend on the comparison outcome:",
        "Several extensions are scoped and do not depend on the comparison outcome:")
a = T[c9].index("\\item \\textbf{Training coverage and comparison repeats.}")
b = T[c9].index("  % CHANGED (round 3, D-13 = (b)): the three-vehicle scenarios are phase 2")
T[c9] = T[c9][:a] + T[c9][b:]; log.append((c9, "first bullet (campaign tasks) removed"))
rep(c9, "\\item \\textbf{Retrained NA-INDI / LINDI reference.} Our notes describe a retrain path for the Cobo-Briesewitz hybrid; it is out of the primary compared set but remains a candidate extension if hardware budget allows.",
        "\\item \\textbf{A retrained single-vehicle hybrid.} The learned variant of the incremental hybrid of Cobo-Briesewitz et al.\\ is not part of the compared set and remains a candidate extension.")
rep(c9, "\\item \\textbf{Residual reinforcement-learning strategies.} Explicitly deferred in the project convention: reinforcement-learning residual policies are not part of this comparison and would require a separate safety and data budget before any flight claim.",
        "\\item \\textbf{Residual reinforcement-learning strategies.} Reinforcement-learning residual policies are not part of this comparison and would require a separate safety and data budget before any flight claim.")
rep(c9, "\\item \\textbf{Hardware validation of onboard prediction.} Simulation does not substitute for flights with the learned term active on the brushless pair; flash-resident vs upload deployment trades reflash cost against lab iteration speed.",
        "\\item \\textbf{Onboard prediction beyond this platform.} The comparison evaluates the learned term on one airframe; other airframes and larger teams are open.")

# ---- minor fixes
rep(c5, "The floor is lowered to \\SI{0.10}{\\metre} for one scenario only, the near-ground pass of Section~\\ref{sec:setup-scenarios}, whose purpose",
        "The floor is lowered to \\SI{0.10}{\\metre} for one scenario only, the near-ground pass of Section~\\ref{sec:setup-scenarios} (defined in the library, not part of the comparison campaign), whose purpose")
rep(c5, "And battery discharge changes the thrust constant slightly within a flight, which appears as a slow drift in the measured residual; the effect is bounded by the null scenarios but not removed.",
        "And a slow drift of the thrust constant within a flight, for example from rotor wear or temperature, appears as a slow drift in the measured residual; it is not compensated, and the null scenarios and the baseline anchors bound it but do not remove it.")
rep(c5, "Battery discharge within a flight is not compensated", "A drift of the thrust constant within a flight is not compensated")
rep("ch3_system_modelling.tex", "& Battery discharge; rotor wear; miscalibration", "& Rotor wear; temperature; miscalibration")

for fn, t in T.items(): (DST / fn).write_text(t)
(ROOT / "closing_edits_log.txt").write_text("\n".join(f"{a}: {b}" for a, b in log) + "\n")
print(len(log), "edits applied")
