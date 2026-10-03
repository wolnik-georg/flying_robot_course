# Cursor prompt — fix the RPM figures (selection/method), clean the Z-tracking figure, small INDI fixes (desk only, 2026-10-03)

Repo `~/Desktop/flying_robot_course`. Same constraints as the previous prompt (never fly/flash/commit/push, no `Co-Authored-By`, don't edit `docs/07`, `docs/meetings/2026-10-03.md`, firmware, yaml, raw logs; all numbers from stated reproducible rules; list all exclusions). Touch only `experiments/analysis/meeting_simple_*.py`, `docs/meetings/assets/2026-10-03/fig_*`, `experiments/analysis/out/meeting_2026-10-03/*`. Naming: "Ours / Omar C / Omar Rust".

## What Claude found (verified by looking at the figures and re-running numbers)

**A. RPM figures — the two picked flights do not show what we want.**
1. `A8_2026-09-21_13-14-36 m3`: lag 6 ms / RMSE 509 RPM are **dominated by one ~50 ms glitch at t≈11.1 s in the DECK trace** (deck dips to ≈5 000 and spikes to ≈24 000 RPM while DShot is smooth). The `|DShot−deck|>10 000` rule cannot tell which source glitched, and replacing only DShot leaves a residual error — the metric is measuring a deck glitch, not a systematic source difference. The raw (grey) DShot spikes are also still drawn (we do not want spike plots).
2. `A2_2026-09-23_19-27-03 cf_second m3` (RMSE 1091 RPM, "best difference"): the motor **shuts down at ≈8.5 s** and the rest of the log is RPM ≈ 0–2 000 / a flat ≈ 700 RPM filtered-DShot tail vs deck 0 — post-shutdown, not a meaningful deck-vs-DShot comparison. The pick rule (largest RMSE) selects artefacts.

**B. `fig_z_tracking_geometric.png` needs cleanup:** (1) excluded flights are partly drawn (dashed traces and "excl" labels cross the panels, Figure-8 panel is cluttered); (2) formation panels are titled "formation top / cf5 ki_z=16" — they are the **top drone `cf_second`** (say so: "A1 — top drone (cf_second), geometric + ki_z=16"); (3) the table's Hover/Figure-8 **max |error| ≈ 10–12 cm** is inconsistent with a steady window starting at liftoff+6 s (the plot shows ≈3 cm at +6 s) — find out whether the window or the liftoff definition (solo logs start at z≈−0.024) lets the convergence transient into the max, fix it or state exactly what the max covers.

**C. INDI table/figure (accepted, small fixes):** (1) legend labels show a truncated date "(026-10-02)" — use the flight id (HH-MM-SS); (2) the flag "crashed" is wrong for flights that **completed** with a large tilt excursion: `A8 Omar C 18-23-23` (max tilt 55.8°, flew 39 s) and `A1 Omar Rust 18-48-33` (62.9°, flew 32 s) — use a separate flag "completed, tilt excursion >45°" vs "crashed (flip/abort)" and report per-variant medians **with and without** the excursion flights, with n; (3) for `A1 Ours 19-15-47` z drops to ≈0 m around t≈16 s and ≈23–25 s — state in the md whether this is touchdown/bounce or a landing and how it enters mean/max.

## Tasks

**T1 — RPM method (change the script, then redo table + two figures).**
- **Airborne window:** use only samples where the motor is running and the vehicle is airborne: deck > 8 000 RPM **and** raw DShot > 8 000 RPM (before any filtering), further restricted to the contiguous flight segment between first and last such sample **per motor**. **Exclude flights/motors** where a motor shuts down or drops below 5 000 RPM for > 0.5 s while z > 0.3 m (list them, e.g. `A2_2026-09-23_19-27-03 cf_second`).
- **Symmetric glitch handling:** a sample is **inconsistent** if `|DShot − deck| > 10 000` RPM or DShot ≥ 60 000. Remove such samples from **both** series (same timestamps); compute bias, RMSE, Pearson r on the remaining aligned samples; for the lag, fill the removed samples by linear interpolation of **both** series (or compute on contiguous segments) and say which. **Classify each removed sample** as DShot-high spike (DShot ≫ deck) vs deck glitch (deck ≫ DShot or deck dip) and report both counts per flight, so we know which source glitched.
- Keep the formulas exactly as stated in the earlier prompt (bias, bias %, RMSE, r, lag = argmax of the cross-correlation of mean-removed series over ±50 ms, positive = DShot later) and write them at the top of the md. State whether filtering changes the lag/RMSE vs the old pipeline (per flight and median).
- **Figures (two flights):** choose among **airborne-clean flights** by stated rules: (1) the flight with the largest systematic |bias %|; (2) the flight with the largest lag whose rolling lag (2 s windows) is stable (e.g. IQR < 2 ms). Each figure: top = deck vs DShot (filtered) over the **airborne full flight**, lines of different colour/style so both are visible, **no raw DShot, no spike markers**, removed samples simply left as gaps; bottom = difference; inset zoom on a fast transient where the few-ms lag is visible; text box with bias, RMSE, r, lag, n removed (DShot-high / deck). Large fonts. If no flight has a visibly different pair of traces, say so plainly in the md (the honest message may be "traces are nearly identical; lag 2–5 ms").
- **Table:** all flights × 4 motors (airborne-clean only) + one summary line (median, IQR over flights); exclusions listed.

**T2 — Clean `fig_z_tracking_geometric.png`** per B (no excluded traces drawn; clear titles; Figure-8 panel readable; consistent axes ±10 cm with ±2 cm band; n in titles) and fix/explain the max |error| inconsistency; update `table_z_tracking_geometric.md` accordingly.

**T3 — INDI fixes** per C (`fig_indi_z_tracking.png`, `table_indi_z_error.md`).

## Deliverable / reporting
Updated PNGs/tables/scripts. Lead the report with: (1) which two RPM flights were chosen, by which rule, and their bias/RMSE/r/lag + counts of DShot-high vs deck glitches, (2) the Z-tracking max-|error| explanation, (3) INDI medians with/without excursion flights. Confidence per claim; list anything you could not resolve.
