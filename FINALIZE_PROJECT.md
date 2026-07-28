# Finalize Project — final week plan (2026-07-24 → presentation)

Semester wrap-up. Investigation mode is closed out (see `flying_drone_stack/docs/results_2026-07-15_brushless.md`
§8-9 for the full brushless shake diagnosis and upgraded-drone comparison). Everything below is
about turning what's already been found into a presentation, not generating new results.

**Status (2026-07-26): SLIDES DONE.** Tasks 4-6 (narrative, slides, scope-cut) complete, across
three content-review passes (2026-07-25 evening plot-correctness pass; 2026-07-26 content-accuracy
pass — removed low-value/redundant slides, corrected an inaccurate causal claim on the slalom
slide, expanded Root Cause, rewrote Real Fix Candidates and Summary; 2026-07-26 appendix pass —
added two appendix slides, motion planning summary + full INDI pseudocode — see §5's "Third",
"Fourth", and "Fifth round" notes for the full list). **Deck is 41 slides, compiles cleanly,
operator has signed off on content — slides are considered final unless new flight data below
changes something.**

**Update (2026-07-27): bonus item 10b turned into a full `loop` inversion commissioning
session** — far beyond "a few more attempts." Full timeline, root causes, fixes, and flight log
inventory now live in `flying_drone_stack/docs/investigation_inverted_loop_2026-07-27.md` — pull
numbers from there directly for slides instead of re-deriving. Short version: `loop`'s inverted
flip is now achieved and repeatable (3/3 clean inversions, roll 163-178° right at the apex);
post-flip recovery is still open (genuine torque-saturation/chatter, not height or trajectory
shape — height was a real, separately-found root cause for the earlier crashes and is fixed).
`teardrop`/`teardrop_wide` got a height-convention fix and inherited the clamp loosening but
were not otherwise re-tuned this session (see doc §5 for why the loop technique didn't
transfer). This is good, honest limitations-slide material if there's room (§4.6-4.7 already use
this tone) — not required for the deck as-is.

**Remaining work — see the table in §11 below for the full list with status.** In short: video
(Task 7, LED ring on), rehearsal (Task 8), buffer day (Task 9), and three optional bonus flight
experiments (§10) that are not required to finish.
yaml switched back to upgraded (`kr: 603.0` active, verified) as the final resting config —
operator still needs to run `make DRONE=upgrade cload` + restart CS2 server to complete the
hardware side of the switch-back (yaml edit alone doesn't reflash firmware). Put prop guards back
on the brushless drone before storing it.

**Closing aside (2026-07-25, brushless, no guards — not part of the planned task list, came up
during Task 3 wrap-up):** operator removed the brushless drone's prop guards as a last-resort test
and re-identified kt from a clean hover (measured mass 36.7g vs 41g with guards). New kt values
pushed live (`kt1..4` in the yaml's brushless block, old guards-on values preserved as comments).
Follow-up hover with the new kt was noticeably noisier (gyro ~450-480°/s vs the ~95-300°/s
guards-on baseline) — not a crash, but a real degradation. Root cause identified but **not
pursued**: prop guards sit at the arm tips (large moment arm), so removing them measurably lowers
roll/pitch rotational inertia; the firmware's compiled `J` matrix was calibrated *with* guards on
and is now mismatched, degrading the INDI increment law (`delta_tau = J*(alpha_ref - alpha_meas)`).
Re-deriving `J` properly needs its own regression campaign (bench data or many stable flight
logs, as was done for the guards-on `J` originally) — correctly not attempted with one day of lab
time left. Guards are back on; this doesn't affect anything else in the project.

Tasks 1-2 done: hover/circle/figure8/oval footage captured on upgraded, plus two independent
clean→crash progressions (oval kt=0.3→0.5, circle kt=0.5→0.7 — the circle one wasn't originally
planned, came from testing and turned out to be good footage on its own). One likely-bad take
flagged (figure8 kt=0.05, only 5.0s — retake if unusable).

**New, added 2026-07-25**: circle kt sweep added as Task 2b, both platforms — oval already has a
cross-platform kt-sweep comparison (§9.6/9.7), circle only ever had brushless data at kt=0.05.
Circle kt=0.4/0.6 Poly4D exports generated (were missing) so the sweep density matches oval's
(0.05/0.1/0.2/0.3/0.4/0.5/0.6). Do the upgraded half now (no config change needed, same session as
Tasks 1-2); do the brushless half combined with Task 3's brushless switch (avoid switching
firmware/yaml twice in one lab session).

## Task table

**Flying/data collection (1, 2, 2b, 3) — all done 2026-07-25.** Remaining work is 100% desk work.

| # | Task | Status |
|---|---|---|
| 1 | Flight footage: hover/circle/figure8/oval on upgraded | ✅ done |
| 2 | Oval kt-sweep clean→crash | ✅ done (bonus circle crash too) |
| 2b | Circle kt sweep, both platforms | ✅ done |
| 3 | Brushless hover clip + circle/oval sweep | ✅ done |

| # | Task | Status |
|---|---|---|
| 4 | Write presentation narrative arc | ✅ done |
| 5 | Build slides using existing plots (5 rounds of revision, see §5) | ✅ done — **deck content-final, 41 slides** |
| 6 | State explicit scope cut (Phase 3/4 descoped, guards-off aside mentioned) | ✅ done |

| # | Remaining task | Why it matters | Priority |
|---|---|---|---|
| 7 | Record narrated video walkthrough — **fly with LED ring on** for footage not yet captured that way | Deliverable requirement | **Critical — next** |
| 8 | Rehearse / time the presentation | Avoid running over on presentation day | High |
| 9 | Buffer day for re-recording, slide fixes | Nothing in this project has gone smoothly on the first try | High |
| 10 | Bonus, only if time allows (see §10) — lower-kt slalom sweep both platforms, more teardrop/inverted-loop attempts, prototype the notch-filter fix | Would strengthen the deck further but the deck is already complete without it | Optional, non-blocking |

---

## 1. Capture flight footage — upgraded drone (hover/circle/figure8/oval)

| # | Step | Command / action |
|---|---|---|
| 1.1 | Confirm config is on the upgraded block (already verified 2026-07-25) | `grep -n "kr:" /home/georg/Desktop/crazyswarm2/crazyflie/config/crazyflies.yaml` → active (uncommented) line must read `kr: 603.0` |
| 1.2 | Confirm CS2 server is up and connected to the drone | check server terminal / `ros2 node list` shows the crazyflie server node |
| 1.3 | Set up camera(s) | one fixed wide shot covering the full flight volume; optional second handheld/phone angle for takeoff/landing close-ups |
| 1.4 | Do a dry run with no camera first if the drone hasn't flown yet today | `ros2 run crazyflie_examples flight -- --trajectory hover --duration 8` — confirms link/arm/land before spending a take on it |
| 1.5 | Start recording, slate the clip (say trajectory+kt out loud or hold up a card) | — |
| 1.6 | Fly hover | `ros2 run crazyflie_examples flight -- --trajectory hover --duration 15` |
| 1.7 | Fly circle kt=0.05 | `ros2 run crazyflie_examples flight -- --trajectory circle --mode 1 --kt 0.05 --onboard` |
| 1.8 | Fly figure8 kt=0.05 | `ros2 run crazyflie_examples flight -- --trajectory figure8 --mode 1 --kt 0.05 --onboard` |
| 1.9 | Fly oval kt=0.05 | `ros2 run crazyflie_examples flight -- --trajectory oval --mode 1 --kt 0.05 --onboard` |
| 1.10 | Re-slate/re-take any clip that didn't come out clean (bad framing, drone out of shot, obvious glitch) | repeat the relevant command from 1.6-1.9 |
| 1.11 | Optional: wide establishing shot of the physical setup (mocap volume, drone on stand) | for an intro/methodology slide |
| 1.12 | Stop recording, back up video files off the recording device immediately | copy to laptop/cloud same session — don't leave it only on a phone/camera SD card |
| 1.13 | Rename/organize clips by trajectory+kt so Task 5 (slides) can grab them without re-scrubbing footage | e.g. `footage/upgraded_hover.mp4`, `footage/upgraded_circle_kt0.05.mp4`, etc. |

## 2. Capture oval kt-sweep flight live (clean → crash)

- Use the already-known-safe/known-risky points from `results_2026-07-15_brushless.md` §9.6:
  kt=0.3 is clean on the upgraded drone, kt=0.5 crashes reliably (3/3 attempts this session).
- Clear the flight area / add crash protection (mats, netting) before the kt=0.5 attempt — this
  is an intentional, expected crash, not a mishap, but still a real impact.
- Film both flights back-to-back from the same fixed camera angle so they cut together cleanly in
  the video (same framing makes the "it's the same maneuver, just faster" point visually obvious).
- Capture audio if possible — the motor/prop sound audibly changes as tracking degrades, useful
  supporting evidence in the video even without overlaying data.
- After filming, no further analysis needed — the RMSE/gyro-σ numbers for both points are already
  in §9.6's table, just cite them in the slide/voiceover.

## 2b. Circle kt sweep, both platforms

Poly4D exports for kt=0.4/0.6 generated 2026-07-25 (were missing) — full sweep 0.05/0.1/0.2/0.3/
0.4/0.5/0.6/0.7 now available on both platforms for flying.

**Upgraded half — now, same session as Tasks 1-2, no config change:**

| # | Step | Command |
|---|---|---|
| 2b.1 | kt=0.1 | `ros2 run crazyflie_examples flight -- --trajectory circle --mode 1 --kt 0.1 --onboard` |
| 2b.2 | kt=0.2 | `ros2 run crazyflie_examples flight -- --trajectory circle --mode 1 --kt 0.2 --onboard` |
| 2b.3 | kt=0.3 | `ros2 run crazyflie_examples flight -- --trajectory circle --mode 1 --kt 0.3 --onboard` |
| 2b.4 | kt=0.4 | `ros2 run crazyflie_examples flight -- --trajectory circle --mode 1 --kt 0.4 --onboard` |
| 2b.5 | kt=0.6 | `ros2 run crazyflie_examples flight -- --trajectory circle --mode 1 --kt 0.6 --onboard` |

(kt=0.05 already have from July 22/23; kt=0.5 clean and kt=0.7 crash already captured today —
no need to repeat those.)

**Brushless half — combine with Task 3's switch (see below), same sweep points 0.05/0.1/0.2/0.3/
0.4/0.5/0.6/0.7, plus expect the crash point to land somewhere in that range (unknown ceiling for
circle on brushless — this is genuinely new data, not a re-confirmation).** Fly these back-to-back
with the Task 3 hover clip while already switched over, so the firmware/yaml only gets touched once.

After both halves are flown: same `analyze_flight.py --compare` workflow already used for oval
(§9.7) will work out of the box for circle too — the calibration fixes from that session are
type-agnostic, no more script changes needed.

**Upgraded half — done, 2026-07-25, results:**
- kt=0.1-0.6 (7 attempts total): **all clean**, zero crashes.
- kt=0.7-1.0 (9 attempts total): **marginal/inconsistent**, roughly 50/50 crash rate (5 crashed,
  4 clean) — not a sharp wall. Same character as the kw-floor finding elsewhere in this project:
  "inconsistent pass/fail = marginal stability, not smooth degradation."
- **Conclusion: hard-clean ceiling ~kt=0.6, marginal zone kt=0.7-1.0, stopped here** — this framing
  (ceiling + marginal band, not a single number) is the presentation-ready finding, don't fly more
  circle attempts chasing a cleaner threshold.

**Slalom — also flown today (Task "same for slalom"), confirms `corner`'s finding, not pursued
further:**
- kt=0.05 (3 attempts): 2 crashed (z=-20.5m, -2.1m, both cut short at 6.5-9.9s — "gets off
  trajectory at the beginning" exactly as observed live), 1 survived a bad dip (z=-5m) but completed.
- kt=0.1 (1 attempt): crashed (z=-4.4m).
- **Conclusion: slalom crashes even at the gentlest tested speed on upgraded — confirmed real
  (3/4 crash-level events), matches `corner`'s bandwidth-ceiling story (§9.3). Stopped here; a fix
  would require a new kr/kw tuning campaign, out of scope for this week.**

## 3. Capture brushless hover clip showing the shake (+ Task 2b brushless circle sweep)

| # | Step | Command / action |
|---|---|---|
| 3.1 | Reflash firmware to brushless | `cd flying_drone_stack/firmware_app && make cload` (DRONE=bl is the default, no flag needed) |
| 3.2 | Edit `crazyswarm2/crazyflie/config/crazyflies.yaml` | comment out the active **CF2.1 UPGRADED MOTORS** block (`kr: 603.0`), uncomment the **CF21BL BRUSHLESS — PARKED** block (preserved intact since 2026-07-23) |
| 3.3 | Restart the CS2 server | so it re-reads the yaml and pushes brushless gains (kr=2400/kw=170) to the freshly-flashed firmware |
| 3.4 | Physically swap in the brushless (CF21BL) airframe | same radio URI, just the physical drone |
| 3.5 | Verify the switch took | `grep -n "kr:" crazyswarm2/crazyflie/config/crazyflies.yaml` → active line should read `kr: 2400.0` |
| 3.6 | Camera rolling, slate the clip | "brushless, hover, shows the shake" |
| 3.7 | Fly hover | `ros2 run crazyflie_examples flight -- --trajectory hover --duration 15` |
| 3.8 | Check footage — wobble should be visible/audible | re-take if not clearly visible |
| 3.9 | Fly brushless circle sweep (Task 2b, new data — brushless has never flown circle above kt=0.05) | `--trajectory circle --mode 1 --kt 0.1 --onboard`, then 0.2/0.3/0.4/0.5/0.6/0.7 in order, stop at first crash |
| 3.10 | **Switch back to upgraded** | `make DRONE=upgrade cload`, revert the yaml comment/uncomment, restart CS2 server |
| 3.11 | Verify switch-back took | `grep -n "kr:" crazyswarm2/crazyflie/config/crazyflies.yaml` → active line should read `kr: 603.0` again |

Step 3.10-3.11 matter — don't skip the switch-back, you're done flying brushless for the rest of
the project and want to leave the rig in the state everything else assumes. This is the only
remaining task that touches the brushless drone — one hover clip plus the circle sweep is the full
scope here, not a reopening of the shake investigation.

**Done, 2026-07-25. Results:**
- Hover clip captured (5 attempts total during the session, all usable — shake visible/audible).
- Oval sweep gaps filled: kt=0.1/0.2/0.3/0.4 all clean (kt=0.3/0.4 needed one retake each after a
  radio/telemetry dropout mid-flight — confirmed via frozen RPM values right before an abrupt
  cutoff, not a crash; retakes were clean full flights). Full brushless oval sweep now:
  0.05-0.5 clean, kt=0.7 crashed (§18.2) — same density as upgraded's oval sweep.
- Circle sweep (new — brushless had never flown circle above kt=0.05): **0.1-0.5 all clean, then a
  marginal zone 0.6-0.9** (2 clean / 3 crashed across those points) — kt=1.0 skipped since 0.8/0.9
  already crashed. **Same qualitative pattern as upgraded circle** (clean-then-marginal-zone, not
  a sharp wall) — a genuine cross-platform finding, not a platform-specific quirk. Rough ceiling
  comparison (small sample, don't over-claim precision): upgraded clean to ~0.6, brushless clean
  to ~0.5 — close enough that a real difference isn't established.
- Yaml switched back to upgraded (`kr: 603.0` confirmed active); firmware reflash
  (`make DRONE=upgrade cload`) and CS2 server restart still need to be run on hardware to complete
  the switch-back.

**No more flying planned for the rest of the project** — Tasks 1-3/2b are the full data-collection
scope. Everything from here is slides/video/writing (Tasks 4-9).

## 4. Write presentation narrative arc

Full outline below, updated 2026-07-25 with everything from `results_2026-07-15_brushless.md`
§8-10. Every number/claim below has a source section — cite the doc, don't re-derive anything.
Suggested slide count per beat is a ceiling, not a target — cut before you pad.

### 4.1 Standard CF2.1 + INDI (1 slide)
Baseline established and working: 3.87cm XY RMSE, kr=1050 (`results_2026-06-20.md`). One line,
move on — this is context, not a result to dwell on.

### 4.2 Brushless integration + the shake investigation (4-5 slides — this is the methodology
centerpiece, worth the most careful treatment even without a fix)
Frame explicitly as a diagnosis story, not a stalled feature:
- **Symptom** (1 slide): ~7Hz self-sustained oscillation, delayed onset (calm ~5s, then jumps to
  ~37% of PWM range and sustains), reproducible in both hover and maneuvering flight. Use the
  brushless hover footage from today here — visible/audible wobble is a strong opener.
- **Root cause, quantified** (1-2 slides): kr=2400 tunes the attitude loop's natural frequency to
  ωₙ≈7.8Hz — matching the measured shake frequency (7.22Hz) almost exactly (§8's resonance
  finding). Actuator lag (~32ms, confirmed via 500Hz SD-card logging, in-flight, real 4-motor
  differential load — not a bench/single-motor artifact) contributes 82° of phase loss at that
  frequency, vs EKF attitude lag's 20° — actuator lag is the dominant, previously-unquantified
  term. Use the `usd_hover_2026-07-23_shake_diagnostic.png` and/or `usd_oval_..._diagnostic.png`
  6-panel figures directly — they were built for exactly this slide.
- **Dead ends, ruled out with evidence** (1 slide): filter tuning (fc_bw=70 measured irrelevant at
  7Hz — a 10× higher cutoff can't attenuate this frequency by construction) and detuning kr (proven
  futile two ways: kr≈600 still shaky *and* the vibration is broadband — §4b-spectrum — so there's
  no quiet frequency to retune into; also directly degrades tracking, a lose-lose lever).
- **Real fix candidates, correctly not attempted** (1 slide): actuator-lag reduction (faster
  ESC/motor/prop response, or a delay-compensating term in the INDI law) or vibration damping
  (prop balancing, mount isolation) — both are hardware-iteration work, explicitly out of scope
  for this project's remaining time. State this plainly; it's a scope decision, not a gap.

### 4.3 Switch to the thrust-upgraded drone (1 slide)
Why: a *different*, already-solved root cause (wrong compiled thrust model, fixed via
`CONFIG_CRAZYFLIE_THRUST_UPGRADE_KIT=y`) — not a retreat from the brushless problem, a deliberate
move to keep making progress while that one waits on hardware. Already flying well before this
session (3.7cm mean XY RMSE, n=11, kr=603).

### 4.4 Trajectory library + kt sweeps, both platforms (3-4 slides — this is where today's new
data lives)
- **Oval kt sweep** (§9.6, §10.1): both platforms clean at low kt, crash walls found — upgraded
  ~kt 0.3-0.4, brushless ~kt 0.5-0.7. Use the oval clean→crash footage from today (Task 2) here;
  it's a strong visual pairing with the numbers.
- **Circle kt sweep** (§10.2, new this session): **both platforms show the same
  clean-then-marginal-zone pattern**, not a sharp wall — upgraded clean to ~0.6 then 4/9 crash
  0.7-1.0; brushless clean to ~0.5 then 3/5 crash 0.6-0.9. Frame this as a genuine cross-platform
  finding (a property of aggressive circular tracking generally, not a platform quirk) — good
  "second, independent confirmation" material after the oval story.
- **Other trajectories at kt=0.05** (§9.1, §9.5): upgraded crashes on `corner`/`slalom` where
  brushless doesn't — a bandwidth-ceiling story (kr=603's lower ωₙ trades the resonant shake for
  less control authority on sharp direction changes), separate from the kt-sweep envelope finding.

### 4.5 Headline comparison result (1-2 slides — the climax; give this the most polish)
**Position/attitude tracking is essentially tied at low aggressiveness, with a kt-dependent
nuance**: circle kt=0.05 ties (1.7cm vs 1.9cm), but at kt=0.3 brushless is measurably tighter
(1.4cm vs 2.2cm, ~36% lower) — don't oversimplify to a blanket "tracking is identical," the honest
finding is *"tied at gentle speeds, brushless pulls ahead as aggressiveness increases, right up
until each platform's own crash wall."* Cite the `--compare` plots directly (§9.7, §10.2) — they
visually show both platforms tracing near-identical paths, so the number and the picture agree.
**The platforms differ in envelope width, not in tracking quality when both are within envelope.**

### 4.6 Two honest complications, briefly (1 slide, don't dwell)
- `helix`: the one trajectory where brushless has an uncontested precision edge (52.8cm vs
  71.1cm RMSE, both real — both platforms show a genuine shape undershoot on this climbing
  spiral, brushless less so).
- `tilted_oval`: a genuine upgraded-specific anomaly (11.9cm vs brushless's 1.3cm shape RMSE),
  confirmed real via a lap-synchronized error pattern, not a script bug (§9.4/§10.5).
These are good "we found more than we were looking for, and we're not hiding the parts that don't
fit a clean story" material — a strength, not a weakness, if framed as such.

### 4.7 Scope, limitations, and what was correctly not chased (1-2 slides — see Task 6 for the
full list)
Phase 3 (third drone), Phase 4 (aggressive maneuvers — blocked by the unresolved shake), the
no-guards/J-mismatch aside (§10.3 — identified, correctly not pursued with one day left).

---

**Total: ~13-17 slides for content** (before title/agenda/conclusion). Keep each beat to its stated
ceiling — the biggest risk with this much verified material is over-including everything just
because it's all true and documented. The comparison result (4.5) is the strongest single finding;
everything else should be paced to build toward it, not compete with it.

## 5. Build slides from existing plots

**Done, 2026-07-25, restructured twice same day per operator feedback.** `FINAL_PRESENTATION_BEAMER.tex`
at repo root — compiles cleanly to a **46-slide** PDF.

**Second restructure round (same day) added, per explicit operator direction:**
- Core section: replaced the plain RMSE table with the actual **5-way bar-chart plots**
  (`5way_per_flight.png`, `5way_mean_comparison.png`) — same visual style as the
  geometric-vs-INDI baseline slide, more informative than a table.
- Upgraded-vs-brushless section: added **full 6-panel multi-platform dashboards** (analysis,
  axes, kinematics, 3D-orientation-overlay, INDI-torque-panel, RPM-balance) for **oval and
  circle**, same treatment as figure-8's 4-way comparison — generated by discovering that the
  "figure8 multi-platform" plotting infrastructure (`_load_figure8_dashboard_entry`,
  `plot_*_multi`) was already fully trajectory-generic under the hood (built on the same
  calibration helpers fixed earlier this session), just named for its original use case.
  Genericized the hardcoded "(figure8, kt=0.05)" title text into a `traj_desc` parameter (10
  call sites in `analyze_flight.py`, 2 needed an `f`-string prefix fix to actually interpolate).
- **Data quality catch during this work**: the brushless kt=0.05 flights from 2026-07-22
  (`oval`/`circle`) have completely frozen `tau_x/y/z` telemetry (constant across the entire
  flight — a real logging fault from that early session, confirmed by checking the raw CSV, not
  a plotting bug). Switched the oval/circle 6-panel dashboards to **kt=0.1** instead (both
  platforms flown same-day, 2026-07-25, valid dynamic tau on both sides) rather than use broken
  data. The kt=0.05 numeric RMSE citations elsewhere (from position telemetry, unaffected) stay
  as-is — only the torque-panel visual needed the kt=0.1 substitution.
- New standalone plot: alpha-raw-vs-filtered and commanded-torque, **upgraded vs brushless
  hover, side by side** (`flying_drone_stack/docs/upgraded_vs_brushless_alp_tau_2026-07-25.png`)
  — extends the "filter is irrelevant" finding to both platforms directly, and makes the torque
  amplitude contrast (upgraded ~0.0003-0.0007 N·m vs brushless ~0.002-0.005 N·m, ~7-10×) visible
  without needing the reader to parse PSD plots.
- `helix` dropped entirely per operator instruction (not considered for detailed comparison);
  `tilted_oval` kept as a standalone complication slide (its own comparison plot, not paired with
  helix anymore).
- Shake investigation section moved to **Limitations & Complications**, near the end, reframed
  as explaining *why* brushless underperforms upgraded on some maneuvers despite winning
  figure-8 outright — an explicit, intentional "counterintuitive combination" framing rather than
  presenting it as a standalone early topic.
- Teardrop/inverted-loop: **left as an open placeholder**, one more attempt planned for tomorrow
  if time allows — not presented as a finished finding.

**Third round (2026-07-25 evening, plot-correctness pass, not a restructure) — a supervisor-facing
double-check of the plots surfaced several real bugs, not just cosmetic issues, all fixed in
`Controls/analyze_flight.py`:**
- **Z RMSE was inflated 15-17cm instead of the true ~1-2cm**, in both the multi-platform loader
  (`_load_figure8_dashboard_entry`) and, separately, the single-flight `plot_analysis()` used by
  the slalom slide — both compared actual Z against the wrong baseline (full-log mean including
  ground/takeoff/landing samples, instead of the trajectory-start-relative reference). Fixed in
  both places; the slalom slide's "Position Error" panel previously showed a nonsensical flat
  ~70-80cm error line next to a "RMSE 3D=4.0cm" box — now consistent.
- **Position vs Time panel** (multi-platform dashboards) was missing the planned x/y lines
  entirely and had no z trace at all — fixed, now shows planned x/y/z (black) plus actual z per
  platform. Same missing-planned-z bug existed in the per-axis dashboard's Z subplot — fixed.
- **Yaw error was never computed anywhere** in the multi-platform path despite roll/pitch both
  existing — added throughout (metrics tables, both kt-sweep LaTeX tables).
- **kt-sweep tables** (oval, circle) restructured twice more per operator feedback: platform-
  paired columns (both platforms' XY/Z RMSE together, then both platforms' Roll/Pitch/Yaw
  together, then both platforms' Max speed together) instead of grouped-by-platform; Z RMSE and
  Max speed columns added, computed directly from the cited flight logs.
- **"Mean thrust [PWM]" row removed** from the metrics tables — always printed 0/-0, not useful.
- **Metrics table headers were overlapping/truncated** at narrow column widths — fixed (taller
  header row + multi-line label instead of flattened single-line).
- **5-way bar chart script** (`flying_drone_stack/scripts/plot_5way_comparison.py`) had its
  `savefig` paths pointing at a stale scratchpad directory from a previous session — fixed to
  write to `flying_drone_stack/docs/` directly. Also added step-over-step % annotations (vs the
  immediately-previous platform, not just vs the Standard Geometric baseline) per operator request.
- Slalom section: removed the "Upgraded Crash, Visualized" slide (upgraded has zero clean
  slalom flights, not a fair comparison), added a "3D Trajectory, Flown vs Planned" slide for
  brushless's clean flight instead.
- Added a caveat note to all three "Alpha Raw/Filtered and Commanded Torque" slides
  (roll/pitch/yaw): confirmed via file-system trace that these use the **~100Hz CS2/radio hover
  logs**, not the 500Hz SD-card capture used for the two shake-diagnostic slides earlier in the
  same section (no SD-logged hover exists for the upgraded drone at all, and the brushless SD
  raw binary was already consumed/decoded and not saved) — noted on-slide rather than re-flown,
  time-boxed out.
- All fixes verified applied consistently to **both** the static PNG and interactive Plotly HTML
  where both exist (figure-8 four-way, oval kt=0.4, circle kt=0.7 — they share one computation,
  so no separate calc path to fall out of sync). Plots that are static-only by design (slalom
  dashboard, both SD shake diagnostics, all three alpha/tau plots, the 5-way bar charts) were not
  expected to have an interactive counterpart — not a regression.
- **Side investigation, no code change**: checked whether the project's safety/output clamps are
  "loose or conservative" per operator question. Finding: the firmware's INDI output clamps
  (`tau_xy_max`, `tau_z_max`, `tilt_max_deg`, `thrust_max`) are **disabled entirely**
  (`g_indi_clamp_en = 0` default, and the yaml block that would enable them is commented out) —
  every flight this project, including every high-kt crash, ran with zero clamp-imposed ceiling.
  Worth stating in Q&A if asked: the crashes aren't a symptom of an overly-conservative clamp,
  there's no active clamp to loosen — the real ceiling is control bandwidth (kr/kw), not output
  clamping.

Deck is still **46 pages** after this round (no slide count change, only content/plot fixes).

Full structure now: Recap (6 slides) → Core figure-8 four-way (9) → Upgraded vs.\ brushless,
oval+circle full 6-panel treatment each + kt sweeps + corner/slalom + nuance + tilted_oval (19)
→ Limitations (8) → Summary (2), plus title/agenda (2) = 46 total.

Remaining slide work is refinement only (wording/spacing polish, deciding exact video cut-in
points) — not building from scratch. If tomorrow's teardrop/inverted-loop attempt happens, that
placeholder slide needs a real result swapped in before the talk; **if it doesn't happen, reword
the placeholder** (currently says "attempt planned for tomorrow" — don't leave a promise for a
specific experiment that never occurred).

**Fourth round (2026-07-26, content-accuracy pass, operator sign-off) — deck cut from 46 to 39
slides.** Removed slides that were low-value or redundant now that the underlying HTML files are
just opened directly during the talk instead of referenced from a slide: the figure-8 per-flight
bar chart (slide 10's mean-comparison chart already covers it), both "Interactive Versions
Available" filename-listing slides, the "Upgraded vs. Brushless" section-intro text slide, and all
three per-axis alpha/tau roll/pitch/yaw comparison slides (their one caveat-worthy point — ~100Hz
CS2 logs, not 500Hz SD — no longer earns three slides once removed). Content fixes:
- **Slalom slide**: removed an incorrect causal claim ("kr=603's lower bandwidth trades away
  the shake but can't correct fast enough for sharp turns") — checked against the data and it
  doesn't hold: upgraded (kr=603) never shows the sustained resonant limit cycle at all (clean or
  crash, never shaky), and brushless itself still shakes at lower kr too. Replaced with an honest
  "genuinely open, not yet explained" statement instead of a plausible-sounding but wrong
  mechanism.
- **Shaky Shaky slide**: removed the "onset is immediate not delayed" bullet (operator call —
  correct but not worth the slide space once the mechanism is covered on the Root Cause slide).
- **Root Cause slide**: table itself unchanged, added a plain-language explanation under each row
  (what it measures, why it matters, how to say it simply in the talk).
- **Real Fix Candidates slide**: expanded from 2 to 4 candidates and reframed around what the data
  actually supports. New primary candidate: a **narrow band-reject (notch) filter around 5-10 Hz**
  instead of a broad low-pass — the shake is a confirmed narrow 7.22 Hz peak, not broadband noise,
  so a plain low-pass cut low enough to touch it just reintroduces the same actuator-lag phase-loss
  problem measured elsewhere on the Root Cause slide; a notch leaves the rest of the spectrum (and
  its phase) alone. Kept the two original hardware candidates (actuator lag, source vibration), and
  added a second, explicitly speculative candidate: ESC/motor-protocol-level rate/current limiting
  (e.g. DShot slew-rate limiting) — a guess, not backed by any data collected this project.
- **Summary slide**: fully rewritten to state overall project achievements as a narrative — motion
  planning pipeline → tuned geometric baseline on the standard drone → full INDI implemented and
  deployed on three platforms → fair four-mode comparison (INDI beats geometric everywhere,
  brushless beats standard/upgraded on raw tracking) → the flight library across trajectories and
  platforms as the project's concrete artifacts → one closing bullet for limitations (shaky shaky,
  no teardrop/inverted loop attempted).

**Fifth round (2026-07-26, appendix pass) — deck 39 → 41 slides, two appendix slides added after
Summary, both checked against the actual codebase before writing:**
- **"Appendix — Motion Planning, In Brief"**: 5-point plain-language summary of the planning
  pipeline (waypoints → minimum-snap polynomial segments → two segment-timing modes, manual vs.
  the $k_t$-automatic Richter mode used throughout this talk → differential flatness mapping the
  flat outputs to the full attitude/rate/thrust reference → the final feedforward reference
  exported and evaluated onboard at 500-1000\,Hz), sourced from `src/planning/mod.rs` and
  `src/planning/flatness.rs`.
- **"Appendix — Full INDI, Pseudocode"**: simplified pseudocode traced directly from
  `firmware_app/src/lib.rs`'s `controller_step()` — both the position-loop increment
  (`a_indi = a_meas - a_model`) and the attitude-loop increment
  (`delta_tau = J*(alpha_ref - alpha_meas)`, added on top of `tau_current` read from motor RPM,
  not a model) are called out explicitly as the two "this is the INDI part" lines, closed with a
  plain-language explanation of the core idea (measure the current state instead of trusting a
  model, correct only the gap). Filtering/clamping details correctly omitted per the operator's
  brief ("not necessarily needing to cover each fine detail").

**Deck is now considered content-final** — operator confirmed "slides are now as I want them" after
round four, and approved the two appendix slides in round five. Any further slide work is
contingent on whether the bonus flights in §10 below produce something worth adding, not scheduled
independently.

Original notes below, kept for reference on which assets map to which slides:

Reuse these directly, no regeneration needed:

- `flying_drone_stack/docs/usd_hover_2026-07-23_shake_diagnostic.png` — the 6-panel shake root
  cause figure (filter aliasing, EKF lag, actuator lag, RPM rate).
- `flying_drone_stack/docs/usd_oval_2026-07-23_shake_diagnostic.png` — same, under maneuvering
  load (confirms the mechanism isn't hover-specific).
- `Controls/logs/circle_mode1_kt0.05_2026-07-23_19-12-43_vs_circle_mode1_kt0.05_2026-07-22_18-26-28.png`
  and the three oval-kt `_vs_` comparison plots (§9.7) — the platform-tied-tracking evidence.
- `Controls/logs/helix_..._vs_..._.png` and `tilted_oval_..._vs_..._.png` — if including the two
  open anomalies as "interesting complications" rather than going deep on them.
- Existing project-plan beamer decks at repo root (`PROGRESS_PRESENTATION_BEAMER.tex`,
  `PROGRESS_PRESENTATION_2_BEAMER.tex`) — reuse their template/style rather than starting a new
  deck from scratch, keeps visual consistency with earlier progress presentations.
- One slide per major finding; put full data tables in backup/appendix slides, not the main flow.

## 6. State explicit scope cut

- One slide, plain and upfront: Phase 3 (third drone) and Phase 4 (accel-pinned aggressive
  maneuvers, slide 20 of the original project plan) were not attempted — reason: Phase 4
  specifically requires the brushless platform's higher thrust/omega, which was blocked by the
  unresolved shake through the remaining project timeline.
- Also worth one line each on `helix` and `tilted_oval` tracking anomalies (§9.5) — real findings,
  flagged, not pursued further given time. Frame as "identified, not chased" rather than omitting.
- This slide protects you in Q&A — better to preempt "did you do X" than be asked cold.

## 7. Record narrated video walkthrough

- Script the narration from the Task 4 outline before recording — don't improvise live over slides.
- Structure: slides + voiceover, cut to flight footage (Tasks 1-3) at the relevant narrative beat
  (e.g. cut to the brushless hover clip right when describing the shake symptom).
- **Fly with the LED ring on** for any footage not already shot that way — better visualization in
  the final cut. Check existing captured clips first; only re-shoot the ones that need it.
- Check the assignment's required video length before scripting — trim the narrative arc to fit
  rather than discovering the overage during editing.
- Export and watch it back once fully assembled before calling it done.

## 8. Rehearse / time the presentation

- Full run-through with a timer, out loud, not silently reading slides.
- If over time: cut from the appendix-worthy material first (kt-sweep gap discussion, anomaly
  deep-dives), never from the headline comparison result.
- If possible, present to one other person first — the shake investigation's causal chain (kr →
  resonance → actuator lag → limit cycle) is dense and worth a comprehension check before the
  real presentation.

## 9. Buffer day

- Reserve explicitly — no new content creation scheduled this day.
- Use for: re-shooting any footage that didn't come out usable, fixing slide typos/layout, video
  export re-runs, submitting deliverables with margin before the deadline rather than at the wire.

## 10. Bonus, only if time allows (not required — deck is already complete without these)

Three practical follow-ups that came up once the slides were locked in, all genuinely optional.
None of them block finishing the project; only pursue if Tasks 7-9 leave spare time.

1. **Slalom kt sweep at a lower kt range, both platforms.** Current data only has kt=0.05 (upgraded
   crashes 3/4, brushless clean) — no sweep exists. Hypothesis to test: at a low-enough kt, brushless
   should track sharp corners \emph{better} than upgraded (matching the oval/circle kt-sweep story),
   giving slalom a real cross-platform comparison plot instead of the current single-platform-only
   slide. Use the same `--kt` sweep methodology as oval/circle (§2, §2b) but start below 0.05 and
   step up until upgraded's first clean flight is found.
2. **A few more teardrop/inverted-loop attempts, brushless only.** Ties to the still-open
   `teardrop_wide` crash (kt=0.05, crashed under plain geometric on both platforms, never
   root-caused) and to Phase 4 (accel-pinned aerobatic maneuvers) being otherwise fully descoped.
   Even one clean brushless attempt would upgrade the "Teardrop / Inverted Loop" limitations slide
   from a placeholder to a real (partial) result.
   **UPDATE 2026-07-27: done, far exceeded scope — see
   `flying_drone_stack/docs/investigation_inverted_loop_2026-07-27.md`.** `loop` now achieves a
   clean, repeatable inversion (3/3 flights, roll 163-178° at the apex); post-flip recovery is
   the one remaining open piece (torque saturation/chatter, actively being instrumented and
   retried — not blocking, already a real partial result for the slide).
3. **Prototype the notch-filter fix candidate** from the "Real Fix Candidates" slide — a band-reject
   filter around 5-10\,Hz instead of the current broad low-pass, targeting the confirmed 7.22\,Hz
   shake peak specifically. This is genuinely new investigation, not slide polish — treat it as a
   real (if small) experiment: implement, fly, and check whether it actually reduces the shake
   without reintroducing the phase-lag problem the current filter has. If it works, it upgrades
   the Real Fix Candidates slide from "diagnosed but not fixed" to a partial fix — a strong note to
   end the limitations section on. If it doesn't fully work in the time available, the current
   slide already honestly frames it as future work, so nothing is lost by trying.

## 11. Everything remaining — full list

| # | Item | Type | Status |
|---|---|---|---|
| 7 | Record narrated video walkthrough, LED ring on for footage not yet shot that way | Required | ⏳ not started |
| 8 | Rehearse / time the full presentation | Required | ⏳ not started |
| 9 | Buffer day (re-shoots, slide-typo fixes, video re-export, submit with margin) | Required | ⏳ reserved, not started |
| 10a | Slalom kt sweep at a lower kt range, both platforms — test whether brushless out-tracks upgraded at sharp corners like it does on oval/circle | Bonus, optional | ⏳ not started |
| 10b | A few more teardrop/inverted-loop attempts, brushless only — upgrade the placeholder limitations slide if one lands clean | Bonus, optional | ✅ far exceeded — clean repeatable inversion achieved, recovery still open, see `docs/investigation_inverted_loop_2026-07-27.md` |
| 10c | Prototype the 5-10\,Hz notch-filter fix candidate — implement, fly, check whether it actually reduces the shake | Bonus, optional | ⏳ not started |

Everything else (narrative, slide deck, scope-cut framing, motion-planning + INDI appendix slides)
is done and signed off — this table is the complete remaining punch list.
