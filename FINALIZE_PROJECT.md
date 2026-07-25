# Finalize Project — final week plan (2026-07-24 → presentation)

Semester wrap-up. Investigation mode is closed out (see `flying_drone_stack/docs/results_2026-07-15_brushless.md`
§8-9 for the full brushless shake diagnosis and upgraded-drone comparison). Everything below is
about turning what's already been found into a presentation, not generating new results.

**Status (2026-07-25): in the lab now, focused on Tasks 1-3 (flight footage capture).** Confirmed
before starting: `crazyflies.yaml` is still on the upgraded block (`kr: 603.0` active, brushless
block parked/commented) — Task 1/2 need no config change, Task 3 is the only one requiring a
firmware/yaml switch (and a switch back afterward).

## Task table

| # | Task | Why it matters | Priority |
|---|---|---|---|
| 1 | Capture flight footage: hover/circle/figure8/oval on upgraded drone | Presentation needs visual proof; drone is currently reliable — do this before anything breaks again | **Critical** |
| 2 | Capture one oval kt-sweep flight live (e.g. kt=0.3 clean → kt=0.5 crash) | Strong "pushing the envelope" visual for slides/video | **Critical** |
| 3 | Capture one brushless hover clip showing the shake | Show-don't-tell for the shake investigation section | High |
| 4 | Write presentation narrative arc (standard → brushless shake investigation → upgraded switch → comparison result) | Turns existing docs into a story; this is the actual deliverable | **Critical** |
| 5 | Build slides using existing plots (§8 shake diagnostics, §9.5-9.7 comparison tables/plots) | Assets already exist — reuse, don't regenerate | **Critical** |
| 6 | State explicit scope cut: Phase 3 (third drone) and Phase 4 (aggressive maneuvers) descoped for time | Honesty about scope reads better than silently dropping it | High |
| 7 | Record narrated video walkthrough | Deliverable requirement | High |
| 8 | Rehearse / time the presentation | Avoid running over on presentation day | High |
| 9 | Buffer day for re-recording, slide fixes, last-minute drone issues | Nothing in this project has gone smoothly on the first try | High |

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

## 3. Capture brushless hover clip showing the shake

- This requires reflashing back to brushless firmware and re-enabling the brushless yaml block
  (currently commented out/parked):
  ```
  cd flying_drone_stack/firmware_app && make cload      # DRONE=bl is the default
  ```
  In `crazyswarm2/crazyflie/config/crazyflies.yaml`: comment out the active **CF2.1 UPGRADED
  MOTORS** block, uncomment the **CF21BL BRUSHLESS — PARKED** block (labeled and preserved intact
  from the 2026-07-23 switch). Restart the CS2 server after editing so it re-reads the yaml.
- Fly a plain hover (`--trajectory hover --duration 15`) — the shake is fully present at hover,
  no maneuver needed (confirmed in §8, kr=2400/kw=170 is the locked brushless config already
  active in that block).
- Film close enough to see the visible high-frequency wobble; audio again helps (the ~7Hz
  oscillation is audible in prop noise).
- **After filming, switch back to the upgraded config** for the rest of the week (reverse the yaml
  edit, reflash `make DRONE=upgrade cload`) so tasks 1-2's platform stays the one you're set up on.
- This is the only task that touches the brushless drone again — do not use this as an opening to
  resume the shake investigation; one demo clip is the full scope here.

## 4. Write presentation narrative arc

Draft a linear outline before touching slides. Suggested structure, pulling directly from existing
docs (no new analysis needed):

1. **Standard CF2.1 + INDI** — baseline established, working (`results_2026-06-20.md` reference:
   3.87cm XY RMSE, kr=1050).
2. **Brushless integration** — why (Phase 2 of the project plan), what INDI on brushless required.
3. **The shake investigation** — tell it as a methodical diagnosis, not a failure:
   - Symptom: ~7Hz self-sustained oscillation, delayed onset, ~37% of PWM range.
   - Root cause chain: kr=2400 tunes loop resonance to ~7.8Hz (matches measured 7.22Hz almost
     exactly) + actuator lag (~32ms, confirmed via 500Hz SD logging, in-flight, real 4-motor load)
     contributing 82° of phase loss at that frequency — the dominant term, more than EKF lag (20°).
   - Dead ends ruled out with evidence, not assumption: filter tuning (fc_bw=70 proven irrelevant
     at 7Hz), detuning kr (proven futile both empirically and structurally — broadband vibration,
     no quiet frequency to retune into).
   - Real fix candidates identified but require hardware iteration beyond this project's remaining
     time: actuator-lag reduction (ESC/motor/prop) or vibration damping (balancing, mounts).
4. **Switch to the upgraded (thrust-kit) drone** — why: different, already-solved root cause
   (compiled thrust model), unblocks forward progress.
5. **Trajectory library + oval kt sweep on the upgraded drone** — envelope found: clean to kt~0.3,
   crash wall at kt~0.5; crashes on corner/slalom at kt=0.05 too (bandwidth ceiling, not resonance).
6. **Headline comparison result**: position/attitude tracking is statistically tied between
   platforms (1.7-3.2cm XY RMSE both, across circle + full oval sweep) — the platforms differ in
   *envelope*, not *precision*. Cite the fixed-and-verified `analyze_flight.py` comparison plots
   (§9.7) directly.
7. **Scope and limitations** (see Task 6).

Keep each section to 2-4 slides max — the temptation with this much material is to over-include;
resist it. The comparison result (step 6) is the strongest single finding — build the talk's climax
around it.

## 5. Build slides from existing plots

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
