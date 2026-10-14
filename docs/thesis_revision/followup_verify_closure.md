# Follow-up 1 — Closing the [VERIFY] items

Sources used: the flight code and its configuration (`firmware_app/src/lib.rs`, `traj_iface.c`, the local
`crazyflie-firmware` tree the firmware app builds against (`firmware_app/Makefile:16`), `crazyswarm2` configuration and
simulator backend), the merge tool `flying_drone_stack/tools/merge_usd_logs.py`, and the 18 held PDFs. Line numbers
refer to `revised/` as of this pass. Five of the nine items are closed (V7 only for its architecture half), and four
remain open. One new decision, **D-11**, came out of V3.

| Item | Place (now) | Status | Evidence | Action |
|---|---|---|---|---|
| V1 | Ch2 L105; Ch4 L274 | **OPEN** | Lee et al. CDC 2010 is not among the held PDFs (`docs/papers/`); the companion arXiv v4 (held) gives the regions as Propositions 3–4 | Author: obtain the CDC 2010 paper and check that it states the 90°/180° regions and how they are numbered. |
| V2 | Ch2 L656 (and Ch1 §1.2 ¶6) | **OPEN** | No Abro et al. PDF held | Author: check every clause of the Abro paragraph against the full text, or take the reduced wording of D-08. |
| V3 | Ch4 L211 | **CLOSED** | One file `crazyswarm2/crazyflie/config/crazyflies.yaml` plus one frozen parameter block per strategy in `crazyswarm2/crazyflie/config/study_strategies.json` (S0, S1, S2, …), checked against read-back values before take-off (its `_documentation` string) | Text changed: "from a single configuration file" → "from the configuration, as one frozen parameter set per strategy" (`% CHANGED (V3)`). **New finding:** the S1 block selects `stabilizer.controller: 9` (`controller_omar_indi.c`), not the `controller_step` INDI (ctrl_mode 3) that Ch4 describes → **D-11** (added to `author_decisions.md`; `% DECISION D-11` comments at Ch4 L198 and L292; notes added to every D-01/D-02 comment that states Strategy 1's configuration). |
| V4 | Ch5 L54 | **CLOSED** | `crazyflie-firmware/src/platform/interface/platform_defaults_cf21bl.h:46` `ARM_LENGTH 0.050f` (centre to motor); `lib.rs:261` `ARM_M = √2/2 × 0.050`. Diagonal motor-to-motor distance 0.10 m. (The CF 2.0 "9 cm rotor-to-rotor" of Shi et al. is the same measure: arm 0.046 m → 0.092 m.) | Text changed to "nominally 41 g, with 10 cm between diagonally opposite motors" (`% CHANGED (V4)`). The 41 g is the configured study mass (`crazyflies.yaml`, `mass: 0.041`); the firmware default 39.3 g is for a different deck set (header L47–52), so the word "nominally" stays. |
| V5 | Ch5 L245 | **CLOSED (moot)** | The radio packet numbers were removed under H1 (`% REMOVED (P5-07)`); the marker applies only if they are restored | No action unless the author restores the numbers; in that case cite the vendor radio specification (proposed reference A31). |
| V6 | Ch5 L285 | **CLOSED** | `merge_usd_logs.py`: docstring L1–32; `--meta/--roles` aligns each vehicle against its own commanded trajectory, described in the help text as "strongly preferred" (L339–342, L393–445); radio cross-correlation with offset and drift (L80–170) is used as a cross-check | First sentence of the alignment paragraph rewritten to the commanded-trajectory method, with radio cross-correlation as a cross-check (`% CHANGED (V6)`). The last sentence of that paragraph is D-05 and is unchanged. The tool's internal note on why cross-drone correlation is unreliable is a development finding and was not used (H1). |
| V7 | Ch5 L463 | **PARTLY CLOSED** | Architecture: `crazyswarm2/crazyflie_sim/crazyflie_sim/backend/neuralswarm.py` L24–56 (φ 6→25→40→40→20, ρ 20→40→40→40→1, H = 20) and gate L78 match the flight network → "in fact of the same architecture" is **confirmed**. Weights: `backend/data/neuralswarm2/{phi_S,phi_L,phi_G,rho_S,rho_L}.pth`, added to upstream Crazyswarm2 by W. Hönig, a Neural-Swarm2 author (commit `51a12dd`, 2024-01-10) | Open half, owned by the **supervisor**: confirm that these weight files are the networks of the Neural-Swarm2 paper. Text unchanged. |
| V8 | Ch5 L510 (Table 5.2 row) | **CLOSED** | `merge_usd_logs.py` L432–438: the merge refuses to proceed when a vehicle's position fit to its own commanded trajectory has RMS ≥ 15 cm; override `--tag-quality-only` (L350–353) | Table cell rewritten to "the merge refuses an alignment whose position fit to the vehicle's own commanded trajectory exceeds 15 cm RMS, unless overridden explicitly" (`% CHANGED (V8)`). The earlier wording "role check against motion capture" was inaccurate. |
| V9 | `proposed_references.md` A28 | **OPEN** | `lib.rs:230–231` names "Busetto et al. 2025 (arXiv:2512.14450), Table in Sec 4.1" as the source of the brushless inertia; no PDF held. The revised chapters give no inertia values, so no thesis sentence depends on it | Author: obtain arXiv:2512.14450 and check the title, authors and inertia table, but only if the thesis is to state inertia values. |

## Re-check of other values in the touched sentences

- 1 kHz loop: `crazyflie-firmware/src/modules/interface/stabilizer_types.h:383` `RATE_MAIN_LOOP RATE_1000_HZ` ✓.
- 500 Hz logging: `merge_usd_logs.py:44` (`DEFAULT_RATE = 500.0`) agrees with `usd_study_config.txt` ✓.
- Alignment error budget "a few milliseconds" (Ch5 §5.4.3): the merge tool's docstring (L17–20) lists about 1 ms of
  propagation, up to 2 ms of task phase and 1–3 ms of drift over 60 s. This is design reasoning, not a measurement;
  consistent ✓.

## Build
Clean after these changes (60 pages, no undefined references or citations).
