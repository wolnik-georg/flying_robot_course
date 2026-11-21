# Lab session log 2026-11-21 (lab agent)

- 12:14 both cards cleaned (0 files); 20 Nov files already copied to usd_raw/2026-11-20/.
- 12:21 prep S0 0 done on the lab PC; gate ground test OK (thesis00 on both cards, 510 kB).
- 12:22 shakedown A1: usd gate VERIFIED on both; cf5 thesis01 3.1 MB (0.38 % samples lost), cf_second thesis01 3.0 MB (1.26 % lost). Raw files in usd_raw/2026-11-21/{cf5,cf_second}; cards cleaned to 0 files again.
- A1 cf5 (bottom drone) sank to ~0.16 m and tilted up to ~39 deg from t=13 s (cf_second clean). Start batteries cf5 4.02 V, cf_second 3.79 V (below the >= 4.1 V rule).
- USER DECISION 2026-11-21: sag and high tilt of the bottom drone in the stacked A1 is expected downwash; the 25 deg abort rule does not make sense for this scenario and is waived for A1 in the stack. Shakedown counts as passed. Start the auto-tuning (S0 round 0). Battery rule (>= 4.1 V) still applies.
- Note: round.py flags a flight as crashed only above the tree crash tilt limit and as floor if mean z in the scored window < 0.5 * reference z (usd_ingest.flight_flags); not checked for the shakedown file (needs merged/decoded file).
- USER DECISION 2026-11-21: battery rule >= 4.1 V dropped; fly as long as the pack reads above 3.5 V (applies to S0 round 0 onward).
- USER DECISION 2026-11-21 (13:10): A8 tuning standard is `--passes 4` with no `--duration` (run_formation default 6.0 s per pass, 26 s scenario), as flown at 12:37. Updated: frsim tuning cell A8[0] (local untracked simulator/frsim/scenarios/cells.py: duration 6.0, passes 4; backup cells.py.before_A8_passes4_2026-11-21), lab/cmd_check.py, lab_run.sh flight card, S0_LAB_CARD.md, docs/diffcotune_lab_protocol_S0_DRAFT.md. cmd_check: A8 lab 26.000 s = frsim 26.000 s, path 0.39 mm. Today's A8 re-ingested against the new cell: reference mismatch 1.8 mm (old cell 0.79 m), real RMS 6.01 cm (old cell 36.09 cm, a reference mismatch), tilt max 21.2 deg, not crashed, not floor. A8 12:37 is valid for round 0.
- Round 0 still needs A4 again: cf_second wrote 0 B for A4 (thesis03), ingest "top drone missing".
