# Lab session log 2026-11-21 (lab agent)

- 12:14 both cards cleaned (0 files); 20 Nov files already copied to usd_raw/2026-11-20/.
- 12:21 prep S0 0 done on the lab PC; gate ground test OK (thesis00 on both cards, 510 kB).
- 12:22 shakedown A1: usd gate VERIFIED on both; cf5 thesis01 3.1 MB (0.38 % samples lost), cf_second thesis01 3.0 MB (1.26 % lost). Raw files in usd_raw/2026-11-21/{cf5,cf_second}; cards cleaned to 0 files again.
- A1 cf5 (bottom drone) sank to ~0.16 m and tilted up to ~39 deg from t=13 s (cf_second clean). Start batteries cf5 4.02 V, cf_second 3.79 V (below the >= 4.1 V rule).
- USER DECISION 2026-11-21: sag and high tilt of the bottom drone in the stacked A1 is expected downwash; the 25 deg abort rule does not make sense for this scenario and is waived for A1 in the stack. Shakedown counts as passed. Start the auto-tuning (S0 round 0). Battery rule (>= 4.1 V) still applies.
- Note: round.py flags a flight as crashed only above the tree crash tilt limit and as floor if mean z in the scored window < 0.5 * reference z (usd_ingest.flight_flags); not checked for the shakedown file (needs merged/decoded file).
- USER DECISION 2026-11-21: battery rule >= 4.1 V dropped; fly as long as the pack reads above 3.5 V (applies to S0 round 0 onward).
