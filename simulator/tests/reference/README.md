# Frozen reference episodes (2026-11-10)

`reference_episodes.json`: 5 strategies x A1/A4/A8 x study cells x variants nominal/md10 x noise seeds 0/1 x gains default/tuned pick (+ S3 A1 md40/md60, + FBL search check), produced ONCE by `make_reference.py` from the existing tuning-simulator code of record and the OLD firmware builds (S0-S3: crazyflie-firmware build of 2026-10-25 17:02; FBL: the FBL firmware build build of 2026-11-04 18:27). Each episode stores inputs and x/y/z RMS, 3-D RMS, max tilt, stable flag, crash time.
`compare_runs.py` compares two runs of the generator (e.g. old vs fresh build). Sanity checks of the table are printed by `make_reference.py assemble`.

Needs the original tuning code (`experiments/tuning/`), which is not part of this branch; the frozen reference table and `test_equivalence.py` do not need it.
