# Reproduce the numbers

The tuning pipeline in `experiments/tuning/` stays the **record** of how the gains were found (search methods, rankings, closing table). `frsim` is the same plant, the same firmware controller code and the same error definitions in a small, documented package; it is checked against the tuning simulator episode by episode (`tests/test_equivalence.py`).

## Rerun a tuned pick
```
python3 scripts/evaluate_gains.py --ctrl S0 --scen A8 --gains tuned      # about 2 min with 4 processes
python3 scripts/evaluate_gains.py --ctrl S1 --scen A1 --gains tuned
```
It computes the selection error (tuning cells x 9 robustness variants x noise seeds 0 - 2) and the test error (study cells x 5 variants x noise seeds 0 - 4) and prints the values recorded in `gains/tuned/<controller>.yaml` next to them. Checked on 10 Nov 2026:

| Pick | Selection error (cm), crashed episodes | Test error nominal / 10 / 40 / 60 ms delay (cm) |
|---|---|---|
| S0 A8 (Bayesian optimisation, seed 1) | 4.773675404295527, 1 - identical to the record | 2.105 / 2.219 / 2.657 / 3.163 |
| S1 A1 (Sobol sampling, seed 4) | 7.2176653680472285, 2 - identical to the record | 3.949 / 4.110 / 4.279 / 4.301 |

The recorded values are the evaluations of the tuning with the exact vehicle inertia (`experiments/tuning/out/closing_exactJ/`), which also feed the closing table.

## Rerun one flight
```
python3 scripts/run_scenario.py --ctrl S3 --scen A1 --gains tuned --variant md60
```
S3 on A1 with 40 / 60 ms delay is a crash-border cell: small differences (for example the rounding of the inertia) change which noise seeds crash, so its test error at md40 / md60 is not stable to compare (marked in `gains/tuned/S3.yaml`).

## Equivalence with the tuning simulator
`tests/reference/reference_episodes.json` holds 199 episodes (5 controllers x A1 / A4 / A8 x nominal and 10 ms delay x noise seeds 0 and 1 x default and tuned gains, plus S3 A1 at 40 / 60 ms delay and the FBL search check), produced once by `tests/reference/make_reference.py` from the old tuning code and the old firmware builds. `tests/test_equivalence.py` reproduces them with `frsim` and a freshly built firmware: stable episodes bit-identical in every RMS field, crashed episodes with the same crash time. Result on 10 Nov 2026: **199 / 199 identical**, maximum difference 0.

`tests/legacy/check_indi_fast_o_identical.py` checks the fast INDI plant of the old code against its standard version (kept for the record; needs the old code).

## Original tuning code
Needs the original tuning code (`experiments/tuning/`), which is not part of this branch; the frozen reference table and `test_equivalence.py` do not need it. (`make_reference.py` and `tests/legacy/` are the only parts that import it.)

## Not covered
* FBL tuned gains (the retuning is running); `frsim` has only the default FBL gains for now.
* The simulator is plant-only: nothing here was flown. Where the plant differs from the drone (see `MODEL.md`), results on the drone can differ; the tuned gains are the ones the plant selected.
