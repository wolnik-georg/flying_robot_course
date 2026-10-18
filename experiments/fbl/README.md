# FBL track — own training pipeline for the learned feedback-linearization controller (Hsieh et al. 2026)

Goal: while the authors' training script is pending, build our own end-to-end pipeline (residual training → inference → closed loop in simulation) on our data, so that the authors' script can later replace only the training step. Status, plan and results: `docs/fbl/STATUS.md`. Background and gap analysis of the received bundle: `docs/73_FBL_Bundle_Analysis.md`.

```
experiments/fbl/
  common/               bundle.py (read-only access to the authors' bundle, vehicle override), vehicle.py (our constants + sources), data.py (pair dataset from the NS2 banks)
  stage0_definitions/   residual definitions, geometry, ROM against our label, coverage
  stage1_training/      supervised fit of the bundle's residual structure on our 73-flight bank (leave-one-flight-out)
  stage2_sim/           closed loop with the bundle's FBL controller in simulation
  out/                  generated results per stage (small json/md/png; checkpoints go to out/stage1/ with a model card)
```
Rules: the authors' bundle (`~/Desktop/learned_fbl_core`, override `FBL_BUNDLE`) is never edited or copied into the repo; scripts import it. Python: `~/.pyenv/versions/flying_robots/bin/python` (torch 2.14 CPU). Data: the merged NS2 banks listed in `experiments/results/ns2_pilot2_2026-10-17/training_manifest_pilot2.json` (regenerate with `merge_training_bank.py` if missing). Nothing here touches the controller tree, the NS2 models or the firmware.

Bundle integrity hash (sha256 over sorted relative path + file hash of all non-pyc files), recorded 2026-10-18: `49089e6c10c4401b…` (full value in `out/stage0/stage0_results.json`).

Run: `~/.pyenv/versions/flying_robots/bin/python experiments/fbl/stage0_definitions/s0_check_definitions.py`
