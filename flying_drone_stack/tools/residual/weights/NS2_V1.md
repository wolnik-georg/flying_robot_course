# NS2 weights v1 (the network flown in all lab sessions up to 2026-10-09, frozen for the comparative study)
- File: `ns2_v1.npz` = copy of `experiments/analysis/out/c2_e2e_2026-10-01/full_bank_c1_complete.npz`
- sha256: `8a43b5b54d0e2023f475a4314f64fe81120a38413d9f6cb6817b2bab6afc7b43` (also in `ns2_v1.sha256`)
- Trained on 26 geometric two-drone flights (A1–A7, no A8), R² 0.944; flash-resident in the `…rpmf…` firmware images (built with `RNN_WEIGHTS_NPZ` = this file).
- Decision 2026-10-13: no v2 (the unused flights add nothing, new data would only give small gains); v1 stays. Known limits: dz 0.75 m and A7 thinly covered, A8 crossings not in the bank (`docs/study_readiness_matrix.md` §2, `docs/ns2_retrain_options.md`).
