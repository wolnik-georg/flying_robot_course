# 49 — 2026-09-26 C.2 E2E + flash-resident RNN — execution record & validator prompt

**Purpose:** Copy from **`--- VALIDATOR PROMPT START ---`** through **`--- VALIDATOR PROMPT END ---`** into Claude (or another agent) to **independently re-verify** the desk execution described below. This doc is both **audit instructions** and **claimed results** (numbers must be re-checked on disk; do not trust prose alone).

**Primary repo:** `/home/georg/Desktop/flying_robot_course`  
**Secondary (read-mostly):** `/home/georg/Desktop/crazyswarm2`, `/home/georg/Desktop/crazyflie-firmware` (SIL bindings)

**Lab context (26 Sep 2026):** Facility mocap attitude fault — **no flights**, desk-only work.

**Git (claimed, local — verify `git log` and that branch is not pushed unless operator says so):**

| Commit | Subject |
|--------|---------|
| `963bef8` | Add residual_nn_flash build for flash-resident RNN weights. |
| `7987999` | Complete C.2 desk extension: LOO, Stage C, SIL predict, docs. |
| `2285b8c` | docs: note flash RNN test in test_residual_nn header; refresh overview C.2 line. |

**Note:** Commits were made with `git -c core.hooksPath=/tmp/githooks_empty commit` to avoid a hook that appended `Co-authored-by: Cursor`. Re-check `git log -1 --format=%B` on each.

---

## VALIDATOR PROMPT START

You are **validating** a completed desk session on a Master's thesis repo. Your job is to **verify claims against the filesystem and re-run key commands**, not to re-implement from scratch unless a check fails.

### Scope of work that was executed

Three task groups:

**Task A — Finish Neural-Swarm2 (Strategy 2) residual-learning E2E**

1. Leave-one-out on the full **18-file** training bank (24 manifest paths; 6 C5 solo excluded; one file with 0 rows).
2. Fix and re-run **Stage C** (real merged CSV → loader → compiled firmware).
3. **Live CS2 SIL** predict-only run with **full-bank trained weights** (not 21-Sep LOO subset only).
4. Append dated section to `docs/40` (do not overwrite prior 26-Sep full-bank train section).

**Task B — Flash-resident weight storage (`residual_nn_flash`)**

1. Storage/delivery only in `residual_nn.rs` / `lib.rs` — **no** changes to `phi_forward` / `rho_forward` / layer sizes / weight count.
2. Build-time `.npz` → embedded const via `build.rs` + `export_weights_rs.py`.
3. Second Cargo feature; **`residual_nn` upload path untouched**.
4. `make DRONE=bl` with flash feature **links** (no RAM overflow); symbol in flash address space.
5. Host test: flash vs upload **bit-for-bit** parity.
6. Append dated section to `docs/45`.

**Task C — Documentation cleanup**

Updated: `docs/07`, `docs/25`, `docs/40`, `docs/45`, `docs/13`, `Thesis_Progress_Overview.html`, `next_flight_card.html`, `lab_sessions/2026-09-24_to_26.md`.

---

### Ground truth (validator must confirm, not re-derive blindly)

| Claim | How to verify |
|-------|----------------|
| **24** C.1 training-eligible manifest paths | Count `training_eligible: true` in `manifest_2026-09-23_c1.json` + `training_eligible_crosswalk.json` (21 Sep) |
| **18** files contribute rows to NS2 train | `experiments/analysis/out/c2_e2e_2026-09-26/stage_a_full_bank.json` → `n_files_with_rows: 18`, `n_samples_total: 181489` |
| **6× C5 solo** excluded | Loader skips no-neighbour solo flights (structural) |
| **`A1_2026-09-21_13-25-10`** → **0** training rows | `a_res` zero throughout — same as 21-Sep Stage A |
| **A2** two flights kept despite `top_align_rms_high` | Manifest dz=0.30, circle; ~28 cm cf_second align RMS = expected Lee + downwash, not merge defect |
| Full-bank train (prior same-day pass) | `full_bank_40.npz`, log `full_bank_train.log`; val RMSE **0.2806**, baseline **1.2900**, **78.2%** reduction, fold **1.17e-06** — see `c2_full_bank_report.json` |
| Stage C first fail root cause | Host `cffirmware` built **without** `--features residual_nn` → `rnn_service()` compiled out |

---

### Claimed results — Task A

#### A.1 LOO

- **Script:** `experiments/analysis/run_c2_loo_18fold_2026_09_26.py`
- **Outputs:** `experiments/analysis/out/c2_e2e_2026-09-26/loo_18fold_results.json`, `loo_18fold_run.log`, `loo_weights_18fold/*.npz` (**17** weight files)
- **Folds:** **17** (not 18) — no LOO for `A1_2026-09-21_13-25-10` (0 rows)

**Per-flight held-out RMSE (m/s²)** — recompute from JSON or trust table after spot-check:

| Flight key | RMSE | % reduction vs predict-zero |
|------------|-----:|----------------------------:|
| A1_12-51-16 | 0.797 | 55.5 |
| A1_17-17-26 | 0.441 | 75.3 |
| A1_17-18-48 | 0.341 | 81.3 |
| A1_17-36-06 | 0.381 | 81.7 |
| A1_17-37-26 | 0.382 | 81.1 |
| A2_19-21-05 | 0.499 | 55.7 |
| A2_19-27-03 | 0.432 | 59.9 |
| A3_13-00-57 | 0.184 | 76.1 |
| A3_13-02-56 | 0.187 | 75.3 |
| A3_13-04-34 | 0.251 | 66.1 |
| A3_17-45-03 | 0.174 | 73.6 |
| A3_17-46-43 | 0.164 | 75.2 |
| A3_17-54-32 | 0.161 | 80.2 |
| A3_17-57-32 | 0.151 | 80.3 |
| A7_19-11-19 | 0.453 | 71.4 |
| A7_19-12-38 | 0.413 | 73.5 |
| A7_19-13-55 | 0.431 | 74.8 |

#### A.2 Stage C — PASS (after rebuild)

**Rebuild (required for Stage C):**

```bash
cd /home/georg/Desktop/flying_robot_course/flying_drone_stack/firmware_app
DRONE_PLATFORM=bl RUSTFLAGS="-C panic=abort" cargo build --release \
  --target x86_64-unknown-linux-gnu --features residual_nn
cd ~/Desktop/crazyflie-firmware && make bindings_python
```

**Re-run (example):**

```bash
cd /home/georg/Desktop/flying_robot_course
python3 flying_drone_stack/tools/residual/test_real_data_pipeline.py \
  --weights experiments/analysis/out/c2_e2e_2026-09-26/full_bank_40.npz \
  --csv experiments/logs/c1_2026-09-23_merged/A3_2026-09-23_17-45-03/A3_2026-09-23_17-45-03_merged_usd.csv
```

**Log:** `experiments/analysis/out/c2_e2e_2026-09-26/stage_c_23sep_A3_rerun.log`

| Check | Claimed value |
|-------|----------------|
| Path A max &#124;NumPy − compiled&#124; | **5.80e-07 m/s²** |
| Path B (differenced peer vel) | **2.06e-06 m/s²** |
| Overall | **PASS** (tol 2e-4) |

#### A.3 SIL inference (full-bank weights)

- **Script:** `experiments/analysis/c2_fullbank_sim_predict.sh`
- **Server yaml:** `experiments/sim_validation/server_c2_fullbank_predict.yaml`
- **Weights:** `experiments/analysis/out/c2_e2e_2026-09-26/full_bank_40.npz`
- **Settings:** `rnn.en=0`, `backend=neuralswarm`, formation A3 dz=0.30, predict+log only
- **Summary JSON:** `experiments/sim_validation/c2_fullbank_sim_inference.json`

| Metric | Claimed |
|--------|---------|
| Log rows | **5206** |
| Upload in log | 19297 weights, val RMSE **0.280608…**, `rnn.en=0`, network ready |
| `rnn.pred_z` finite, non-NaN | yes |
| `rnn.pred_z` RMS (cf231_active) | **~7.13 m/s²** |
| Clamp | min/max **±8.0** (~20% samples at clamp — `OUT_CLAMP`) |
| corr(`a_res_z`, `rnn_pred_z`) | **~0.159** (open-loop predict; low correlation not claimed as failure) |

**Not re-run in this session:** Stage D/E on full bank; 21-Sep Stage E remains closed-loop reference (`c2_e2e_stage_e.json`).

**Doc:** `docs/40_C2_Residual_Pipeline_E2E_Validation_Plan.md` — § **Extension — LOO, Stage C, SIL — 2026-09-26** (prior § **Full bank — 2026-09-26** preserved).

---

### Claimed results — Task B (flash-resident)

| Item | Detail |
|------|--------|
| Feature | `residual_nn_flash` in `flying_drone_stack/firmware_app/Cargo.toml` |
| Embed | `build.rs` + `flying_drone_stack/tools/residual/export_weights_rs.py` |
| Kbuild | `CF_CARGO_FEATURES="--features residual_nn_flash"`, `CF_RNN_WEIGHTS_NPZ=...` |
| Upload path | `residual_nn` unchanged; flash: `rnn_service()` sets `g_rnn_ready=1`, upload no-ops |

**Flash link verification (CF21BL):**

```bash
cd /home/georg/Desktop/flying_robot_course/flying_drone_stack/firmware_app
CF_RNN_WEIGHTS_NPZ=/home/georg/Desktop/flying_robot_course/experiments/analysis/out/c2_e2e_2026-09-26/full_bank_40.npz \
CF_CARGO_FEATURES="--features residual_nn_flash" make DRONE=bl
# Read RAM | and Flash | lines from build output
arm-none-eabi-nm -S build/cf21bl.elf | rg EMBEDDED_RNN
```

| Metric | Baseline (no RNN) | `residual_nn` (RAM) | `residual_nn_flash` |
|--------|------------------:|--------------------:|--------------------:|
| RAM used | 94304 | **overflow +40732 B** | **94312** (36760 free) |
| RAM bss | 84004 | +weight buffer | **84004** |
| Flash used | 393140 | link fails | **475972** |
| Symbol | — | — | **`0805b084 00012d84 T EMBEDDED_RNN_WEIGHTS`** |

**Host parity:**

```bash
python3 /home/georg/Desktop/flying_robot_course/flying_drone_stack/tools/residual/test_flash_vs_upload.py
# Claimed: PASS, |Δz| = 0 (both may hit OUT_CLAMP at -8.0 on test inputs)
```

**Default flight build restored:** plain `make DRONE=bl` without `CF_CARGO_FEATURES` → no RNN features, baseline RAM.

**Doc:** `docs/45_Residual_NN_RAM_Budget_Investigation.md` — top status updated + § **Implementation — flash-resident weights — 2026-09-26**.

---

### Claimed results — Task C (docs)

Validator: for each file, confirm **dated addendum** exists and **does not contradict** other docs.

| Doc | Expected change |
|-----|-----------------|
| `docs/07_Thesis_Progress_Checklist.md` | WHERE WE ARE, History **(56)**, C.2 step, prep table |
| `docs/25_C1_Data_Collection_Plan.md` | Desk-while-blocked line |
| `docs/40_*` | Status line + Extension section |
| `docs/45_*` | Status + Implementation section |
| `docs/13_Residual_Learning.md` | Dual deployment (upload vs flash), not RAM-only |
| `docs/Thesis_Progress_Overview.html` | C.2 / desk line |
| `docs/next_flight_card.html` | C.2 desk + NS2 bullet |
| `docs/lab_sessions/2026-09-24_to_26.md` | Evening addendum |

Also committed: `docs/48_2026-09-26_Desk_Priority_And_Needle_Moving_Prompt.md` (separate prioritization prompt — not the execution spec).

---

### Original execution rules (still apply for any follow-up)

- Do **not** modify network math in `residual_nn.rs` for RAM fixes.
- Read-only: `experiments/logs/usd_raw/`, merged CSVs.
- No git push unless operator asks.
- No Co-authored-by / session trailers on commits.
- Ground every number in something on disk or a re-run.

---

### Your deliverables as validator

1. **Pass/fail table** for each claimed result (LOO row count, Stage C numbers, SIL CSV stats, flash link RAM/flash, nm symbol, parity script).
2. **Doc consistency:** list any stale or contradicting sentences found outside the updated files.
3. **Gaps:** explicitly list what was **not** done (17 vs 18 LOO folds, Stage D/E full bank, full-bank train not re-run in extension commit, etc.).
4. **Recommendations:** only if a check fails — minimal fix path.

Suggested verification order:

1. `git log -3`, `git status`, confirm not pushed if policy requires.
2. `jq` / `python3` on `loo_18fold_results.json`, `c2_fullbank_sim_inference.json`, `stage_a_full_bank.json`.
3. Grep `stage_c_23sep_A3_rerun.log` for PASS and max diff lines.
4. Re-run `test_flash_vs_upload.py` (slow — two cargo + bindings builds).
5. Optional: flash `make DRONE=bl` + nm (moderate time).
6. Read `docs/40` Extension + `docs/45` Implementation sections.

---

## VALIDATOR PROMPT END

---

## Operator notes

- **Prior desk prompt (portfolio ranking while mocap down):** `docs/48_2026-09-26_Desk_Priority_And_Needle_Moving_Prompt.md` — different purpose from this doc.
- **Authoritative narrative after validation:** `docs/40` (C.2), `docs/45` (RAM/flash), `docs/07` History (56).
- **Python for analysis:** `~/.pyenv/versions/flying_robots/bin/python` if system matplotlib broken.
