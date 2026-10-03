# Neural-Swarm2 reference comparison (public repo vs papers vs this project)

**Reference clone (outside repo):** `~/Desktop/neural-swarm-ref` @ **`48b185118edde247f46ab76ee821ec23fc1b98ed`** (sparse checkout: `learning/`, `hardware/nn-export/`, `hardware/datacollection/`, `hardware/neural-swarm-ros-pkg/`, `planning/`, `systemid/`, `README.md` — no `data/`).  
**Papers (pdftotext in clone):** `~/Desktop/neural-swarm-ref/papers/ns1_icra2020.txt`, `ns2_tro2021.txt`.

**Live SIL (R3):** `experiments/sim_validation/run_ns2_div_sim.py` → `experiments/sim_validation/ns2_div_sil_results.json`.

---

## Executive summary (read first)

### Ranked behavioural differences (impact → lab relevance)

| Rank | Topic | Verdict | Impact | Before next lab? |
|------|--------|---------|--------|------------------|
| 1 | **NN call rate / decimation** | Paper: ~550 µs/network, mocap **100 Hz**; **no explicit “evaluate NN at 100 Hz”** in public code. We run at **1 kHz** unless `rnn.div=10`. | **High** (MCU + peer-velocity semantics) | **Yes** — Bench B timing + predict-only A8; not a feature-definition fix |
| 2 | **Train/serve velocity** | Training: **500 Hz** merged `stateEstimate` diffs; firmware: **~100 Hz** peer packets + **hold** + optional decimation | **Medium** | Monitor `rnn_pred_*` / neighbour gate; no desk code change until bench proves gap |
| 3 | **SIL plant vs labels** | `np` backend has **no** downwash; `a_res` ≪ training scale → pred-vs-`a_res` metrics misleading in SIL | **Medium** (interpretation) | **No** for hardware gate; use hardware `rnn.en=0` logs |
| 4 | **Output clamp 8 m/s²** | **Ours only** (firmware + training checks); not in public `nn.c` / `compute_Fa` | **Low–medium** | Only if predictions saturate in uSD |
| 5 | **Max 3 neighbours** | Firmware buffer; reference `compute_Fa` loops all neighbours | **Low** (2-drone lab) | No |
| 6 | **phi_L / rho_L** | Reference supports large/small; we export **zeros** (Crazyflie-only fleet) | **None** for current lab | No |
| 7 | **C vs Python validation** | Reference `validate.py` = **heatmaps only**; no numeric tolerance test | **None** | No — we have `test_pipeline.py` / `test_residual_nn.py` |

### Evaluation-rate verdict (one paragraph)

**Neither NS1 nor NS2 states a fixed onboard NN evaluation frequency.** NS1 gives **~550 µs** per network on STM32 @ 168 MHz and **f_a < 4 ms for ≤6 neighbours**, with **motion capture at 100 Hz** (NS1 text ~L467–611; NS2 ~L1691). That **bounds** feasible rate (~100 Hz) but does **not** document “we call the network once per position loop.” Public `hardware/nn-export/` has **no stabilizer integration**; onboard call site is **UNKNOWN (private `crazyflie-firmware`)**. Our **`rnn.div=10` (100 Hz hold) is inferred**, consistent with timing + mocap, **not** a reference-documented setting.

### R3 — does 100 Hz hold degrade predictions/control? (corrected 2026-10-03)

**Method:** `run_ns2_div_sim.py` — **fresh subprocess per config**, custom **A8-like** pass (not library A8), `np`
plant (**pred-vs-`a_res` not interpretable**). **100 Hz peer mode:** new position+timestamp every **10 ms** (wrapper
on `oot_set_peer`); contrast mode uses default SIL **1 kHz** peer stamps (`crazyflie_sil.py` behaviour).

| Mode | div1 vs div10 predict-only (headline) | Confidence |
|------|----------------------------------------|------------|
| **100 Hz peer packets** | Mean RMS **~0.001 m/s²**; max RMS **~0.003 m/s²**; frac \|Δ\|>0.1 **~0%** (3 phases) | **High** (re-run script) |
| **1 kHz SIL peers** | Larger diffs (e.g. phase 1.1 RMS **~0.21**, max **~7.4**; clamp events) — **artefact of 1 ms differencing**, not hardware | **High** |

**Retracted:** Appendix I claim (RMS **0.93 m/s²**, lag **130 ms**) — **State not reset** between div runs + cross-correlation on spiky signals (Validation 9).

**Closed-loop `rnn.en=1`:** Kalman assert in some subprocess runs — **not reported** here; predict-only is the gating read for div hold.

---

## Comparison matrix

Columns: **Reference** (`~/Desktop/neural-swarm-ref/…`), **Paper**, **Ours**, **Status**, **Impact**, **Recommended action**.

| # | Item | Reference (file:line) | Paper (section) | Ours (file:line) | Status | Impact | Action |
|---|------|----------------------|-----------------|------------------|--------|--------|--------|
| 1 | φ/ρ architecture 6→25→40→40→H=20, ρ H→40→40→40→1 | `hardware/nn-export/nn.c:36-79`, `learning/nns.py:6-38` | NS2 §IV, Table III | `firmware_app/src/residual_nn.rs` (module doc) | **MATCH** | none | none |
| 2 | Activation ReLU×3, last linear | `nn.c:14-33`, `nns.py:16-20` | NS2 §IV | `residual_nn.rs` `layer_relu` / `layer_linear` | **MATCH** | none | none |
| 3 | Weight layout PyTorch `Linear`: export **W.T** | `hardware/nn-export/gen.py:23-24` | — | `tools/residual/model.py` `flatten`; `OFF_*` in `residual_nn.rs:119-125` | **MATCH** | none | Keep `test_pipeline.py` |
| 4 | φ_S/L input: **neighbour − self** (pos, vel) | `planning/robots.py:255-256` | NS1 eq. (relative states) | `residual_nn.rs` gate + `lib.rs` peer rel | **MATCH** | none | none |
| 5 | φ_G: **[z_g−z, −vx, −vy, −vz]** (ground at z=0) | `planning/robots.py:266-268`; `neuralswarm.py:88-90` | NS2 ground / φ_env | `residual_nn.rs` ground term | **MATCH** | none | none |
| 6 | Neighbour gate **\|dx\|, \|dy\| < 0.2**, **\|dvx\| < 1.5** | `planning/robots.py:257` | — (not in paper eqs) | `residual_nn.rs:139-140`, `281-283` | **MATCH** | none | Omar line **79** copied from **`planning/robots.py:257`** |
| 7 | Output **f_a,z scalar**; x/y = 0 | `neuralswarm.py:100`; `nn.c:111-124` | NS2 eq. (9), z force | `residual_nn.rs` `compute_Fa` → `(0,0,faz)` | **MATCH** | none | **`rnn_pred_x/y ≡ 0` expected by design** |
| 8 | Units: network **grams** → N → `/mass` | `neuralswarm.py:136-137` | NS2 §V-C (~grams) | `residual_nn.rs:136`, `136-137` | **MATCH** | none | none |
| 9 | Input normalisation | Training uses **raw** m, m/s in `get_data` (`learning/utils.py:200-230`); **no fold** in `gen.py` | — | **Folded into fc1** (`train.py:87-88`, `fold_normalisation`) | **DIFFERS** (equivalent if fold correct) | low | **Verified** ~1e-6 via `test_pipeline.py` (**high**) |
| 10 | Training label **f_a,z** | `Fa()` thrust−IMU, **`fa_delay`** 0.16 filter (`utils.py:160-191`); `training.py:49` | NS2 data / load cell | **`indi.a_res_*`**, merge @ **500 Hz** | **DIFFERS** | **medium** | Compare hardware logs to merge stats; no retrain before bench |
| 11 | Logging / sample rate | `config_datacollection.txt:1` → **100 Hz** | — | uSD **500 Hz** (`merge_usd_logs.py:44`) | **DIFFERS** | medium | Quote measured merge offset each flight |
| 12 | Peer velocity in training | Cubic interp logs; **vel from data** (`utils.py` pipelines) | — | **Finite diff** @ 500 Hz in merge (`merge_usd_logs.py:212-214`) | **DIFFERS** | **medium** | Document hold + 100 Hz packets in `docs/13` |
| 13 | Peer velocity onboard | **UNKNOWN** (private firmware) | — | Diff on **timestamp change**, else **hold** (`lib.rs:852-860`) | **UNKNOWN-private** vs **DIFFERS** from 500 Hz merge | **medium** | Bench A8 predict-only |
| 14 | NN eval rate | **Not stated** in `nn-export/` | ~550 µs; mocap 100 Hz | **1 kHz** call; **`rnn.div` default 10** (`lib.rs:874-891`) | **DIFFERS** | **high** | Bench B; keep 100 Hz build |
| 15 | Spectral norm **Lip=3**, batch 256, 20 epochs | `training.py:44-45,47,1007-1080` | NS2 §IV-C | Adam + val split; full-bank LOO (`train.py`, `docs/13`) | **PARTIAL MATCH** | low | — |
| 16 | Val split | 50 chunks, **20% val** (`utils.py:414-425`) | NS2 §V-C | Block / LOO on 26-flight bank | **DIFFERS** | low | — |
| 17 | Training filter (spatial) | `data_filter` defaults **0.4** (`utils.py:235`); training overrides **0.35** (`training.py:50-51`) | — | Dataset gates in `dataset.py` | **DIFFERS** (thresholds) | low | — |
| 18 | MAX neighbours | No hard cap in `compute_Fa` loop | ≤6 neighbours timing | **`MAX_NEIGHBOURS=3`** | **DIFFERS** | low (2 drones) | none |
| 19 | Robot type | small / large paths | Heterogeneous | **small only**; φ_L exported **0** | **DIFFERS** (by design) | none | none |
| 20 | Output clamp | No clamp in `nn.c` / `compute_Fa` | — | **`OUT_CLAMP=8.0` m/s²** (`residual_nn.rs:134`) | **DIFFERS** | medium | Watch `rnn.clamped` in logs |
| 21 | C vs Python numeric check | `validate.py` **visual only** | — | `test_residual_nn.py`, `test_pipeline.py` ~**1e-6** | **DIFFERS** (ours stricter) | none | none |
| 22 | Static scratch buffers | `nn.c:9-12` static temps | — | `residual_nn.rs` static `PHI_*`, `RHO_*` | **MATCH** | none | none |
| 23 | **`rnn_pred_z` magnitude at rest / lock** | — | NS2 example fa,z **grams** (e.g. **5±8.9 g** ground, §V-C) | Host harness (full-bank weights): lone φ_G at rest **z=0.1…1.2 m** → **−0.64…−0.05 m/s²**; **not −4.5**. Neighbour at **zero offset** → **−2.4**; 0.5 m above in gate → **−2.9**. Hardware **−4.5** on 10-03 while **estimate locked / flipping** — lock artefact, not φ_G scale alone | **DIFFERS** (interpretation) | medium | A8: preds must **vary** with peer motion; constant z-only pred ≠ healthy neighbour path |

**Counts (verified rows only):** **MATCH 10** · **DIFFERS 11** · **UNKNOWN-private 1** · **PARTIAL 1** · **Plausible / not verified 1**.

---

## Section notes

### 1. Architecture & validation

Reference `validate.py` builds heatmaps via SWIG `nnexport` (`validate.py:23-46`); it does **not** compare against PyTorch on random tensors. Our **`host/test_residual_nn.py`** and **`tools/residual/test_pipeline.py`** fill that gap (~**1e-6** m/s²).

### 2. Features & sign conventions

Training tensor uses **D2−D1** for relative position/velocity (`utils.py:224-225`). Heatmap code uses **x_2 − x_self** (`utils.py:462-465`). Planning **`x_neighbor - x`** (`robots.py:256`). All consistent with our firmware.

**validate.py quirk:** for `ground`, it passes `c[2:]` from the **last** neighbour loop iteration (`validate.py:35-36`) — misleading for multi-neighbour plots; **not** the production ground path in `robots.py`.

### 3. Normalisation

Reference ships **unnormalised** weights in `gen.py`; normalisation is **not** applied in `nn.c`. We **fold** μ/σ into first layers at export (`train.py:170`) so firmware sees raw SI units — documented in `docs/13` and `model.py:43-55`.

### 4. Gating provenance

**Omar `neuralswarm.py:79`** matches **`planning/robots.py:257`** (same three tests). Training **`data_filter`** uses separate **spatial** thresholds (0.35 m), not the 0.2/1.5 inference gate.

### 5. Timing & integration (what is / is not stated)

| Source | States |
|--------|--------|
| NS1 paper text | 100 Hz mocap; ~**550 µs**/network; **<4 ms** for ≤6 neighbours |
| NS2 paper text | 100 Hz mocap; controller+EKF+NN **onboard** |
| `config_datacollection.txt` | Log **100 Hz**, mode **1 = synchronous stabilizer** |
| `nn-export/main.c` | Demo single eval only |
| Private firmware | **Call period, peer broadcast rate — UNKNOWN** |

---

## Before next lab session (R2 → checklist)

**No feature-definition mismatch** requiring a **desk code change** before Checklist G. **Do** complete **Bench B** (timing), **predict-only A8** (peer path + preds), and treat SIL pred-vs-`a_res` as **non-gating**.

---

## Changelog

| Date | Change |
|------|--------|
| 2026-10-03 | Initial matrix + SIL div sweep (Cursor desk close-out) |
