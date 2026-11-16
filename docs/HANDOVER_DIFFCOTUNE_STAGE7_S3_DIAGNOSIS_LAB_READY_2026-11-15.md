# DiffCoTune, Stage 7: find and fix why S3 does not tune, an NS2 weights check, and the final lab-readiness pack (desk only, 2026-11-15)

Paste everything below the line into the tuning (main) agent.

---

Your Stage 5c and Stage 6 reports were checked against the files:
- the rdfix evaluations: S1 9.04 → 2.30 / 5.73 → 1.81 cm; S3 9.02 → 9.02 / 5.78 → 5.78 cm, best round 1;
- S2: 12.36 → 5.25 / 6.77 → 5.26 cm;
- the per-round logs, the Stage 6 timeline (4.82 → 3.28 h, 5.01 → 3.45 h) and the scenario sheet.

All confirmed. S0, S1 and S2 are lab-ready on the desk side. This stage makes S3 work if that is possible without changing the method, and closes the lab preparation.

## User decisions (15 Nov)

1. **Scenarios: option A** (tuning A1, A8, CIRCLE, FIG8; test A4 cells 1 / 2). Write it into the protocol as decided. There is no option "E": drop that open point.
2. **Lab sessions:**
   - session 1 is S0 + S2 interleaved;
   - session 2 is S1, alone or with S3 if this stage makes S3 lab-ready;
   - S3 stays out of the first session.
3. **Switch overhead** (strategy switch, `--solo` relaunch): measured in the first lab block, no desk work.
4. **Which PC runs `round.py` in the lab:** still open (user). Write both cases into the lab card: this PC (two desk steps at once shown to work), or the lab PC (one at a time unless it is measured there).
5. **NS2 weights:** add a check to the preflight (step 3).
6. **S3:** diagnose and fix in this stage (steps 1–2). Lab-ready only if criteria a–c pass with the frozen method version.

## What the S3 logs show (rdfix, `dryrun/stage5c_rdfix/logs/dry_S3.log`)

- **Round 0 tuning:**
  - the simulator cost rises from the start iterate (Jn 1.000 at epoch 0, 1.453 at epoch 50);
  - the best iterate is epoch 0, so θ₁ = θ₀ and the round-1 flight repeats the default gains.
- **Rounds 1–3:** the best iterates are at epochs 23, 17 and 12, and Jn rises again after them. The real cost stays at 10.5–10.95 cm.
- **The fit:**
  - motor lag drifts from −0.62 to −1.37 (log error);
  - plume from +0.53 to +0.95;
  - the fit cost does not fall much.
- **The gradient check passes** (J_sysId mixed, J_task forward compiled, J_task noisy reverse), so the gradient is the exact derivative of the model's cost.
- **S1 is the same controller without NS2 and tunes well** (10.3 → 2.9 cm). **S2 is NS2 on the geometric controller and tunes well.** So the suspect is the combination INDI + NS2 in the S3 model or its cost landscape. That is a hypothesis, not a fact.

## Do (one pool at a time, in this order)

### 1. Diagnose S3 (point θ₀ and β of round 0 of the rdfix run, noise seeds of round 0)

**a. Descent test:**
- Evaluate the noisy J_task (the 2 seeds of the tuning step, all 4 scenarios, summed as in the loop) along −g, in the coordinates Adam updates.
- Step lengths: 1e-5, 1e-4, 1e-3, 1e-2 and one Adam step at lr 0.01.
- Do the same per scenario, and noise-free.
- Report J against the step and the predicted first-order decrease g·Δ:
  - J falls for small steps: the gradient is a valid descent direction, and the problem is the step (landscape too rough for lr 0.01).
  - J rises even at 1e-5: the cost and the gradient disagree, which means a model bug (step 1c).

**b. Landscape:**
- 1-D noisy cost slices along each gain (±25 %, 21 points) and along −g (±0.05, 21 points), per scenario, for S3 and for S1 at the same point.
- Report roughness (for example the number of local minima, or the spread of the slope from neighbouring differences against g), and which scenario dominates g and J.

**c. NS2 path (same checks as S2, where it works):**
- the S3 model with NS2 off equals the S1 model (forward, ≤ 1e-12);
- the S3 model with NS2 on equals frsim S3 (the Stage 5 forward check; repeat on A1 and A8 at θ₀ and at the round-3 θ);
- the derivative of the NS2 output with respect to the measured state: autograd against finite differences at 3 points on A1 and A8;
- where the clamp at 8 m/s² or the hold is active, report how often it is active on A1 / A8 at θ₀;
- any `detach` or `no_grad` on the NS2 path that differs between S2 and S3 (code diff, with line numbers).

**d. INDI + NS2 interaction:** on A1 and A8 at θ₀:
- the NS2 term against the INDI disturbance estimate (`a_prev` / the filtered acceleration term);
- whether the two counteract each other in the model and in frsim alike.

Report it as a fact table. If model and frsim agree, it is plant behaviour, not a bug.

**e. Ablation tuning (round 0 only, the loop's settings):**
- S3 with NS2 off in the tuning model only (the plant stays S3): does θ₁ improve the real cost?
- S3 started from S1's θ₀ is the same as θ₀; skip it if identical.
- Information only: these are not adopted methods.

### 2. Fix, only if the cause is a bug

- **A bug in the S3 model** (it disagrees with frsim, or a wrong detach / clamp gradient): fix it in `experiments/diffcotune/` so the model matches frsim again. Rerun the S3 forward check and gradient check, then the S3 rdfix dry run, round test and gradient check (same seeds, plants, frozen method version, grad_mode mixed, RPM delay fixed). Report against criteria a–c.
- **No bug** (the model is right and the landscape is too rough or the gradient points the wrong way for a finite step):
  - do **not** change the method;
  - list the options as **new deviations** for the user, each with its expected cost and the test that would decide it. Examples: an S3-only smaller learning rate, more noise seeds in the tuning step, starting S3 from S1's θ_best, leaving S3 untuned (default gains in the study);
  - run at most one round-0 probe per option (information only, marked "not adopted").
- Either way: write **"S3 lab-ready on the desk side: yes / no"** with the reason.

### 3. NS2 weights check in the preflight

- Find the source of record for the NS2 weights each strategy flies with: the study manifest, the frozen block of `run_formation --study`, and the image or upload tool (read only). Also find the hash of `simulator/frsim/data/ns2_v2.npz`, which the S2 / S3 models use. If the source of record is not stated, write so and list it as open.
- `lab/preflight.py --controller S2|S3`:
  - check that the model's weights file hash equals the recorded v2 hash;
  - check that the lab yaml of that strategy has the NS2 parameters of record (`rnn.en`, `res_sign`, as `lab_check_strategy.py` defines them);
  - for S0 / S1, check that NS2 is off.
- Desk only, no radio. On a mismatch, PREFLIGHT FAILS with the reason.
- Mock test: one wrong hash and one wrong yaml value must both fail.

### 4. Lab-readiness pack

- **Run `preflight.py`** for S0, S1 and S2 with the final defaults (S1: RPM delay fixed, grad_mode mixed; S2: mixed; S0: reverse). Also for S3 if step 2 made it lab-ready. All must print PREFLIGHT PASSED.
- **`lab/LAB_READINESS.md`**, one page:
  - per controller: lab-ready yes / no, round time, the defaults `round.py` uses, the preflight result;
  - session 1 (S0 + S2 interleaved) and session 2 (S1 [+ S3]) with the cards to use (`INTERLEAVED_LAB_CARD.md`, `S0_LAB_CARD.md`), their order and the estimated hours from the Stage 6 timeline;
  - what is measured in the first block (switch overhead, `--solo` relaunch, readback time, battery use);
  - the open items with who decides (PC for `round.py`, flights per pack, uSD files per flight, supervisor agreement).
- **Interleaved lab card:** option A written in as decided; the two PC cases (decision 4); the NS2 preflight line for S2.
- **Protocol draft:** scenario option A decided (user, 15 Nov); session plan; S3 status. Mark everything "Stage 7".
- **Spec / inventory / READMEs:** marked "Stage 7". Any S3 model fix is listed with its check; any S3 option is listed as a proposed deviation, not adopted.

## Report back (short): `dryrun/stage7/report_stage7.md`

1. S3 diagnosis: the descent table, roughness, the NS2 checks (c, d) and the ablation; the cause, stated as found / not found.
2. Fix or options: what changed and its checks, or the deviation options with probes; the S3 dry run against criteria a–c if rerun; "S3 lab-ready: yes / no".
3. The NS2 weights check: the source of record, the hashes, the mock-test result.
4. Preflight results per controller; the link to `LAB_READINESS.md`.
5. Open points for the user, with the decision each needs.

## Rules

- **Resources:**
  - `nice -n 19`;
  - **one worker pool at a time**; every job through `dryrun/stage3c/job_runner.py` (load ≤ 22 and MemAvailable ≥ 3 GB, read from `/proc/loadavg` and `/proc/meminfo` before and one minute after each start); if a limit breaks, stop that job (exact PID) and report;
  - at most 16 CPU processes; no GPU (driver mismatch, do not touch it);
  - torch threads 2 per worker; `TORCHINDUCTOR_COMPILE_THREADS=1`;
  - jobs strictly one after another.
- **Shells:** no command that waits on standard input (no bare `cat >`); use heredocs or the edit tools.
- **Python:** `/usr/bin/python3` (3.10, torch 2.5.1) for frsim and the models; `~/.pyenv/versions/flying_robots/bin/python` for plots and log analysis; no pip installs, no torch upgrade.
- **Edits only in:**
  - `experiments/diffcotune/` (with `dryrun/stage7/` and `lab/`); new switches default to the current behaviour; a model bug fix must keep or restore the frsim match and is stated with its check;
  - `docs/diffcotune_spec.md`, `docs/diffcotune_inventory.md` (marked "Stage 7");
  - the protocol draft (scenario decision, session plan, S3 status).
- **Do not touch:**
  - `simulator/`, `flying_drone_stack/`, `~/Desktop/crazyswarm2`, `~/Desktop/crazyflie-firmware`, `~/Desktop/fbl_v2_firmware`: read only; read-only git commands only (no checkout, no branch switch); no firmware rebuild;
  - `experiments/tuning/`, `out/`, the old docs, `docs/papers/` (reading is fine), the supervisor summary, the meeting docs, the thesis, `docs/comparative_study_protocol.md` (reading is fine), everything FBL;
  - Stage 3b–6 results (write new results under `stage7/`).
- **Method:** frozen (DiffCoTune Split-Alternate, 50 epochs with the best iterate, J_sysId = p + v + q, noise with 2 seeds, ±25 % limit, lr 0.01, Adam reset, 5 rounds, keep-best, `--skip-last-fit`). No change for S3 without the user's decision.
- **Data:** complete uSD logs only, never radio CSVs.
- **Git and hardware:**
  - no branch switch, checkout, stash, commit or push in any repository;
  - no Co-Authored-By;
  - nothing flashed or flown; no connection to a drone or radio;
  - no reboot; do not touch the NVIDIA driver.
- **Stopping processes:** no `pkill`, `killall` or kill of a PID found by pattern. Only exact PIDs you started yourself, checked with `ps -o pid,ppid,args -p <PID>`.
- **Facts:** never invent numbers or rules; write "not stated" and list it as open. The NS2 suspicion is a hypothesis; report what the tests show.
- **Wording:**
  - time stamps only from the real `date` output;
  - method names in full in docs;
  - FBL is just "FBL";
  - say "test error", never "study score".
