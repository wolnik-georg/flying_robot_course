# DiffCoTune lab session log 2026-11-18 (pair S0 + S2; laptop = round.py/session.py only, lab PC = CS2 + flights)

| time | event |
|---|---|
| 16:09 | `~/Desktop/LAB_SESSION_ACTIVE` created by the lab agent at the user's request (was missing on the laptop). `~/lab_yaml/`, `~/lab_inbox/` created. |
| 16:09 | `~/lab_yaml/crazyflies.yaml` was NOT copied from the lab PC (missing). Agent wrote a PROVISIONAL copy: `git -C ~/Desktop/crazyswarm2 show main:crazyflie/config/crazyflies.yaml` (main = 452342128e10, read only, checkout stays on study_FBL_stock). sha256 704a865160aa683a6388fa8475badd9a3de403994e5497df9a25901c9d1f9763. Grep: cf5 ctrlOmarIndi.indi = 0, rnn.en = 0, no ctrlFbl. |
| 16:09 | Preflight S0 on that copy: PREFLIGHT PASSED (incl. CIRCLE/FIG8 pad checks, pad [0.0, 0.3]). |
| 16:10 | Preflight S2 on that copy: FAILED (3c, 7b: yaml has indi/rnn.en = (0,0), S2 needs (0,1)). `set_tree_strategy.py S2` on a scratch copy: "anchor not found ... is the tree_S*.patch applied to this yaml?" -> the main-branch yaml has no tree-strategy anchors, so this provisional copy is probably NOT the lab PC's yaml. |

Open: real lab-PC yaml needed (sha256sum on both machines) before preflight S2 can be judged; batteries, uSD cards, `round.py --init` not done yet.

## Hardware and image (reported by the user, addendum aa0b2653; not verified by the agent)
- New bottom drone (yaml cf5) radio://0/80/2M/E7E7E7BB02 after the cf5 power fault of 26 Oct; top = cf_second radio://0/80/2M/E7E7E7E7E7 (was ...E9). Which physical drone is new: ASK the user, not yet recorded.
- Image on both drones: fbl_v2b_candidate.bin, sha256 87bd963e8299ff6a4bda7dd190355651df7da4e905d90c505bd4cd663b299be4 (4 Nov); has NOT flown before; image of record for the campaign. Preflight 7a still names study_tree_v2.bin (note only). check_active_controller.py ... 9 on both drones still to run.
- uSD: THESIS1/THESIS2 old 17 files each backed up to ~/usd_card_backup/THESIS{1,2}_2026-11-18_pre_diffcotune/ and archived on card; study config sha256 d247134c...; both start at thesis00; check_usd_deck.py still to run.
- Lab PC path: ~/flying_robot_course on feature/auto-tuning (there is no ~/flying_robot_course_dct).

## Yaml (agent)
- 18:0x: ~/lab_yaml/crazyflies.yaml replaced by the user/planner (earlier agent copy kept as crazyflies.yaml.before_2026-11-18_address, sha 704a8651...). Now sha256 bd6c05e966b29ef3010fcb0c991e63c22e3cc7e24345d14b74bc9ecb425da43d = `git show main:` of ~/Desktop/crazyswarm2 main (80c54c8, address fix). Both URIs as in the addendum; no ctrlFbl; cf5 indi 0, rnn.en 0 (S0). Lab-PC sha256 not yet compared (ask user).
- FINDING: set_tree_strategy.py S2 fails on this yaml ("anchor not found" for the rnn.en comment "tree strategy: 1 = NS2"); no branch (main, study_S1/S2/S3) has that comment, so `session.py switch` S0->S2 will not work as printed. study_S2 yaml differs from main in exactly one line (rnn.en 0->1, line 246), sha256 f373260c42690f3b614781c19f5b6c4ee7c3a11152cedf2737e96a55b1c606f8. Proposed switch: use the study_S2 branch yaml (or an equivalent one-line edit). Awaiting user decision; no repo tool edited.
- Tool fix by the planning agent: commit 7b9b00dd on feature/auto-tuning (set_tree_strategy.py accepts the current rnn.en comment). Laptop repo already contains it. Instruction: use session.py switch as written, no crazyswarm2 branch switch, no study_S2 yaml copy; run the same switch on ~/lab_yaml/crazyflies.yaml and on the lab PC yaml, sha256 after each switch. (Agent test on a scratch copy: S2 sha256 shown in the next line.)
- 17:4x (per user): check_active_controller ... 9 passed on both drones after the flash: "CONTROLLER: Using OutOfTree4 (9) controller", all log variables present (agent did not see the output). check_usd_deck, batteries, supervisor OK: pending (user).
- New bottom drone identity and lab-PC yaml sha256: user message contained unfilled placeholders ("<which one>", "<value>"); NOT recorded, asked again.
- 18:2x (per user): check_usd_deck both drones: usd.canLog 1 OK, gyro group present, usd.logging 0; usd.bcUSD missing on both (known, same as docs/lab_sessions/2026-10-19.md:17). Batteries good, supervisor's OK given (user statement).
- 18:19: first S0 preflight hung (PID 3450533, 11 min CPU, load ~11-12, stuck 15 Nov processes); stopped by exact PID, rerun started (output scratchpad/pf_S0.txt).

## Setup facts confirmed by the planning agent (status update 2026-11-18 evening; agent did not verify)
- Assignment by radio address: cf5 = bottom = radio://0/80/2M/E7E7E7BB02; cf_second = top = radio://0/80/2M/E7E7E7E7E7 (was ...E7E9, no longer answers). Both drones identical builds; physical marking after the shakedown. Hardware change against the 26 Oct setup (cf5 power fault, docs/lab_sessions/2026-10-26.md:21).
- Image of record (both drones): fbl_v2b_candidate.bin sha256 87bd963e8299ff6a4bda7dd190355651df7da4e905d90c505bd4cd663b299be4 (4 Nov), flashed on the laptop 17:4x (BB02 541699 bytes OK, E7E7E7E7E7 541699 bytes OK). Preflight 7a still names study_tree_v2.bin = note only. check_active_controller 9: console "Using OutOfTree4 (9) controller", missing [].
- crazyswarm2 main 80c54c8 (address fix); lab PC yaml sha256 bd6c05e9...da43d (= laptop copy, per planning agent). flying_robot_course feature/auto-tuning 7b9b00dd on laptop and lab PC (~/flying_robot_course).
- uSD: THESIS1/2 backed up and archived, study config d247134c..., start thesis00; check_usd_deck OK (usd.bcUSD missing = known). Batteries and supervisor OK: stated by the user earlier (to be confirmed once more before the shakedown).

## NVIDIA hang (agent findings)
- Cause confirmed: nvidia-smi "Driver/library version mismatch"; torch.cuda.is_available() hangs > 60 s. Two earlier agent preflight runs (S0 PID 3450533, then S0 3453646 + S2 3453584) hung in torch._dynamo -> torch.cuda.is_available; all three stopped by exact PID.
- CUDA_VISIBLE_DEVICES=-1 alone: still hangs. PYTORCH_NVML_BASED_CUDA_CHECK=1 alone: still hangs. Both together: plain `torch.cuda.is_available()` returns False in 1.5 s. BUT preflight S0 with both variables still did not finish in 250 s (stack: still inside torch._dynamo transform, s0_model.py:239 rollout).
- With both variables PLUS TORCHDYNAMO_DISABLE=1 (eager rollout): S0 preflight 22 s, PREFLIGHT PASSED (1c 18 s); S2 preflight on the copy switched by the fixed tool (sha256 f373260c...606f8): PREFLIGHT PASSED (1c 23 s; 3c, 7a, 7b PASS; 7a image note only). No file under the repo changed. lab_env.sh was not found by the agent (find over the repo), not sourced.
- OPEN: whether TORCHDYNAMO_DISABLE=1 is acceptable for round.py/desk (eager vs compiled path; not verified to give identical results). Question to the planning agent via the user.

## After reboot (19:00-19:05)
- uptime 3 min at 19:00; nvidia-smi OK (driver 580.178.04); torch import 1.8 s, cuda available True; used CUDA_VISIBLE_DEVICES=-1, no TORCHDYNAMO_DISABLE.
- HEAD 7b9b00dd; ~/lab_yaml/crazyflies.yaml sha256 bd6c05e9... (S0); LAB_SESSION_ACTIVE present.
- Preflight S0 (~/lab_yaml yaml) PASSED (27 s). Preflight S2 (scratch copy switched with set_tree_strategy.py, sha f373260c...) PASSED.
- Not yet done: round.py --init S0/S2, shakedown, flights. session_helpers/lab_env.sh does not exist in this tree.
