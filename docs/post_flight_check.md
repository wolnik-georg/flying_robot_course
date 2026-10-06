# Post-flight check (`post_flight_check.py`)

Quick QA after each formation flight: radio CSV pair, optional pose bag, optional uSD `run_tag`.

## Python environment

Bag decoding needs MCAP ROS 2 support in the **flying_robots** env:

```bash
~/.pyenv/versions/flying_robots/bin/pip install mcap mcap-ros2-support
```

## Usage

```bash
REPO=~/georg/flying_robot_course   # or ~/Desktop/flying_robot_course on the laptop
~/.pyenv/versions/flying_robots/bin/python \
  "$REPO/experiments/analysis/post_flight_check.py" \
  --date 2026-10-05 \
  --logs "$REPO/experiments/logs" \
  --bags "$REPO/experiments/logs/rosbags" \
  --md "$REPO/experiments/analysis/out/post_flight_check_2026-10-05.md" \
  --json "$REPO/experiments/analysis/out/post_flight_check_2026-10-05.json"
```

## Inputs

- **Radio:** `{scenario}_cf5_{date}_{HH-MM-SS}.csv` paired with `{scenario}_cf_second_{date}_{HH-MM-SS}.csv` and `{scenario}_{date}_{HH-MM-SS}.meta.json`.
- **Bag (optional):** auto-matched under `--bags` (see below).
- **uSD (optional):** `experiments/logs/usd_raw/{cf5,cf_second}_*_{date}_{HH-MM-SS}.bin` — checks size and `run_tag` vs `usd_run_tag` in meta.

## Thresholds (fixed)

| Check | Threshold |
|--------|-----------|
| Liftoff | first `pos_z` > 0.05 m |
| Radio pose step flag | single-sample 3D step > **15 cm** |
| Battery “loaded” min | samples with `vbat` > 2 V |
| Battery failure | min `vbat` **< 3.0 V** and that drone’s log ends **> 5 s** before partner (or collapse pattern on cf_second) |
| Log end mismatch flag | partner log ends **> 5 s** apart |
| Aborted flight | duration **< 15 s** and both drones `pos_z` end **< 0.05 m** |
| Bag match | bag `starting_time` ≤ `t_start_sim` from meta, lead **0–60 s**, pick **latest** bag start still preceding the flight |
| Bag duplicate pose | 3D distance **< 3 cm** for **> 0.2 s** while both `z` > 0.3 m |
| Est vs bag RMS | onboard estimate vs `/poses` for same name; first `z` > 0.2 m aligned; lag search **±0.3 s** in **10 ms** steps; xy RMS (airborne samples) |

## Verdicts

- **CLEAN** — no battery/ pose failure rules triggered.
- **NOT CLEAN** — **BATTERY**, **POSE_SWAP**, or **OTHER** with one-line evidence.
- **ABORTED** — short flight, both landed at end.

**POSE_SWAP** (radio): cf5 single step > 15 cm while cf5 `vbat` ok, and post-step position within **5 cm** of cf_second’s estimate at that time; or bag duplicate-pose flag.

## Bag auto-match

Uses `t_start_sim` from the flight `.meta.json` (same epoch as `usd_run_tag` source). Each bag directory’s `metadata.yaml` supplies `starting_time.nanoseconds_since_epoch`. The tool chooses the bag whose start is **closest before** the flight (maximum bag start among those with `0 ≤ t_start_sim - bag_start ≤ 60` s). Typical evening A8 leads were **~20–25 s** (bag started earlier than the 5–15 s rule-of-thumb but within 60 s).

## Acceptance run — 2026-10-05

Expected (from lab session close-out) vs **actual script output** (2026-10-06 desk run; thresholds **not** tuned).

| Time | Expected | Actual | Match? |
|------|----------|--------|--------|
| 17:39:27 | CLEAN | CLEAN | yes |
| 17:41:09 | CLEAN | CLEAN | yes |
| 17:59:30 | CLEAN | CLEAN | yes |
| 18:01:01 | NOT CLEAN, POSE_SWAP (~10.9 s) | NOT CLEAN, POSE_SWAP @ **18.2 s** (radio swap pattern) | partial (time) |
| 18:02:38 | NOT CLEAN, POSE_SWAP (~15.1 s) | NOT CLEAN, POSE_SWAP @ **22.4 s** (first swap-pattern step; later 750 cm tumble) | partial (time) |
| 18:18:58 | NOT CLEAN, POSE_SWAP or OTHER | NOT CLEAN, **POSE_SWAP @ 12.3 s** (swap then flip; uSD noted ~5.7 s in session doc) | partial (time / classification) |
| 18:34:10 | NOT CLEAN, BATTERY cf_second 2.53 V, log ~28.6 s | NOT CLEAN, BATTERY cf_second **2.53 V**, log **28.6 s** (cf5 48.2 s) | yes |
| 18:55:34 A1 | NOT CLEAN, BATTERY cf_second 2.61 V, log ~21.9 s, cf5 flip ~0.4 s later | NOT CLEAN, BATTERY cf_second **2.61 V**, log **21.9 s**; cf5 max tilt **179°** after partner log ended | yes |
| 19:17:04 | CLEAN, bag a8_2, est–bag RMS ~0.1–0.2 cm, step ≤ 1.2 cm, min dist 50 cm | CLEAN, **ns2_pose_a8_2**, bag max step **1.16 cm**, min dist **50 cm**, est–bag RMS **~0.14 cm** (xy, lag-aligned) | yes† |
| 19:19:27 | CLEAN, a8_3 | CLEAN, **ns2_pose_a8_3**, bag step **1.14 cm**, min dist **50 cm** | yes† |
| 19:23:26 | CLEAN, a8_5 | CLEAN, **ns2_pose_a8_5**, bag step **1.18 cm**, min dist **50 cm** | yes† |
| 19:21:54 | ABORTED ~8 s, no bag/uSD | **ABORTED** **8.2 s**, no bag, uSD missing | yes |

† The "2–3 cm" in the first draft of the session doc was a normalization error in a hand check (sum instead of mean over samples). Recomputed independently (3D, 1 ms lag grid): **~0.14–0.16 cm** at lags of 10–40 ms. The script is right; the session doc has been corrected.

Full machine output: `experiments/analysis/out/post_flight_check_2026-10-05.json`.

## Time base used in the evidence strings
Times such as `t=18.2 s` are the radio CSV `time_s` column as logged (absolute, starts at about 7 s), **not** seconds since liftoff or since the first sample. The session notes quote seconds since the first sample of the pair (about 7.3 s earlier): 18:01:01 swap 18.2 s ≡ 10.9 s, 18:02:38 22.4 s ≡ 15.1 s, 18:18:58 12.3 s ≡ 5.0 s. These are the same events. At 20 Hz radio logging the order of swap and flip at 18:18:58 cannot be resolved; use the uSD log (it starts after the event in that flight).
