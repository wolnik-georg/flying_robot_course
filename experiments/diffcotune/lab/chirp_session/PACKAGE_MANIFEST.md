# Package manifest — chirp session (REPORT_59 L1). Every file the lab agent must copy, and where
Source of everything: branch `desk/s1-simulator-gap` (worktree `~/Desktop/frc_desk_s1gap`), directory `experiments/desk_s1_gap/proposal/lab_pkg/` unless stated.

| file | copy to | purpose |
|---|---|---|
| `firmware/fbl_v2b_dshot_usd128_candidate.bin` (+ `.sha256`) | flash to **cf5 only** (cfloader / the usual procedure) | image 9e133f76 reproduced byte for byte + `usddeck_limit128_ring16k_AB.patch` (limit 128, 16 KB ring buffer) |
| `firmware/usddeck_limit128_ring16k_AB.patch`, `firmware/reproduce_build_ns.sh` | keep for the record | the patch; the namespace build that reproduces / rebuilds the image |
| `config.txt` | root of cf5's uSD card as `config.txt` | config v3: 96 current variables unchanged + 19 added = 115, buffer 16384 (needs the flashed image above) |
| `chirp_flight.py` | `experiments/diffcotune/lab/chirp_session/chirp_flight.py` (NO crazyswarm2 change; run `python3 chirp_flight.py --height 0.6 --amp 0.3 --flight F1` from a foreground terminal in the sourced lab workspace) | streamed chirp with the ramp, uSD gate (ground precheck, usec.reset on the ground, start in hover, stop after landing, usd.logging 0 in finally), aborts (keys, tilt / rate, stale, excursion > 0.3 m, z < 0.25 m), TTY check |
| `crazyflies_chirp_telemetry_PROPOSED.yaml` (in `proposal/`) | merge the `firmware_logging` fragment into the cf5 block of the crazyflies yaml (optional, solo only) | attitude / gyro_acc 100 Hz for the abort monitor |
| `cfusdlog.py`, `decode_usd_log_v3.py`, `v3_variables.json`, `rebuild.py`, `q2_loop.py`, `q2_timing.py`, `chirp_lib.py`, `usd_check.py`, `usd_analyse.py` | one directory on the analysis PC, e.g. `flying_drone_stack/tools/chirp_session/` (all together; `cfusdlog.py` is a copy of crazyflie-firmware/tools/usdlog) | L3 GO / NO-GO and L4a analysis; `decode_usd_log_v3.py` is the repo decoder + the 19 new names |
| `decode_usd_log_v3.patch` | `git apply` in the repo root only if the other tools (merge, plots) must read the new names | adds the RENAME entries to `flying_drone_stack/tools/decode_usd_log.py` |
| `flight_card_chirp.md`, `PACKAGE_MANIFEST.md` | read in the lab | the card (preconditions, G1-G8, F1-F5, commands) |
| **sim side (L4b / L5)**: check out the branch `desk/s1-simulator-gap` (do not copy single files) | any PC with the repo: `experiments/desk_s1_gap/` | needs `simulator/frsim` (new opt-in options `command_delay_fractional`, `torque_leadlag_*`), `lab_sim.py`, `sim_block.py`, `t5_sim.py`, `unified.py`, `u_summary.py`, `hold_out.py`, `common.py`, `aa_edge.py`, `base_r33m_uextra.json`, `lab_run.sh` |

## Commands (from the analysis directory / from experiments/desk_s1_gap)
* after every flight (< 2 min): `python3 usd_check.py <file> --axis y --amp 0.3 --kr <KR> --kw 0.00117812 --tilt-go <8|10|12>`  -> `GO` / `NO-GO: <failed checks>`
* after the flights: `python3 usd_analyse.py --out <dir> F1=<file>:y F2=<file>:y F3=<file>:y F5=<file>:y F6=<S3 file>:y`  -> `<dir>/analysis.json` + summary
* simulator block: `./lab_run.sh sim_block.py identify --analysis <dir>/analysis.json --flight F2 --out <dir>`  then  `./lab_run.sh sim_block.py passtest --block <dir>/block_measured.json --tag lab1`
* rehearsal (frsim flights in the real uSD format with a known block): `./lab_run.sh dryrun.py --out results/dryrun [--passtest]`
