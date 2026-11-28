17:11:34 session start (user in lab); step 0.1 laptop bin check
17:16:52 cards: THESIS1 (29 archived) + THESIS2 (20 archived) -> v3 config, hashes match; old configs in ~/chirp_backup
17:17:26 flash cf5
17:18:10 cf5 flash command finished; flash cf_second
17:23:33 both flashed (a152c922 image), boot console: mount OK, config read OK, malloc 16384 OK (cf5 verified; cf_second see output); controller 9 readback OK both, canLog 1 both
17:48:39 G7 attempt: no takeoff (z 0.05 m, floor abort). Cause: chirp_flight.py lacked cf.arm(True) + EKF reset that run_formation does (brushless cf5 never takes off without arm). Fixed in chirp_flight.py (differs from sim b17da76b by that block only).
17:54:46 F1 aborted: telemetry silent > 0.5 s. User order: telemetry-silent abort disabled (STALE_S=1e9); other aborts unchanged.
17:58:41 F1: no usable flight log. thesis00 gate test, thesis01 8 s ground file (usd_check NO-GO, drone on ground), thesis02 0 B unclosed; fsck -a recovered 20 OLD fragments (40/77-var formats), none v3. Card archived to _archive_2026-11-28_F1_attempt_and_fsck, copies in experiments/logs/usd_raw/2026-11-28 (not committed).
18:0x-18:56 (planning agent took over on user order): chirp_flight.py fixes pushed: 058ce3ea (close file 3x), 0ee39007 + ac1b3e6e (streamed landing at the hover point instead of HLC land after cmdFullState; excursion / z-floor / rate aborts removed, tilt = usual 25 deg / 0.5 s rule), 93e68298 (2 s touchdown before disarm).
Flights (all complete, 115 vars, ~0.5 % lost rows, copies byte-verified in experiments/logs/usd_raw/2026-11-28/, analysis in chirp_session/out/F*_analysis/):
  F1 S1 r1 thesis01: roll Ms 13.3 PM 19 GM 2.9 | pitch 10.7 / 16 / 3.9
  F2 S1 r1 thesis03: roll 11.5 / 17 / 4.0 | pitch 11.7 / 18 / 3.7
  F6 S3 r1 thesis05 (KR 0.00707, NS2 rnn.pred_z active): roll 11.5 / 19 / 6.0 | pitch 11.1 / 17 / 3.7
  F3 S1 KR x0.9 thesis06 (KR_x 0.00693 logged): roll 10.7 / 16 / 5.1 | pitch 10.7 / 16 / 4.5
  F5 S1 r1 thesis07: roll 10.4 / 18 / 5.2 | pitch 11.5 / 20 / 4.3
  29 Hz INDI line in every flight (28.7-28.9 Hz); gyro delay minus mocap latency -5.5..+1.5 ms. Failed / empty files: thesis00, 02, 04 (0 B, aborted attempts). G1-G8 not run (user decision); hover at ~0.8 m (height after takeoff).
