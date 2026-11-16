# Round 2 report (S0, method version "A + B")

Real cost (mean of the 4 tuning flights, 3-D RMS): **17.80 cm**; keep-best: round 2 (17.80 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S0/round_2/A1_r2_merged_usd.csv | 1 | ok | False | True | 32.47 | yes |
| A8 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S0/round_2/A8_r2_merged_usd.csv | 1 | ok | False | False | 11.75 | yes |
| CIRCLE | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S0/round_2/CIRCLE_r2_merged_usd.csv | 1 | ok | False | False | 6.35 | yes |
| FIG8 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S0/round_2/FIG8_r2_merged_usd.csv | 1 | ok | False | False | 5.92 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.06, -0.0158, 0.06, -0.0191]
theta flown (factor to the default gains): {'Pxy': 1.011, 'Pz': 1.0407, 'Dxy': 0.9611, 'Dz': 0.9906, 'Iz': 1.0408, 'KRxy': 1.0313, 'KRz': 0.993, 'Kwxy': 1.0405, 'Kwz': 0.9677, 'KIxy': 0.9614}
theta of the next round (factor to this round): {'Pxy': 1.0187, 'Pz': 1.0115, 'Dxy': 0.9812, 'Dz': 1.018, 'Iz': 1.02, 'KRxy': 0.9964, 'KRz': 0.995, 'Kwxy': 1.0196, 'Kwz': 0.9802, 'KIxy': 0.9803}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

Desk time of this command: 139 s (ingest 1 s, fit 73 s, tune 64 s).
