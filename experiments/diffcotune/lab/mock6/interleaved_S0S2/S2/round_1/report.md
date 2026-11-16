# Round 1 report (S2, method version "A + B")

Real cost (mean of the 4 tuning flights, 3-D RMS): **15.85 cm**; keep-best: round 0 (15.71 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S2/round_1/A1_r1_merged_usd.csv | 1 | ok | False | True | 29.38 | yes |
| A8 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S2/round_1/A8_r1_merged_usd.csv | 1 | ok | False | False | 8.32 | yes |
| CIRCLE | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S2/round_1/CIRCLE_r1_merged_usd.csv | 1 | ok | False | False | 6.21 | yes |
| FIG8 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S2/round_1/FIG8_r1_merged_usd.csv | 1 | ok | False | False | 5.81 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.04, 0.0328, 0.0395, 0.0342]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 1.0029, 'Pz': 1.0081, 'Dxy': 0.9924, 'Dz': 1.0069, 'Iz': 1.0202, 'KRxy': 0.9957, 'KRz': 1.0037, 'Kwxy': 1.0067, 'Kwz': 1.0201, 'KIxy': 0.9927}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

Desk time of this command: 213 s (ingest 1 s, fit 127 s, tune 83 s).
