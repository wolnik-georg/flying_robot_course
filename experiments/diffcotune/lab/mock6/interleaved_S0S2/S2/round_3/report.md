# Round 3 report (S2, method version "A + B")

Real cost (mean of the 4 tuning flights, 3-D RMS): **14.91 cm**; keep-best: round 3 (14.91 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S2/round_3/A1_r3_merged_usd.csv | 1 | ok | False | True | 27.57 | yes |
| A8 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S2/round_3/A8_r3_merged_usd.csv | 1 | ok | False | False | 8.08 | yes |
| CIRCLE | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S2/round_3/CIRCLE_r3_merged_usd.csv | 1 | ok | False | False | 5.94 | yes |
| FIG8 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S2/round_3/FIG8_r3_merged_usd.csv | 1 | ok | False | False | 5.38 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.0795, -0.0046, 0.0786, 0.0179]
theta flown (factor to the default gains): {'Pxy': 1.013, 'Pz': 0.9981, 'Dxy': 0.9826, 'Dz': 0.9969, 'Iz': 1.0304, 'KRxy': 1.0057, 'KRz': 0.9937, 'Kwxy': 1.0169, 'Kwz': 1.0304, 'KIxy': 1.0027}
theta of the next round (factor to this round): {'Pxy': 0.9841, 'Pz': 1.0198, 'Dxy': 1.01, 'Dz': 1.0174, 'Iz': 1.0202, 'KRxy': 1.0154, 'KRz': 0.9802, 'Kwxy': 0.9888, 'Kwz': 0.9849, 'KIxy': 1.0038}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

Desk time of this command: 212 s (ingest 1 s, fit 127 s, tune 81 s).
