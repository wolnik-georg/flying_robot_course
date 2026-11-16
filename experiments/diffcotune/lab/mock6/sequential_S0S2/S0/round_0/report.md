# Round 0 report (S0, method version "A + B")

Real cost (mean of the 4 tuning flights, 3-D RMS): **19.99 cm**; keep-best: round 0 (19.99 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/sequential_S0S2/session_logs/S0/round_0/A1_r0_merged_usd.csv | 1 | ok | False | True | 37.13 | yes |
| A8 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/sequential_S0S2/session_logs/S0/round_0/A8_r0_merged_usd.csv | 1 | ok | False | False | 11.82 | yes |
| CIRCLE | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/sequential_S0S2/session_logs/S0/round_0/CIRCLE_r0_merged_usd.csv | 1 | ok | False | False | 6.51 | yes |
| FIG8 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/sequential_S0S2/session_logs/S0/round_0/FIG8_r0_merged_usd.csv | 1 | ok | False | False | 6.15 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.02, -0.0198, 0.02, -0.0198]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 0.9924, 'Pz': 1.0201, 'Dxy': 0.9802, 'Dz': 1.0104, 'Iz': 1.0202, 'KRxy': 1.0189, 'KRz': 1.0129, 'Kwxy': 1.0199, 'Kwz': 0.9864, 'KIxy': 0.9808}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

Desk time of this command: 134 s (ingest 1 s, fit 67 s, tune 64 s).
