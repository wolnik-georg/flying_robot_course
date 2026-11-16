# Round 1 report (S0, method version "A + B")

Real cost (mean of the 4 tuning flights, 3-D RMS): **18.63 cm**; keep-best: round 1 (18.63 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S0/round_1/A1_r1_merged_usd.csv | 1 | ok | False | True | 34.60 | yes |
| A8 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S0/round_1/A8_r1_merged_usd.csv | 1 | ok | False | False | 10.52 | yes |
| CIRCLE | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S0/round_1/CIRCLE_r1_merged_usd.csv | 1 | ok | False | False | 6.50 | yes |
| FIG8 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S0/round_1/FIG8_r1_merged_usd.csv | 1 | ok | False | False | 6.11 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.04, -0.0345, 0.04, -0.0375]
theta flown (factor to the default gains): {'Pxy': 0.9924, 'Pz': 1.0201, 'Dxy': 0.9802, 'Dz': 1.0104, 'Iz': 1.0202, 'KRxy': 1.0189, 'KRz': 1.0129, 'Kwxy': 1.0199, 'Kwz': 0.9864, 'KIxy': 0.9808}
theta of the next round (factor to this round): {'Pxy': 1.0187, 'Pz': 1.0201, 'Dxy': 0.9806, 'Dz': 0.9804, 'Iz': 1.0202, 'KRxy': 1.0121, 'KRz': 0.9803, 'Kwxy': 1.0202, 'Kwz': 0.9811, 'KIxy': 0.9803}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

Desk time of this command: 146 s (ingest 1 s, fit 80 s, tune 63 s).
