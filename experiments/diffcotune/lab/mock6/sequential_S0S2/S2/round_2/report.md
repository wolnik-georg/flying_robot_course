# Round 2 report (S2, method version "A + B")

Real cost (mean of the 4 tuning flights, 3-D RMS): **15.11 cm**; keep-best: round 2 (15.11 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/sequential_S0S2/session_logs/S2/round_2/A1_r2_merged_usd.csv | 1 | ok | False | True | 27.52 | yes |
| A8 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/sequential_S0S2/session_logs/S2/round_2/A8_r2_merged_usd.csv | 1 | ok | False | False | 9.48 | yes |
| CIRCLE | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/sequential_S0S2/session_logs/S2/round_2/CIRCLE_r2_merged_usd.csv | 1 | ok | False | False | 6.02 | yes |
| FIG8 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/sequential_S0S2/session_logs/S2/round_2/FIG8_r2_merged_usd.csv | 1 | ok | False | False | 5.48 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.0595, 0.0147, 0.0586, 0.0371]
theta flown (factor to the default gains): {'Pxy': 1.0029, 'Pz': 1.0081, 'Dxy': 0.9924, 'Dz': 1.0069, 'Iz': 1.0202, 'KRxy': 0.9957, 'KRz': 1.0037, 'Kwxy': 1.0067, 'Kwz': 1.0201, 'KIxy': 0.9927}
theta of the next round (factor to this round): {'Pxy': 1.0101, 'Pz': 0.99, 'Dxy': 0.99, 'Dz': 0.99, 'Iz': 1.0101, 'KRxy': 1.0101, 'KRz': 0.99, 'Kwxy': 1.0101, 'Kwz': 1.0101, 'KIxy': 1.0101}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

Desk time of this command: 225 s (ingest 1 s, fit 127 s, tune 94 s).
