# Round 0 report (S2, method version "A + B")

Real cost (mean of the 4 tuning flights, 3-D RMS): **15.71 cm**; keep-best: round 0 (15.71 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S2/round_0/A1_r0_merged_usd.csv | 1 | ok | False | True | 28.87 | yes |
| A8 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S2/round_0/A8_r0_merged_usd.csv | 1 | ok | False | False | 9.09 | yes |
| CIRCLE | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S2/round_0/CIRCLE_r0_merged_usd.csv | 1 | ok | False | False | 6.15 | yes |
| FIG8 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S2/round_0/FIG8_r0_merged_usd.csv | 1 | ok | False | False | 5.78 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.02, 0.02, 0.02, 0.02]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

Desk time of this command: 212 s (ingest 1 s, fit 128 s, tune 81 s).
