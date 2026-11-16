# Round 3 report (S0, method version "A + B")

Real cost (mean of the 4 tuning flights, 3-D RMS): **16.35 cm**; keep-best: round 3 (16.35 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S0/round_3/A1_r3_merged_usd.csv | 1 | ok | False | True | 30.10 | yes |
| A8 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S0/round_3/A8_r3_merged_usd.csv | 1 | ok | False | False | 9.58 | yes |
| CIRCLE | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S0/round_3/CIRCLE_r3_merged_usd.csv | 1 | ok | False | False | 6.26 | yes |
| FIG8 | /home/georg/Desktop/flying_robot_course/experiments/diffcotune/lab/mock6/interleaved_S0S2/session_logs/S0/round_3/FIG8_r3_merged_usd.csv | 1 | ok | False | False | 5.76 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.08, -0.0192, 0.0799, -0.0337]
theta flown (factor to the default gains): {'Pxy': 1.0298, 'Pz': 1.0527, 'Dxy': 0.943, 'Dz': 1.0084, 'Iz': 1.0616, 'KRxy': 1.0276, 'KRz': 0.988, 'Kwxy': 1.0608, 'Kwz': 0.9486, 'KIxy': 0.9424}
theta of the next round (factor to this round): {'Pxy': 1.0155, 'Pz': 1.0126, 'Dxy': 0.9817, 'Dz': 0.9948, 'Iz': 1.0201, 'KRxy': 0.9958, 'KRz': 0.9831, 'Kwxy': 1.0163, 'Kwz': 0.9823, 'KIxy': 0.9807}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

Desk time of this command: 144 s (ingest 1 s, fit 79 s, tune 63 s).
