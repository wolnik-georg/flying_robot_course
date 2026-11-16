# Round 0 report (S2, method version "A + B")

Real cost (mean of the 4 tuning flights, 3-D RMS): **15.71 cm**; keep-best: round 0 (15.71 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1_dry_merged_usd.csv | 1 | ok | False | True | 28.87 | yes |
| A8 | A8_dry_merged_usd.csv | 1 | ok | False | False | 9.09 | yes |
| CIRCLE | CIRCLE_dry_merged_usd.csv | 1 | ok | False | False | 6.15 | yes |
| FIG8 | FIG8_dry_merged_usd.csv | 1 | ok | False | False | 5.78 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.3608, -0.0658, 0.402, -0.0586]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 1.2328, 'Pz': 1.25, 'Dxy': 0.848, 'Dz': 0.8618, 'Iz': 1.25, 'KRxy': 1.1523, 'KRz': 1.0591, 'Kwxy': 1.203, 'Kwz': 0.8, 'KIxy': 0.8102}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 22, 'Dxy': 0, 'Dz': 0, 'Iz': 26, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 23, 'KIxy': 0}

Desk time of this command: 1560 s (ingest 0 s, fit 792 s, tune 764 s).
