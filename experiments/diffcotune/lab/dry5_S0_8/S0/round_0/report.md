# Round 0 report (S0, method version "A + B")

Real cost (mean of the tuning flights, 3-D RMS): **18.24 cm**; keep-best: round 0 (18.24 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1_dry_merged_usd.csv | 1 | ok | False | True | 37.13 | yes |
| A8 | A8_dry_merged_usd.csv | 1 | ok | False | False | 11.82 | yes |
| CIRCLE | CIRCLE_dry_merged_usd.csv | 1 | ok | False | False | 6.51 | yes |
| FIG8 | FIG8_dry_merged_usd.csv | 1 | ok | False | False | 6.15 | yes |
| A4 | A4_dry_merged_usd.csv | 1 | ok | False | False | 8.04 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.1007, -0.1099, 0.1373, -0.1021]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 0.8963, 'Pz': 1.138, 'Dxy': 0.983, 'Dz': 1.1094, 'Iz': 0.8918, 'KRxy': 1.1287, 'KRz': 1.1067, 'Kwxy': 1.1382, 'Kwz': 1.1365, 'KIxy': 0.9547}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

Desk time of this command: 535 s (ingest 1 s, fit 286 s, tune 243 s).
