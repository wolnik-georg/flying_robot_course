# Round 0 report (S3, method version "A + B")

Real cost (mean of the 4 tuning flights, 3-D RMS): **10.75 cm**; keep-best: round 0 (10.75 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1_dry_merged_usd.csv | 1 | ok | False | False | 18.93 | yes |
| A8 | A8_dry_merged_usd.csv | 1 | ok | False | False | 6.64 | yes |
| CIRCLE | CIRCLE_dry_merged_usd.csv | 1 | ok | False | False | 5.82 | yes |
| FIG8 | FIG8_dry_merged_usd.csv | 1 | ok | False | False | 5.13 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.1224, -0.2142, -0.0934, 0.1476, -0.3156]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 0.9587, 'Pz': 1.0455, 'Dxy': 1.0413, 'Dz': 0.952, 'Iz': 0.9512, 'KRxy': 1.0427, 'KRz': 0.9512, 'Kwxy': 0.9599, 'Kwz': 0.9553, 'KIxy': 1.0323}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

Desk time of this command: 1705 s (ingest 1 s, fit 875 s, tune 826 s).
