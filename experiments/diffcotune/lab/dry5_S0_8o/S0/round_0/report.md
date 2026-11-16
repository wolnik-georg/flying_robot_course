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

beta (log-factors) after the fit: [-0.2778, -0.4, -0.1706, 0.1376]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 1.2051, 'Pz': 1.2088, 'Dxy': 0.8, 'Dz': 0.9354, 'Iz': 1.25, 'KRxy': 1.0702, 'KRz': 0.9445, 'Kwxy': 1.2369, 'Kwz': 0.8, 'KIxy': 0.8046}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 5, 'Dz': 0, 'Iz': 26, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 20, 'KIxy': 3}

Desk time of this command: 2777 s (ingest 1 s, fit 1209 s, tune 1558 s).
