# Round 0 report (S0, method version "A + B")

Real cost (mean of the 4 tuning flights, 3-D RMS): **19.99 cm**; keep-best: round 0 (19.99 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1_dry_merged_usd.csv | 1 | ok | False | True | 37.13 | yes |
| A8 | A8_dry_merged_usd.csv | 1 | ok | False | False | 11.82 | yes |
| CIRCLE | CIRCLE_dry_merged_usd.csv | 1 | ok | False | False | 6.51 | yes |
| FIG8 | FIG8_dry_merged_usd.csv | 1 | ok | False | False | 6.15 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.2396, -0.0402, 0.3215, -0.147]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 1.25, 'Pz': 1.2258, 'Dxy': 0.8, 'Dz': 1.2388, 'Iz': 1.25, 'KRxy': 1.1513, 'KRz': 1.0906, 'Kwxy': 1.1831, 'Kwz': 0.8925, 'KIxy': 0.8}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 6, 'Pz': 0, 'Dxy': 8, 'Dz': 0, 'Iz': 27, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 18}

Desk time of this command: 1149 s (ingest 1 s, fit 571 s, tune 576 s).
