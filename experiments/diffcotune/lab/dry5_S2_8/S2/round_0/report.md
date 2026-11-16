# Round 0 report (S2, method version "A + B")

Real cost (mean of the tuning flights, 3-D RMS): **14.38 cm**; keep-best: round 0 (14.38 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1_dry_merged_usd.csv | 1 | ok | False | True | 28.87 | yes |
| A8 | A8_dry_merged_usd.csv | 1 | ok | False | False | 9.09 | yes |
| CIRCLE | CIRCLE_dry_merged_usd.csv | 1 | ok | False | False | 6.15 | yes |
| FIG8 | FIG8_dry_merged_usd.csv | 1 | ok | False | False | 5.78 | yes |
| A4 | A4_dry_merged_usd.csv | 1 | ok | False | False | 6.85 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.0387, -0.0324, -0.0115, -0.0396]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 1.017, 'Pz': 0.9836, 'Dxy': 0.9836, 'Dz': 0.985, 'Iz': 0.9812, 'KRxy': 0.986, 'KRz': 1.02, 'Kwxy': 1.0175, 'Kwz': 1.0201, 'KIxy': 0.9895}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

Desk time of this command: 396 s (ingest 1 s, fit 209 s, tune 179 s).
