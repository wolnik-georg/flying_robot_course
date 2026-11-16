# Round 0 report (S1, method version "A + B")

Real cost (mean of the 4 tuning flights, 3-D RMS): **10.32 cm**; keep-best: round 0 (10.32 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1_dry_merged_usd.csv | 1 | ok | False | False | 17.72 | yes |
| A8 | A8_dry_merged_usd.csv | 1 | ok | False | False | 7.04 | yes |
| CIRCLE | CIRCLE_dry_merged_usd.csv | 1 | ok | False | False | 5.90 | yes |
| FIG8 | FIG8_dry_merged_usd.csv | 1 | ok | False | False | 5.25 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.0867, 0.0858, 0.0027, -0.0563, -0.0478]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 1.1151, 'Pz': 0.8463, 'Dxy': 1.0755, 'Dz': 1.0853, 'Iz': 1.25, 'KRxy': 1.063, 'KRz': 1.015, 'Kwxy': 1.0564, 'Kwz': 0.9842, 'KIxy': 1.1053}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 28, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

Desk time of this command: 1275 s (ingest 1 s, fit 622 s, tune 650 s).
