# Round 1 report (S0, method version "A + B")

Real cost (mean of the 4 tuning flights, 3-D RMS): **9.71 cm**; keep-best: round 1 (9.71 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1_dry_merged_usd.csv | 1 | ok | False | False | 16.27 | yes |
| A8 | A8_dry_merged_usd.csv | 1 | ok | False | False | 7.67 | yes |
| CIRCLE | CIRCLE_dry_merged_usd.csv | 1 | ok | False | False | 5.31 | yes |
| FIG8 | FIG8_dry_merged_usd.csv | 1 | ok | False | False | 5.06 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.1097, 0.1638, 0.1652, -0.0276]
theta flown (factor to the default gains): {'Pxy': 1.25, 'Pz': 1.2258, 'Dxy': 0.8, 'Dz': 1.2388, 'Iz': 1.25, 'KRxy': 1.1513, 'KRz': 1.0906, 'Kwxy': 1.1831, 'Kwz': 0.8925, 'KIxy': 0.8}
theta of the next round (factor to this round): {'Pxy': 0.8142, 'Pz': 0.8622, 'Dxy': 1.0533, 'Dz': 1.0146, 'Iz': 1.2117, 'KRxy': 1.1202, 'KRz': 1.0464, 'Kwxy': 1.0015, 'Kwz': 1.0292, 'KIxy': 1.0474}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 9, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

Desk time of this command: 1232 s (ingest 1 s, fit 608 s, tune 621 s).
