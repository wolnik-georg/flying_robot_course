# Round 0 report (S2, method version "A + B")

Real cost (mean of the 4 tuning flights, 3-D RMS): **15.71 cm**; keep-best: round 0 (15.71 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1_dry_merged_usd.csv | 1 | ok | False | True | 28.87 | yes |
| A8 | A8_dry_merged_usd.csv | 1 | ok | False | False | 9.09 | yes |
| CIRCLE | CIRCLE_dry_merged_usd.csv | 1 | ok | False | False | 6.15 | yes |
| FIG8 | FIG8_dry_merged_usd.csv | 1 | ok | False | False | 5.78 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.019, 0.028, 0.0191, 0.0243]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 0.8875, 'Pz': 0.97, 'Dxy': 1.0415, 'Dz': 1.0412, 'Iz': 1.25, 'KRxy': 1.1235, 'KRz': 1.0229, 'Kwxy': 0.959, 'Kwz': 1.0137, 'KIxy': 1.0157}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 7, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

Desk time of this command: 1470 s (ingest 1 s, fit 711 s, tune 757 s).
