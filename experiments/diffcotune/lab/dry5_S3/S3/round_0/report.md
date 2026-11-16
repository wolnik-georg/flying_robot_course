# Round 0 report (S3, method version "A + B")

Real cost (mean of the 4 tuning flights, 3-D RMS): **10.75 cm**; keep-best: round 0 (10.75 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1_dry_merged_usd.csv | 1 | ok | False | False | 18.93 | yes |
| A8 | A8_dry_merged_usd.csv | 1 | ok | False | False | 6.64 | yes |
| CIRCLE | CIRCLE_dry_merged_usd.csv | 1 | ok | False | False | 5.82 | yes |
| FIG8 | FIG8_dry_merged_usd.csv | 1 | ok | False | False | 5.13 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.0, 0.0, 0.0, 0.0, 0.0]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 0.9772, 'Pz': 1.0613, 'Dxy': 0.9987, 'Dz': 0.9348, 'Iz': 0.9342, 'KRxy': 1.0217, 'KRz': 0.864, 'Kwxy': 0.9968, 'Kwz': 1.1511, 'KIxy': 0.9876}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

Desk time of this command: 1581 s (ingest 1 s, fit 773 s, tune 805 s).
