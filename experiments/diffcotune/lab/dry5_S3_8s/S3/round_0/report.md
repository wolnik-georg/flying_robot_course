# Round 0 report (S3, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **10.08 cm**; keep-best: round 0 (10.08 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1_dry_merged_usd.csv | 1 | ok | False | False | 18.93 | yes |
| A8 | A8_dry_merged_usd.csv | 1 | ok | False | False | 6.64 | yes |
| CIRCLE | CIRCLE_dry_merged_usd.csv | 1 | ok | False | False | 5.82 | yes |
| FIG8 | FIG8_dry_merged_usd.csv | 1 | ok | False | False | 5.13 | yes |
| A4 | A4_dry_merged_usd.csv | 1 | ok | False | False | 6.72 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.2011, -0.4881, -0.2476, -0.2845, 0.0]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 0.9562, 'Pz': 0.9576, 'Dxy': 1.0518, 'Dz': 0.9596, 'Iz': 0.9565, 'KRxy': 1.0507, 'KRz': 1.0675, 'Kwxy': 0.9852, 'Kwz': 1.0601, 'KIxy': 1.0349}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 5.77 cm, best flown set 6.13 cm (24 s)
  A1      new: 13.4 cm | 13.4 cm                    best: 13.9 cm | 13.6 cm
  A8      new: 6.9 cm | 8.1 cm                      best: 7.6 cm | 9.1 cm
  CIRCLE  new: 2.3 cm | 2.9 cm                      best: 2.2 cm | 2.8 cm
  FIG8    new: 2.2 cm | 2.1 cm                      best: 2.1 cm | 2.0 cm
  A4      new: 3.5 cm | 3.0 cm                      best: 4.1 cm | 3.9 cm
```

Desk time of this command: 2562 s (ingest 1 s, fit 1233 s, tune 1294 s).
