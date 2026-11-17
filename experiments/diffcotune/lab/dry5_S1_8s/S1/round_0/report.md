# Round 0 report (S1, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **9.54 cm**; keep-best: round 0 (9.54 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1_dry_merged_usd.csv | 1 | ok | False | False | 17.72 | yes |
| A8 | A8_dry_merged_usd.csv | 1 | ok | False | False | 7.04 | yes |
| CIRCLE | CIRCLE_dry_merged_usd.csv | 1 | ok | False | False | 5.90 | yes |
| FIG8 | FIG8_dry_merged_usd.csv | 1 | ok | False | False | 5.25 | yes |
| A4 | A4_dry_merged_usd.csv | 1 | ok | False | False | 5.40 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.3209, -0.3393, -0.4382, -0.4361, 0.0]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 1.2121, 'Pz': 0.8, 'Dxy': 1.25, 'Dz': 1.25, 'Iz': 1.25, 'KRxy': 1.25, 'KRz': 1.25, 'Kwxy': 0.8051, 'Kwz': 1.25, 'KIxy': 1.25}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 9, 'Dxy': 28, 'Dz': 13, 'Iz': 27, 'KRxy': 26, 'KRz': 9, 'Kwxy': 0, 'Kwz': 23, 'KIxy': 27}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 3.25 cm, best flown set 5.06 cm (24 s)
  A1      new: 9.5 cm | 9.5 cm                      best: 12.3 cm | 12.1 cm
  A8      new: 2.1 cm | 2.3 cm                      best: 4.4 cm | 4.9 cm
  CIRCLE  new: 1.5 cm | 1.6 cm                      best: 2.3 cm | 2.2 cm
  FIG8    new: 1.5 cm | 1.5 cm                      best: 2.2 cm | 2.1 cm
  A4      new: 1.6 cm | 1.5 cm                      best: 4.3 cm | 3.8 cm
```

Desk time of this command: 2229 s (ingest 1 s, fit 1058 s, tune 1142 s).
