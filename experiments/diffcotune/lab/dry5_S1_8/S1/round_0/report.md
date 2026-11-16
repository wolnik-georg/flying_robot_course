# Round 0 report (S1, method version "A + B")

Real cost (mean of the tuning flights, 3-D RMS): **9.54 cm**; keep-best: round 0 (9.54 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1_dry_merged_usd.csv | 1 | ok | False | False | 17.72 | yes |
| A8 | A8_dry_merged_usd.csv | 1 | ok | False | False | 7.04 | yes |
| CIRCLE | CIRCLE_dry_merged_usd.csv | 1 | ok | False | False | 5.90 | yes |
| FIG8 | FIG8_dry_merged_usd.csv | 1 | ok | False | False | 5.25 | yes |
| A4 | A4_dry_merged_usd.csv | 1 | ok | False | False | 5.40 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.3048, -0.2894, 0.294, -0.2373, 0.0]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 0.7347, 'Pz': 1.466, 'Dxy': 1.3343, 'Dz': 1.2806, 'Iz': 1.566, 'KRxy': 1.4673, 'KRz': 0.8905, 'Kwxy': 0.9566, 'Kwz': 1.0711, 'KIxy': 0.9828}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 4.15 cm, best flown set 6.51 cm (37 s)
  A1      new: 11.3 cm | 11.5 cm                    best: 14.8 cm | 14.0 cm
  A8      new: 3.5 cm | 4.1 cm                      best: 7.9 cm | 9.5 cm
  CIRCLE  new: 1.8 cm | 1.8 cm                      best: 2.4 cm | 2.6 cm
  FIG8    new: 1.8 cm | 1.7 cm                      best: 2.4 cm | 2.4 cm
  A4      new: 2.1 cm | 1.9 cm                      best: 4.6 cm | 4.5 cm
```

Desk time of this command: 2185 s (ingest 1 s, fit 877 s, tune 1265 s).
