# Round 0 report (S2, method version "A + B")

Real cost (mean of the 4 usable tuning flights, 3-D RMS): **7.09 cm**; keep-best: none yet (no eligible set).

**R3: ['A1'] failed 3 attempts; fallback flight(s) with theta_best (round n/a, no theta_best yet) not used for the fit; this set is NOT eligible for keep-best.**

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1_dry_merged_usd.csv | 1 | ok | False | True | 28.87 | no |
| A1 | A1_dry_a2_merged_usd.csv | 2 | ok | False | True | 28.96 | no |
| A1 | A1_dry_a3_merged_usd.csv | 3 | ok | False | True | 29.11 | no |
| A8 | A8_dry_merged_usd.csv | 1 | ok | False | False | 9.09 | yes |
| CIRCLE | CIRCLE_dry_merged_usd.csv | 1 | ok | False | False | 6.15 | yes |
| FIG8 | FIG8_dry_merged_usd.csv | 1 | ok | False | False | 5.78 | yes |
| A4 | A4_dry_merged_usd.csv | 1 | ok | False | False | 6.85 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.0331, 0.1098, -0.0028, -0.0522]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 1.0175, 'Pz': 0.95, 'Dxy': 0.9769, 'Dz': 0.9037, 'Iz': 1.25, 'KRxy': 1.0598, 'KRz': 1.0283, 'Kwxy': 1.0705, 'Kwz': 0.9306, 'KIxy': 0.9341}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 24, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 5.62 cm, best flown set 6.84 cm (27 s)
  A1      new: 15.3 cm tilt+floor | 15.2 cm tilt+floor best: 16.8 cm tilt+floor | 16.6 cm tilt+floor
  A8      new: 5.4 cm | 4.5 cm                      best: 7.8 cm | 7.3 cm
  CIRCLE  new: 1.5 cm | 1.6 cm                      best: 2.5 cm | 2.6 cm
  FIG8    new: 1.4 cm | 1.4 cm                      best: 2.2 cm | 2.2 cm
  A4      new: 4.9 cm | 4.9 cm                      best: 5.2 cm | 5.2 cm
  events the best flown set shows too (not counted): A1 seed 2000 tilt, A1 seed 2000 floor, A1 seed 2001 tilt, A1 seed 2001 floor
```

Desk time of this command: 2235 s (ingest 2 s, fit 1055 s, tune 1140 s).
