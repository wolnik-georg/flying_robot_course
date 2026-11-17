# Round 0 report (S0, method version "A + B")

Real cost (mean of the 4 usable tuning flights, 3-D RMS): **8.43 cm**; keep-best: none yet (no eligible set).

**R3: ['A1'] failed 3 attempts; fallback flight(s) with theta_best (round n/a, no theta_best yet) not used for the fit; this set is NOT eligible for keep-best.**

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1_dry_merged_usd.csv | 1 | ok | False | True | 37.13 | no |
| A1 | A1_dry_a2_merged_usd.csv | 2 | ok | False | True | 37.23 | no |
| A1 | A1_dry_a3_merged_usd.csv | 3 | ok | False | True | 37.18 | no |
| A8 | A8_dry_merged_usd.csv | 1 | ok | False | False | 11.82 | yes |
| CIRCLE | CIRCLE_dry_merged_usd.csv | 1 | ok | False | False | 6.51 | yes |
| FIG8 | FIG8_dry_merged_usd.csv | 1 | ok | False | False | 6.15 | yes |
| A4 | A4_dry_merged_usd.csv | 1 | ok | False | False | 8.04 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.236, 0.0283, 0.1268, -0.1457]
theta flown (factor to the default gains): {'Pxy': 1.0, 'Pz': 1.0, 'Dxy': 1.0, 'Dz': 1.0, 'Iz': 1.0, 'KRxy': 1.0, 'KRz': 1.0, 'Kwxy': 1.0, 'Kwz': 1.0, 'KIxy': 1.0}
theta of the next round (factor to this round): {'Pxy': 1.218, 'Pz': 1.25, 'Dxy': 0.8, 'Dz': 1.114, 'Iz': 1.25, 'KRxy': 1.25, 'KRz': 0.9087, 'Kwxy': 1.1557, 'Kwz': 0.8, 'KIxy': 0.8}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 20, 'Dxy': 8, 'Dz': 0, 'Iz': 27, 'KRxy': 4, 'KRz': 0, 'Kwxy': 0, 'Kwz': 18, 'KIxy': 9}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 8.54 cm, best flown set 14.50 cm (27 s)
  A1      new: 17.1 cm floor | 17.1 cm floor        best: 37.7 cm tilt+floor | 37.8 cm tilt+floor
  A8      new: 7.0 cm | 6.7 cm                      best: 11.2 cm floor | 11.0 cm floor
  CIRCLE  new: 5.6 cm | 5.6 cm                      best: 6.8 cm | 6.8 cm
  FIG8    new: 5.3 cm | 5.3 cm                      best: 6.3 cm | 6.4 cm
  A4      new: 7.7 cm floor | 7.9 cm floor          best: 10.4 cm floor | 10.5 cm floor
  events the best flown set shows too (not counted): A1 seed 2000 floor, A1 seed 2001 floor, A4 seed 2000 floor, A4 seed 2001 floor
```

Desk time of this command: 1969 s (ingest 1 s, fit 910 s, tune 1025 s).
