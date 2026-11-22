# Round 3 report (S0, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **5.42 cm**; keep-best: round 3 (5.42 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 4.22 | yes |
| A8 | A8.csv | 1 | ok | False | False | 5.52 | yes |
| A4 | A4.csv | 1 | ok | False | False | 4.68 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 6.32 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 6.04 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.1568, -0.203, 0.0739, -0.5424]
theta flown (factor to the default gains): {'Pxy': 1.0117, 'Pz': 1.2481, 'Dxy': 1.4782, 'Dz': 1.9173, 'Iz': 1.9531, 'KRxy': 1.648, 'KRz': 0.7309, 'Kwxy': 0.8069, 'Kwz': 0.6633, 'KIxy': 1.2791}
theta of the next round (factor to this round): {'Pxy': 1.25, 'Pz': 1.25, 'Dxy': 1.25, 'Dz': 1.2387, 'Iz': 1.25, 'KRxy': 1.25, 'KRz': 1.25, 'Kwxy': 1.2252, 'Kwz': 1.2347, 'KIxy': 1.25}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 26, 'Pz': 24, 'Dxy': 26, 'Dz': 0, 'Iz': 27, 'KRxy': 27, 'KRz': 24, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 26}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 1.06 cm, best flown set 1.59 cm (18 s)
  A1      new: 1.1 cm | 1.3 cm                      best: 1.8 cm | 1.8 cm
  A8      new: 1.4 cm | 1.3 cm                      best: 1.8 cm | 1.8 cm
  CIRCLE  new: 0.6 cm | 0.6 cm                      best: 0.8 cm | 0.7 cm
  FIG8    new: 0.4 cm | 0.4 cm                      best: 0.5 cm | 0.5 cm
  A4      new: 1.8 cm | 1.7 cm                      best: 3.0 cm | 3.0 cm
```

Desk time of this command: 1373 s (ingest 2 s, fit 645 s, tune 704 s).
