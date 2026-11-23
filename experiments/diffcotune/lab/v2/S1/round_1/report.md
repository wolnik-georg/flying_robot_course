# Round 1 report (S1, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **2.82 cm**; keep-best: round 1 (2.82 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 1.98 | yes |
| A8 | A8.csv | 1 | ok | False | False | 4.16 | yes |
| A4 | A4.csv | 1 | ok | False | False | 2.03 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 3.09 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 2.21 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.3496, -0.3364, -0.2402, -0.6963, -0.2321]
theta flown (factor to the default gains): {'Pxy': 1.2282, 'Pz': 0.8, 'Dxy': 1.25, 'Dz': 0.8001, 'Iz': 1.25, 'KRxy': 1.25, 'KRz': 1.25, 'Kwxy': 1.1963, 'Kwz': 1.197, 'KIxy': 1.25}
theta of the next round (factor to this round): {'Pxy': 0.9441, 'Pz': 0.8421, 'Dxy': 1.25, 'Dz': 1.25, 'Iz': 1.25, 'KRxy': 1.25, 'KRz': 0.9103, 'Kwxy': 0.9595, 'Kwz': 0.8, 'KIxy': 1.228}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 15, 'Dz': 25, 'Iz': 23, 'KRxy': 25, 'KRz': 0, 'Kwxy': 0, 'Kwz': 24, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 2.98 cm, best flown set 5.01 cm (24 s)
  A1      new: 6.7 cm | 7.0 cm                      best: 10.0 cm floor | 10.5 cm
  A8      new: 1.5 cm | 1.7 cm                      best: 3.3 cm | 3.2 cm
  CIRCLE  new: 2.4 cm | 2.5 cm                      best: 4.1 cm | 4.1 cm
  FIG8    new: 2.7 cm | 2.4 cm                      best: 4.0 cm | 3.8 cm
  A4      new: 1.5 cm | 1.3 cm                      best: 3.6 cm | 3.4 cm
```

Desk time of this command: 2138 s (ingest 3 s, fit 1004 s, tune 1103 s).
