# Round 4 report (S0, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **4.20 cm**; keep-best: round 4 (4.20 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 2.02 | yes |
| A8 | A8.csv | 1 | ok | False | False | 3.60 | yes |
| A4 | A4.csv | 1 | ok | False | False | 3.76 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 5.56 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 5.09 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.1046, -0.3049, -0.1731, -0.6517]
theta flown (factor to the default gains): {'Pxy': 1.2646, 'Pz': 1.5601, 'Dxy': 1.8478, 'Dz': 2.375, 'Iz': 2.4414, 'KRxy': 2.06, 'KRz': 0.9137, 'Kwxy': 0.9886, 'Kwz': 0.819, 'KIxy': 1.5989}
theta of the next round (factor to this round): {'Pxy': 1.25, 'Pz': 1.25, 'Dxy': 1.25, 'Dz': 1.2106, 'Iz': 1.25, 'KRxy': 1.2363, 'KRz': 1.25, 'Kwxy': 0.9957, 'Kwz': 1.2099, 'KIxy': 1.25}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 25, 'Pz': 28, 'Dxy': 25, 'Dz': 0, 'Iz': 27, 'KRxy': 19, 'KRz': 27, 'Kwxy': 0, 'Kwz': 12, 'KIxy': 25}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 0.68 cm, best flown set 0.98 cm (19 s)
  A1      new: 0.8 cm | 0.8 cm                      best: 1.2 cm | 1.2 cm
  A8      new: 0.8 cm | 0.8 cm                      best: 1.1 cm | 1.1 cm
  CIRCLE  new: 0.5 cm | 0.5 cm                      best: 0.7 cm | 0.6 cm
  FIG8    new: 0.4 cm | 0.4 cm                      best: 0.5 cm | 0.5 cm
  A4      new: 0.9 cm | 0.8 cm                      best: 1.5 cm | 1.4 cm
```

Desk time of this command: 1386 s (ingest 2 s, fit 654 s, tune 707 s).
