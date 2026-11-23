# Round 1 report (S3, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **3.65 cm**; keep-best: round 1 (3.65 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 4.19 | yes |
| A8 | A8.csv | 1 | ok | False | False | 2.43 | yes |
| A4 | A4.csv | 1 | ok | False | False | 2.71 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 4.02 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 4.43 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.0787, -0.0207, 0.0905, -0.1419, -0.1383]
theta flown (factor to the default gains): {'Pxy': 0.99, 'Pz': 0.99, 'Dxy': 1.0101, 'Dz': 0.99, 'Iz': 0.99, 'KRxy': 1.0101, 'KRz': 0.99, 'Kwxy': 0.99, 'Kwz': 1.0101, 'KIxy': 1.0101}
theta of the next round (factor to this round): {'Pxy': 0.9918, 'Pz': 0.9314, 'Dxy': 1.066, 'Dz': 1.0141, 'Iz': 1.0392, 'KRxy': 1.0599, 'KRz': 1.0518, 'Kwxy': 0.947, 'Kwz': 1.0166, 'KIxy': 0.9839}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 0, 'Dxy': 0, 'Dz': 0, 'Iz': 0, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 5.66 cm, best flown set 6.01 cm (23 s)
  A1      new: 15.5 cm | 15.4 cm                    best: 15.5 cm | 15.6 cm
  A8      new: 5.5 cm | 5.0 cm                      best: 6.2 cm | 5.7 cm
  CIRCLE  new: 2.2 cm | 2.9 cm                      best: 2.2 cm | 2.8 cm
  FIG8    new: 1.9 cm | 1.5 cm                      best: 1.9 cm | 1.5 cm
  A4      new: 3.8 cm | 3.0 cm                      best: 4.0 cm | 4.6 cm
```

Desk time of this command: 4911 s (ingest 3 s, fit 1177 s, tune 3699 s).
