# Round 1 report (S3, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **3.99 cm**; keep-best: round 1 (3.99 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 2.88 | yes |
| A8 | A8.csv | 1 | ok | False | False | 5.03 | yes |
| A4 | A4.csv | 1 | ok | False | False | 2.58 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 4.38 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 4.49 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.0099, 0.0489, 0.1885, -0.138, -0.0413]
theta flown (factor to the default gains): {'Pxy': 0.99, 'Pz': 0.99, 'Dxy': 1.0101, 'Dz': 0.99, 'Iz': 0.99, 'KRxy': 1.0101, 'KRz': 0.99, 'Kwxy': 0.99, 'Kwz': 1.0101, 'KIxy': 1.0101}
theta of the next round (factor to this round): {'Pxy': 1.0024, 'Pz': 1.0382, 'Dxy': 1.001, 'Dz': 0.9982, 'Iz': 0.9233, 'KRxy': 0.9973, 'KRz': 0.9835, 'Kwxy': 1.019, 'Kwz': 0.9873, 'KIxy': 0.9943}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 1, 'Dxy': 0, 'Dz': 0, 'Iz': 24, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.47 cm, best flown set 6.38 cm (26 s)
  A1      new: 15.9 cm | 16.6 cm                    best: 15.9 cm | 16.6 cm
  A8      new: 6.2 cm | 5.7 cm                      best: 6.1 cm | 5.6 cm
  CIRCLE  new: 2.9 cm | 3.3 cm                      best: 2.7 cm | 3.2 cm
  FIG8    new: 2.6 cm | 2.1 cm                      best: 2.4 cm | 1.9 cm
  A4      new: 4.3 cm | 5.1 cm                      best: 4.7 cm | 4.8 cm
```

Desk time of this command: 2633 s (ingest 3 s, fit 1253 s, tune 1336 s).
