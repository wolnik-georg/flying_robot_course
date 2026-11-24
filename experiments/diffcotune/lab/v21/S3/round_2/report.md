# Round 2 report (S3, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **3.24 cm**; keep-best: round 2 (3.24 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 3.24 | yes |
| A8 | A8.csv | 1 | ok | False | False | 2.42 | yes |
| A4 | A4.csv | 1 | ok | False | False | 2.42 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 3.42 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 4.30 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [0.0139, 0.0354, 0.1878, -0.1376, -0.0556]
theta flown (factor to the default gains): {'Pxy': 0.9924, 'Pz': 1.0278, 'Dxy': 1.0111, 'Dz': 0.9883, 'Iz': 0.9142, 'KRxy': 1.0073, 'KRz': 0.9737, 'Kwxy': 1.0089, 'Kwz': 0.9972, 'KIxy': 1.0043}
theta of the next round (factor to this round): {'Pxy': 0.9114, 'Pz': 1.0493, 'Dxy': 1.1, 'Dz': 1.0911, 'Iz': 0.9852, 'KRxy': 1.0676, 'KRz': 1.0989, 'Kwxy': 0.9065, 'Kwz': 0.9421, 'KIxy': 1.0955}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 21, 'Pz': 0, 'Dxy': 26, 'Dz': 23, 'Iz': 0, 'KRxy': 8, 'KRz': 3, 'Kwxy': 23, 'Kwz': 0, 'KIxy': 23}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 5.80 cm, best flown set 6.46 cm (23 s)
  A1      new: 15.4 cm | 15.3 cm                    best: 16.2 cm | 16.4 cm
  A8      new: 5.1 cm | 4.7 cm                      best: 6.2 cm | 5.7 cm
  CIRCLE  new: 2.9 cm | 3.2 cm                      best: 2.8 cm | 3.3 cm
  FIG8    new: 2.5 cm | 2.1 cm                      best: 2.6 cm | 2.0 cm
  A4      new: 3.6 cm | 3.1 cm                      best: 4.3 cm | 5.1 cm
```

Desk time of this command: 2518 s (ingest 3 s, fit 1182 s, tune 1301 s).
