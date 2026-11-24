# Round 1 report (S1, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **3.18 cm**; keep-best: round 1 (3.18 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 2.99 | yes |
| A8 | A8.csv | 1 | ok | False | False | 5.08 | yes |
| A4 | A4.csv | 1 | ok | False | False | 2.47 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 2.33 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 2.06 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.1696, -0.6697, -0.1943, -0.7161, -0.6335]
theta flown (factor to the default gains): {'Pxy': 1.1, 'Pz': 0.9, 'Dxy': 1.1, 'Dz': 0.9, 'Iz': 1.1, 'KRxy': 1.1, 'KRz': 1.1, 'Kwxy': 1.0245, 'Kwz': 1.1, 'KIxy': 1.1}
theta of the next round (factor to this round): {'Pxy': 1.064, 'Pz': 0.9, 'Dxy': 1.1, 'Dz': 1.1, 'Iz': 1.1, 'KRxy': 1.1, 'KRz': 1.1, 'Kwxy': 0.9, 'Kwz': 1.1, 'KIxy': 1.1}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 40, 'Dxy': 41, 'Dz': 41, 'Iz': 41, 'KRxy': 41, 'KRz': 40, 'Kwxy': 40, 'Kwz': 40, 'KIxy': 40}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 3.91 cm, best flown set 4.88 cm (23 s)
  A1      new: 9.3 cm | 9.5 cm                      best: 10.5 cm | 10.4 cm
  A8      new: 2.0 cm | 2.1 cm                      best: 2.6 cm | 2.7 cm
  CIRCLE  new: 2.7 cm | 2.8 cm                      best: 3.8 cm | 4.0 cm
  FIG8    new: 2.7 cm | 2.5 cm                      best: 3.6 cm | 3.5 cm
  A4      new: 2.9 cm | 2.8 cm                      best: 3.8 cm | 3.8 cm
```

Desk time of this command: 1922 s (ingest 2 s, fit 760 s, tune 1129 s).
