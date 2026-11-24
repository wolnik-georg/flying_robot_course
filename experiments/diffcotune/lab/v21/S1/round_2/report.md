# Round 2 report (S1, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **17.05 cm**; keep-best: round 1 (3.18 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 3.05 | yes |
| A8 | A8.csv | 1 | ok | False | False | 17.81 | yes |
| A4 | A4.csv | 1 | ok | False | False | 8.56 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 23.22 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 22.69 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.479, -0.9935, -0.3914, -1.0531, -0.7577]
theta flown (factor to the default gains): {'Pxy': 1.1704, 'Pz': 0.81, 'Dxy': 1.21, 'Dz': 0.99, 'Iz': 1.21, 'KRxy': 1.21, 'KRz': 1.21, 'Kwxy': 0.922, 'Kwz': 1.21, 'KIxy': 1.21}
theta of the next round (factor to this round): {'Pxy': 1.0141, 'Pz': 1.1, 'Dxy': 1.1, 'Dz': 0.9, 'Iz': 1.1, 'KRxy': 0.92, 'KRz': 0.9, 'Kwxy': 0.9, 'Kwz': 0.9, 'KIxy': 1.1}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 40, 'Dxy': 40, 'Dz': 40, 'Iz': 41, 'KRxy': 3, 'KRz': 40, 'Kwxy': 40, 'Kwz': 40, 'KIxy': 31}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 10.95 cm, best flown set 20.74 cm (25 s)
  A1      new: 22.8 cm floor | 21.1 cm floor        best: 29.0 cm floor | 28.5 cm floor
  A8      new: 8.7 cm floor | 8.6 cm floor          best: 19.2 cm floor | 19.1 cm floor
  CIRCLE  new: 8.0 cm | 8.1 cm                      best: 18.5 cm | 18.6 cm
  FIG8    new: 8.2 cm | 8.1 cm                      best: 18.7 cm | 18.6 cm
  A4      new: 8.2 cm floor | 7.7 cm floor          best: 18.9 cm floor | 18.4 cm floor
  events the best flown set shows too (not counted): A1 seed 2000 floor, A1 seed 2001 floor, A8 seed 2000 floor, A8 seed 2001 floor, A4 seed 2000 floor, A4 seed 2001 floor
```

Desk time of this command: 2130 s (ingest 3 s, fit 1001 s, tune 1096 s).
