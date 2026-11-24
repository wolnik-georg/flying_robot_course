# Round 3 report (S0, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **7.74 cm**; keep-best: round 3 (7.74 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 8.77 | yes |
| A8 | A8.csv | 1 | ok | False | False | 4.83 | yes |
| A4 | A4.csv | 1 | ok | False | False | 8.23 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 8.25 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 7.98 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.0704, 0.1877, 0.7379, -0.3134]
theta flown (factor to the default gains): {'Pxy': 0.748, 'Pz': 1.331, 'Dxy': 0.9405, 'Dz': 1.3266, 'Iz': 1.2461, 'KRxy': 1.2514, 'KRz': 0.9222, 'Kwxy': 1.0602, 'Kwz': 0.8696, 'KIxy': 0.8895}
theta of the next round (factor to this round): {'Pxy': 0.9, 'Pz': 1.1, 'Dxy': 1.0366, 'Dz': 1.0656, 'Iz': 1.1, 'KRxy': 1.0292, 'KRz': 0.9563, 'Kwxy': 1.0195, 'Kwz': 0.9196, 'KIxy': 1.0427}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 21, 'Pz': 26, 'Dxy': 0, 'Dz': 0, 'Iz': 10, 'KRxy': 0, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.56 cm, best flown set 7.10 cm (23 s)
  A1      new: 12.7 cm floor | 13.2 cm floor        best: 14.4 cm floor | 15.2 cm floor
  A8      new: 4.7 cm | 4.7 cm                      best: 5.1 cm | 5.1 cm
  CIRCLE  new: 3.8 cm | 3.9 cm                      best: 4.1 cm | 4.1 cm
  FIG8    new: 3.7 cm | 3.7 cm                      best: 3.9 cm | 3.9 cm
  A4      new: 7.6 cm floor | 7.7 cm floor          best: 7.6 cm floor | 7.5 cm floor
  events the best flown set shows too (not counted): A1 seed 2000 floor, A1 seed 2001 floor, A4 seed 2000 floor, A4 seed 2001 floor
```

Desk time of this command: 4276 s (ingest 3 s, fit 935 s, tune 3309 s).
