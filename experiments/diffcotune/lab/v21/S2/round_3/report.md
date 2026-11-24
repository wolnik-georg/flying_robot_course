# Round 3 report (S2, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **7.35 cm**; keep-best: round 3 (7.35 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 9.80 | yes |
| A8 | A8.csv | 1 | ok | False | False | 4.25 | yes |
| A4 | A4.csv | 1 | ok | False | False | 5.45 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 7.85 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 8.03 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.2685, -0.3768, 0.2227, -0.5558]
theta flown (factor to the default gains): {'Pxy': 0.9027, 'Pz': 0.8859, 'Dxy': 0.729, 'Dz': 1.0337, 'Iz': 1.331, 'KRxy': 1.2915, 'KRz': 0.7291, 'Kwxy': 1.3253, 'Kwz': 1.0308, 'KIxy': 0.7434}
theta of the next round (factor to this round): {'Pxy': 1.1, 'Pz': 0.9789, 'Dxy': 0.9, 'Dz': 1.1, 'Iz': 1.1, 'KRxy': 1.081, 'KRz': 0.9, 'Kwxy': 1.0633, 'Kwz': 0.9, 'KIxy': 0.9805}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 35, 'Pz': 0, 'Dxy': 40, 'Dz': 40, 'Iz': 41, 'KRxy': 0, 'KRz': 35, 'Kwxy': 0, 'Kwz': 39, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 6.15 cm, best flown set 6.51 cm (19 s)
  A1      new: 13.9 cm | 14.1 cm                    best: 13.9 cm floor | 13.9 cm
  A8      new: 2.4 cm | 2.4 cm                      best: 2.9 cm | 2.8 cm
  CIRCLE  new: 3.1 cm | 3.1 cm                      best: 3.8 cm | 3.8 cm
  FIG8    new: 3.5 cm | 3.5 cm                      best: 4.0 cm | 3.9 cm
  A4      new: 7.8 cm | 7.7 cm                      best: 8.1 cm | 7.9 cm
```

Desk time of this command: 4157 s (ingest 3 s, fit 3291 s, tune 838 s).
