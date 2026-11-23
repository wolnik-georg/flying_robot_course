# Round 1 report (S2, method version "A + B")

Real cost (mean of the 5 usable tuning flights, 3-D RMS): **6.48 cm**; keep-best: round 1 (6.48 cm).

| scenario | file | attempt | gate | crashed | floor | 3-D RMS cm | used |
|---|---|---|---|---|---|---|---|
| A1 | A1.csv | 1 | ok | False | False | 10.03 | yes |
| A8 | A8.csv | 1 | ok | False | False | 3.18 | yes |
| A4 | A4.csv | 1 | ok | False | False | 4.35 | yes |
| CIRCLE | CIRCLE.csv | 1 | ok | False | False | 6.26 | yes |
| FIG8 | FIG8.csv | 1 | ok | False | False | 6.41 | yes |

Test flights (not used for the update): {}

beta (log-factors) after the fit: [-0.1254, 0.0037, -0.0246, -0.3251]
theta flown (factor to the default gains): {'Pxy': 1.1716, 'Pz': 0.8694, 'Dxy': 0.8213, 'Dz': 1.0121, 'Iz': 1.25, 'KRxy': 1.25, 'KRz': 1.023, 'Kwxy': 1.0962, 'Kwz': 0.9498, 'KIxy': 0.8894}
theta of the next round (factor to this round): {'Pxy': 0.9107, 'Pz': 0.8125, 'Dxy': 1.2383, 'Dz': 1.0354, 'Iz': 1.25, 'KRxy': 1.25, 'KRz': 1.0606, 'Kwxy': 0.8179, 'Kwz': 0.8986, 'KIxy': 1.2191}
gain-limit hits per gain (of 51 evaluations): {'Pxy': 0, 'Pz': 5, 'Dxy': 0, 'Dz': 0, 'Iz': 26, 'KRxy': 18, 'KRz': 0, 'Kwxy': 0, 'Kwz': 0, 'KIxy': 0}

## Safety preview of the next gain set (frsim, fitted beta, 2 noise seeds; never changes the gains)

```
PREVIEW PASSED
predicted 3-D RMS (mean over the 5 tuning scenarios and 2 seeds): new set 2.82 cm, best flown set 5.68 cm (19 s)
  A1      new: 5.2 cm | 5.1 cm                      best: 13.6 cm | 13.3 cm floor
  A8      new: 1.6 cm | 1.6 cm                      best: 2.4 cm | 2.3 cm
  CIRCLE  new: 1.6 cm | 1.6 cm                      best: 2.5 cm | 2.6 cm
  FIG8    new: 1.8 cm | 1.7 cm                      best: 2.4 cm | 2.5 cm
  A4      new: 4.0 cm | 4.0 cm                      best: 7.1 cm | 8.1 cm
```

Desk time of this command: 1644 s (ingest 2 s, fit 770 s, tune 847 s).
