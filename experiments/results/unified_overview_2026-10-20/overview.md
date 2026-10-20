### A8 swap (dz 0.5, 0.5 m/s, 4 crossings per flight)
| strategy | flights | mean dip [cm] | std of the flight means | worst crossing | z RMS [cm] | motors max |
|---|---|---|---|---|---|---|
| S0 geometric | 4 | **-12.5** | 0.4 | -15.1 | 3.8 | 76–80 % |
| S1 INDI | 4 | **-8.4** | 2.8 | -13.9 | 3.1 | 100–100 % |
| S2 geometric + NS2 | 4 | **-5.6** | 0.7 | -9.3 | 1.8 | 79–82 % |
| S3 INDI + NS2 | 5 | **-3.8** | 0.9 | -8.6 | 2.1 | 100–100 % |

### A1 stacked hold (dz 0.5, hold 15 s, window 2–15 s)
| strategy | flights | z error mean [cm] | std of the flight means | z RMS [cm] | max roll/pitch | motors max |
|---|---|---|---|---|---|---|
| S0 geometric | 4 | **-17.9** | 2.4 | 18.2 | 42° | 92–100 % |
| S1 INDI | 4 | **-2.0** | 2.2 | 3.1 | 22° | 100–100 % |
| S2 geometric + NS2 | 6 | **-9.0** | 1.6 | 10.5 | 42° | 90–100 % |
| S3 INDI + NS2 | 4 | **+2.6** | 2.1 | 5.1 | 24° | 100–100 % |

### A4 offset line (dz 0.5, 0.5 m/s): z RMS [cm], mean ± std over flights (n)
| offset | S0 geometric | S1 INDI | S2 geometric + NS2 | S3 INDI + NS2 |
|---|---|---|---|---|
| 0.05 m | 17.2 ± 1.2 (n=3) | 5.0 ± 0.5 (n=3) | 11.7 ± 1.7 (n=3) | 3.8 ± 0.8 (n=3) |
| 0.10 m | 10.5 ± 1.1 (n=3) | 2.8 ± 0.1 (n=3) | 5.0 ± 1.5 (n=3) | 3.2 ± 0.5 (n=3) |
| 0.15 m | 6.2 ± 1.5 (n=3) | 2.4 ± 0.6 (n=3) | 2.3 ± 0.9 (n=3) | 2.7 ± 1.1 (n=3) |
