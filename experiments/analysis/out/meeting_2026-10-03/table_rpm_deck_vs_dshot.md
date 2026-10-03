# Table — deck vs DShot RPM (airborne-clean, Hampel Δ filter)

## Formulas (metrics on glitch-removed aligned samples, deck & DShot > 0)
- **bias** = mean(s − d) [RPM]; **bias %** = 100 · mean(s − d) / mean(d)
- **RMSE** = √(mean((s − d)²)) [RPM]
- **Pearson r** between d and s
- **lag τ** [ms] = argmax cross-correlation of mean-removed d,s over ±50 ms; **positive = DShot later**

## Airborne window (unchanged)
- Raw deck > 8000 RPM and raw DShot > 8000 RPM; contiguous first→last.
- Exclude RPM < 5000 for > 0.5 s while z > 0.3 m, or airborne segment < 25 s.

## Glitch removal (Hampel on Δ = DShot − deck)
- Flag if |Δ − median(Δ, centred 21-sample window)| > max(600 RPM, 6.0 · 1.4826 · MAD(Δ)), or |Δ| > 10000, or DShot ≥ 60000.
- Remove flagged samples from **both** series (gaps in plots); metrics on remaining samples.
- **Lag:** linear interpolation of both series at removed times, then ±50 ms cross-correlation.

## Old |Δ|>10k rule vs Hampel (included motor-rows)
- Total removed samples: **254** (Hampel+rules) vs **245** (|Δ|>10k only).
- Removed on top-1% |d(deck)/dt| samples: **1** (0.4% of removals) — should stay low so real transients remain.

### Figure BIAS — largest |bias %| (-0.32 %)
- `cf_second_thesis47_2026-10-02_18-52-26` cf_second m1: bias **-0.32 %**, RMSE **105.0**, r **0.942**, lag **2.00 ms**; removed DShot-high **3**, deck **0** (old rule would remove **3**)

### Figure LAG — largest |lag| with rolling-lag IQR < 2 ms (IQR=0.00 ms)
- `cf_second_thesis55_2026-10-02_19-18-12` cf_second m3: bias **-0.19 %**, RMSE **98.2**, r **0.977**, lag **2.00 ms**; removed DShot-high **7**, deck **1** (old rule would remove **7**)

Fleet median lag: **2.00 ms** — traces nearly identical; systematic lag **≈2–5 ms**.

## Exclusions

## Fleet table (airborne-clean only)

| flight | vehicle | motor | n | removed | old rule | DShot-hi | deck | fast-deck | bias % | RMSE | r | lag | IQR |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| cf_second_A8_thesis00_2026-10-03_13-17-25 | cf_second | 1 | 13623 | 8 | 8 | 8 | 0 | 0 | -0.17 | 103.2 | 0.861 | 4.00 | 4.00 |
| cf_second_A8_thesis00_2026-10-03_13-17-25 | cf_second | 2 | 13620 | 11 | 11 | 11 | 0 | 0 | -0.25 | 106.4 | 0.830 | 2.00 | 0.00 |
| cf_second_A8_thesis00_2026-10-03_13-17-25 | cf_second | 3 | 13625 | 6 | 6 | 6 | 0 | 1 | -0.11 | 96.8 | 0.823 | 2.00 | 2.00 |
| cf_second_A8_thesis00_2026-10-03_13-17-25 | cf_second | 4 | 13620 | 11 | 11 | 11 | 0 | 0 | -0.25 | 120.7 | 0.828 | 2.00 | 4.00 |
| cf_second_A8_thesis01_2026-10-03_13-17-26 | cf_second | 1 | 13629 | 11 | 11 | 11 | 0 | 0 | -0.16 | 100.3 | 0.815 | 2.00 | 4.00 |
| cf_second_A8_thesis01_2026-10-03_13-17-26 | cf_second | 2 | 13626 | 14 | 14 | 14 | 0 | 0 | -0.27 | 107.6 | 0.784 | 2.00 | 0.00 |
| cf_second_A8_thesis01_2026-10-03_13-17-26 | cf_second | 3 | 13629 | 11 | 10 | 10 | 1 | 0 | -0.16 | 98.4 | 0.779 | 2.00 | 2.00 |
| cf_second_A8_thesis01_2026-10-03_13-17-26 | cf_second | 4 | 13629 | 11 | 11 | 11 | 0 | 0 | -0.24 | 120.7 | 0.777 | 0.00 | 4.00 |
| cf_second_thesis43_2026-10-02_18-28-10 | cf_second | 1 | 13549 | 8 | 8 | 8 | 0 | 0 | -0.29 | 103.3 | 0.937 | 2.00 | 2.00 |
| cf_second_thesis43_2026-10-02_18-28-10 | cf_second | 2 | 13547 | 10 | 10 | 10 | 0 | 0 | -0.16 | 100.7 | 0.918 | 4.00 | 4.00 |
| cf_second_thesis43_2026-10-02_18-28-10 | cf_second | 3 | 13553 | 4 | 4 | 4 | 0 | 0 | -0.24 | 99.7 | 0.962 | 0.00 | 0.00 |
| cf_second_thesis43_2026-10-02_18-28-10 | cf_second | 4 | 13550 | 7 | 7 | 7 | 0 | 0 | -0.17 | 108.0 | 0.906 | 0.00 | 3.00 |
| cf_second_thesis44_2026-10-02_18-28-10 | cf_second | 1 | 13485 | 3 | 3 | 3 | 0 | 0 | -0.25 | 101.8 | 0.926 | 0.00 | 2.00 |
| cf_second_thesis44_2026-10-02_18-28-10 | cf_second | 2 | 13477 | 11 | 11 | 11 | 0 | 0 | -0.21 | 103.5 | 0.923 | 2.00 | 4.00 |
| cf_second_thesis44_2026-10-02_18-28-10 | cf_second | 3 | 13478 | 10 | 9 | 9 | 1 | 0 | -0.18 | 95.4 | 0.939 | 0.00 | 2.00 |
| cf_second_thesis44_2026-10-02_18-28-10 | cf_second | 4 | 13479 | 9 | 9 | 9 | 0 | 0 | -0.21 | 109.2 | 0.885 | -2.00 | 4.00 |
| cf_second_thesis47_2026-10-02_18-52-26 | cf_second | 1 | 13560 | 3 | 3 | 3 | 0 | 0 | -0.32 | 105.0 | 0.942 | 2.00 | 2.00 |
| cf_second_thesis47_2026-10-02_18-52-26 | cf_second | 2 | 13556 | 7 | 6 | 6 | 1 | 0 | -0.25 | 107.3 | 0.932 | 2.00 | 2.00 |
| cf_second_thesis47_2026-10-02_18-52-26 | cf_second | 3 | 13559 | 4 | 4 | 4 | 0 | 0 | -0.27 | 103.1 | 0.956 | 2.00 | 2.00 |
| cf_second_thesis47_2026-10-02_18-52-26 | cf_second | 4 | 13555 | 8 | 8 | 8 | 0 | 0 | -0.18 | 109.5 | 0.918 | -2.00 | 4.00 |
| cf_second_thesis48_2026-10-02_18-52-26 | cf_second | 1 | 13545 | 6 | 6 | 6 | 0 | 0 | -0.28 | 102.6 | 0.901 | 2.00 | 2.00 |
| cf_second_thesis48_2026-10-02_18-52-26 | cf_second | 2 | 13541 | 10 | 6 | 6 | 4 | 0 | -0.24 | 106.9 | 0.893 | 2.00 | 2.00 |
| cf_second_thesis48_2026-10-02_18-52-26 | cf_second | 3 | 13549 | 2 | 2 | 2 | 0 | 0 | -0.27 | 104.4 | 0.915 | 2.00 | 2.00 |
| cf_second_thesis48_2026-10-02_18-52-26 | cf_second | 4 | 13542 | 9 | 9 | 9 | 0 | 0 | -0.22 | 109.2 | 0.859 | 0.00 | 2.00 |
| cf_second_thesis54_2026-10-02_19-18-11 | cf_second | 1 | 13508 | 7 | 7 | 7 | 0 | 0 | -0.31 | 112.5 | 0.951 | 0.00 | 2.00 |
| cf_second_thesis54_2026-10-02_19-18-11 | cf_second | 2 | 13511 | 4 | 3 | 3 | 1 | 0 | -0.12 | 102.0 | 0.951 | 2.00 | 4.00 |
| cf_second_thesis54_2026-10-02_19-18-11 | cf_second | 3 | 13508 | 7 | 7 | 7 | 0 | 0 | -0.22 | 98.6 | 0.963 | 2.00 | 2.00 |
| cf_second_thesis54_2026-10-02_19-18-11 | cf_second | 4 | 13504 | 11 | 11 | 11 | 0 | 0 | -0.17 | 109.2 | 0.939 | 0.00 | 4.00 |
| cf_second_thesis55_2026-10-02_19-18-12 | cf_second | 1 | 13525 | 5 | 5 | 5 | 0 | 0 | -0.29 | 105.5 | 0.974 | 0.00 | 2.00 |
| cf_second_thesis55_2026-10-02_19-18-12 | cf_second | 2 | 13520 | 10 | 10 | 10 | 0 | 0 | -0.14 | 101.6 | 0.968 | 2.00 | 2.00 |
| cf_second_thesis55_2026-10-02_19-18-12 | cf_second | 3 | 13522 | 8 | 7 | 7 | 1 | 0 | -0.19 | 98.2 | 0.977 | 2.00 | 0.00 |
| cf_second_thesis55_2026-10-02_19-18-12 | cf_second | 4 | 13522 | 8 | 8 | 8 | 0 | 0 | -0.17 | 108.4 | 0.965 | -2.00 | 4.00 |

**Summary (n=32):** bias % median -0.22 (IQR -0.26…-0.17); RMSE median 103.9 RPM (IQR 101.4…108.1).
