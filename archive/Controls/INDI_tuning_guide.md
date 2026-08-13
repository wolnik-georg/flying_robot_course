# INDI + Position Gain Tuning Guide

## What each parameter controls

### `fc_bw` — filter cutoff on angular acceleration (alp)

INDI feeds back measured angular acceleration (`alp`). The raw signal (`alp_raw`) is noisy, so it is low-pass filtered to `alp` before use.

| Direction | Effect |
|-----------|--------|
| Higher fc_bw | Less filtering, faster/more accurate feedforward, but tau chatters |
| Lower fc_bw | Cleaner signal but adds phase lag → feedforward goes stale, INDI degrades toward PD |

**Rule:** raise `fc_bw` until `tau` visibly chatters in the `_indi_panel.png`, then back off ~20%.
Phase lag (reported by `analyze_indi.py`) should stay under `τ_dom / 4`.

Current: **25 Hz** → ~10 ms lag, noise reduction ≈ 0 dB (barely filtering) → safe to try 50 Hz.

---

### `KR` — attitude stiffness

```
ωn = sqrt(KR)     ← natural frequency of attitude loop
```

- Higher → faster attitude correction, shorter settling time
- Too high → oscillations, noise amplification, tau saturation
- Current: KR=1325 → ωn=36.4 rad/s

---

### `KW` — attitude damping

```
ζ      = KW / (2·ωn)    ← damping ratio
τ_dom  ≈ KW / KR        ← dominant time constant (settling time, overdamped)
```

- Higher → smoother, less overshoot, longer settling
- **ζ target: 1.0–1.5** (critical to slightly overdamped). Below 1 = oscillation; above 2 = very slow.
- Current: KW=114 → ζ=1.57, τ_dom=86 ms — overdamped, safe, room to go faster.
- In incremental mode (RPM deck active): Routh stability does NOT require KW > KR.

---

### `KP` — position stiffness

```
pos bandwidth ≈ sqrt(KP)   [rad/s]
```

- Higher → drone chases position error harder, reduces lag behind reference
- Too high → overshoot, oscillation around trajectory
- **Bandwidth rule:** pos bandwidth should be 3–5× trajectory frequency and 5–10× below attitude ωn
- Current: KP=28 → bandwidth ≈ 5.3 rad/s (figure-8 at ~1 rad/s → 5× ratio ✓)

---

### `KV` — position damping

```
ζ_pos = KV / (2·sqrt(KP))    ← position loop damping
critical damping: KV ≈ 2·sqrt(KP)
```

- Higher → damps oscillation around reference, reduces cross-track error
- Too low → position loop underdamped → drone overshoots reference
- Current: KP=28, KV=6 → ζ_pos=0.57 — **underdamped**. Target KV=9–10.

---

## Tuning order — always inner before outer

```
1. fc_bw   →  raise until tau chatters, back off 20%
2. KR/KW   →  raise KR to speed up, keep ζ ≈ 1.0–1.3, verify in hover first
3. KV      →  raise toward 2·sqrt(KP) for critical damping
4. KP      →  raise only after attitude loop is faster and KV is set
```

If the attitude loop is slow or shaky, raising position gains makes it worse — inner loop always first.

---

## Reading `analyze_indi.py` output

| Output field | What it means | Action |
|---|---|---|
| `noise reduction` ≈ 0 dB | Filter barely active — signal already clean | fc_bw can be raised |
| `noise reduction` < −6 dB | Filter is working hard (noisy signal) | Keep fc_bw, don't raise |
| `phase lag` > τ_dom/4 | Filter delay degrading feedforward | Lower fc_bw |
| `tau_sat` < 5% | Large headroom | Can raise KR |
| `tau_sat` > 20% | Approaching saturation | Do NOT raise KR further |
| `t_shift` in analyze_flight | Along-track lag → attitude too slow | Raise KR (reduce τ_dom) |
| Geom RMSE ≈ Poly4D RMSE | Shape and timing both imperfect | Both loops need work |
| Geom RMSE << Poly4D RMSE | Shape good, timing off | Attitude loop (KR/KW) |

---

## Baseline — June 15 2026 (first INDI flights, fc_bw=25 Hz)

**Active gains:** KR=1125, KW=122, KP=28, KV=6, fc_bw=25 Hz
- ωn=33.5, ζ=1.82, τ_dom=108 ms

| Flight | XY RMSE | Along-track t_shift | Peak tau | Tau sat |
|--------|---------|--------------------|---------:|--------|
| Hover | — | — | 2.6 mN·m | 0% |
| Figure-8 kt0.1 (5.9s lap) | 9.3 cm | +70 ms | 3.8 mN·m | 0% |
| Figure-8 kt0.2 (4.91s lap) | 12.9 cm | −10 ms | 6.6 mN·m | 0% |

## Baseline — June 16 2026 (fc_bw raised to 80 Hz, same gains)

**Active gains:** KR=1125, KW=122, KP=28, KV=6, fc_bw=80 Hz

| Flight | XY RMSE | Along-track t_shift | Peak tau | Tau sat |
|--------|---------|--------------------|---------:|--------|
| Hover | — | — | ~2.0 mN·m | 0% |
| Figure-8 kt0.1 (5.9s lap) | 7.1 cm | +40 ms | ~4.1 mN·m | 0% |
| Figure-8 kt0.2 (4.91s lap) | 12.9 cm | −20 ms | ~4.5 mN·m | 0% |

Key observations:
- Hover Roll RMS improved 0.78°→0.53°, Pitch 0.59°→0.48°
- kt0.1 RMSE improved 9.3→7.1 cm; t_shift halved 70→40 ms — fc_bw was contributing lag
- kt0.2 unchanged — at that speed attitude loop speed (τ_dom=108 ms) is the bottleneck, not filter
- Filter at 80 Hz: noise reduction ≈ 0 dB, phase lag ≈ 10 ms → clean, no chatter
- Along-track lag +40 ms ≈ τ_dom/2.7 → **KR is now the primary bottleneck**
- KV=6 gives ζ_pos=0.57 (underdamped) → cross-track oscillation still present

---

## Recommended next steps

### Step 1 — Raise KR/KW (hover verify first)

From KR=1125, KW=122 (τ_dom=108 ms), step ~25% at a time:

| Step | KR | KW | ζ | τ_dom | Expected t_shift |
|---|---|---|---|---|---|
| **Step 1** | **1400** | **128** | 1.71 | 91 ms | ~+25 ms |
| Step 2 | 1700 | 138 | 1.67 | 81 ms | ~+15 ms |
| Step 3 | 2000 | 150 | 1.68 | 75 ms | ~+10 ms |

Verify in hover after each step: att RMS stays <1°, tau_sat stays <20%.

### Step 2 — Raise KV (no hover verify needed, pure damping)
```
KV=6 → KV=9   →  ζ_pos=0.85  (near-critical, first try)
KV=6 → KV=10  →  ζ_pos=0.94  (critical damping)
```
Reduces cross-track oscillation. No stability risk.

### Step 3 — fc_bw (optional, low urgency)
```
fc_bw=25 Hz → fc_bw=50 Hz
```
Current 0 dB noise reduction means the signal is already clean above 25 Hz.
Verify tau stays smooth after the raise.

### Step 4 — KP raise (after Steps 1+2)
```
KP=35, KV=11  →  pos BW=5.9 rad/s, ζ_pos=0.93
```
Only meaningful once τ_dom is shorter (attitude loop faster).

---

---

## How to read the plots for fc_bw tuning

Three plots, each answering a different question.

### 1. `_indi_dashboard.png` → Row 1 tau time series — **chatter check (primary)**

Look at the tau_x, tau_y, tau_z time traces.

- **fc_bw too high**: rapid high-frequency oscillations riding on the smooth curve — looks like "grass" or a fuzzy line
- **fc_bw too low**: tau is very smooth but attitude overshoots (see plot 3)
- **just right**: clean smooth curve following the maneuver, no visible oscillations

Also check the **tau PSD** (lower panels in Row 1). A clean system rolls off at high frequencies. If noise gets in, you'll see an elevated plateau at high frequencies instead of a clean rolloff.

### 2. `_indi_dashboard.png` → Row 2 (alp_raw vs alp) — **filter lag check**

Gray = raw angular acceleration, green = filtered (what INDI actually uses).

- **fc_bw too low**: green visibly lags behind gray — you can see it shifted right in time. `analyze_indi.py` terminal reports this as `phase lag ≈ X ms`. Keep lag under ~20 ms (τ_dom/4).
- **fc_bw too high**: green ≈ gray, barely any difference — filter is transparent
- **just right**: green follows gray closely, slight smoothing, no visible time shift

Terminal line to read:
```
X (roll): noise reduction=-3.5 dB   phase lag≈12 ms
```
Noise reduction more negative = filter doing more work. Phase lag increasing = more delay being introduced.

### 3. `_analysis_axes.png` → Attitude row (roll/pitch/yaw) — **overshoot check**

Compare solid actual line against dashed planned line.

- **Overshoot (fc_bw too low or KW too low)**: actual exceeds planned at peaks then comes back — solid line goes above dashed at the turns
- **Lag (fc_bw too low or KR too low)**: actual arrives consistently late at each peak
- **just right**: actual tracks planned closely, same peak amplitude, minimal phase offset

### Overshoot diagnosis

If you see overshoot in the attitude plots, even though ζ=1.57 should be overdamped:
- The theoretical ζ assumes perfect INDI inner loop
- Filter lag makes the effective ζ lower than the formula predicts — feedforward is stale, controller overcorrects
- **Root cause is usually fc_bw too low, not KR too high**
- Fix: raise fc_bw first. If overshoot persists even at high fc_bw, then raise KW.
- Do NOT raise KR when there is overshoot — that makes it worse.

### Decision logic after each flight

```
tau smooth?
  NO  →  fc_bw too high, back off 10–15 Hz

phase lag > 20 ms?
  YES →  fc_bw too low, raise it

overshoot in attitude axes plot?
  YES + lag also high  →  fc_bw too low (lag is causing it)
  YES + lag fine       →  KW too low (true underdamping, not filter problem)
  NO                   →  fc_bw is in the right range
```

### Supervisor-recommended approach: start high, step down

Start at **fc_bw = 80 Hz** and step down rather than starting low and going up. This finds the noise floor empirically:

```
80 Hz  →  hover: check tau for chatter, check alp_raw vs alp lag
           figure-8: check attitude overshoot in axes plot
60 Hz  →  repeat checks
40 Hz  →  repeat checks
25 Hz  →  (current baseline)
```

The optimum is the highest fc_bw where tau stays smooth (no chatter). With the current 0 dB noise reduction at 25 Hz, 50–80 Hz is very likely clean — but you need the flight data to confirm.

---

## Lab session order

```
1.  Set KR=1722, KW=137 in YAML
2.  Hover → analyze_indi.py  (check att RMS <1°, tau_sat <20%)
3.  Figure-8 kt0.1 → analyze_flight.py  (read new RMSE, check t_shift)
4.  If clean: set KV=9, re-fly kt0.1, compare RMSE
5.  If still clean: KR=2120, KW=160
6.  Later: KP=35, KV=11 once attitude is faster
```
