# 67 — RPM DShot sentinel: hold-last-good (prepared 2026-10-08)

**Status:** Code + host tests + log replay prepared; **not flashed**. Apply **after NS2 sign test**, **before** INDI “Omar + Iz” lab block (`docs/lab_session_pack_signtest.md` §4b).

**Scope:** `rpm_get_all()` DShot path only (`indi_gains.rpm_source != 0`). **Omar C** (controller 9) reads optical deck `rpm.m1..4` directly — unchanged. **Geometric / NS2** (`ctrl_mode=0`) does not use RPM for control.

---

## Facts

- DShot telemetry uses **`0xFFFF`** as invalid / no-value. Until 2026-10-08 firmware mapped sentinel → **0** before spike filtering; that zeroed one motor for one tick in INDI / Omar Rust thrust reconstruction.
- Existing DShot guards (2026-09-29, `docs/43`): reject **> 28 000 RPM** and **> 10 000 RPM** step vs last accepted sample; on reject **hold last good** (do not zero).
- Clean **2026-10-05 A8** uSD logs: **26–42** sentinel samples **per flight** across four motors (**~0.05–0.11 %** of motor RPM rows); almost all run length **1**; longest burst **5** ticks on one motor (`thesis00` 17-44-01, m2).
- Real hover RPM **22–24 k**; 28 k cap is conservative headroom.

---

## Change (summary)

| Input | Old output | New output (`rpm_source != 0`) |
|--------|------------|--------------------------------|
| `0xFFFF`, `rpm_prev > 0` | 0 | **hold `rpm_prev`** (`rpm_prev` unchanged) |
| `0xFFFF`, `rpm_prev == 0` | 0 | 0 (unchanged) |
| `0xFFFF`, deck (`rpm_source == 0`) | 0 | 0 (unchanged) |
| All other inputs | (spike filter as today) | **bit-identical** to pre-change logic |

Implementation: `flying_drone_stack/firmware_app/rpm_filter.h` (`rpm_filter_step`), called from `traj_iface.c` `rpm_get_all()`.

---

## Host tests

Run from repo:

```bash
flying_drone_stack/firmware_app/host/test_rpm_filter.sh 1   # refactor only (legacy 0→0 sentinel)
flying_drone_stack/firmware_app/host/test_rpm_filter.sh 2   # after sentinel change
```

| Gate | Result |
|------|--------|
| **Step 1 — bit-identical refactor** | **PASS** — extracted `rpm_filter_step` vs verbatim legacy on edge cases, jump sequences (9 999 / 10 000 / 10 001 RPM), **1×10⁶** random inputs per `rpm_source` 0/1; **0 mismatches** (run with sentinel → 0 in header). |
| **Step 2 — sentinel behaviour** | **PASS** — non-sentinel **5×10⁵** random inputs match legacy; DShot sentinel with `rpm_prev=19 000` → **19 000** (legacy → 0); deck sentinel still → 0; hover+sentinel sequences stay in **[0, 28 000]**. |

---

## Log replay (2026-10-05 A8, DShot channels)

Script: `experiments/analysis/rpm_sentinel_usd_quantify.py`  
Artifacts: `experiments/analysis/out/rpm_sentinel/usd_quantify.json`

Files: `cf5_A8_thesis00/01/02` at **17-44-01** and **19-17/19/23** (`experiments/logs/usd_raw/`).

Thrust proxy at sentinel ticks: **Σ kt·rpm²**, **kt = 4.1×10⁻¹⁰** (ours), comparing old (one motor forced to 0) vs new (held RPM).

| Flight | Sentinel motor-ticks | Max \|ΔRPM\| (any motor) | Max \|Δ thrust proxy\| @ sentinel |
|--------|----------------------|---------------------------|-----------------------------------|
| thesis00 17-44-01 | 28 | 17 667 | **0.128 N** |
| thesis00 19-17-04 | 26 | 18 382 | **0.139 N** |
| thesis01 17-44-01 | 42 | 19 379 | **0.154 N** |
| thesis01 19-19-27 | 29 | 17 241 | **0.122 N** |
| thesis02 19-23-26 | 41 | 19 305 | **0.153 N** |

Interpretation: at sentinel instants, old logic drops one motor’s thrust term to zero for one tick; new logic keeps the last hover RPM (~17–19 k), a **~0.10–0.15 N** correction on total thrust proxy — small vs hover (~0.4 N level) but aligned with “do not zero INDI” spike-filter philosophy.

---

## ARM build (scratch, no flash)

```bash
SCRATCH=$(mktemp -d)
rsync -a --exclude target --exclude build --exclude '*.o' \
  flying_drone_stack/firmware_app/ "$SCRATCH/"
cd "$SCRATCH" && make -j4
```

| Check | Result |
|-------|--------|
| CF21BL link | **PASS** — Flash **39 %**, RAM **75 %** (2026-10-08 scratch build) |
| Host bindings untouched | **PASS** — `~/Desktop/crazyflie-firmware/build/_cffirmware*.so` and `cffirmware_wrap.c` mtimes **unchanged** (2026-10-08 13:49:42 / 13:49:51) |

No Rust/SWIG interface change; **do not** rebuild host bindings for this patch alone.

---

## Recommendation

- **Apply on firmware** when flashing for the **Omar + Iz** block (after NS2 sign test completes), not before — NS2 geometric flights do not depend on RPM; avoids mixing variables on the sign-test day.
- **Risk (low):** holding stale RPM for 1–5 ticks if DShot stays invalid slightly longer than today’s zeros; mitigated by rarity and existing slew/abs caps on real samples.
- **Risk (theoretical):** if sentinel appeared with `rpm_prev` stale after a long dropout, hold could be wrong — not seen in 10-05 logs (sentinels occur in hover with recent good samples).
- **Not in this change:** Omar C deck path; clamping raw spikes when `rpm_prev==0`; logging which path produced each RPM (separate lab logging patch).

---

## References

- `docs/43_RPM_Source_Quality.md` (control path, spike guard)
- `docs/65_Omar_Z_Offset_Plan.md` (RPM source / filter coverage)


## Claude review and final form (2026-10-08)
- The sentinel hold-last-good is now behind a compile-time flag **`RPM_FILTER_HOLD_SENTINEL`, default 0** (`rpm_filter.h`): the **default firmware build behaves exactly as the firmware flown on 10-02/10-05** (test mode 1: 0 mismatches vs the verbatim legacy logic on edges, jump sequences and 10⁶ random inputs; scratch ARM build of the default: flash 402 372 B / text 391 536 B, identical to the build before this change). Enable with `-DRPM_FILTER_HOLD_SENTINEL=1` (test mode 2 passes) — only after the NS2 sign test and before the Omar + Iz INDI block.
- Reason: the NS2 sign test must run on unchanged firmware; geometric `ctrl_mode=0` does not use RPM for control anyway.


## In-flight check 2026-10-08 (flag ON, Omar Rust, 6 A8 flights)
184 raw DShot sentinels (0.057 % of airborne motor samples); controller thrust within 4 ticks of a sentinel: median 0.005 N, p95 0.013 N, max 0.029 N (zeroing a motor would be 0.10–0.15 N). No flight anomalies. Not covered: same-flight A/B with the flag off; the >28 000 RPM and >10 000 RPM-jump paths did not fire (no spikes in these flights). Omar C does not use this filter (deck RPM, no dropouts in 6 flights).
