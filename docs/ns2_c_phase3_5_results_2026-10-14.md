# NS2 in C on the Omar C tree — Phase 3, 4, 5 results (final build, 2026-10-14)

Build: `crazyflie-firmware` with `firmware_patches/` (README order, incl. `ns2_residual_module.patch`); ARM image `…_ns2c.bin` sha256 `dfac761a…2d5`. Nothing flashed, nothing flown.

## Phase 3 — `rnn.en = 0` is bit-identical to the old Omar C (`host/test_omar_ns2_off_identical.py`)
Replay of logged flights through the same worker (`indi_replay_equal_worker.py`, side `omar_c`), old build (backup before the hook) against the final build, byte-for-byte comparison of the output CSV:
| flight | indi | rows | sha256 old = new |
|---|---|---|---|
| A8 2026-10-02 | 3 | 25 655 | a300021ed3b0 |
| A8 2026-10-02 | 0 | 25 655 | 404a9d4373d3 |
| A1 | 3 | 15 037 | 9466b5a18dea |
| A1 | 0 | 15 037 | 81e66789889c |
Result: **ALL IDENTICAL**. (Also in the closed-loop SIL: NS2-off run bit-identical to the old build once two host-only artifacts were removed, see below.)

## Phase 4 — C against the Rust flight code and NumPy (`host/test_ns2_c_equivalence.py`, tolerance 1e-5 relative)
ALL PASS: weights bit-for-bit from the npz (sha256 embedded); C vs NumPy 3 000 random cases, worst relative difference 2.3e-6; **C vs the Rust flight path 400 stateful random cases, worst difference 0 (bit-identical)**; gate boundaries (dx, dy, dvx both sides; dz, dvy, dvz not gated); ±8 m/s² clamp; non-finite handling (mass 0, NaN/overflow states) equal to Rust; peer-history rules (first seen, same ts, normal, gap ≥ 500 ms, backwards, recovery) equal to Rust; decimation at the first call and every 10th main tick (100 Hz).

## Phase 5 — closed-loop two-drone SIL (`experiments/analysis/omar_ns2_closed_loop_sil.py`)
Bottom drone Omar C (indi 0 / 3, Kpos_Iz 1.5, cmd gain 1.14), top drone our geometric (downwash source), plant = the same downwash network as the controller's (perfect prediction = upper bound); scale = real force / predicted force; A8, 4 crossings; dip = lowest z error relative to the steady offset.
| indi | NS2 | scale | dip [cm] | offset [cm] |
|---|---|---|---|---|
| 0 | off | 1.0 / 1.1 / 1.3 / 1.5 | −0.4 / −1.1 / **−11.5** / **−15.3** | 0.7 / 1.3 / 2.9 / 3.9 |
| 0 | on | 1.0 / 1.1 / 1.3 / 1.5 | +0.1 / 0.0 / −0.2 / −0.2 | 0.3 |
| 3 | off | 1.0 / 1.3 | +0.7 / +0.8 | 0.1 |
| 3 | on | 1.0 / 1.3 | +0.5 / +0.5 | 0.2 |
Reading: with the real force at or above the prediction, NS2 removes the geometric Omar's crossing dip (direction/sign correct: a wrong sign would enlarge it). Full INDI shows no dip in this simulation with or without NS2 (the SIL gives it perfect RPM information; on the hardware Omar INDI dips ≈ −9 cm), so **the SIL cannot show whether S3 helps**. The reversed-force negative control (scales −1.0 … −2.0) was inconclusive (almost no response in either direction, asymmetric to the +1.3 response; not explained) and is not used as evidence. The sign is fixed by construction: the C module subtracts `a_nn` exactly like the Rust path with `res_sign −1`, and the two are bit-identical (Phase 4).

## Two host-only artifacts found and removed (hardware unaffected)
1. The host's `usecTimestamp()` stub advances a fake clock by 2000 µs per call; the hook's two timing calls changed Omar INDI's `dt` in simulation. Timing code is now `#ifndef UNIT_TEST_MODE` (kept on the drone).
2. `rnn.ready` is a shared host global: the hook setting it made the top drone's Rust controller call the same clock. The hook now publishes `rnn.ready` only in firmware builds (at init and in the update), and the host `ns2_active()` does not depend on it.
One transient closed-loop crash (full INDI, NS2 off, scale 1.1) occurred in the build with artifact 1 and did not recur in 28 later runs; attributed to the artifact, not proven.
