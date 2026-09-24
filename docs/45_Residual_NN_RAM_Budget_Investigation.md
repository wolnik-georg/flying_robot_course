# 45 — Neural-Swarm2 onboard RAM budget: investigation, findings, and a plan for later

**Status: investigation and planning only. Nothing implemented, nothing flown, nothing changed
in any file that affects current flight builds or the next lab session.** `residual_nn` stays
exactly as it is today — a Cargo feature, default off, never enabled in any build that flies.
This document exists so the work already done (three sessions: initial scoping, a Cursor
investigation, and independent re-verification) isn't lost and doesn't need repeating.

**Date:** 2026-09-24 (desk-only, parallel to the C.1/A4 lab session).

---

## 1. The problem, in one paragraph

`firmware_app/src/residual_nn.rs` is a byte-faithful Rust port of the Neural-Swarm2 paper's
residual-force network (19297 `f32` weights, ~77KB). It's gated behind a Cargo feature
(`residual_nn`, default off) because the full firmware does not fit in RAM on the real embedded
target (STM32F4, Cortex-M4) with it enabled — confirmed by linker error, not estimated. This
blocks Strategy 2 (geometric + learned residual) from ever running on real hardware, independent
of whether trained weights exist yet (they don't — C.2 needs C.1's full data bank first). This
document is about *making the network fit*, not about training it.

**Standing operator rule, unchanged and not up for reinterpretation by this investigation**: the
network itself must never be modified to fix this. No pruning, no quantization, no dropping
`phi_L`/`rho_L`/`phi_G`, no layer-size changes. Quote (`docs/07_Thesis_Progress_Checklist.md`
History (29)): *"leave the architecture exactly as ported... it should work out of the box as it
is."* Whatever fix is pursued has to come from elsewhere — build configuration, memory layout,
or how weights are delivered to the chip — never from the network's math.

---

## 2. Verified baseline (reproduced three times independently, identical every time)

```
$ cd flying_drone_stack/firmware_app && make DRONE=bl

Flash |  393140/1032192 (38%),  639052 free | text: 382840, data: 10300, ccmdata: 0
RAM   |   94304/131072  (72%),   36768 free | bss: 84004, data: 10300
CCM   |   55792/65536   (85%),    9744 free | ccmbss: 55792, ccmdata: 0
```

**With `residual_nn` enabled** (requires a temporary edit to `firmware_app/Kbuild`'s hardcoded
`cargo build` line — it has no `--features` flag or env passthrough today; every test below
reverted this before finishing):

```
ld: region `RAM' overflowed by 40732 bytes
```

Reproduced identically in two separate sessions today. This is the number every approach below
is measured against.

---

## 3. Thread A — trim genuinely-unused subsystems (Kconfig, no code deleted)

**Method**: `build/.config` is gitignored scratch space (safe to edit for testing). Flip a
`CONFIG_*=y` to `# CONFIG_* is not set`, rebuild with the feature enabled, read the overflow
number off the linker, revert. Same mechanism this project already uses for
`CONFIG_CONTROLLER_OOT2/3/4` — nothing is deleted, code stays in the tree, only what's compiled
into *this particular test build* changes.

### Results (every number below independently reproduced by a second pass — exact matches)

| Config change | RAM overflow | Freed | Verified independently? |
|---|---:|---:|---|
| Baseline (nothing off) | 40732 | 0 | ✅ (×3) |
| `DECK_LIGHTHOUSE=n` alone | 37324 | 3408 | ✅ exact match |
| `DECK_LOCO=n` alone | 38396 | 2336 | ✅ exact match |
| `LIGHTHOUSE` + `LOCO` | 34996 | 5736 | ✅ exact match (two sessions) |
| `DECK_BUZZ=n` | 40724 | 8 | not re-checked |
| `DECK_LEDRING=n` | 40004 | 728 | not re-checked |
| `DECK_COLORLED=n` | 40652 | 80 | not re-checked |
| `ENABLE_CPX` + `ENABLE_CPX_ON_UART2=n` | 40732 | 0 | not re-checked |
| `DECK_AI=n` | 38844 | 1888 | not re-checked — **see caveat below** |
| `CONTROLLER_OOT2=n` alone | 40732 | 0 | ✅ exact match |
| `CONTROLLER_OOT3=n` alone | 40732 | 0 | not re-checked |
| `CONTROLLER_OOT4=n` alone | 40732 | 0 | not re-checked |
| `OOT2`+`OOT3`+`OOT4` combined | 40732 | 0 | ✅ exact match |
| `LH`+`LOCO`+`LEDRING` | 34268 | 6464 | not re-checked |
| `LH`+`LOCO`+`AI` | 33108 | 7624 | not re-checked |
| **Best combo: `LH`+`LOCO`+`LEDRING`+`AI`** | **32292** | **8440 (~21% of gap)** | not re-checked |
| Every free cut except `AI` (`LH`,`LOCO`,`LEDRING`,`BUZZ`,`COLORLED`,`CPX`) | 34180 | 6552 | not re-checked |

**Do not cut, confirmed genuinely used by this project**: `DECK_MULTIRANGER`, `DECK_OA`,
`DECK_FLOW`, `DECK_ZRANGER`/`ZRANGER2`, `DECK_RPM`, `DECK_ACTIVE_MARKER`, `DECK_USD` — safety/omap,
optical flow, RPM telemetry (both deck and DShot), mocap markers, and thesis logging all depend
on these.

**`DECK_AI` caveat**: disabling it is reasonable for a dedicated "mocap-only Neural-Swarm2"
firmware image, but this project's SLAM/perception work (`flying_drone_stack/CLAUDE.md`,
`SLAM_STATUS.md`) uses the AI Deck. Cutting it is a real functional tradeoff, not a free win —
only appropriate for a build variant where SLAM is explicitly out of scope for that flight.

**Stock controllers (PID/Mellinger/INDI/Brescianini/Lee)**: unlike our own `OOT*` slots, these
are **not** Kconfig-gated at all today — `controller.c` unconditionally includes and dispatches
all of them. Making them optional would require new conditional-compilation work (new Kbuild
`obj-$(CONFIG_...)` lines, `#ifdef` around `controller.c`'s dispatch table and includes, checking
every place that references them by name). Measured RAM footprint of their static state: on the
order of 1-2KB total (`indi`=316B, `indiOuter`=416B, `g_self` variants 160-384B each) — the same
order of magnitude as a single FreeRTOS task stack slice. **Not worth the implementation effort
for RAM alone** — this would be a flash/build-hygiene project, not a RAM fix.

### Thread A conclusion

**Ceiling: ~8.4KB (~21% of the 40.7KB gap), even in the most aggressive tested combination.**
Real, free, zero functional loss (except `AI`, which has a real tradeoff) — worth doing as
margin — but cannot close the gap by itself, by a wide margin.

---

## 4. Thread B — flash-resident weights (the one approach that can close the whole gap)

### 4.1 The idea

Today, trained weights are uploaded over radio into a **RAM** array
(`static mut RNN: ResidualNet`, `w: [f32; 19297]` in `.bss`) via CRTP params
(`g_rnn_wi`/`wv`/`wc` → `RNN.set_weight(...)`). This costs ~78KB of the scarce ~37KB-free RAM
budget.

The alternative: bake trained weights into the firmware image as a **read-only constant**
(`.rodata`), which lives in **flash** — 639KB free, no contest. The network reads its weights
from flash at inference time instead of from a RAM buffer that was populated by radio upload.

**This does not change the network's math, architecture, layer sizes, or weight count in any
way** — only where the numbers physically live and how they get onto the chip.

### 4.2 Independently verified: this is architecturally real on this exact target/toolchain

This was the one claim worth proving from scratch rather than trusting a description, since it's
the linchpin of the whole recommendation. Method: added a genuine 77KB `const` array (same size
as the real weight array) directly into `firmware_app/src/lib.rs`, built the complete real
firmware, checked the result, then fully reverted (`lib.rs` diffed clean against a backup,
rebuild matches baseline exactly).

**Result:**
```
Before (baseline, no extra array):  RAM 94304/131072, 36768 free
After (+77KB const array added):    RAM 94304/131072, 36768 free   <- UNCHANGED
                                     Flash 470324/1032192 (was 393140, +77184B)
Symbol address: 0x08059a7c   <- 0x08xxxxxx = STM32F4 FLASH, not 0x20xxxxxx RAM
```

**The 77KB array cost exactly zero bytes of RAM.** It landed entirely in flash, confirmed by
both the build's own RAM/flash accounting and the symbol's actual link address. This is not a
theoretical possibility — it's proven on the real linker, the real target, the real build
system this project actually uses.

### 4.3 What would actually need to change (scoped, not implemented)

| Area | What changes |
|---|---|
| **Weight storage** (`residual_nn.rs`) | `w: [f32; N_WEIGHTS]` moves from a mutable, upload-populated field to a `const`/`&'static [f32]` reference to a flash-resident blob, generated at build time from a trained `.npz`. |
| **Upload path** (`residual_nn.rs`, `traj_iface.c`) | `set_weight`/`begin_upload`/`finish_upload` either become no-ops on a flash-weights build, or are removed for that build variant (behind a second Cargo feature, e.g. `residual_nn,flash_weights`). |
| **`lib.rs`** | `static mut RNN` sizing changes; `rnn_service()` (the CRTP upload handler) becomes a stub or is conditionally compiled out. |
| **Host tests** (`test_residual_nn.py`, `tools/residual/test_pipeline.py`) | Need a second code path — inject known weights at build/link time to test the flash-resident path, alongside the existing upload-path test. |
| **Deployment workflow** | **This is the real tradeoff.** Today: retrain → upload over radio, no reflash needed. Flash-resident: retrain → regenerate the weight blob → rebuild firmware → reflash the drone. Every weight change becomes a full firmware update. For a thesis where the model is trained a handful of times and then flown for comparison, this is a minor inconvenience, not a blocker — but it is a real change to how iteration works, and should be a deliberate choice, not a surprise. |

### 4.4 Does this "touch the network"? — flagged for an explicit decision, not assumed

**Unchanged**: layer sizes, 19297 weights, phi/rho math, the neighbour gate, the ground-effect
term — the network computes exactly the same function it does today.

**Changed**: where the weights live, how they arrive on the chip, mutability, and yes — this
requires editing `residual_nn.rs`'s own code (the storage declaration and the upload
functions), even though it never touches a single line of the actual math
(`phi_forward`/`rho_forward`/`eval`).

This is a genuine, narrow question worth an explicit answer before anyone implements it: does
"never modify the network" mean "never change what it computes" (in which case this is fine —
nothing about its behavior changes), or "never touch this file for any reason" (in which case
this needs a different framing, e.g. weights injected via `lib.rs` through an opaque pointer,
which is more awkward given today's struct layout). **Not assumed either way in this
investigation** — this is the one open product decision, not a technical uncertainty.

---

## 5. Combined estimate

| Stage | Main RAM used | Free |
|---|---:|---:|
| Baseline, no `residual_nn` | 94304 | 36768 |
| + `residual_nn`, RAM-resident weights (today's design) | — | **overflow by 40732** |
| + Thread A best combo (−8440B used) | ~85864 | ~45208 (before adding any weights) |
| + Thread B (weights moved to flash, ~0B RAM cost instead of ~78KB) | ~85864 + (metadata only, <1KB) | **~44KB free — comfortable margin, not a tight fit** |

**Thread A alone never reaches a successful link.** Thread B alone (with no Thread A trimming)
already closes the gap with real margin — Thread A on top is extra headroom, not strictly
required, but cheap and worth doing regardless.

---

## 6. Recommendations, ranked

1. **If onboard Strategy 2 inference is pursued at all**: flash-resident weights (Thread B) is
   the only approach that closes the gap on its own, and it's now verified architecturally sound
   on this exact toolchain. The open item is the deployment-workflow tradeoff (§4.3) and the
   "does this count as touching the network" framing (§4.4) — both need your explicit sign-off
   before implementation, not a technical blocker.
2. **Do the Thread A trims regardless**, as margin: `LIGHTHOUSE` + `LOCO` off is free with zero
   functional loss for this project (neither is used). `LEDRING` similarly. `AI` only if a given
   flight profile genuinely doesn't need SLAM.
3. **Disable `OOT2`/`OOT3`/`OOT4`** for a dedicated "Neural-Swarm2-only" build if you want a
   smaller/cleaner flash image — confirmed zero RAM benefit, so this is about build hygiene, not
   the RAM problem.
4. **Do not pursue Kconfig-gating the stock controllers** — confirmed real but small win
   (~1-2KB), not worth the implementation cost for this problem.
5. **If onboard Strategy 2 is not pursued this semester**: current default (feature off,
   sim/host-only validation) is a legitimate, already-working fallback — nothing here is urgent.

---

## 7. What to do next, when there's time for it

This is a plan for a **future session**, not a to-do for now:

1. Get an explicit decision on §4.4's framing question.
2. If flash-resident weights is approved: scope the actual `residual_nn.rs`/`lib.rs` changes as
   a real patch (not just a plan), including the build-time `.npz` → `.rodata` generation step.
3. Extend `test_residual_nn.py`/`test_pipeline.py` for the new code path.
4. Re-verify the full combined build (Thread A trims + Thread B) actually links successfully
   end to end, not just as separate estimates.
5. Only then does this become a real candidate for the C.0 hardware-validation ladder — same
   discipline as any other new control-path change in this project.

---

## 8. Confirmed clean — nothing left modified

Every test in this investigation (Thread A's Kconfig flips, Thread B's probe array, the
temporary `Kbuild` feature patch) was reverted before moving to the next test, and the final
state was re-verified against the baseline in §2, byte-for-byte, at the end of every session
that touched this. `git status` on both `flying_robot_course` and `crazyflie-firmware` shows no
stray changes attributable to this investigation. **The next lab session's build is unaffected —
nothing in this document changes what gets flashed to `cf5` or `cf_second` today.**

---

## Related

- `docs/07_Thesis_Progress_Checklist.md` — History (29) for the original operator decision;
  History (51) for this investigation's summary entry.
- `flying_drone_stack/firmware_app/src/residual_nn.rs` — module docstring, corrected 2026-09-24
  to remove a stale example-fix that contradicted the standing decision.
- `docs/13_Residual_Learning.md` — the residual-learning design doc this feeds into.
- `docs/papers/summaries/shi2022neuralswarm2.md` — confirms the original paper also ran residual
  inference onboard the STM32 in real time (not offline) — only their multi-robot trajectory
  *planner* was host-side. Their own selling point: "small 32-bit microcontroller."
