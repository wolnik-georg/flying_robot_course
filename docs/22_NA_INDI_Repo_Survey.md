# NA-INDI repository survey

**What it is:** the authors' own code for Cobo-Briesewitz, Wahba & Hönig (2026), *Learned
Incremental Nonlinear Dynamic Inversion for Quadrotors with and without Slung Payloads*
(L4DC, arXiv 2503.09441v2). Received directly from the first author. Paper summary:
[`papers/summaries/cobobriesewitz2026lindi.md`](papers/summaries/cobobriesewitz2026lindi.md).

**Location:** `~/Desktop/NA-INDI` · `github.com/Tupryk/NA-INDI` · HEAD `00302a2`
("updated data analysis scripts and added online learning") · 691 MB, 864 files.

**Status of this survey:** read-only reconnaissance, 2026-09-10. Nothing in their repo was
modified. Their firmware branch was cloned separately, read-only, to `~/Desktop/NA-INDI-firmware`
— see §2b. **`~/Desktop/NA-INDI-firmware` is THEIRS; `~/Desktop/crazyflie-firmware` is the one
we build and flash. Never confuse the two.**

---

## 1. The headline finding — four "controllers" are a 2×2, not four implementations

`latex_results_generator.ipynb` selects the method by **two independent booleans**:

| Folder / method key | `using_nn` | `using_indi` | Paper label |
|---|---|---|---|
| `lee` | false | false | **Lee** (geometric baseline) |
| `indi` | false | true | **INDI** (classic, needs RPM) |
| `ilndi` | true | false | **IL-NDI** — learned residual, *no* INDI, *no* RPM at runtime |
| `naindi` | true | true | **NA-INDI** — learned residual **and** INDI on the leftover |

```python
if   method == "ilndi":  using_nn = True;  using_indi = False
elif method == "indi":   using_nn = False; using_indi = True
elif method == "naindi": using_nn = True;  using_indi = True
elif method == "lee":    using_nn = False; using_indi = False
```

So their contribution is a **clean 2×2 factorial ablation** over "learned residual on/off" ×
"INDI on/off" — not four separate control laws. That is architecturally the same shape as our
own strategy set (geometric / INDI / Geometric+NN / hybrid), and the same shape as our runtime
flags `indi_gains.ctrl_mode` × `rnn.en`. **This is the most useful single fact in the repo for
us**: it means their comparison and ours are structurally comparable, and their naming maps
onto our modes directly.

⚠️ **Naming trap:** the paper and this repo use **IL-NDI** (folder `ilndi`), *not* "L-INDI".
Our own cite key is `cobobriesewitz2026lindi` and the summary title says "LINDI / NA-INDI".
Keep the three apart: **LINDI/IL-NDI = NN only**; **INDI = RPM only**; **NA-INDI = both**.

---

## 2. What is present, and what is declared but missing

| Path | What it is |
|---|---|
| `LMCE/` | The whole Python side. Data prep, residual computation, MLP, trainers, model→C export, uSD log decode |
| `LMCE/c_utils/` | Hand-written C: `nn_utils.c/h`, `nn_main.c`, `online_nn_main.c`, `tree_utils.c/h`, `tree_main.c`, `nn_null.c/h` |
| `data_old/` | ~28 flight-data folders (the bulk of the 691 MB) — `circle_*`, `figure8_*`, `helix_*`, `payload_*`, `indi_no_payload`, `new_gains`, … |
| `*.ipynb` | `main`, `main_payload`, `main_tree`, `latex_results_generator` (+ a `copy`), `test` |
| `uav_trajectories` | Gitlink, commit `860e5b6` — **not checked out** |
| `create_null_net.py`, `data_standardization.py`, `pwm2thrust.py` | Standalone helpers |

**Not in this repo — but since located (2026-09-10):**

`.gitmodules` declares two submodules that **are not tracked in `HEAD` at all** (no gitlink
entry, directories absent):

```
[submodule "crazyflie-data-collection"] url = https://github.com/IMRCLab/crazyflie-data-collection.git
[submodule "crazyflie-firmware"]        url = ../crazyflie-firmware.git   branch = upstream_lee
```

The onboard controller C code is **not** in what we received. `git submodule update --init`
cannot recover it — the gitlinks were never committed.

**The declared URL is also a dead end.** `../crazyflie-firmware.git` is *relative*, and git
resolves it against the origin path — `github.com/Tupryk/NA-INDI.git` → **`github.com/Tupryk/
crazyflie-firmware.git`**. That fork exists but has only `master` and `dev-leePayload`; it has
**no `upstream_lee` branch at all**. So the submodule declaration cannot be satisfied as
written.

**Where the controller actually is — found by search, see §4.** Not on `upstream_lee` (that
branch has no NN code whatsoever), but on **`Tupryk/crazyflie-firmware`, branch
`dev-leePayload`** — the first author's own working branch, HEAD `ef2858fd` *("updated nn")*.
Cloned read-only to **`~/Desktop/NA-INDI-firmware`** (47 MB, deliberately named so it can
never be confused with our own `~/Desktop/crazyflie-firmware`).

`git submodule status` also errors (`no submodule mapping found ... for path
'uav_trajectories'`) — the repo's submodule config and tree disagree, so submodule commands
fail outright.

---

## 2b. The controller itself — `controller_lee.c` on `Tupryk/dev-leePayload`

**There is no new `ControllerType`.** The enum is stock (`PID / Mellinger / INDI /
Brescianini / Lee / Oot`). Everything lives **inside `controller_lee.c`**, selected by **two
`PARAM_UINT8` bitmasks**, both in the `ctrlLee` group:

| Param | Bit | Effect |
|---|---|---|
| `ctrlLee.use_nn` | `&1` | NN supplies **lateral** force residual → `a_nn.x`, `a_nn.y` |
| | `&2` | NN supplies **vertical** force residual → `a_nn.z` |
| | `&4` | NN supplies **torque** residual → `u_nn` (`nn_output[3..5]`) |
| `ctrlLee.indi` | `&1` | **force/position INDI** → `a_indi` |
| | `&2` | **moment/attitude INDI** → `tau_rpm` |

So the notebook's `using_nn` / `using_indi` booleans are a *simplification* for labelling
recorded flights — on the drone these are **finer-grained bitmasks**, and force vs torque and
lateral vs vertical can be enabled independently. The network has **6 outputs**: 3 force, 3
torque. Its input vector is ~19 wide and includes **motor PWM ratios** (`motorsGetRatio(0..3)`)
and gyro — so it is *not* RPM-free in the sense of using no actuator information; it uses
commanded PWM instead of measured RPM.

Files added on this branch: `src/modules/src/controller/nn.c` (generated weights) and
`nn_utils.c`. Also present, unrelated to this paper: `controller_lee_payload.c`,
`controller_rl.c`.

### ⚠️ Their residual sign is the sign that crashed us

```c
struct vec F_d = vsub2(a_d, a_indi, a_nn);   // vsub2(a,b,c) = a - b - c   (math3d.h:303)
a_indi = vsub(self->a_imu_filtered, self->a_rpm_filtered);   // measured - model
```

So their **published, working** law is `F_d = a_d − a_res − a_nn` — i.e. **exactly our
`.sub(a_indi).sub(a_nn)`**, the derivation-correct sign that we parked at `res_sign=+1` after
it crashed on 2026-09-09. That is meaningful independent evidence that the *sign* is right and
our failure has a different cause.

**And here is the difference that most plausibly explains why it works for them:**

| | NA-INDI (`controller_lee.c`) | Ours (`lib.rs`) |
|---|---|---|
| Clamp before differencing | `vclampnorm(a_rpm, 10)` **and** `vclampnorm(a_imu, 10)` | none |
| Filter before differencing | 2nd-order Butterworth, **80 Hz, on both sides** | none |
| Resulting `a_res` | difference of two clamped, filtered signals | **raw instantaneous difference** |

Our `a_res` is `a_meas.sub(a_model)` computed fresh every tick with no filter and no bound,
and it feeds `f_d` → `thrust_vec` → `desired_rot()` → `Rd` → the attitude command. Theirs is
heavily conditioned first. **An unfiltered, unclamped residual driving the commanded attitude
is exactly the noise/lag path suspected on 09-09** — and this is the concrete, published
counter-design.

**Also: they structurally prevent double-counting.** When both are on, `a_nn` is folded into
the *model* side before the INDI difference is taken:

```c
self->a_rpm = vadd( ...model..., a_nn);      // NN prediction added to the model
a_indi = vsub(a_imu_filtered, a_rpm_filtered);  // INDI then sees only the leftover
```

So INDI corrects only what the network failed to predict — *that is what "NA-INDI" means*. Our
implementation applies both terms independently (`.sub(a_indi).sub(a_nn)`), and our own code
comment admits double-counting "is not prevented here, because preventing it would remove the
comparison". Their design is the alternative worth measuring against ours in C.3.

---

## 3. The residual they learn — identical formalism to ours

`LMCE/residual_calculation.py`:

```python
f_a = total_mass * acc_world - rowan.rotate(q, f_u)          # no payload
f_a = total_mass * acc_world - rowan.rotate(q, f_u) - T_p    # with payload (T_p = cable tension)
```

That is **exactly our `indi.a_res_*`**, in force rather than acceleration units:

$$ f_a = m\,a_{\text{meas}} - R\,f_{\text{thrust}} \qquad\Longleftrightarrow\qquad a_{\text{res}} = a_{\text{meas}} - a_{\text{model}} = f_a/m $$

**But the disturbance source is different.** Theirs is a **slung point-mass payload** (and a
no-payload case for the plain residual); ours is **another vehicle's downwash**. Same
formalism, different physics, different input features. Their no-payload numbers are the
comparable ones for us.

Notable methodological detail: residuals are computed on **offline-smoothed** data
(`SplineFitter`, `make_smooth`, `spline_segments`) — they fit splines to `omega` and
differentiate the spline rather than the raw signal. That is a training-data quality step we
do not currently do; our `a_res` is the raw onboard value.

---

## 4. The learned model

`LMCE/models.py`:

- **`MLP`** — `Linear → LeakyReLU` stack, default hidden `[24, 24, 24]`, all biases
  zero-initialised (comment: *"Improves Torque Z output"*).
- **`DTE`** — a decision-tree variant (`main_tree.ipynb`, `tree_utils.c`). A second model
  family, not just the MLP.

**Comparison with ours:** a plain MLP over a single vehicle's own state. Ours is a
**deep-sets** network (φ/ρ, permutation-invariant, 987 weights) over *neighbour* relative
states — necessarily different, because our residual depends on other vehicles and theirs does
not. Their architecture is not directly reusable for us; their *pipeline* is.

**`LMCE/model_to_c_conversion.py`** is the closest analogue to our
`tools/residual/` + `rnn.*` upload path, and is worth reading properly:
- `exportNet()` writes a generated `nn.c`
- `nn_c_model_test()` compiles it with `gcc` and **numerically compares C output against the
  PyTorch output** (`compare_outputs`, `err_thresh=1e-4`)
- same pattern for the tree (`exportTree`, `tree_c_model_test`)

That is the same verify-the-C-against-the-reference discipline as our
`host/test_residual_nn.py`, arrived at independently. Their route is *code generation*
(weights baked into generated C, reflash to change); ours is *runtime upload* (`rnn.wi/wv/wc`
over CRTP, no reflash). Ours is more flexible; theirs is simpler and has no upload protocol to
get wrong.

**`online_nn_main.c`** — `add_dataset_entry()` + `backprop()` running in C. HEAD's commit
message says "added online learning". This is **onboard/at-runtime training**, which neither
the paper's main claims nor our thesis currently covers. Worth understanding before assuming
the repo matches the published method exactly.

---

## 5. What this repo is useful for, concretely

1. **A published counter-design for the exact bug we are stuck on.** Their working law uses
   our "fixed" sign, but conditions the residual first (clamp + 80 Hz Butterworth on both
   sides) and folds the NN into the model so INDI sees only the leftover. That is a concrete,
   flight-proven alternative to test — see §2b.
2. **Their 2×2 is a template for our comparison table.** Same ablation structure, already
   validated in a published paper. Worth aligning our reporting to it where honest.
3. **`model_to_c_conversion.py`'s numerical C-vs-PyTorch check** — independent confirmation
   that our own equivalent test is the right discipline, and a second implementation to
   compare against if ours ever disagrees.
4. **The spline-smoothing step on residual training data** is a concrete technique we do not
   currently use and could evaluate for C.2.
5. **`data_old/` is real flight data from a published method** — usable as a sanity target for
   our own analysis tooling, though it is single-drone-with-payload, not formation.
6. **Their no-payload results are the directly comparable baseline** to our single-drone case.

**What it is NOT useful for:** the onboard controller is not in *this* repo — it is in the
separately cloned `~/Desktop/NA-INDI-firmware` (§2b). Their network is also not reusable for
us: it predicts a single vehicle's own payload/aero residual from its own state, while ours
must depend on neighbours.

---

## 6. Open questions before this repo can be used further

1. ~~Fetch the firmware to read the controller~~ — **DONE 2026-09-10.** Not `upstream_lee`
   (no NN code there at all); the controller is on `Tupryk/crazyflie-firmware`, branch
   `dev-leePayload`. Cloned read-only to `~/Desktop/NA-INDI-firmware`. See §2b.
2. ~~What do the flags map to on the drone~~ — **ANSWERED.** Two `PARAM_UINT8` bitmasks,
   `ctrlLee.use_nn` (bits: lateral / vertical / torque) and `ctrlLee.indi` (bits: force /
   moment), both inside stock `controller_lee.c`. No new `ControllerType`.
3. **What are the MLP's input features?** `input_size` is a constructor argument and the call
   sites are inside notebooks; not yet traced. This matters — it is the main design decision
   we would compare against.
4. **Is the online-learning code (`online_nn_main.c`) part of the paper's method or work after
   it?** HEAD post-dates the arXiv v2; treat as after unless shown otherwise.
5. **`data_old/` naming suggests a `data/` that is not present** — the results notebook reads
   `data/figure8_online`, which does not exist in the tree. The published results may not be
   reproducible from this snapshot alone.

---

## 7. Repository status for the thesis

NA-INDI is now a **reference repository** for this thesis — read-only, third-party, never
modified and never pushed to. It sits alongside:

| Repo | Ours? | Role |
|---|---|---|
| `flying_robot_course` | yes, pushed | thesis, firmware source, logs, docs |
| `crazyswarm2` | yes, pushed | flight scripts, configs |
| `crazyflie-firmware` | no — upstream, local mods only | bitcraze base, patch-preserved |
| **`NA-INDI`** | **no — third-party reference** | **authors' code for a paper we cite. Read-only** |

Same rule as `crazyflie-firmware`: **do not fork, do not push, do not modify.** If we ever
need changes to run it, they belong in our repo as a script or patch, not in their tree.
