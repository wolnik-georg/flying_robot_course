# Citation baseline — 27 August 2026

**Status: CLOSED.** Every work cited by the thesis as of this date has been audited against the
source PDF, and the corrections applied to the `.tex`.

**Source of truth remains the `.tex`, not this file.** This file exists so the next audit is a
**diff, not a restart**.

---

## Frozen files

SHA256 at freeze. Five chapter files and the bibliography carry every citation and every number
about prior work; the `*_standalone.tex` files are thin wrappers that `\input` those chapters and
contain no claims of their own — they are hashed for completeness, not because they need auditing.

| File | SHA256 |
|---|---|
| `ch1_introduction.tex` | `b407746bb8673a1e3aaeb1ab38d2b1d688725b50352a83588fff76d0bbff06c7` |
| `ch1_standalone.tex` | `48d5a382b42f2f2f859abf502632a6b02d69a827d623f1705e0f1be4a924c5de` |
| `ch2_related_work.tex` | `2fad78b514d10507f590d3bd19347da09f43f3e1cd61a9098c7febafd651063b` |
| `ch2_standalone.tex` | `3375d3e79abd6e2f06d29872741f66d1b198bd67a2850f1280348d4d72238e31` |
| `ch3_standalone.tex` | `ccdea274e4c588975b42df2d7ff6346c328ff378be67656695702d67429e1c59` |
| `ch3_system_modelling.tex` | `96a01ce36d3f9d88069b9b94173dde79edeb6239d8f5e300c6c49d0493e3f075` |
| `ch4_control_architectures.tex` | `17bce0b793a4b812e451f85f22c9cdcc4e553ce8bf42a237f56b6f58e9acec1d` |
| `ch4_standalone.tex` | `fa5123b44b5b1a24f9af0ed8b486864ec9ada2d33af1015d427380d215aeb6e7` |
| `ch5_experimental_setup.tex` | `e15890f4d705da21cc834a93fea4556b38e6f84d1db129a1006798fbd22c73aa` |
| `ch5_standalone.tex` | `6d546276ebbc954f299b010b4d28b75a5074bde4a4fb01c3914265087b6792ab` |
| `docs/references.bib` | `1fcb0536ec0d4b6bd3ef349ca1cd01afbb0c69aa7b0abe8e5f0f61a03b5b0583` |

**At freeze:** 41 pages, 0 undefined citations, 0 undefined references, 0 overfull boxes.

---

## Frozen cite keys

Twenty keys, every one audited against its PDF. One key per line, sorted. No claims recorded here
by design.

```
abro2025fractional
bauersfeld2024airflow
chee2024knodedwmpc
cobobriesewitz2026lindi
gielis2023aggregate
gu2025comparative
hsieh2025onlineadaptation
hsieh2026flatness
kiran2025downwash
lee2010geometricse3
lee2010geometrictracking
li2023ndpnmpc
shankar2023docking
shi2020neuralswarm
shi2022neuralswarm2
smeur2017cascadedindi
smith2023so2equivariant
tal2018indi
yang2025flatnessresiduals
zhang2024proxfly
```

---

## Locked papers

**Do not reopen any of these unless the `.tex` sentence that cites them changes.** Listed by the
actual cite key.

| Key | Locked |
|---|---|
| `shi2020neuralswarm` | ✔ |
| `shi2022neuralswarm2` | ✔ |
| `hsieh2026flatness` | ✔ |
| `hsieh2025onlineadaptation` | ✔ |
| `cobobriesewitz2026lindi` | ✔ |
| `smith2023so2equivariant` | ✔ |
| `chee2024knodedwmpc` | ✔ |
| `gielis2023aggregate` | ✔ |
| `gu2025comparative` | ✔ |
| `abro2025fractional` | ✔ |
| `zhang2024proxfly` | ✔ |
| `tal2018indi` | ✔ |
| `smeur2017cascadedindi` | ✔ |
| `lee2010geometrictracking` | ✔ |
| `lee2010geometricse3` | ✔ |
| `kiran2025downwash` | ✔ |
| `bauersfeld2024airflow` | ✔ |
| `shankar2023docking` | ✔ |
| `yang2025flatnessresiduals` | ✔ |
| `li2023ndpnmpc` | ✔ |

---

## Rule for the next audit

1. `sha256sum` the frozen files and compare against the table above.
2. **Hash unchanged → that file is still baseline. Skip it entirely.**
3. **Hash changed →** diff it against this baseline. Only sentences that are **new or edited**
   *and* that contain a `\cite` or a number about prior work go into a verification pack.
4. **New cite key →** a new pack, checked against the PDF.
   **Old key, unchanged sentence → do not reopen it.**

```bash
cd docs/thesis && sha256sum ch*.tex ../references.bib
```

**The manuscript sources were brought into git on 27 August 2026 and tagged
`citation-audit-2026-08-27`**, so step 3 is a real diff:

```bash
git diff citation-audit-2026-08-27 -- docs/thesis docs/references.bib
```

The hashes above remain valid as an offline cross-check and as the record for anyone working from
a copy outside the repository. **Only the manuscript sources are tracked** --- the paper library
(`docs/papers/`), the archived citation notes and the compiled PDF stay out of git.

---

## What this file is not

Not a literature matrix. Not a claim ledger. **Do not copy claims into it.** The superseded
matrix, ledger, structure note and review brief are archived at
`docs/_archive/citation_notes/` and are not sources of truth; where they disagree with the `.tex`,
the `.tex` wins.
