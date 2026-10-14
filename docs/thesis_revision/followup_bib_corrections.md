# Follow-up 2 — Bibliography corrections checked against the held PDFs

`docs/references.bib` was **not edited**. For each of the 11 rows of `proposed_references.md` §B this file gives the
version held, the venue data the evidence supports, whether the content the thesis cites is in that version, and an
entry ready to paste. Evidence levels: **PDF** = printed on the held PDF; **ref-list** = from the reference list of
another held paper (secondary; named); **open** = not settleable locally. The text extracts used are in the session
scratchpad (`pdftotext -layout docs/papers/*.pdf`).

`\cite` keys are unchanged in every proposal, so the chapters need no edit. `plain` ignores unknown fields
(`yearnote`, `file`), so they can stay.

---

### 1. `shi2022neuralswarm2`
- **Held:** arXiv:2012.05457v2 (16 Jul 2021), running header "IEEE TRANSACTIONS ON ROBOTICS" (PDF p. 1).
- **Venue:** IEEE T-RO **38(2):1063–1079**; DOI **10.1109/TRO.2021.3098436** (ref-list: `cobobriesewitz2026lindi`, which
  gives 2022). The year is printed as 2022 (Cobo-Briesewitz, ProxFly), 2021 (Smith, Chee, Hsieh 2025: "Apr. 2021") and
  2020 (Gu). Volume 38 is the 2022 volume; 2021 is the online-first year.
- **Content:** every cited statement was checked against v2 (R02). v2 is the T-RO-formatted version, but whether it
  is identical to the published version is **open**.
```bibtex
@article{shi2022neuralswarm2,
  title   = {Neural-Swarm2: Planning and Control of Heterogeneous Multirotor Swarms using Learned Interactions},
  author  = {Guanya Shi and Wolfgang Hönig and Xichen Shi and Yisong Yue and Soon-Jo Chung},
  journal = {IEEE Transactions on Robotics},
  volume  = {38},
  number  = {2},
  pages   = {1063--1079},
  year    = {2022},
  doi     = {10.1109/TRO.2021.3098436},
  note    = {arXiv:2012.05457},
  file    = {papers/shi2022neuralswarm2.pdf},
}
```

### 2. `cobobriesewitz2026lindi`
- **Held:** arXiv:2503.09441v2 (14 Apr 2026); header "Proceedings of Machine Learning Research vol vvv:1–15, 2026 —
  8th Annual Conference on Learning for Dynamics and Control" (**PDF** p. 1). The PMLR volume number is a placeholder
  ("vvv") → **open**.
- **Content:** the cited Table 1, Figs. 5–6 and §1/§4/§5 statements are in v2 (R07). Whether v1 (2025) had Table 1 was
  not checked; cite v2/L4DC.
```bibtex
@inproceedings{cobobriesewitz2026lindi,
  title     = {Learned Incremental Nonlinear Dynamic Inversion for Quadrotors with and without Slung Payloads},
  author    = {Eckart Cobo-Briesewitz and Khaled Wahba and Wolfgang Hönig},
  booktitle = {Proceedings of the 8th Annual Conference on Learning for Dynamics and Control (L4DC)},
  series    = {Proceedings of Machine Learning Research},
  year      = {2026},
  note      = {arXiv:2503.09441v2},
  file      = {papers/cobobriesewitz2026lindi.pdf},
}
```
*(Add `volume` once PMLR assigns it.)*

### 3. `tal2018indi`
- **Held:** arXiv:1809.04048**v2** (7 Jun 2020), which states "A preliminary version of this article was presented at
  57th [IEEE CDC]" (**PDF** p. 1, footnote; text line 70).
- **Venue of the held version:** IEEE T-CST **29(3):1203–1218, 2021**, DOI **10.1109/TCST.2020.3001117** (ref-list:
  `cobobriesewitz2026lindi`). The CDC 2018 version is pp. 4282–4288 (ref-list: another held paper's references).
- **Content:** the cited material (response analysis §IV, eqs. 18–31, 609 g vehicle, 12.9 m/s, encoders, Butterworth
  30 Hz) is in v2 (R13; "12.9 m/s" occurs 4× in the text). Whether all of it is in the CDC paper is **open**. The thesis
  cites content of the extended version, so the T-CST entry is the right one.
```bibtex
@article{tal2018indi,
  title   = {Accurate Tracking of Aggressive Quadrotor Trajectories using Incremental Nonlinear Dynamic Inversion and Differential Flatness},
  author  = {Ezra Tal and Sertac Karaman},
  journal = {IEEE Transactions on Control Systems Technology},
  volume  = {29},
  number  = {3},
  pages   = {1203--1218},
  year    = {2021},
  doi     = {10.1109/TCST.2020.3001117},
  note    = {Extended version of the CDC 2018 paper; arXiv:1809.04048v2},
  file    = {papers/tal2018indi.pdf},
}
```
*(The key keeps "2018" for stability; renaming it would change \cite keys, which H6 forbids.)*

### 4. `lee2010geometricse3`
- **Held:** arXiv:1003.2005**v4** (9 Sep 2011) (**PDF** p. 1). The stability results are Propositions 3 and 4 (text
  lines 366, 372).
- **Content:** Ψ, $e_R$, $e_\Omega$, the torque law, the position-mode thrust and the stability regions are in v4 (R15).
  The bib `note` says the checks were made against v1, but only v4 is held.
```bibtex
@misc{lee2010geometricse3,
  title        = {Control of Complex Maneuvers for a Quadrotor UAV using Geometric Methods on SE(3)},
  author       = {Taeyoung Lee and Melvin Leok and N. Harris McClamroch},
  year         = {2010},
  howpublished = {arXiv:1003.2005},
  eprint       = {1003.2005},
  archivePrefix= {arXiv},
  primaryClass = {math.OC},
  note         = {Companion of the CDC 2010 paper; version 4 (2011) held and used for all equation checks
                  (attitude error function eq. 6, errors eqs. 8--9, torque law eqs. 11/20, Propositions 3--4)},
  file         = {papers/lee2010geometricse3.pdf},
}
```
*(`primaryClass` corrected from `cs.RO` to `math.OC`, as printed on the PDF.)*

### 5. `lee2010geometrictracking`
- **Held:** none. Venue (CDC 2010, pp. 5420–5425, DOI 10.1109/cdc.2010.5717652) is consistent with the held papers that
  cite it (Cobo-Briesewitz, Smeur: "Geometric tracking control of a quadrotor UAV on SE(3)"), but the proposition
  numbering is **open** (V1).
- **Proposal:** keep the entry and shorten the `note` so that it no longer asserts the numbering:
```bibtex
  note = {PRIMARY reference for the geometric SE(3) controller (Strategy 0). Stability regions (exponential below
          90 degrees, almost-global attractiveness below 180 degrees) checked against the v4 companion
          (Propositions 3--4 there); numbering in this paper not checked},
```

### 6. `gu2025comparative`
- **Held:** journal version, J. Intell. Robot. Syst. **(2025) 111:98**, DOI 10.1007/s10846-025-02305-5, open access
  ("© The Author(s) 2025") (**PDF** p. 1). The note "full text not held (paywalled)" is stale.
```bibtex
@article{gu2025comparative,
  title   = {Learning uncertainties online for quadrotor flight control: A comparative study},
  author  = {Weibin Gu and Jiance Zhao and Alessandro Rizzo},
  journal = {Journal of Intelligent \& Robotic Systems},
  volume  = {111},
  pages   = {98},
  year    = {2025},
  doi     = {10.1007/s10846-025-02305-5},
  file    = {papers/gu2025comparative.pdf},
}
```

### 7. `bauersfeld2024airflow`
- **Held:** arXiv:2403.13321v3 (12 Dec 2024), header "IEEE RAL. Preprint version. Accepted Dec, 2024" (**PDF** p. 1).
- **Venue:** RA-L **10(2):1241–1248** (ref-list: `hsieh2026flatness`, giving 2024; `kiran2025downwash`, giving 2025). Volume 10
  is the 2025 volume, so **2025** is proposed; confirm on IEEE Xplore.
- **Content:** 230 g–6.3 kg, six platforms, >16 h, 2.5 motor-distances, near field = CFD, controller demo (Table III)
  are all in v3 ✓.
```bibtex
@article{bauersfeld2024airflow,
  title   = {Robotics meets Fluid Dynamics: A Characterization of the Induced Airflow below a Quadrotor as a Turbulent Jet},
  author  = {Leonard Bauersfeld and Koen Muller and Dominic Ziegler and Filippo Coletti and Davide Scaramuzza},
  journal = {IEEE Robotics and Automation Letters},
  volume  = {10},
  number  = {2},
  pages   = {1241--1248},
  year    = {2025},
  note    = {arXiv:2403.13321v3},
  file    = {papers/bauersfeld2024airflow.pdf},
}
```

### 8. `yang2025flatnessresiduals`
- **Held:** arXiv:2504.04324**v3** (27 Jan 2026), no venue printed (**PDF** p. 1; primary class eess.SY).
- **Venue:** "2025 IEEE 64th Conference on Decision and Control (CDC), pages 3035–3042" (ref-list: `hsieh2026flatness`).
  Whether v3 (dated after CDC 2025) equals the CDC paper is **open**: v3 has 17 pages including appendices, while CDC
  papers have 8. The thesis cites the planar simulation and the theorems, which are likely in both, but this is not
  checked. Keeping the arXiv v3 entry is the safe choice.
```bibtex
@misc{yang2025flatnessresiduals,
  title        = {Learning Flatness-Preserving Residuals for Pure-Feedback Systems},
  author       = {Fengjun Yang and Jake Welde and Nikolai Matni},
  year         = {2025},
  howpublished = {arXiv:2504.04324v3},
  eprint       = {2504.04324},
  archivePrefix= {arXiv},
  primaryClass = {eess.SY},
  note         = {A conference version appeared at IEEE CDC 2025, pp. 3035--3042; the extended arXiv v3 is cited},
  file         = {papers/yang2025flatnessresiduals.pdf},
}
```

### 9. `li2023ndpnmpc`
- **Held:** arXiv:2304.07794v2 (12 Sep 2023), no venue printed. **Venue:** "2023 62nd IEEE Conference on Decision and
  Control (CDC), pp. 2122–2128" (ref-list: `hsieh2025onlineadaptation`, `zhang2024proxfly`). The cited content (DShot
  labels, TX2 NX, shuffled split, 75.37 %) is in v2 ✓.
```bibtex
@inproceedings{li2023ndpnmpc,
  title     = {Nonlinear {MPC} for Quadrotors in Close-Proximity Flight with Neural Network Downwash Prediction},
  author    = {Jinjie Li and Liang Han and Haoyang Yu and Yuheng Lin and Qingdong Li and Zhang Ren},
  booktitle = {62nd IEEE Conference on Decision and Control (CDC)},
  pages     = {2122--2128},
  year      = {2023},
  note      = {arXiv:2304.07794},
  file      = {papers/li2023ndpnmpc.pdf},
}
```

### 10. `shankar2023docking`
- **Held:** arXiv:2311.13988v1, Springer LNCS-style layout, no venue printed. **Venue:** "International Symposium on
  Experimental Robotics. Springer, 2023, pp. 427–437" (ref-list: `chee2024knodedwmpc`). Content ✓ (R17).
```bibtex
@inproceedings{shankar2023docking,
  title     = {Docking Multirotors in Close Proximity using Learnt Downwash Models},
  author    = {Ajay Shankar and Heedo Woo and Amanda Prorok},
  booktitle = {International Symposium on Experimental Robotics (ISER)},
  publisher = {Springer},
  pages     = {427--437},
  year      = {2023},
  note      = {arXiv:2311.13988},
  file      = {papers/shankar2023docking.pdf},
}
```

### 11. `shi2020neuralswarm`
- **Held:** arXiv:2003.02992v1 (6 Mar 2020), no venue printed. **Venue:** "IEEE International Conference on Robotics and
  Automation (ICRA), pp. 3241–3247, 2020" (ref-list: several held papers, e.g. `gielis2023aggregate`,
  `cobobriesewitz2026lindi`). The cited content is in v1 (R01). Whether the ICRA version is identical is **open** but
  likely.
```bibtex
@inproceedings{shi2020neuralswarm,
  title     = {Neural-Swarm: Decentralized Close-Proximity Multirotor Control Using Learned Interactions},
  author    = {Guanya Shi and Wolfgang Hönig and Yisong Yue and Soon-Jo Chung},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  pages     = {3241--3247},
  year      = {2020},
  note      = {arXiv:2003.02992},
  file      = {papers/shi2020neuralswarm.pdf},
}
```

---

## Re-confirmation of cited statements in revised Ch1–Ch5

Method:
1. Every sentence in Ch1–Ch5 that carries a `\cite` and a number had each number searched for in the text of the
   cited PDF(s) (script output in the scratchpad).
2. Every verbatim quotation (``…'') attributed to a paper was searched for in all held PDFs.
3. Every attribution without a number was covered by the Stage 1 records and the pass-D audits.

Results:
- **Numbers:** four sentences were flagged, and none is a real miss. (i) Bauersfeld "0.23 kg": the PDF writes
  "230 g" / "0.230 kg" (lines 111, 256) ✓. (ii) Table 2.1 ProxFly "0.50 m", "0.10 m": the PDF writes "50 cm", "10cm" ✓.
  (iii) Ch3 "0.2 m … 0.4 m": a code value (`residual_nn.rs:139–140`) placed next to a Gielis comparison of kind, not a
  Gielis number ✓. (iv) Ch3 assumptions table: column widths (`0.30\linewidth`), not data ✓.
- **Quotations:** all six attributed quotations occur verbatim in the cited paper (Shi 2020 ×3, NS2 ×1, Gu ×1, plus "to
  the best of our knowledge", which is the author's own phrase). The other quoted strings are the author's own scare
  quotes, not attributions.
- **Equation and proposition numbers of papers:** none appear in the printed text of Ch1–Ch5 (they are only in
  comments). Nothing to re-check.
- **Not confirmable locally:** the Lee CDC 2010 statements (V1) and the Abro paragraph (V2). Both are already marked.
