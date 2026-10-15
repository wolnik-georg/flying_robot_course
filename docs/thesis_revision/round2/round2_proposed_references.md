# Round 2 — proposed references

No new `\cite` key was added in round 2. Every citation in the revised chapters uses one of the 20 existing keys of
`docs/references.bib`, which is unchanged. The round-2 text cites the held PDFs for the equations it now follows:
- Cobo-Briesewitz et al., eqs. (8a), (8b), (10a), (10b) and §4.4: read in the PDF, §4.1–4.2 and §4.4;
- Tal & Karaman, §III-B/D;
- Smeur et al.;
- Lee et al., CDC paper and v4 companion;
- Shi et al. 2022: the grams unit, PDF §VII-C and Fig. 5.

**Status of everything below: UNVERIFIED — the author must check the primary source before adding it.**

| # | Work (as known from the code comment or general knowledge) | Where it would go | What it would support |
|---|---|---|---|
| R2-1 | J. Förster, *System Identification of the Crazyflie 2.0 Nano Quadrocopter*, Bachelor thesis, ETH Zürich, 2015 (*year from memory*); named in the controller's source comment as the origin of the inertia values | Ch4 §S0 deviations, Table 4.2 (inertia) | the inertia used by the controller, and that it was identified for the Crazyflie 2.0 |
| R2-2 | Busetto et al., arXiv:2512.14450 (round-1 A28; title, authors and content not checked) | Ch4 Table 4.2, if a brushless inertia replaces the Crazyflie 2.0 values (open item V10) | a Crazyflie 2.1 Brushless identification |
| R2-3 | D. Mellinger, V. Kumar, "Minimum snap trajectory generation and control for quadrotors", ICRA 2011 (round-1 A2) | Ch4 §common, desired attitude from $a_{cmd}$ and the heading vector $y_c$; $\omega_{des}$ from jerk and snap | the construction used by the flown controller, which differs from Lee et al.'s heading projection |

The round-1 list `../proposed_references.md` (A1–A31, §B bib corrections) and `../followup_bib_corrections.md` remain
valid.
