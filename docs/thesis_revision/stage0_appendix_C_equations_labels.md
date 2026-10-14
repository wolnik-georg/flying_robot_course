# Stage 0 appendix C — equation blocks and cross-reference check

## Equation blocks (Chapters 1–5)

| chapter:line | env | labels | content (truncated) |
|---|---|---|---|
| ch2:59 | align | eq:rw-trans, eq:rw-rot | `\dot{p}_i &= v_i, & m\,\dot{v}_i &= -m g e_3 + f_i R_i e_3 + f_{\mathrm{res},i}, \\ \dot{R}_i &= R_i \hat{\om` |
| ch3:66 | align | eq:model-trans, eq:model-rot | `\dot{p}_i &= v_i, & m\,\dot{v}_i &= -m g e_3 + f_i R_i e_3 + f_{\mathrm{res},i}, \\ \dot{R}_i &= R_i \hat{\om` |
| ch3:112 | equation | eq:model-ares | `\frac{f_{\mathrm{res},i}}{m} \;=\; \underbrace{\dot{v}_i}_{a_{\mathrm{meas}}} - \underbrace{\left( \frac{f_i}` |
| ch3:164 | equation | eq:model-sign | `f_i R_i e_3 \;=\; m\,a_{\mathrm{des}} + m g e_3 - f_{\mathrm{res},i},` |
| ch3:189 | equation | eq:model-F | `f_{\mathrm{res},i} \;\approx\; \mathcal{F}\!\left( \{\, p_j - p_i,\; v_j - v_i \,\}_{j \neq i} \right),` |
| ch3:204 | equation | eq:model-deepsets | `\hat{a}_{\mathrm{res},i} \;=\; \rho\!\left( \sum_{j \neq i} \phi\!\left( p_j - p_i,\; v_j - v_i \right) \righ` |
| ch3:301 | equation | eq:model-fold | `W_1 \frac{x - \mu}{\sigma} + b_1 \;=\; \underbrace{\left(W_1 \oslash \sigma\right)}_{W_1'} x + \underbrace{\l` |
| ch4:89 | equation | eq:ctrl-ades | `a_{\mathrm{cmd}} \;=\; \underbrace{a_{\mathrm{ff}}}_{\text{trajectory}} + K_p e_p + K_v e_v + K_i \!\int\! e_` |
| ch4:122 | equation | eq:ctrl-tau-geo | `\tau = -K_R^{\mathrm{geo}} e_R - K_\omega^{\mathrm{geo}} e_\omega + \omega \times J\omega - k_{i,\mathrm{att}` |
| ch4:167 | equation | eq:ctrl-tau-lee | `M = -k_R e_R - k_\Omega e_\Omega + \Omega \times J\Omega - J\!\left( \hat{\Omega}\, R^\top R_d \Omega_d - R^\` |
| ch4:203 | equation | eq:ctrl-alpharef | `\alpha_{\mathrm{ref}} = \alpha_{\mathrm{des}} - K_R^{\mathrm{INDI}} e_R - K_\omega^{\mathrm{INDI}} e_\omega,` |
| ch4:215 | equation | eq:ctrl-indi-tau | `\tau = \tau_{\mathrm{cur}} + J\left(\alpha_{\mathrm{ref}} - \alpha_{\mathrm{meas}}\right).` |

## Cross-reference check

- labels defined: 105; `\ref` / `\eqref` uses: 169; **undefined references: 0**; labels never referenced: 47 (mostly section labels; list below).

Unreferenced labels: `ch:intro`, `eq:ctrl-alpharef`, `eq:model-fold`, `eq:rw-rot`, `eq:rw-trans`, `sec:conclusion-close`, `sec:conclusion-engineering`, `sec:conclusion-rq`, `sec:conclusion-summary`, `sec:ctrl-common`, `sec:ctrl-s2`, `sec:ctrl-summary`, `sec:disc-contrib`, `sec:disc-rq1`, `sec:disc-rq2`, `sec:disc-rq3`, `sec:disc-validity`, `sec:intro-gap`, `sec:intro-motivation`, `sec:intro-outline`, `sec:intro-problem`, `sec:intro-scope`, `sec:intro-strategies`, `sec:model-assumptions`, `sec:model-notation`, `sec:model-summary`, `sec:model-superposition`, `sec:res2-hybrid`, `sec:res2-nn`, `sec:res2-overview`, `sec:res3-campaign`, `sec:res3-n4sketch`, `sec:res3-summary`, `sec:rw-baseline`, `sec:rw-gap`, `sec:rw-superposition`, `sec:setup-campaign`, `sec:setup-collect`, `sec:setup-gate`, `sec:setup-metrics`, `sec:setup-null`, `sec:setup-protocol`, `sec:setup-sim`, `sec:setup-summary`, `sec:setup-volume`, `tab:intro-scope`, `tab:setup-scenarios`
