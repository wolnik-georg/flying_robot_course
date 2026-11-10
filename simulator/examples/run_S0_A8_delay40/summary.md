# S0 on A8

- gains: tuned gains from S0.yaml (Bayesian optimisation (constrained, best observed point), seed 1)
- variant: md40 (state delay 40 ms)
- noise seed: 0, cells: study
- command: `python3 scripts/run_scenario.py --ctrl S0 --scen A8 --gains tuned --delay-ms 40 --out examples/run_S0_A8_delay40`

| cell | settings | x RMS cm | y RMS cm | z RMS cm | 3-D cm | max tilt deg | result |
|---|---|---|---|---|---|---|---|
| 0 | dz=0.4, span=1.0, duration=6.25, settle=2.0, passes=2 | 0.39 | 2.36 | 0.94 | 2.57 | 18.0 | ok |

Files: [trace_S0_A8_md40_cell0.csv](trace_S0_A8_md40_cell0.csv), [trace_S0_A8_md40_cell0.png](trace_S0_A8_md40_cell0.png)
[summary.json](summary.json)
