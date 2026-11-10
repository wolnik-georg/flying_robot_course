# Gain evaluation: S1 on A1

- gains: tuned gains from S1.yaml (Sobol sampling, seed 4)
- command: `python3 scripts/evaluate_gains.py --ctrl S1 --scen A1 --gains tuned --procs 4 --out examples/evaluate_S1_A1`

```
selection error: 7.2176653680472285 cm  (2 crashed episodes)
recorded:        7.2176653680472285 cm  (2 crashed episodes)  -> IDENTICAL
test error (cm): nominal 3.949, ffs0.4 3.960, md10 4.110, md40 4.279, md60 4.301
recorded:        nominal 3.949, md10 4.110, md40 4.279, md60 4.301  (3 decimals)
```

Episodes: [episodes.csv](episodes.csv) (79 rows), [summary.json](summary.json)
