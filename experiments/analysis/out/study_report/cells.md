# Study report (generated from `study_rows.csv`, metric `primary_metric`)

## A8

Cells (statistics over flights):

```
scenario   setting strategy  n_clean  n_attempts  failure_rate    mean   std  median  incomplete  not_reached  decision_unknown
      A8 speed=0.4       S0        4           4           0.0  -5.915 0.158  -5.930        True        False             False
      A8 speed=0.4       S1        4           4           0.0 -10.270 2.137 -10.661        True        False             False
      A8 speed=0.4       S2        3           3           0.0  -2.841 0.233  -2.975        True        False             False
      A8 speed=0.4     ours        6           6           0.0  -9.565 2.321  -9.252       False        False             False
```

Strategy vs baseline per setting (unpaired bootstrap 95 % CI):

```
scenario   setting strategy baseline  n  n_baseline  mean_diff  diff_ci_lo  diff_ci_hi  ratio  ratio_ci_lo  ratio_ci_hi                             note
      A8 speed=0.4       S1       S0  4           4     -4.354      -5.968      -2.309  1.736        1.392        2.012 n<5 in a group: descriptive only
      A8 speed=0.4       S2       S0  3           4      3.074       2.844       3.343  0.480        0.438        0.511 n<5 in a group: descriptive only
      A8 speed=0.4     ours       S0  6           4     -3.650      -5.469      -2.096  1.617        1.353        1.926 n<5 in a group: descriptive only
```

