# results/

Every script run writes one folder here (these folders are git-ignored; only this file is tracked):

```
results/<date>_<time>_<script>_<controllers>_<scenario>[_<variant>]/
    summary.md      readable summary: table, links to the files
    summary.json    everything needed to rerun: command line, date, controller(s), scenario, cells, variant, seeds,
                    the full gain set and its source, plant settings, firmware build (.so path, time, patch hashes),
                    frsim commit, python / numpy / scipy versions, all results
    trace_*.csv     position error and tilt over time (run_scenario.py, compare.py)
    *.png           plots
    episodes.csv    one row per episode (evaluate_gains.py)
    compare_*.csv   comparison table (compare.py)
```

`--out DIR` writes into DIR instead; `--no-save` writes nothing. To rerun a result, take the `command` and the gains from its
`summary.json` (see `docs/REPRODUCE.md`). Copy a folder out of `results/` (for example into `examples/`) to keep it in git.
