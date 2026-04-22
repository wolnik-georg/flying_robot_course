Run the post-flight diagnostic plot on the latest CSV in `runs/`.

```bash
~/.pyenv/versions/flying_robots/bin/python scripts/plot_flight_diagnostic.py
```

If a CSV path is given as an argument, pass it explicitly:
`$ARGUMENTS`

If `$ARGUMENTS` is non-empty, run:
```bash
~/.pyenv/versions/flying_robots/bin/python scripts/plot_flight_diagnostic.py $ARGUMENTS
```

Report what plots were generated and where they were saved.
