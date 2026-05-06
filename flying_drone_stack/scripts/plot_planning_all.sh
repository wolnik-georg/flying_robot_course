#!/usr/bin/env bash
set -euo pipefail

PYENV_PYTHON="${PYENV_PYTHON:-$HOME/.pyenv/versions/flying_robots/bin/python}"

if [[ ! -x "$PYENV_PYTHON" ]]; then
  echo "Error: Python interpreter not found at: $PYENV_PYTHON" >&2
  echo "Install or adjust PYENV_PYTHON, e.g.:" >&2
  echo "  PYENV_PYTHON=$HOME/.pyenv/versions/flying_robots/bin/python scripts/plot_planning_all.sh" >&2
  exit 1
fi

"$PYENV_PYTHON" scripts/plot_planning_reference.py
"$PYENV_PYTHON" scripts/plot_planning_sim.py
