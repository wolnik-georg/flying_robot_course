"""Flight scenarios A1 / A4 / A8 (vendored from crazyswarm2, see SOURCE.md) and the cell definitions used for tuning and testing."""
from . import curves, poly4d, safety, scenarios  # noqa: F401
from .cells import STUDY_CELLS, TUNING_CELLS, study_cells, tuning_cells  # noqa: F401
from .motion import Motion  # noqa: F401
