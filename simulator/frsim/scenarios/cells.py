"""Cells = one scenario with fixed settings. Tuning cells are used to tune and select gains, study cells (never used for tuning) give the test error.

A1: two vehicles hover one above the other (dz = vertical distance in m, hold = hover time in s).
A4: the top vehicle moves in a line next to the bottom one (offset = horizontal offset in m, speed in m/s).
A8: figure-eight of the bottom vehicle under the top one (span m, duration s per pass, settle s, passes).
"""
A8_STUDY = dict(dz=0.4, span=1.0, duration=6.25, settle=2.0, passes=2)
A4_LINE = dict(axis='y', motion='line')

# Tuning cells (dz 0.375 is the smallest hardware distance of the network training grid; none of them is a study cell)
TUNING_CELLS = {
    'A1': [dict(dz=0.375, hold=15.0), dict(dz=0.75, hold=15.0)],
    'A4': [dict(dz=0.375, offset=0.05, speed=0.2, **A4_LINE), dict(dz=0.5, offset=0.1, speed=0.3, **A4_LINE), dict(dz=0.75, offset=0.15, speed=0.4, **A4_LINE)],
    'A8': [dict(dz=0.375, span=1.0, duration=8.0, settle=2.0, passes=2), dict(dz=0.75, span=1.0, duration=5.0, settle=2.0, passes=2)],
}
STUDY_CELLS = {
    'A1': [dict(dz=0.45, hold=15.0)],
    'A4': [dict(dz=0.4, offset=0.125, speed=0.35, **A4_LINE), dict(dz=0.4, offset=0.075, speed=0.35, **A4_LINE)],
    'A8': [dict(A8_STUDY)],
}


def tuning_cells(scenario):
    """List of cell settings (dicts, copies) for tuning and selection."""
    return [dict(c) for c in TUNING_CELLS[scenario]]


def study_cells(scenario):
    """List of cell settings (dicts, copies) for the test error."""
    return [dict(c) for c in STUDY_CELLS[scenario]]
