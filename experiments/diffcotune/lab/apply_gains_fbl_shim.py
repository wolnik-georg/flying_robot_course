"""Stage F2: lets preflight import lab/apply_gains.py with the group switched to ctrlFbl (same as fbl/apply_gains_fbl.py)."""
import apply_gains as AG
AG.GROUP = 'ctrlFbl'
