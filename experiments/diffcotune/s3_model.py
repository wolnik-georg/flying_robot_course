#!/usr/bin/env python3
"""Differentiable S3 model (S1 = INDI mode 3, plus NS2 in the controller) on the frsim plant (Stage 5). Everything is in s1_model.py (`rollout(..., ns2=True)`) and s2_model.py (`ns2_hold`)."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import s1_model as S1
from s1_model import (GAIN_NAMES, GAIN_DEFAULT, BETA_NAMES, BETA_START, N_BETA, TILT_CRASH_DEG, tilt_deg, frsim_noise, M, G)  # noqa: F401


def rollout(ref, phi, psi, *a, **k):
    return S1.rollout(ref, phi, psi, *a, ns2=True, **k)
