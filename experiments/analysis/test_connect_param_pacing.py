#!/usr/bin/env python3
"""Verify crazyswarm2 connect-time param pacing (syslink queue overflow guard)."""
from pathlib import Path

CPP = (
    Path(__file__).resolve().parents[2].parent
    / "crazyswarm2/crazyflie/src/crazyflie_server.cpp"
)


def test_connect_param_pacing_present():
    text = CPP.read_text()
    assert "syslink RX queue" in text
    assert "sleep_for(pace)" in text
    assert "std::chrono::milliseconds(150)" in text
