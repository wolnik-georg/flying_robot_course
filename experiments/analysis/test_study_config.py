#!/usr/bin/env python3
"""Desk test of the crazyswarm2 --study gate (no hardware): pure validation functions + the ORDER of the gate in
run_formation.py (pre-takeoff check before apply('takeoff'); airborne mismatch lands before exiting)."""
import json, re, sys, tempfile
from pathlib import Path

CS2 = Path.home() / "Desktop" / "crazyswarm2" / "crazyflie_examples"
sys.path.insert(0, str(CS2))
from crazyflie_examples import study_config as sc  # noqa: E402


def main():
    # strategy blocks
    s0 = {"controller": 6, "ctrl_mode": 0, "indi": {"res_sign": 1.0}, "pos": {"ki_z": 16.0}}
    assert sc.validate_intended("S0", s0) == []
    assert any("ctrl_mode" in p for p in sc.validate_intended("S0", {**s0, "ctrl_mode": 3}))
    assert any("ki_z" in p for p in sc.validate_intended("S0", {**s0, "pos": {"ki_z": 0.0}}))
    s2 = {"controller": 6, "ctrl_mode": 0, "indi": {"res_sign": -1.0}, "pos": {"ki_z": 16.0}}
    assert sc.validate_intended("S2", s2) == []
    assert any("res_sign" in p for p in sc.validate_intended("S2", {**s2, "indi": {"res_sign": 1.0}}))
    assert sc.validate_intended("S1", {"controller": 9, "ctrl_mode": 0, "indi": {}, "pos": {}}) == []
    assert any("controller" in p for p in sc.validate_intended("S1", {"controller": 6, "ctrl_mode": 0, "indi": {}, "pos": {}}))
    # static (yaml-only) keys, before takeoff
    ok = {"rnn_en": 1.0}
    assert sc.validate_readback("S2", ok, static_only=True) == []
    assert any("rnn.en" in p for p in sc.validate_readback("S2", {"rnn_en": 0.0}, static_only=True))
    assert any("missing" in p for p in sc.validate_readback("S2", {}, static_only=True))
    s1_static = {"ctrlOmarIndi_Kpos_Iz": 1.5, "ctrlOmarIndi_indi": 3, "deck_bcRpm": 1}
    assert sc.validate_readback("S1", s1_static, static_only=True) == []
    assert any("Kpos_Iz" in p for p in sc.validate_readback("S1", {**s1_static, "ctrlOmarIndi_Kpos_Iz": 0.0}, static_only=True))
    # full airborne readback
    full_s0 = {"stabilizer_controller": 6, "indi_gains_ctrl_mode": 0, "pos_gains_ki_z": 16.0, "rnn_en": 0}
    assert sc.validate_readback("S0", full_s0) == []
    assert sc.validate_readback("S0", {**full_s0, "stabilizer_controller": 9})
    # readback key list: full groups, no write-only weight params, no nonexistent usd.configId
    ks = set(sc.READBACK_KEYS)
    assert {"indi_gains.res_sign", "indi_gains.rpm_source", "ctrlOmarIndi.Kpos_Iz", "ctrlOmarIndi.indi", "rnn.en", "rnn.div", "ctrlOot5.kpos_iz", "deck.bcRpm"} <= ks
    assert not any(k in ks for k in ("rnn.wi", "rnn.wv", "rnn.wc", "usd.configId")) and len(ks) > 80
    assert "stabilizer.controller" in sc.keys_for_airborne_check("S1") and "ctrlOmarIndi.Kpos_Iz" in sc.keys_for_airborne_check("S1")
    # firmware disk check
    with tempfile.TemporaryDirectory() as d:
        f = Path(d) / "fw.bin"; f.write_bytes(b"abc")
        good = sc.sha256_file(f)
        m = {"cf5": {"firmware_file": str(f), "firmware_sha256": good}, "cf_second": {"firmware_file": "", "firmware_sha256": "x"}}
        pr, info = sc.check_firmware_disk(m); assert pr == [] and info["cf5_firmware_disk_check"] == "ok"
        m["cf5"]["firmware_sha256"] = "0" * 64
        pr, info = sc.check_firmware_disk(m); assert pr and info["cf5_firmware_disk_check"] == "MISMATCH"
        m["cf5"]["firmware_file"] = str(Path(d) / "nope.bin")
        pr, info = sc.check_firmware_disk(m); assert pr == [] and "unverified" in info["cf5_firmware_disk_check"]
    # manifest gate
    assert sc.manifest_ok({"cf5": {"firmware_sha256": "a"}, "cf_second": {"firmware_sha256": ""}})[0] is False
    # ORDER of the gate in run_formation.py
    src = (CS2 / "crazyflie_examples" / "run_formation.py").read_text()
    pre = src.index("static_only=True")
    takeoff = src.index("apply('takeoff', _RAMP_CONTROLLER, _RAMP_CTRL_MODE")
    assert pre < takeoff, "pre-takeoff study check must come before apply('takeoff')"
    pre_block = src[src.rindex("if args.study:", 0, pre):takeoff]
    assert "NOT taking off" in pre_block and "return" in pre_block
    air = src.index("ABORT (airborne)")
    land = src.index("_land_all()", air)
    exit_ = src.index("sys.exit('[formation] --study refused after landing", air)
    start_traj = src.index("allcfs.startTrajectory(0")
    assert takeoff < air < land < exit_ < start_traj, "airborne mismatch must land before exiting and before the trajectory starts"
    assert src.count("def _land_all") == 1 and src.count("_land_all()") == 3  # the def line, the end-of-flight call, the airborne-abort call
    print("PASS study gate tests")


if __name__ == "__main__":
    main()
