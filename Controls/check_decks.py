#!/usr/bin/env python3
"""
Quick diagnostic: connect to CF and print deck detection + sensor status.
Run before flying to confirm Flow Deck v2 and Z-ranger are active.

Usage:
  python3 Controls/check_decks.py
  python3 Controls/check_decks.py radio://0/80/2M
"""
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"

uri = sys.argv[1] if len(sys.argv) > 1 else uri_helper.uri_from_env(default=DEFAULT_URI)
print(f"Connecting to {uri} ...")

cflib.crtp.init_drivers()
cf_obj = Crazyflie(rw_cache="./cache")
cf_obj.console.receivedChar.add_callback(lambda t: print(t, end=""))

with SyncCrazyflie(uri, cf=cf_obj) as scf:
    cf = scf.cf

    print("\n--- Deck detection params ---")
    deck_params = [
        ("deck.bcFlow2", "Flow Deck v2 (optical flow + z-ranger)"),
        ("deck.bcZRanger2", "Z-Ranger v2 (standalone)"),
        ("deck.bcMultiranger", "Multi-ranger"),
        ("deck.bcUSD", "USD (logging deck)"),
    ]
    for param, label in deck_params:
        try:
            val = cf.param.get_value(param)
            status = "✓ DETECTED" if val == "1" else "✗ not present"
            print(f"  {param:30s} {status}  ({label})")
        except Exception as e:
            print(f"  {param:30s} ERROR: {e}")

    print("\n--- Live sensor readings (5 seconds) ---")
    # CF log block limit is ~26 bytes. Split into two blocks:
    #   Block A: flow + range  (2+2+2 = 6 bytes)
    #   Block B: kalman vars + state  (4+4+4+4+4+4 = 24 bytes)
    log_a = LogConfig(name="DiagFlow", period_in_ms=200)
    log_a.add_variable("range.zrange", "uint16_t")  # 2 bytes
    log_a.add_variable("motion.deltaX", "int16_t")  # 2 bytes
    log_a.add_variable("motion.deltaY", "int16_t")  # 2 bytes

    log_b = LogConfig(name="DiagKal", period_in_ms=200)
    log_b.add_variable("kalman.varPX", "float")  # 4 bytes each
    log_b.add_variable("kalman.varPY", "float")
    log_b.add_variable("kalman.varPZ", "float")
    log_b.add_variable("kalman.stateX", "float")
    log_b.add_variable("kalman.stateY", "float")
    log_b.add_variable("kalman.stateZ", "float")

    print(
        f'  {"zrange_mm":>10}  {"flow_dx":>8}  {"flow_dy":>8}'
        f'  {"varPX":>10}  {"varPY":>10}  {"varPZ":>10}'
        f'  {"estX":>7}  {"estY":>7}  {"estZ":>7}'
    )

    latest = {}  # merged latest values from both log blocks

    def _cb_a(ts, data, _cfg):
        latest.update(data)

    def _cb_b(ts, data, _cfg):
        latest.update(data)

    log_a.data_received_cb.add_callback(_cb_a)
    log_b.data_received_cb.add_callback(_cb_b)

    cf.log.add_config(log_a)
    cf.log.add_config(log_b)
    log_a.start()
    log_b.start()

    t_start = time.time()
    try:
        while time.time() - t_start < 5.0:
            time.sleep(0.2)
            if latest:
                print(
                    f'  {latest.get("range.zrange",  float("nan")):>10.0f}'
                    f'  {latest.get("motion.deltaX", float("nan")):>8.0f}'
                    f'  {latest.get("motion.deltaY", float("nan")):>8.0f}'
                    f'  {latest.get("kalman.varPX",  float("nan")):>10.5f}'
                    f'  {latest.get("kalman.varPY",  float("nan")):>10.5f}'
                    f'  {latest.get("kalman.varPZ",  float("nan")):>10.5f}'
                    f'  {latest.get("kalman.stateX", float("nan")):>7.3f}'
                    f'  {latest.get("kalman.stateY", float("nan")):>7.3f}'
                    f'  {latest.get("kalman.stateZ", float("nan")):>7.3f}'
                )
    finally:
        log_a.stop()
        log_b.stop()

    print("\n--- What to look for ---")
    print("  Flow Deck v2 healthy:")
    print("    deck.bcFlow2 = 1")
    print("    range.zrange = 20–50 mm when sitting on floor (not 0, not 65535)")
    print(
        "    motion.deltaX/Y fluctuate slightly even when still (sensor noise), NOT stuck at 0"
    )
    print("    kalman.varPX/Y converge to < 0.01 within ~30 s")
    print(
        "  If zrange = 0 or 65535: Z-ranger not working (deck not seated / deck upside down)"
    )
    print("  If deltaX/Y always 0: optical flow sensor not working")
    print("  If varPX/Y keep growing: no flow input → Kalman diverges (what you saw)")
