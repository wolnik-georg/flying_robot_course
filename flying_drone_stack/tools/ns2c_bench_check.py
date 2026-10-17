#!/usr/bin/env python3
"""Phase 7 bench check of the Omar C tree image with NS2 in C (docs/plan_omar_tree_ns2_2026-10-14.md).
GROUND ONLY, motors must not be commanded: this firmware has no arming gate, so NO position setpoint is sent here.
CS2 must be stopped. Static checks only; the NS2 hook itself runs only with a position setpoint (first hover, Phase 8).

For one drone: select controller 9 (and read the console line), read the Omar and rnn parameters, check that the log
variables the study uSD config needs exist, check that rnn.en is writable, and put the controller back to 6.

  python3 flying_drone_stack/tools/ns2c_bench_check.py radio://0/80/2M/E7E7E7BB02          # cf5
  python3 flying_drone_stack/tools/ns2c_bench_check.py radio://0/80/2M/E7E7E7E7E9          # cf_second
  python3 flying_drone_stack/tools/ns2c_bench_check.py <uri> --readback     (read-only: prints controller, indi, Kpos_Iz, rnn.en, rnn.ready;
                                                                              writes nothing; run with CS2 stopped, drone still powered, to
                                                                              prove which strategy the last block ran)
  python3 flying_drone_stack/tools/ns2c_bench_check.py <uri> --post-flight                 # after a flight, drone still powered, CS2 stopped:
                                                                                           # prints rnn.us_last/us_max/us_avg and rnn.pred_*, rnn.clamped

Expected: console 'Using OutOfTree4 (9)'; rnn.ready 1, rnn.div 10, rnn.en 0; ctrlOmarIndi.mass 0.0427, Kpos_P 7/7/7,
Kpos_D 4/4/4; all log variables present. Exit code 0 only if every check passes."""
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

uri = sys.argv[1]
POST = "--post-flight" in sys.argv
READ = "--readback" in sys.argv
console: list[str] = []
fails: list[str] = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{('  ' + str(detail)) if detail != '' else ''}")
    if not ok:
        fails.append(name)


def pget(cf, name):
    return cf.param.get_value(name, timeout=3)


cflib.crtp.init_drivers()
if READ:
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        names = ["stabilizer.controller", "ctrlOmarIndi.indi", "ctrlOmarIndi.Kpos_Iz", "ctrlOmarIndi.Kpos_Px", "ctrlOmarIndi.Kpos_Dx",
                 "ctrlOmarIndi.mass", "rnn.en", "rnn.ready", "rnn.div"]
        vals = {n: pget(cf, n) for n in names}
        for n in names:
            print(f"  {n:24s} {vals[n]}")
        ind, en = int(float(vals["ctrlOmarIndi.indi"])), int(float(vals["rnn.en"]))
        try:   # 2026-10-17: Version A/B switch; absent on images built before it (then Version A)
            mode = int(float(pget(cf, "ctrlOmarIndi.ns2_mode")))
            print(f"  {'ctrlOmarIndi.ns2_mode':24s} {mode}")
        except Exception:
            mode = 0
            print(f"  {'ctrlOmarIndi.ns2_mode':24s} (not in this image: Version A)")
        strat = {(0, 0): "S0 geometric", (3, 0): "S1 INDI", (0, 1): "S2 geometric + NS2", (3, 1): "S3 INDI + NS2 (Version A, plain sum)"}.get((ind, en), f"other (indi {ind}, rnn.en {en})")
        if (ind, en) == (3, 1) and mode == 1:
            strat = "S3B INDI + NS2 (Version B, INDI on what NS2 left over)"
        print(f"  => strategy loaded on this drone: {strat}")
    sys.exit(0)
if POST:
    from cflib.crazyflie.log import LogConfig
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        got = {}
        lc = LogConfig(name="ns2t", period_in_ms=100)
        for n in ["rnn.us_last", "rnn.us_max", "rnn.us_avg", "rnn.pred_x", "rnn.pred_y", "rnn.pred_z"]:
            lc.add_variable(n, "float" if n.endswith(("avg", "_x", "_y", "_z")) else "uint16_t")
        lc.add_variable("rnn.clamped", "uint8_t")
        cf.log.add_config(lc)
        lc.data_received_cb.add_callback(lambda ts, d, _l: got.update(d))
        lc.start(); time.sleep(1.0); lc.stop()
        print("post-flight NS2 timing / prediction:", got)
        print("  rnn.us_max = %s us (single NS2 evaluation, worst case since boot or rnn.rst)" % got.get("rnn.us_max"))
    sys.exit(0)

with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
    cf = scf.cf
    cf.console.receivedChar.add_callback(console.append)
    toc = cf.log.toc.toc

    need_log = ["rnn.pred_x", "rnn.pred_y", "rnn.pred_z", "rnn.clamped", "rnn.us_last", "rnn.us_max", "rnn.us_avg",
                "ctrlOmarIndi.a_rpm_fx", "ctrlOmarIndi.a_imu_fx", "ctrlOmarIndi.thrustSi", "ctrlOmarIndi.torquex"]
    missing = [n for n in need_log if n.split(".")[0] not in toc or n.split(".")[1] not in toc[n.split(".")[0]]]
    check("log variables present in the image", not missing, f"missing {missing}" if missing else f"{len(need_log)} checked")

    del console[:]
    cf.param.set_value("stabilizer.controller", "9")
    time.sleep(1.5)
    line = "".join(console)
    check("console reports controller 9 (OutOfTree4)", "OutOfTree4 (9)" in line, repr(line.strip()[-80:]))

    vals = {n: pget(cf, n) for n in ["rnn.ready", "rnn.en", "rnn.div", "ctrlOmarIndi.mass", "ctrlOmarIndi.indi",
                                       "ctrlOmarIndi.Kpos_Px", "ctrlOmarIndi.Kpos_Dx", "ctrlOmarIndi.Kpos_Iz"]}
    print("  values:", {k: (float(v) if not isinstance(v, str) else v) for k, v in vals.items()})
    check("rnn.ready == 1 (flash weights linked)", int(float(vals["rnn.ready"])) == 1)
    check("rnn.div == 10 (100 Hz)", int(float(vals["rnn.div"])) == 10)
    check("rnn.en == 0 at boot", int(float(vals["rnn.en"])) == 0)
    check("Omar mass 0.0427 kg", abs(float(vals["ctrlOmarIndi.mass"]) - 0.0427) < 1e-4)
    check("Omar Kpos_P 7, Kpos_D 4 (defaults)", abs(float(vals["ctrlOmarIndi.Kpos_Px"]) - 7.0) < 1e-6 and abs(float(vals["ctrlOmarIndi.Kpos_Dx"]) - 4.0) < 1e-6)

    cf.param.set_value("rnn.en", "1"); time.sleep(0.3)
    en1 = int(float(pget(cf, "rnn.en")))
    cf.param.set_value("rnn.en", "0"); time.sleep(0.3)
    en0 = int(float(pget(cf, "rnn.en")))
    check("rnn.en writable (1 then back to 0)", en1 == 1 and en0 == 0)

    cf.param.set_value("stabilizer.controller", "6"); time.sleep(1.0)

print()
print("RESULT:", "ALL PASS" if not fails else f"{len(fails)} FAIL: {fails}")
sys.exit(1 if fails else 0)
