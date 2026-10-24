#!/usr/bin/env python3
"""Bench / post-flight check of the FBL branch (Strategy 3) in the Omar C controller. Written 2026-10-20 on the desk, NOT yet run on a drone.
GROUND ONLY, motors must not be commanded: no position setpoint is sent. CS2 must be stopped.

  python3 flying_drone_stack/tools/fbl_bench_check.py <uri>              # static checks (controller 9 console line, ctrlFbl params and log variables, preset 1 gains,
                                                                          #   en writable, steps stay 0 on the ground), controller put back to 6
  python3 flying_drone_stack/tools/fbl_bench_check.py <uri> --readback   # read-only: prints en, preset, gains, indi, rnn.en and the strategy they mean
  python3 flying_drone_stack/tools/fbl_bench_check.py <uri> --post-flight  # after a flight, drone still powered, CS2 stopped: steps, faults, tripped, active, thrust;
                                                                          #   FBL ran only if steps > 0 (the Omar thrust model does not describe FBL)
Expected at boot: ctrlFbl.en 0, preset 1, gains kp 14/11.25, kv 9.5/5.4, ki 12/12, kr 59.5/35 (preset 0 = the authors' gains 56/45, 38/21.6, 48/3, 238/140).
Exit code 0 only if every check passes."""
import sys, time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

uri = sys.argv[1]; POST = "--post-flight" in sys.argv; READ = "--readback" in sys.argv
console: list[str] = []; fails: list[str] = []
GAINS = ["kp_xy", "kp_z", "kv_xy", "kv_z", "ki_xy", "ki_z", "kr_xy", "kr_z"]
P1 = dict(kp_xy=14.0, kp_z=11.25, kv_xy=9.5, kv_z=5.4, ki_xy=12.0, ki_z=12.0, kr_xy=59.5, kr_z=35.0)
P0 = dict(kp_xy=56.0, kp_z=45.0, kv_xy=38.0, kv_z=21.6, ki_xy=48.0, ki_z=3.0, kr_xy=238.0, kr_z=140.0)
LOGV = [("active", "uint8_t"), ("tripped", "uint8_t"), ("steps", "uint32_t"), ("faults", "uint32_t"), ("res_az", "float"), ("thrust", "float")]


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{('  ' + str(detail)) if detail != '' else ''}")
    if not ok: fails.append(name)


def pget(cf, name):
    # cflib's get_value returns the value cached at connect time (2026-10-24: stale reads after a controller switch / preset write), so ask the drone again first
    try: cf.param.request_param_update(name); time.sleep(0.12)
    except Exception: pass
    return cf.param.get_value(name, timeout=3)


def log_once(cf, secs=1.0):
    got = {}
    lc = LogConfig(name="fblchk", period_in_ms=100)
    for n, t in LOGV: lc.add_variable("ctrlFbl." + n, t)
    cf.log.add_config(lc); lc.data_received_cb.add_callback(lambda ts, d, _l: got.update({k.split(".")[1]: v for k, v in d.items()}))
    lc.start(); time.sleep(secs); lc.stop(); return got


cflib.crtp.init_drivers()
with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
    cf = scf.cf
    if READ:
        names = ["stabilizer.controller", "ctrlOmarIndi.indi", "rnn.en", "ctrlFbl.en", "ctrlFbl.preset", "ctrlFbl.cd", "ctrlFbl.z_on"] + ["ctrlFbl." + g for g in GAINS]
        v = {n: pget(cf, n) for n in names}
        for n in names: print(f"  {n:24s} {v[n]}")
        g = {k: float(v["ctrlFbl." + k]) for k in GAINS}
        gset = "preset 1 (ours)" if all(abs(g[k] - P1[k]) < 1e-3 for k in GAINS) else "preset 0 (authors')" if all(abs(g[k] - P0[k]) < 1e-3 for k in GAINS) else "custom gains"
        print(f"  => FBL {'ON' if int(float(v['ctrlFbl.en'])) else 'off'} (indi {v['ctrlOmarIndi.indi']}, rnn.en {v['rnn.en']}), gains: {gset}")
        sys.exit(0)
    if POST:
        got = log_once(cf)
        print("post-flight FBL counters:", got)
        check("FBL ran (steps > 0)", int(got.get("steps", 0)) > 0, f"steps {got.get('steps')}")
        check("no faults", int(got.get("faults", 1)) == 0, f"faults {got.get('faults')}")
        print("  tripped =", got.get("tripped"), "(1 = the tilt latch fell back to Omar during the flight)")
        sys.exit(0 if not fails else 1)
    cf.console.receivedChar.add_callback(console.append)
    toc = cf.log.toc.toc
    missing = [n for n, _ in LOGV if "ctrlFbl" not in toc or n not in toc["ctrlFbl"]]
    check("ctrlFbl log variables present in the image", not missing, f"missing {missing}" if missing else f"{len(LOGV)} checked")
    del console[:]
    cf.param.set_value("stabilizer.controller", "9"); time.sleep(1.5)
    check("console reports controller 9 (OutOfTree4)", "OutOfTree4 (9)" in "".join(console), repr("".join(console).strip()[-80:]))
    check("ctrlFbl.en == 0 at boot", int(float(pget(cf, "ctrlFbl.en"))) == 0)
    check("preset == 1 at boot", int(float(pget(cf, "ctrlFbl.preset"))) == 1)
    g = {k: float(pget(cf, "ctrlFbl." + k)) for k in GAINS}
    check("preset-1 gains", all(abs(g[k] - P1[k]) < 1e-3 for k in GAINS), g)
    cf.param.set_value("ctrlFbl.preset", "0"); time.sleep(0.3)
    g0 = {k: float(pget(cf, "ctrlFbl." + k)) for k in GAINS}
    check("preset 0 loads the authors' gains", all(abs(g0[k] - P0[k]) < 1e-3 for k in GAINS), g0)
    cf.param.set_value("ctrlFbl.preset", "1"); time.sleep(0.3)
    g1 = {k: float(pget(cf, "ctrlFbl." + k)) for k in GAINS}
    check("preset 1 restores ours", all(abs(g1[k] - P1[k]) < 1e-3 for k in GAINS))
    cf.param.set_value("ctrlFbl.en", "1"); time.sleep(0.3); en1 = int(float(pget(cf, "ctrlFbl.en")))
    got = log_once(cf)
    cf.param.set_value("ctrlFbl.en", "0"); time.sleep(0.3); en0 = int(float(pget(cf, "ctrlFbl.en")))
    check("ctrlFbl.en writable (1 then back to 0)", en1 == 1 and en0 == 0)
    check("on the ground FBL does not step (steps 0, active 0)", int(got.get("steps", -1)) == 0 and int(got.get("active", -1)) == 0, got)
    cf.param.set_value("stabilizer.controller", "6"); time.sleep(1.0)
print(); print("RESULT:", "ALL PASS" if not fails else f"{len(fails)} FAIL: {fails}"); sys.exit(1 if fails else 0)
