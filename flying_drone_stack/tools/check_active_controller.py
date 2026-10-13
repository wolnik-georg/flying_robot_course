#!/usr/bin/env python3
"""Which controller does the drone really run after `stabilizer.controller` is set? (CS2 must be stopped; drone on the ground, motors not armed.)

For each value in the list: set the parameter, read it back, capture the firmware console line "Using <name> (<n>) controller" and sample a few log
variables for ~1 s. The Tal & Karaman controllers (11 Rust, 12 C) write ctrlTk.* every tick; Omar Rust (10) and the geometric controller do not.

  python3 flying_drone_stack/tools/check_active_controller.py radio://0/80/2M/E7E7E7BB02 [6 10 11 12 6]

Added 2026-10-13 after the "Tal & Karaman" flights logged the signature of controller 6 (indi.tau 0, ctrlTk.* 0, alp and rnn_pred non-zero)."""
import sys, time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

uri = sys.argv[1]
seq = [int(x) for x in sys.argv[2:]] or [6, 10, 11, 12, 6]
console = []


def on_char(text):
    console.append(text)


cflib.crtp.init_drivers()
with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
    cf = scf.cf
    cf.console.receivedChar.add_callback(on_char)
    toc = cf.log.toc.toc
    want = [("ctrlTk.thrust", "float"), ("ctrlTk.rpm_ok", "float"), ("ctrlTk.tau_f", "float"), ("indi.e_r_norm", "float"), ("indi.tau_x", "float"), ("rnn.pred_z", "float")]
    have = [(n, t) for n, t in want if n.split(".")[0] in toc and n.split(".")[1] in toc[n.split(".")[0]]]
    print("log variables present:", [n for n, _ in have], " missing:", [n for n, _ in want if (n, _) not in have])
    for c in seq:
        del console[:]
        cf.param.set_value("stabilizer.controller", str(c))
        time.sleep(1.5)
        got = {}
        if have:
            lc = LogConfig(name="chk", period_in_ms=20)
            for n, t in have:
                lc.add_variable(n, "float")

            def cb(ts, data, _lc):
                got.update(data)

            cf.log.add_config(lc)
            lc.data_received_cb.add_callback(cb)
            lc.start()
            time.sleep(1.0)
            lc.stop()
        text = "".join(console).replace("\n", " | ")
        readback = cf.param.get_value("stabilizer.controller")
        print(f"set {c:2d}  readback {readback}  console: {text.strip()[-120:]!r}")
        print("        ", {k: round(v, 5) for k, v in got.items()})
