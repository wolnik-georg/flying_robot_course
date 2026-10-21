#!/usr/bin/env python3
"""Dry run of fbl_bench_check.py against a fake drone (mocked cflib): checks that the script runs through, passes on a correct image and FAILS on wrong ones.
  python3 flying_drone_stack/tools/test_fbl_bench_check_mock.py"""
import runpy, sys, types
from pathlib import Path
P1 = dict(kp_xy=14.0, kp_z=11.25, kv_xy=9.5, kv_z=5.4, ki_xy=12.0, ki_z=12.0, kr_xy=59.5, kr_z=35.0)
P0 = dict(kp_xy=56.0, kp_z=45.0, kv_xy=38.0, kv_z=21.6, ki_xy=48.0, ki_z=3.0, kr_xy=238.0, kr_z=140.0)
def fake(state):
    class Param:
        def get_value(self, n, timeout=3): return str(state["p"][n])
        def set_value(self, n, v):
            state["p"][n] = type(state["p"][n])(float(v)) if not isinstance(state["p"][n], str) else v
            if n == "ctrlFbl.preset" and state["preset_works"]: state["p"].update({"ctrlFbl." + k: (P1 if int(float(v)) == 1 else P0)[k] for k in P1})
            if n == "stabilizer.controller": state["console"]("Using OutOfTree4 (9)\n" if v == "9" and state["console_ok"] else "Using something else\n")
    class Console:
        def __init__(s): s.receivedChar = types.SimpleNamespace(add_callback=lambda cb: state.__setitem__("cb", cb))
    class Toc: toc = {"ctrlFbl": {k: 1 for k in ("active", "tripped", "steps", "faults", "res_az", "thrust")}} if state["log_ok"] else {"ctrlFbl": {}}
    class Log:
        toc = Toc()
        def add_config(self, lc): state["lc"] = lc
    class CF:
        param = Param(); console = Console(); log = Log()
        def __init__(self, **k): pass
    class LC:
        def __init__(self, name, period_in_ms): self.vars = []; self.data_received_cb = types.SimpleNamespace(add_callback=lambda cb: setattr(self, "cb", cb))
        def add_variable(self, n, t): self.vars.append(n)
        def start(self): self.cb(0, {"ctrlFbl.active": 0, "ctrlFbl.tripped": 0, "ctrlFbl.steps": state["steps"], "ctrlFbl.faults": 0, "ctrlFbl.res_az": 0.0, "ctrlFbl.thrust": 0.0}, self)
        def stop(self): pass
    class Sync:
        def __init__(self, uri, cf=None): self.cf = CF()
        def __enter__(self): state["console"] = lambda s: [c for c in s] and [state["cb"](ch) for ch in s]; return self
        def __exit__(self, *a): return False
    m = types.ModuleType("cflib"); m.crtp = types.SimpleNamespace(init_drivers=lambda: None)
    mods = {"cflib": m, "cflib.crtp": m.crtp, "cflib.crazyflie": types.SimpleNamespace(Crazyflie=CF), "cflib.crazyflie.log": types.SimpleNamespace(LogConfig=LC), "cflib.crazyflie.syncCrazyflie": types.SimpleNamespace(SyncCrazyflie=Sync)}
    return mods
def run(args, **kw):
    state = dict(p={"stabilizer.controller": "6", "ctrlFbl.en": 0, "ctrlFbl.preset": 1, "ctrlOmarIndi.indi": 0, "rnn.en": 0, **{"ctrlFbl." + k: v for k, v in P1.items()}}, preset_works=True, console_ok=True, log_ok=True, steps=0); state.update(kw)
    sys.modules.update(fake(state)); sys.argv = ["fbl_bench_check.py", "radio://0/80/2M/E7E7E7BB02"] + args
    try: runpy.run_path(str(Path(__file__).parent / "fbl_bench_check.py"), run_name="__main__"); return 0
    except SystemExit as e: return int(e.code or 0)
cases = [("correct image, static checks", [], {}, 0), ("readback", ["--readback"], {}, 0), ("post-flight with steps > 0", ["--post-flight"], dict(steps=1234), 0),
         ("post-flight with steps = 0 must FAIL", ["--post-flight"], dict(steps=0), 1), ("image without ctrlFbl log variables must FAIL", [], dict(log_ok=False), 1),
         ("preset parameter that does not load gains must FAIL", [], dict(preset_works=False), 1), ("controller 9 console line missing must FAIL", [], dict(console_ok=False), 1)]
bad = 0
for name, args, kw, want in cases:
    print(f"\n=== {name}"); rc = run(args, **kw); ok = (rc == want); bad += not ok; print(f"--> exit {rc}, expected {want}: {'OK' if ok else 'WRONG'}")
print("\nRESULT:", "ALL AS EXPECTED" if not bad else f"{bad} unexpected"); sys.exit(1 if bad else 0)
