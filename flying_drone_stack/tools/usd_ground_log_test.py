"""Per-drone GROUND logging smoke test over cflib (no CS2, no motors): usd.logging 1 for 5 s, then 0, prints the drone's own console
(`uSD: Logging to: thesisNN` / `uSD: Wrote ... B`). CS2 must be stopped (radio shared). Adds one file (runTag 777) to each card.
2026-11-18: validated on cf5 + cf_second (both wrote thesis00, ~886 kB)."""
import sys, time, cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
cflib.crtp.init_drivers()
URIS={'cf5':'radio://0/80/2M/E7E7E7BB02','cf_second':'radio://0/80/2M/E7E7E7E7E7'}
for name,uri in URIS.items():
    buf=[]
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='/tmp/usd_ground_log_cache')) as scf:
        cf=scf.cf
        cf.console.receivedChar.add_callback(lambda t,b=buf: b.append(t))
        print(name,'canLog',cf.param.get_value('usd.canLog'),'logging',cf.param.get_value('usd.logging'))
        cf.param.set_value('usd.runTag','777'); time.sleep(0.3)
        cf.param.set_value('usd.logging','1'); print(name,'logging -> 1 at',time.strftime('%H:%M:%S'))
        time.sleep(5)
        cf.param.set_value('usd.logging','0'); print(name,'logging -> 0')
        time.sleep(3)
    print('--- console of',name); print(''.join(buf)[-1500:])
