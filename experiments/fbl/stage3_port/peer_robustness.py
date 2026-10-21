#!/usr/bin/env python3
"""Peer-feed robustness of FBL (and S2 for contrast): measured feed = ~90 Hz updates with the empirical interval distribution (median 9.9 ms, p95 20 ms; peer_intervals.npy from the
uSD logs), ~1 mm noise per update, latency unknown (+-40..60 ms, sign not resolvable from the merged logs) -> sweep. Output out/stage3/peer_robustness.json
  CFFIRMWARE_DIR=<fw>/build python3 experiments/fbl/stage3_port/peer_robustness.py"""
import json, sys
from pathlib import Path
sys.argv = sys.argv[:1]; sys.path.insert(0, str(Path(__file__).parent)); import c_in_the_loop_real as C  # noqa: E402
R = dict(delay_ms=4, motor_tau_ms=44, gyro_noise_dps=0.5, pos_noise_m=0.0005, vel_noise=0.005)
PEERS = {"ideal (100 Hz, truth)": None, "measured jitter, 1 mm": dict(emp=1, noise_m=0.001, lat_ms=0)}
for lat in (25, 50, 100): PEERS[f"measured jitter, 1 mm, latency {lat} ms"] = dict(emp=1, noise_m=0.001, lat_ms=lat)
for nz in (3, 10): PEERS[f"measured jitter, noise {nz} mm"] = dict(emp=1, noise_m=nz / 1000, lat_ms=0)
out = {}
for scen, kw, met in (("A8", {}, "z_dip_cm"), ("A1", {}, "z_mean_cm")):
    for pn, pk in PEERS.items():
        for m in ("s2", "fbl_rom"):
            r = C.run(m, 0.5, real=R, scen=scen, peer=pk, **kw); out[f"{scen}|{pn}|{m}"] = dict(metric=met, value=r[met], z_rms=r["z_rms_cm"], tilt=r["tilt_max_deg"], tripped=r["tripped"])
            print(f"{scen} {pn:40s} {m:8s} {met} {r[met]:7.2f} z_rms {r['z_rms_cm']:6.2f} tilt {r['tilt_max_deg']:5.1f} tripped {r['tripped']}", flush=True)
json.dump(out, open(Path(__file__).parents[1] / "out/stage3/peer_robustness.json", "w"), indent=1)
