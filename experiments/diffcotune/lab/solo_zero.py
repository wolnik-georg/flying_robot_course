#!/usr/bin/env python3
"""Time zero for the solo tuning flights (CIRCLE, FIG8), which have no meta.json (planning agent, 2026-11-21).

Method: match the speed of the drone's own logged setpoint (ctrltarget_vx/vy) to the speed of the frsim TUNING-cell
reference, and take the best shift as time zero. It uses only the commanded setpoint, never the measured position, so it is
not circular (no cost search). The speed of the setpoint does not depend on the frame (rotation, offset).
Check on the 8 solo flights of 2026-11-21: setpoint speed error < 1 mm/s; after the shift the ingest's frame lag is 0.000 s
and the reference mismatch is about 3 mm. The server-log start_trajectory offset used before was 0.08-0.22 s early: S2 round 1
hit the +-0.2 s lag limit and inflated the cost (CIRCLE 7.44 -> 6.26 cm, FIG8 7.30 -> 6.41 cm).

Usage:  python3 solo_zero.py <in.csv> <CIRCLE|FIG8> <out.csv>
in.csv = single-drone merge of the solo flight (merge_usd_logs.py --roles bottom). If it is already shifted, the shift
in its '# t shifted by -X s' comment line is undone first. Writes out.csv with '# meta:t_zero=scenario_start' and prints
the shift, the speed error and the ingest result (RMS, lag, mismatch). Stop if |lag| > 0.02 s or mismatch > 10 mm.
"""
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import reference as R  # noqa: E402
import round as RD  # noqa: E402
import usd_ingest as U  # noqa: E402


def main(src, scen, dst):
    lines = open(src).read().split('\n')
    applied = 0.0
    for ln in lines:
        m = re.search(r'shifted by -([0-9.]+)', ln) if ln.startswith('#') else None
        if m: applied = float(m.group(1))
    L = [ln for ln in lines if ln and not ln.startswith('#')]; h = L[0].split(',')
    a = np.array([[float(x) if x not in ('', 'nan') else np.nan for x in ln.split(',')] for ln in L[1:]])
    ix = {k.split('.', 1)[1]: i for i, k in enumerate(h) if '.' in k}
    traw = a[:, 0] + applied
    ref = R.build(scen, RD.cell_of(scen, 'tuning', 0)); rt = np.asarray(ref.time_s); rp = np.asarray(ref.pos)
    rsp = np.linalg.norm(np.gradient(rp[:, :2], rt, axis=0), axis=1)
    csp = np.hypot(a[:, ix['ctrltarget_vx']], a[:, ix['ctrltarget_vy']])
    best = None
    for sh in np.arange(0.0, 4.0, 0.002):
        m = (traw - sh >= rt[0]) & (traw - sh <= rt[-1])
        e = float(np.sqrt(np.nanmean((csp[m] - np.interp(traw[m] - sh, rt, rsp)) ** 2)))
        if best is None or e < best[0]: best = (e, float(sh))
    err, sh = best
    with open(dst, 'w') as f:
        f.write('# meta:t_zero=scenario_start\n')
        f.write('# t shifted by -%.3f s = time zero from the logged setpoint speed vs the tuning-cell reference (solo_zero.py, speed error %.1f mm/s)\n' % (sh, err * 1000))
        f.write(L[0] + '\n')
        for row, tt in zip(L[1:], traw): f.write('%.6f,%s\n' % (tt - sh, row.split(',', 1)[1]))
    g = U.ingest_flight(dst, scen, RD.cell_of(scen, 'tuning', 0), rotate_deg=-90.0)
    lag = g['frame']['lag_s']; mis = g['reference_check']['max_abs_scored_after_offset_and_lag_m'] * 1000
    ok = abs(lag) <= 0.02 and mis <= 10 and err < 0.01
    print('%s %s: shift %.3f s (was %.3f), setpoint speed error %.1f mm/s, real RMS %.2f cm, lag %+.3f s, mismatch %.1f mm -> %s'
          % (Path(dst).name, scen, sh, applied, err * 1000, g['rms3d_cm'], lag, mis, 'OK' if ok else 'STOP: check by hand'))
    return 0 if ok else 1


if __name__ == '__main__':
    if len(sys.argv) != 4: sys.exit(__doc__)
    sys.exit(main(*sys.argv[1:]))
