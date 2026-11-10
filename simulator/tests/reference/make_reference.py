#!/usr/bin/env python3
"""Needs the original tuning code (`experiments/tuning/`), which is not part of this branch; the frozen reference table
and `test_equivalence.py` do not need it.
ONE-TIME generator of the frozen reference table (simulator Stage 2, 2026-11-10). Not part of the test suite and not
needed to run the simulator.

It imports the EXISTING tuning-simulator code of record (experiments/tuning, unchanged) per strategy and runs a fixed
set of episodes with the host bindings given in
CFFIRMWARE_DIR; the same script is run once with the old builds (reference) and once with the fresh build
(build_bindings.sh) to compare the firmware only.

  python3 make_reference.py run --group S0|S1|S2|S3|FBL --cfw <build dir> --out <json> [--procs 4] [--reverse] [--mtpc
    N]
  python3 make_reference.py assemble --old <dir with S0.json..FBL.json> --out reference_episodes.json      (also
    prints the sanity check against out/closing_exactJ and the fblv2o search logs)

Plant code of record per group (imported, not copied): S0 / S2 plant_core_o (standard plant, trace files redirected into
a scratch dir); S1 / S3 indi_jobs_o with the fast INDI plant
(plant_core_indi_fast_o, gate flag out/plant_indi_fast_accepted, S1_PLANT=old); FBL fbl_jobs_v2o with the fast plant
(plant_core_fast_o, flag out/plant_fast_accepted_fblv2o).
Episodes: scenarios A1 / A4 / A8, their study cells, variants nominal and md10, noise seeds 0 and 1, gains default and
tuned pick (S0-S3: out/closing/picks_s0|s1.json, i.e. the
picks of closing_exactJ; FBL: default = preset 1, candidate = best observed evaluation of the finished search
fblv2o_<sc>_bo2_s0); S3 A1 pick additionally md40 / md60 (crash border).
FBL also gets a search check: the 'per_episode' list of that candidate's search record is recomputed (tuning cells x 5
search variants, noise seed 0).
"""

import argparse, glob, json, os, sys, time
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
TUN = REPO / 'experiments/tuning'
OUTV = Path(os.environ.get('REF_SCRATCH', str(REPO / 'simulator/_build/ref_scratch')))
VARIANTS = {'nominal': {}, 'md10': {'md': 10}, 'md40': {'md': 40}, 'md60': {'md': 60}}
SC_CELLS = {
    'A1': slice(0, 1),
    'A4': slice(1, 3),
    'A8': slice(3, 4),
}  # study cells per scenario, as in diff/rank_final.py


def wait_resources(procs):
    """Wait until load + procs <= 22 and MemAvailable >= 3 GB."""
    while True:
        load = float(open('/proc/loadavg').read().split()[0])
        mem = [int(l.split()[1]) for l in open('/proc/meminfo') if l.startswith('MemAvailable')][0] / 1048576
        if load + procs <= 22 and mem >= 3.0:
            return
        print('waiting: load %.1f + %d procs > 22 or MemAvailable %.1f GB < 3' % (load, procs, mem), flush=True)
        time.sleep(60)


def load_group(group):
    """Import the plant code of record for the group; returns (P, core, JOBS, extra) and sets up the module state
    exactly like the record chains."""
    sys.path.insert(0, str(TUN))
    if group in ('S0', 'S2'):
        import plant_core_o

        sys.modules['plant_core'] = plant_core_o
        import core, plant_core as P
        from run_job import JOBS, S0_PARAMS

        defaults = {k: v[0] for k, v in S0_PARAMS.items()}
    elif group in ('S1', 'S3'):
        os.environ.setdefault('S1_PLANT', 'old')
        import indi_jobs_o
        from indi_jobs_o import JOBS
        import core, plant_core as P
        from run_job import S0_PARAMS

        defaults = {k: v[0] for k, v in S0_PARAMS.items()}
    else:
        import fbl_jobs_v2o
        from fbl_jobs_v2o import JOBS, PRESET1
        import core, plant_core as P

        defaults = dict(PRESET1)
    P.OUT = OUTV / 'traces'
    (P.OUT / 'stock_inner_traces').mkdir(
        parents=True, exist_ok=True
    )  # the standard plant writes a trace file per episode: keep it out of experiments/
    return P, core, JOBS, defaults


def pick_gains(group, sc, jobs_defaults):
    """Tuned gains and their source for the group and scenario."""
    if group in ('S0', 'S2'):
        return json.load(open(TUN / 'out/closing/picks_s0.json'))[sc][
            'gains'
        ], 'out/closing/picks_s0.json (%s)' % json.load(open(TUN / 'out/closing/picks_s0.json'))[sc]['candidate']
    if group in ('S1', 'S3'):
        return json.load(open(TUN / 'out/closing/picks_s1.json'))[sc][
            'gains'
        ], 'out/closing/picks_s1.json (%s)' % json.load(open(TUN / 'out/closing/picks_s1.json'))[sc]['candidate']
    f = TUN / f'out/fblv2o_{sc}_bo2_s0.jsonl'
    L = [json.loads(x) for x in open(f)]
    assert len(L) == 240
    b = min(L, key=lambda r: r['score'])
    return b['gains'], 'out/fblv2o_%s_bo2_s0.jsonl best observed (index %d, search score %.6f)' % (
        sc,
        b['index'],
        b['score'],
    )


def build_list(group, P, core, JOBS, defaults):
    """[(meta, cfg)] in a fixed order."""
    ep = []
    pref = {'S0': 's0q_', 'S1': 's1q_', 'S2': 's2q_', 'S3': 's3q_', 'FBL': 'fblv2o_'}[group]
    for sc in ('A1', 'A4', 'A8'):
        job = JOBS[pref + sc]()
        cells = P.STUDY_CELLS[SC_CELLS[sc]]
        pg, src = pick_gains(group, sc, defaults)
        for glab, g in (('default', defaults), ('pick', pg)):
            vlist = ['nominal', 'md10'] + (
                ['md40', 'md60'] if (group == 'S3' and sc == 'A1' and glab == 'pick') else []
            )
            for vl in vlist:
                for seed in (0, 1):
                    for ci, c in enumerate(core.episodes(job, g, [VARIANTS[vl]], cells)):
                        c['seed'] = seed
                        ep.append(
                            (
                                dict(
                                    kind='study',
                                    group=group,
                                    scenario=sc,
                                    cell_index=ci,
                                    cell=cells[ci][1],
                                    variant=vl,
                                    variant_cfg=VARIANTS[vl],
                                    seed=seed,
                                    gains_label=glab,
                                    gains=g,
                                    gains_source=('defaults' if glab == 'default' else src),
                                ),
                                c,
                            )
                        )
        # search check: the per_episode list of the candidate's search record, recomputed
        # (tuning cells x search variants, noise seed 0)
        if group == 'FBL':
            f = TUN / f'out/fblv2o_{sc}_bo2_s0.jsonl'
            rec = min((json.loads(x) for x in open(f)), key=lambda r: r['score'])
            for k, c in enumerate(core.episodes(job, pg, None, None)):
                c['seed'] = 0
                ep.append(
                    (
                        dict(
                            kind='search_check',
                            group=group,
                            scenario=sc,
                            index=k,
                            search_per_episode=rec['per_episode'][k],
                            gains=pg,
                            gains_source=src,
                        ),
                        c,
                    )
                )
    return ep


def summarize(r):
    """Fields of one episode result that go into the reference table."""
    ts = r['ts']
    stable = bool(r['stable'])
    return dict(
        stable=stable,
        completed=bool(r['completed']),
        x_rms_cm=r['x_rms_cm'],
        y_rms_cm=r['y_rms_cm'],
        z_rms_cm=r['z_rms_cm'],
        rms3d_cm=(r['x_rms_cm'] ** 2 + r['y_rms_cm'] ** 2 + r['z_rms_cm'] ** 2) ** 0.5,
        tilt_max_deg=r['tilt_max_deg'],
        n_samples=len(ts),
        t_end_s=float(ts[-1]),
        crash_time_s=(None if r['completed'] else float(ts[-1])),
        score=(100.0 if not stable else (r['x_rms_cm'] ** 2 + r['y_rms_cm'] ** 2 + r['z_rms_cm'] ** 2) ** 0.5),
    )


def run_group(a):
    """Run all episodes of one group with the old plant code and write the JSON file."""
    cfw = os.environ['CFFIRMWARE_DIR'] = str(Path(a.cfw).resolve() if 'fbl_v2_firmware' not in a.cfw else a.cfw)
    P, core, JOBS, defaults = load_group(a.group)
    import cffirmware

    so = cffirmware._cffirmware.__file__ if hasattr(cffirmware, '_cffirmware') else cffirmware.__file__
    ep = build_list(a.group, P, core, JOBS, defaults)
    order = list(range(len(ep)))
    if a.reverse:
        order.reverse()
    wait_resources(a.procs)
    from multiprocessing import Pool

    t0 = time.time()
    with Pool(a.procs, maxtasksperchild=a.mtpc or None) as pool:
        res = pool.map(P.run, [ep[i][1] for i in order], chunksize=1)
    out = [None] * len(ep)
    for i, r in zip(order, res):
        out[i] = dict(meta=ep[i][0], result=summarize(r))
    info = dict(
        group=a.group,
        cffirmware_dir=cfw,
        so=so,
        so_mtime=time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(os.path.getmtime(so))),
        plant_module=P.__name__ + ' (' + os.path.basename(P.__file__) + ')',
        reverse=a.reverse,
        maxtasksperchild=a.mtpc,
        procs=a.procs,
        wall_s=time.time() - t0,
        n_episodes=len(out),
        python=sys.version.split()[0],
    )
    json.dump(dict(info=info, episodes=out), open(a.out, 'w'), indent=1)
    print('wrote', a.out, len(out), 'episodes, %.0f s' % info['wall_s'], flush=True)


def assemble(a):
    """Merge the group files of the OLD builds into reference_episodes.json and check them against the existing
    numbers."""
    d = Path(a.old)
    ref = dict(
        info=dict(
            generated=time.strftime('%Y-%m-%d %H:%M'),
            generator='simulator/tests/reference/make_reference.py (one-time)',
            note='old builds = the numbers of record; compare with the fresh build via tests/',
        ),
        groups={},
    )
    for g in ('S0', 'S1', 'S2', 'S3', 'FBL'):
        j = json.load(open(d / f'{g}.json'))
        ref['groups'][g] = j
    json.dump(ref, open(a.out, 'w'), indent=1)
    print('wrote', a.out)
    import numpy as np

    print('sanity vs closing_exactJ (study mean over cells per noise seed; max |diff| in cm):')
    worst = 0.0
    rows = []
    for g in ('S0', 'S1', 'S2', 'S3'):
        for sc in ('A1', 'A4', 'A8'):
            e = json.load(open(TUN / f"out/closing_exactJ/eval_{'s2' if g in ('S0', 'S2') else 's3'}_{sc}.json"))
            key = {'S0': 'ref_s0', 'S2': 'ns2_s2', 'S1': 'ref_s1', 'S3': 'ns2_s3'}[g]
            for glab, ek in (('default', 'baseline'), ('pick', 'pick')):
                for vl in ('nominal', 'md10', 'md40', 'md60'):
                    ex = e[key][ek].get('study_' + vl)
                    for seed in (0, 1):
                        s = [
                            x['result']['score']
                            for x in ref['groups'][g]['episodes']
                            if x['meta']['scenario'] == sc
                            and x['meta']['gains_label'] == glab
                            and x['meta']['variant'] == vl
                            and x['meta']['seed'] == seed
                        ]
                        if not s:
                            continue
                        dd = abs(float(np.mean(s)) - ex[str(seed)])
                        worst = max(worst, dd)
                        rows.append((g, sc, glab, vl, seed, float(np.mean(s)), ex[str(seed)], dd))
    for r in rows:
        if r[-1] > 0:
            print('  DIFF', r)
    print(
        '  compared %d (group, scenario, gains, variant, seed) means; max |diff| = %.3g cm; nonzero: %d'
        % (len(rows), worst, sum(1 for r in rows if r[-1] > 0))
    )
    ch = [
        (x['meta']['scenario'], x['meta']['index'], x['meta']['search_per_episode'], x['result']['score'])
        for x in ref['groups']['FBL']['episodes']
        if x['meta']['kind'] == 'search_check'
    ]
    dd = [abs(c[2] - c[3]) for c in ch]
    print(
        'FBL search check: %d episodes vs jsonl per_episode: max |diff| = %.3g, nonzero %d'
        % (len(ch), max(dd), sum(1 for x in dd if x > 0))
    )


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest='cmd', required=True)
    r = sp.add_parser('run')
    r.add_argument('--group', required=True)
    r.add_argument('--cfw', required=True)
    r.add_argument('--out', required=True)
    r.add_argument('--procs', type=int, default=4)
    r.add_argument('--reverse', action='store_true')
    r.add_argument('--mtpc', type=int, default=0)
    s = sp.add_parser('assemble')
    s.add_argument('--old', required=True)
    s.add_argument('--out', required=True)
    a = ap.parse_args()
    {'run': run_group, 'assemble': assemble}[a.cmd](a)
