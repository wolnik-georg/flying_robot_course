#!/usr/bin/env python3
"""Stage 5 A1: offline preflight of one DiffCoTune controller in one command (desk PC or lab PC; nothing is flown, flashed, connected or written to crazyswarm2).
    /usr/bin/python3 lab/preflight.py --controller S0 [--pad X Y] [--yaml <crazyflies.yaml>]
Checks (each prints PASS / FAIL / NOTE; exit code 0 only if there is no FAIL):
  1 imports        frsim bindings (cffirmware) importable with this interpreter; the differentiable model compiles and runs one short rollout; scipy / yaml / torch present
  2 flight files   fly_file.py --check for CIRCLE (turned) and FIG8 at the pad (default: the pad of the yaml, else 0.0 0.3); OVAL is refused
  3 round state    round.py --init in a temporary directory: round_0/gains.yaml = default gains, yaml round trip, merge into a copy of the yaml changes only the 15 gains (apply_gains.py)
  4 settings       flights.json template parses; n-rounds 5 reaches the settings; tuning cells and rotate_deg are the protocol's (CIRCLE and FIG8 -90)
  5 resources      free disk for 5 rounds of logs + 12 test flights (size from the 23 Oct merged files, repeats bound 3x); load and MemAvailable against the standing limits (22, 3 GB)
  6 paths          which lab scripts hold a path of this desk PC without a flag (listed)
Other controllers: per-controller entries in lab/controllers.py (Stage 5 B4)."""
import argparse, json, os, re, subprocess, sys, tempfile, time
from pathlib import Path
sys.dont_write_bytecode = True
os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
HERE = Path(__file__).resolve().parent; TOP = HERE.parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(TOP)); sys.path.insert(0, str(TOP / 'dryrun'))
FAILS = []
if '--controller' in sys.argv:            # Stage 5: select the model of loop_lib before it is imported
    os.environ['DIFFCOTUNE_CONTROLLER'] = sys.argv[sys.argv.index('--controller') + 1]
if '--variant' in sys.argv:               # Stage F2: FBL variant base | rom | both (round.py reads --variant from sys.argv too)
    os.environ['DIFFCOTUNE_FBL_VARIANT'] = sys.argv[sys.argv.index('--variant') + 1]


def res(ok, name, msg='', note=False):
    tag = 'NOTE' if note else ('PASS' if ok else 'FAIL')
    if not ok and not note: FAILS.append(name)
    print('[%s] %s %s' % (tag, name, msg), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0]); ap.add_argument('--controller', default='S0'); ap.add_argument('--variant', choices=['base', 'rom', 'both'], default=None, help='FBL only (Stage F2)'); ap.add_argument('--pad', nargs=2, type=float); ap.add_argument('--yaml'); ap.add_argument('--ns2-file', help='mock test: weights file to hash instead of the one the model loads'); ap.add_argument('--ns2-yaml', help='mock test: yaml text to check item 7b on instead of --yaml / the strategy branch'); a = ap.parse_args()
    import numpy as np
    ctrl = a.controller; print('preflight', ctrl, time.strftime('%Y-%m-%d %H:%M:%S'), flush=True)
    # ---- 1 imports
    try:
        import reference as R
        from frsim import config, controllers  # noqa
        import cffirmware  # noqa
        res(True, '1a frsim bindings', 'cffirmware from %s' % Path(cffirmware.__file__).parent, )
    except Exception as e:
        res(False, '1a frsim bindings', repr(e)); return end()
    for mod in ('scipy', 'yaml', 'torch'):
        try: __import__(mod); res(True, '1b import ' + mod)
        except Exception as e: res(False, '1b import ' + mod, repr(e))
    import round as _round_first  # noqa  (sets DIFFCOTUNE_GRAD_MODE from its per-controller table before loop_lib is imported; Stage 5c)
    import loop_lib as L
    from frsim.scenarios import tuning_cells
    try:
        import torch; torch.set_num_threads(2)
        MODEL = _model_module(ctrl)
        ref = R.build('A1', tuning_cells('A1')[0]); t0 = time.time()
        with torch.no_grad():
            o = MODEL.rollout(ref, np.zeros((1, L.NG)), np.zeros((1, MODEL.N_BETA if hasattr(MODEL, 'N_BETA') else 4)), compile=True)
        x = o['x'].numpy(); res(bool(np.isfinite(x).all()), '1c model rollout', '%s compiled, A1 reference, %d ticks, finite, %.0f s' % (MODEL.__name__, x.shape[0], time.time() - t0))
    except Exception as e:
        res(False, '1c model rollout', repr(e))
    # ---- 2 flight files
    import fly_file as FF
    yp, yo = FF.yaml_pads(); pad = a.pad or yp
    for cell in ('CIRCLE', 'FIG8'):
        import io, contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf): rc = FF.check(cell, pad, yo)
        last = [l for l in buf.getvalue().splitlines() if 'margin' in l]
        res(rc == 0, '2 fly_file --check ' + cell, 'pad %s: %s' % (pad, '; '.join(l.strip() for l in last)))
    r = subprocess.run([sys.executable, str(HERE / 'fly_file.py'), '--cell', 'OVAL', '--check'], capture_output=True, text=True)
    res(r.returncode != 0 and 'dropped' in (r.stdout + r.stderr), '2 OVAL refused', (r.stderr or r.stdout).strip()[:90])
    # ---- 3 round state
    import round as RD, apply_gains as AG, yaml
    with tempfile.TemporaryDirectory() as td:
        sd = Path(td) / ctrl; RD.init_state(sd); rd = sd / 'round_0'
        st = json.load(open(rd / 'state.json')); phi = RD.yaml_to_phi(rd / 'gains.yaml')
        res(float(np.abs(phi).max()) < 1e-12 and float(np.abs(np.array(st['phi'])).max()) == 0.0, '3a round_0 gains', 'yaml round trip: max |phi| = %.1e (default gains)' % float(np.abs(phi).max()))
        if ctrl == 'FBL':          # Stage F2: the FBL fragment = ctrlFbl en 1, inner 1, cd 0.55, dw_en / nn_en of the variant, the 6 gains = preset 1 (FBL_DEFAULT_GAINS)
            import apply_gains_fbl_shim as AGF  # noqa
            g = AGF.AG.load_fragment(rd / 'gains.yaml'); dflt = {p: float(L.GAIN_DEFAULT[i]) for i, (n, ps) in enumerate(RD.GAIN_PARAMS) for p in ps}; dw, nn = RD.FBL_SWITCHES[RD.FBL_VARIANT]
            fixed = dict(en=1.0, inner=1.0, cd=0.55, dw_en=float(dw), nn_en=float(nn))
            res(all(abs(g[k] - v) <= 1e-12 * abs(v) for k, v in dflt.items()) and all(g.get(k) == v for k, v in fixed.items()) and len(g) == 11 and 'kr_xy' not in g, '3b FBL fragment = preset 1 + switches', 'variant %s: %d values: 6 gains = FBL_DEFAULT_GAINS, en 1, inner 1, cd 0.55, dw_en %d, nn_en %d, no kr' % (RD.FBL_VARIANT, len(g), dw, nn))
        else:
            g = AG.load_fragment(rd / 'gains.yaml'); dflt = {p: float(L.GAIN_DEFAULT[i]) for i, (n, ps) in enumerate(RD.GAIN_PARAMS) for p in ps}
            res(all(abs(g[k] - v) <= 1e-12 * abs(v) for k, v in dflt.items()) and len(g) == 15, '3b gains = firmware defaults', '%d values equal TREE_DEFAULT_GAINS' % len(g))
        src = Path(a.yaml) if a.yaml else FF.YAML
        if src.exists():
            out = Path(td) / 'merged.yaml'; ns = argparse.Namespace(fragment=str(rd / 'gains.yaml'), yaml=str(src), out=str(out), strategy=(None if ctrl == 'FBL' else ctrl))
            try:
                import contextlib, io
                with contextlib.redirect_stdout(io.StringIO()): AG.cmd_merge(ns)
                res(out.exists(), '3c merge on a copy of the yaml', 'only the 15 ctrlOmarIndi values differ, switches of %s unchanged (checked inside cmd_merge): %s' % (ctrl, src))
            except SystemExit as e:
                if 'wrong crazyswarm2 branch' in str(e) and not a.yaml:
                    res(True, '3c merge on this PC\'s yaml', 'NOT the branch of %s (%s); on the lab PC the yaml of the strategy branch must be in place and this check must PASS' % (ctrl, str(e)[:110]), note=True)
                else:
                    res(False, '3c merge on a copy of the yaml', str(e)[:200])
        else:
            res(False, '3c merge', 'yaml not found: %s (pass --yaml)' % src, note=True)
        # merge on the yaml of the strategy's branch of crazyswarm2 (read only: git show, nothing checked out) when the repo is here
        br = {'S0': 'main', 'S1': 'study_S1', 'S2': 'study_S2', 'S3': 'study_S3', 'FBL': 'study_FBL_stock'}[ctrl]; repo = FF.YAML.parents[2]
        if (repo / '.git').exists():
            r = subprocess.run(['git', '-C', str(repo), 'show', '%s:crazyflie/config/crazyflies.yaml' % br], capture_output=True, text=True)
            if r.returncode == 0:
                y = Path(td) / 'branch.yaml'; y.write_text(r.stdout); ns = argparse.Namespace(fragment=str(rd / 'gains.yaml'), yaml=str(y), out=str(Path(td) / 'b_merged.yaml'), strategy=(None if ctrl == 'FBL' else ctrl))
                try:
                    with contextlib.redirect_stdout(io.StringIO()): AG.cmd_merge(ns)
                    res(True, '3d merge on the yaml of branch ' + br, ('cf5 indi / rnn.en = %s kept' % (AG.SWITCHES[ctrl],)) if ctrl != 'FBL' else 'only cf5 ctrlFbl values differ (branch study_FBL_stock, read with git show)')
                except SystemExit as e:
                    res(False, '3d merge on the yaml of branch ' + br, str(e)[:200])
    # ---- 4 settings
    import flights_template as FT
    m = FT.template(test=True); idx_ok = all(c == 0 for x in m if x['role'] == 'tuning' for c in [x['cell']]) and sorted(x['scenario'] for x in m if x['role'] == 'tuning') == sorted(s for s, _ in L.TUNING)
    nt = len(L.TUNING); rot = {x['scenario']: x['rotate_deg'] for x in m[:nt]}
    res(idx_ok and rot['CIRCLE'] == -90.0 and rot['FIG8'] == -90.0 and (rot.get('A4') == -90.0 or 'A4' not in rot) and len(m) == nt + 6, '4a flights.json template', 'tuning order %s (lab order), rotate_deg %s, %d test flights' % ([x['scenario'] for x in m[:nt]], rot, sum(x['role'] == 'test' for x in m)))
    old = RD.SETTINGS['n_rounds']; sys.argv = ['round.py']
    S_ = RD.SETTINGS; scen_ok = [s for s, _ in L.TUNING] == ['A1', 'A8', 'CIRCLE', 'FIG8', 'A4'] and L.TUNING_SET == '5'
    frozen = (S_['method'] == 'ours' and S_['n_rounds'] == 6 and S_['skip_last_fit'] and S_['states'] == ['p', 'v', 'q'] and S_['gain_limit'] == 0.25 and S_['lr'] == 0.01 and not S_['paper_stop'] and S_['noise'] and S_['n_noise_seeds'] == 2
              and S_['epochs'] == 50 and S_['best_iterate'] and S_['grad_mode'] == 'cfwd' and scen_ok and (L.FIX_RD if L.CTRL in ('S1', 'S3') else not L.FIX_RD))
    res(frozen, '4b frozen lab variant (Stage 8b, decision 2)', 'method %s, rounds %d, tuning %s, states %s, 50 epochs best iterate %s, Adam lr %g, gain limit %s, noise seeds %d, paper stop %s, grad mode %s, RPM delay %s: must be the frozen "ours" variant' % (S_['method'], S_['n_rounds'], [s for s, _ in L.TUNING], ''.join(S_['states']), S_['best_iterate'], S_['lr'], S_['gain_limit'], S_['n_noise_seeds'], S_['paper_stop'], S_['grad_mode'], ('fixed 4.0502 ms' if L.FIX_RD else 'fitted') if L.CTRL in ('S1', 'S3') else 'n/a'))
    gm = RD.GRAD_MODE_DEFAULT.get(L.CTRL); res(gm == RD.SETTINGS['grad_mode'] == L.GRAD_MODE, '4d grad mode', 'gradient mode of %s: %s (round.py default table), loop_lib.GRAD_MODE %s' % (L.CTRL, gm, L.GRAD_MODE))
    # ---- 5 resources
    sizes = [p.stat().st_size for p in (TOP.parent / 'logs' / 'bank_unseen_S0_2026-10-23').glob('*/*_merged_usd.csv')]
    if sizes:
        per = max(sizes); need = per * (20 + 12) * 3; free = os.statvfs(str(TOP)).f_bavail * os.statvfs(str(TOP)).f_frsize
        res(free > need, '5a disk', 'free %.1f GB, need %.2f GB (largest 23 Oct merged file %.1f MB x 32 flights x 3 repeats bound)' % (free / 1e9, need / 1e9, per / 1e6))
    else:
        res(False, '5a disk', 'no 23 Oct merged files to size the logs', note=True)
    load = os.getloadavg()[0]; mem = [int(l.split()[1]) for l in open('/proc/meminfo') if l.startswith('MemAvailable')][0] / 1048576
    res(load <= 22 and mem >= 3, '5b load / memory', 'load %.1f (<= 22), MemAvailable %.1f GB (>= 3; a pool needs about 6 GB)' % (load, mem))
    # ---- 6 paths
    hits = []
    for f in ('round.py', 'fly_file.py', 'apply_gains.py', 'lab_pool.py', 'flights_template.py', 'preflight.py', 'fake_usd.py'):
        for i, line in enumerate((HERE / f).read_text().splitlines(), 1):
            if re.search(r"/home/|Desktop|~/", line) and not line.strip().startswith('#') and 'path' not in f:
                hits.append('%s:%d %s' % (f, i, line.strip()[:110]))
    code = [h for h in hits if 'sys.argv' not in h]
    res(True, '6 desk-PC paths', '%d lines name a desk path in the lab scripts (lab-run scripts: all through a flag or Path.home()): %s' % (len(code), code[:4]), note=True)
    # ---- 6b rule R3 in round.py (Stage 8d, user decision 16 Nov): version marker
    rtxt = (HERE / 'round.py').read_text(); mk = re.search(r"R3_VERSION = '([^']+)'", rtxt)
    res(mk is not None and "role'] == 'fallback'" in rtxt, '6b round.py has rule R3', ('marker %s' % mk.group(1)) if mk else 'R3_VERSION marker not found in round.py (old round.py: a scenario with 3 failed attempts would stop the round)')
    res('extension_round_added' in rtxt and 'NO_ELIGIBLE_SET.md' in rtxt, '6c round.py has the extension rounds (no eligible set after round 5: up to 2 more rounds, never theta_0)', 'markers extension_round_added / NO_ELIGIBLE_SET.md' if 'extension_round_added' in rtxt else 'old round.py: with no eligible set the last round would write no extension (update stage 8d2)')
    rsm = re.search(r"RS_VERSION = '([^']+)'", rtxt)
    res(rsm is not None and 'two R3 rounds in a row' in rtxt and 'restart_from_best' in rtxt, '6d round.py has rule RS (R3 round: tuning restarts from theta_best; two R3 rounds in a row: stop, final gains = theta_best)', ('marker %s' % rsm.group(1)) if rsm else 'RS_VERSION marker not found in round.py (old round.py: an R3 round would tune on from the crashed set)')
    # ---- 7 NS2 weights and switches (Stage 7; desk only, read only)
    if ctrl == 'FBL': fbl_checks(a, res)
    else: ns2_checks(ctrl, a, res)
    return end()


def fbl_checks(a, res):
    """Stage F2 (FBL): 8a the variant and its model flags / the frsim variant switch (simulator patch 0001); 8b the yaml of branch study_FBL_stock has ctrlFbl en 1, inner 1, cd 0.55 (read with git show); 8c the candidate image named in docs/fbl/LAB_PLAN.md
    exists and has the sha256 of its .sha256 file (the file only; which image is on the drones is not checkable offline); 8d the usd config usd_fbl_config.txt holds every channel usd_ingest reads; 8e crash limit 45 deg for FBL."""
    import hashlib, yaml as _y, fly_file as FF
    import fbl_model as FM
    var = os.environ.get('DIFFCOTUNE_FBL_VARIANT'); flg = FM.flags(var) if var in FM.VARIANTS else None
    import frsim.controllers as FC
    res(flg is not None and var in ('base', 'rom', 'both'), '8a FBL variant', 'variant %s: residual %s, ROM %s, net %s (model flags); frsim switch FRSIM_FBL_VARIANT read in controllers.configure: %s' % (var, *(flg or ('?', '?', '?')), 'FRSIM_FBL_VARIANT' in Path(FC.__file__).read_text()))
    res(FM.SG_R == 0 and not FM.QUANT and FM.TILT_CRASH_DEG == 45.0, '8b model settings (decisions D1, D5, D7)', 'SG_R %g, quantisation %s, crash tilt %g deg' % (FM.SG_R, FM.QUANT, FM.TILT_CRASH_DEG))
    repo = Path.home() / 'Desktop' / 'crazyswarm2'
    if (repo / '.git').exists():
        r = subprocess.run(['git', '-C', str(repo), 'show', 'study_FBL_stock:crazyflie/config/crazyflies.yaml'], capture_output=True, text=True)
        if r.returncode == 0:
            c = _y.safe_load(r.stdout)['robots']['cf5']['firmware_params']; f = c.get('ctrlFbl', {})
            res(f.get('en') == 1 and f.get('inner') == 1 and abs(float(f.get('cd', 0)) - 0.55) < 1e-9, '8c yaml of branch study_FBL_stock', 'cf5 ctrlFbl en %s, inner %s, cd %s (read only); the yaml values nn_en %s / dw_en %s are replaced by the fragment of the variant' % (f.get('en'), f.get('inner'), f.get('cd'), f.get('nn_en'), f.get('dw_en')))
        else: res(False, '8c yaml of branch study_FBL_stock', 'git show failed: ' + r.stderr.strip()[:100], note=True)
    else: res(False, '8c yaml of branch study_FBL_stock', 'crazyswarm2 repo not found at %s' % repo, note=True)
    root = Path.home() / 'Desktop' / 'flying_robot_course'; art = root / 'flying_drone_stack' / 'firmware_app' / 'build_artifacts'; img = art / 'study_tree_v2_fbl_stockinner_nn4_full_candidate.bin'; shaf = art / 'study_tree_v2_fbl_stockinner_nn4_full_candidate.sha256'
    if img.exists() and shaf.exists():
        h = hashlib.sha256(img.read_bytes()).hexdigest(); want = shaf.read_text().split()[0]
        res(h == want, '8d candidate image (LAB_PLAN.md:10)', 'sha256 %s %s the .sha256 file; which image the drones run: not checkable offline (check_active_controller.py / fbl_bench_check.py in the lab)' % (h[:12], '=' if h == want else '!='))
    else: res(False, '8d candidate image', 'file not found in %s' % art, note=True)
    cfg = root / 'flying_drone_stack' / 'tools' / 'usd_fbl_config.txt'; sys.path.insert(0, str(root / 'flying_drone_stack' / 'tools'))
    try:
        from decode_usd_log import RENAME
        inv = {v: k for k, v in RENAME.items()}; have = set(cfg.read_text().split())
        import usd_ingest as U
        need = list(U.STATE_COLS) + list(U.Q_COLS); miss = [n for n in need if inv.get(n) not in have]
        res(not miss, '8e uSD channels', 'usd_fbl_config.txt logs the raw variable of every channel usd_ingest reads (%d of %d)%s' % (len(need) - len(miss), len(need), '; missing %s' % miss if miss else ''))
    except Exception as e:
        res(False, '8e uSD channels', repr(e)[:150], note=True)


def ns2_checks(ctrl, a, res):
    """7a the NS2 weights file of the models (S2 / S3: simulator/frsim/data/ns2_v2.npz, ns2_torch.DEFAULT_WEIGHTS) has the sha256 of record: registry.json entry ns2_v2 of flying_drone_stack/tools/residual/weights, which also equals the .sha256 next to the weights
    (git-tracked copy); 7b the yaml (--yaml, else the strategy's branch of crazyswarm2 read with git show) has cf5 ctrlOmarIndi.indi / rnn.en of the strategy, NS2 off on cf_second, and for S0 / S1 NS2 off."""
    import hashlib, yaml as _y, fly_file as FF
    root = HERE.parents[2]; wdir = root / 'flying_drone_stack' / 'tools' / 'residual' / 'weights'
    try:
        reg = json.load(open(wdir / 'registry.json')); ents = reg if isinstance(reg, list) else reg.get('models', reg); ents = ents if isinstance(ents, list) else list(ents.values())
        rec = [e for e in ents if isinstance(e, dict) and e.get('id') == 'ns2_v2'][0]; rec_hash = rec.get('sha256') or rec.get('sha256_full')
        side = (wdir / 'ns2_v2.sha256').read_text().split()[0]
    except Exception as e:
        res(False, '7a NS2 weights record', 'registry / sha256 record not readable: %r' % e); return
    if ctrl in ('S2', 'S3'):
        import ns2_torch as N
        f = Path(a.ns2_file) if a.ns2_file else Path(N.DEFAULT_WEIGHTS); h = hashlib.sha256(f.read_bytes()).hexdigest()
        res(rec_hash is not None and h == rec_hash == side, '7a NS2 weights hash', 'model file %s sha256 %s; registry ns2_v2 %s; %s' % (f.name, h[:12], str(rec_hash)[:12], 'equal' if h == rec_hash == side else 'MISMATCH (registry %s, .sha256 file %s)' % (str(rec_hash)[:12], side[:12])))
        img = 'study_tree_v2.bin (sha256 3ae7304a...) contains the ns2_v2 weights per docs/s3_indi_ns2_decision_2026-10-18.md; the image on the drones is not read by this check (no radio)'; res(True, '7a image of record', img, note=True)
    else:
        res(True, '7a NS2 weights', '%s has no NS2: the model does not load the weights' % ctrl, note=True)
    # 7b yaml
    txt = None; src = ''
    if a.ns2_yaml: txt = Path(a.ns2_yaml).read_text(); src = a.ns2_yaml
    elif a.yaml: txt = Path(a.yaml).read_text(); src = a.yaml
    else:
        br = {'S0': 'main', 'S1': 'study_S1', 'S2': 'study_S2', 'S3': 'study_S3'}[ctrl]; repo = FF.YAML.parents[2]
        if (repo / '.git').exists():
            r = subprocess.run(['git', '-C', str(repo), 'show', '%s:crazyflie/config/crazyflies.yaml' % br], capture_output=True, text=True)
            if r.returncode == 0: txt = r.stdout; src = 'branch ' + br + ' (git show)'
    if txt is None: res(True, '7b yaml NS2 switches', 'no yaml given and no crazyswarm2 repo here: pass --yaml <lab crazyflies.yaml> on the lab PC', note=True); return
    try:
        rb = _y.safe_load(txt)['robots']; fp = lambda n: rb[n].get('firmware_params', {}); want = {'S0': (0, 0), 'S1': (3, 0), 'S2': (0, 1), 'S3': (3, 1)}[ctrl]
        got = (int(fp('cf5')['ctrlOmarIndi']['indi']), int(fp('cf5')['rnn']['en'])); top = (int(fp('cf_second')['ctrlOmarIndi']['indi']), int(fp('cf_second')['rnn']['en']))
        ok = got == want and top == (0, 0)
        res(ok, '7b yaml NS2 switches', '%s: cf5 indi / rnn.en %s (strategy %s needs %s), cf_second %s (needs (0, 0)); %s' % (src, got, ctrl, want, top, 'NS2 off as required' if want[1] == 0 and ok else ('NS2 on' if ok else 'MISMATCH')))
        def _find(d, k):
            if isinstance(d, dict):
                if k in d: return d[k]
                for v in d.values():
                    r = _find(v, k)
                    if r is not None: return r
        rs = _find(fp('cf5'), 'res_sign')
        res(True, '7b res_sign', 'res_sign in the cf5 block: %s (lab_check_strategy.py defines only indi and rnn.en as the strategy switches; res_sign is "irrelevant for Omar" per the yaml comment)' % rs, note=True)
    except Exception as e:
        res(False, '7b yaml NS2 switches', 'yaml of %s not parsed: %r' % (src, e))


def _model_module(ctrl):
    import importlib
    if ctrl == 'S0': return importlib.import_module('s0_model')
    if ctrl == 'FBL': return importlib.import_module('fbl_model')
    return importlib.import_module('%s_model' % ctrl.lower())


def end():
    print('PREFLIGHT %s' % ('FAILED: ' + ', '.join(FAILS) if FAILS else 'PASSED'), flush=True); return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
