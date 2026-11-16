"""Stage 5b: S0 is unchanged in reverse mode: worker_rollout of loop_lib before (loop_lib.py.before_stage5b) and after Stage 5b, S0, A8 and A1 cells, J_task, J_task with 2 noise seeds, J_sysId: value and gradient compared bit for bit (max abs difference)."""
import importlib.util, sys, json
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'stage5'))
import loop_lib as NEW  # noqa
from importlib.machinery import SourceFileLoader
spec = importlib.util.spec_from_loader('loop_lib_old', SourceFileLoader('loop_lib_old', str(HERE / 'loop_lib.py.before_stage5b'))); OLD = importlib.util.module_from_spec(spec); spec.loader.exec_module(OLD)
import checks5 as C5  # noqa
import torch; torch.set_num_threads(2)
assert NEW.CTRL == 'S0' and NEW.GRAD_MODE == 'reverse'
x = np.r_[np.random.default_rng(3).uniform(-0.1, 0.1, 10), np.log([0.8, 1.5, 1.3, 0.7])]; out = {}
for i in (0, 1):
    s, c = NEW.TUNING[i]; tr = [C5.fr((s, c, NEW.gains_of(np.zeros(10)), NEW.PLANTS['P1']['overrides'], 0, False))]
    for name, args in (('task', ('task', s, c, x, None)), ('task_noise', ('task', s, c, x, None, NEW.ALL_STATES, (1000, 1001))), ('sysid', ('sysid', s, c, x, tr, ('p', 'v', 'q')))):
        a = OLD.worker_rollout(args); b = NEW.worker_rollout(args)
        out['%s_%s' % (s, name)] = dict(dJ=abs(a[0] - b[0]), dgrad=float(np.abs(a[1] - b[1]).max())); print(s, name, out['%s_%s' % (s, name)], flush=True)
out['identical'] = all(v['dJ'] == 0 and v['dgrad'] == 0 for v in out.values()); print('S0 reverse mode identical to Stage 5:', out['identical'])
json.dump(out, open(HERE / 'results' / 's0_identity.json', 'w'), indent=1)
