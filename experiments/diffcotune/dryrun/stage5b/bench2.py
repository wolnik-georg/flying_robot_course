import re
src = open('bench_blk.py').read().split("def timeit")[0]
exec(src)
import torch._dynamo as D
tst1 = tuple(t[:1].reshape(t.shape[1:]) for t in tst); tz1 = tz[0]
with torch.no_grad():
    for name, fn, args in (('single jvp', lambda a, b, c, d: one(a, b, c, d), (tuple(st), z0, tst1, tz1)), ('vmap jvp', batched, (tuple(st), z0, tst, tz))):
        try:
            ex = D.explain(fn)(*args); print(name, 'graphs', ex.graph_count, 'breaks', ex.graph_break_count, [str(b.reason)[:200] for b in ex.break_reasons[:3]], flush=True)
        except Exception as e: print(name, 'explain failed', type(e).__name__, str(e)[:300], flush=True)
