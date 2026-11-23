#!/usr/bin/env python3
"""Regression helper (protocol v2.1, 23 Nov): number of differing fields between two round report.json files, `seconds*` keys excluded (wall-clock times). Usage: compare_reports.py A.json B.json
Exact comparison of every number and string (floats must be identical); prints the differing paths; exit 0 only if there are none."""
import json, sys
def walk(a, b, path, out):
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b)):
            if k.startswith('seconds') or k == 'text': continue          # 'text' = the preview text, whose last field is the preview wall time
            if k not in a or k not in b: out.append((path + '/' + k, 'missing', a.get(k, '<none>'), b.get(k, '<none>'))); continue
            walk(a[k], b[k], path + '/' + k, out)
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b): out.append((path, 'length', len(a), len(b))); return
        for i, (x, y) in enumerate(zip(a, b)): walk(x, y, '%s[%d]' % (path, i), out)
    elif a != b and not (a != a and b != b): out.append((path, 'value', a, b))
if __name__ == '__main__':
    A, B = json.load(open(sys.argv[1])), json.load(open(sys.argv[2])); out = []; walk(A, B, '', out)
    for o in out[:40]: print('DIFF', *o)
    print('differing fields:', len(out)); sys.exit(1 if out else 0)
