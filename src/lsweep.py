"""Clash/connection check over the station cycle. Usage: python3 lsweep.py module N"""
import sys, importlib, numpy as np, io, contextlib
sys.path.insert(0, '/home/claude/scripts')
import lcheck
mod = importlib.import_module(sys.argv[1]); N = int(sys.argv[2]) if len(sys.argv) > 2 else 24
bad = {}
for k in range(N):
    t = (k + 0.5)/N; st = mod.cycle(t)
    P = mod.build(st)
    with contextlib.redirect_stdout(io.StringIO()): rep = lcheck.run(P, verbose=False)
    issues = [('CLASH',) + c[:3] for c in rep['clashes']] + [('BADFIT',) + tuple(map(str, c)) for c in rep['bad_fits']] + [('UNCONN', u) for u in rep['unconnected']]
    print(f"t={t:.3f} {st['phase']:8s} theta={np.degrees(st['theta']):7.1f} beta={np.degrees(st['beta']):7.1f} -> {len(issues)} issues", flush=True)
    for i in issues: print('    ', i); bad.setdefault(i[:3], []).append(round(t, 3))
print('SUMMARY', len(bad), 'distinct issues')
for k, v in bad.items(): print(' ', k, 'at t =', v[:6])
