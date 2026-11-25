"""Closing with slack: link 0 (and the head behind it) held back by DL along the run while link 33 is posted, hooked and
swung down to beta B1; then the head comes forward (gap closes) with X at B1; then X swings to 0 (pins snap into link 0)."""
import sys; sys.path.insert(0, '/home/claude/lego-factory/sim/closing')
import numpy as np, path2 as p, depth2 as D, chainst as cs
from asm2 import T
DL = float(sys.argv[1]); B0 = float(sys.argv[2]); B1 = float(sys.argv[3]); PHI = float(sys.argv[4]) if len(sys.argv) > 4 else 0
back = T(0, 0, DL)                                    # the bottom run travels -z: holding the head back = +z
NB0 = dict(p.NB)
def dep(M, sh):
    out = {}
    for n, Mn in NB0.items():
        Mn2 = (T(0, 0, sh) @ Mn) if n in ('l0', 'l1') else Mn
        d = D.depth(M, Mn2)
        if d >= 0: out[n] = d
    return out
u = np.array([0, np.cos(np.radians(PHI)), -np.sin(np.radians(PHI))])
worst = {}
def rec(stage, dd):
    for n, v in dd.items():
        if v > worst.get(stage, (0, ''))[0]: worst[stage] = (v, n)
for h in (30, 20, 14, 10, 7, 4, 2, 0): rec('post', dep(T(*(h*u)) @ p.pose(B0, 4.0), DL))
for d in (3, 2, 1, 0): rec('hook', dep(p.pose(B0, d), DL))
for b in np.arange(B0, B1 - 0.1, -5): rec('swing1', dep(p.pose(b), DL))
for sh in np.arange(DL, -0.1, -1): rec('close gap', dep(p.pose(B1), sh))
for b in np.arange(B1, -0.1, -2): rec('swing2', dep(p.pose(b), 0))
print(f'DL {DL} B0 {B0} B1 {B1} PHI {PHI}:', {k: (v[0], v[1]) for k, v in worst.items()})
