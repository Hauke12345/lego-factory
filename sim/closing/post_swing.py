"""Kinematic check (real LDraw meshes, lcheck triangle tests) of the 'post and swing' closing path for link 33 (X).
X turns about its front joint J (link 32's pin) by beta (forward bend: rear end inward), approaches along -opening by d,
then swings beta -> 0 so its rear pins enter link 0's hooks from the inside."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, lcheck, chainst as cs, worker2
from asm2 import T, R
P_ = cs.PITCH; S = cs.S_SLOT
def lk(n, s): return dict(name=n, part='3873', col=0, M=cs.link_M(s), body=n)
base = [lk('l32', S + P_), lk('l31', S + 2*P_), lk('l0', S - P_), lk('l1', S - 2*P_)]
Q = []; worker2.build(M0=np.eye(4), tag='x', body='x', P=Q, modules={'trackL'})
for q in Q:
    if '_spr_' in q['name']: base.append(dict(name=q['name'].split('_spr_')[1], part=q['part'], col=0, M=q['M'], body='spr'))
MX = cs.link_M(S)
J = (MX @ np.array([0, 0, 0, 1.0]))[:3]          # X front hooks (at S + P) on link 32 pins
def chk(beta, d, sgn):
    Rb = T(*J) @ R('x', sgn*beta) @ T(*(-J))
    M = Rb @ MX
    # opening direction of X's hooks: local +y rotated 17 deg towards ... approximate with local +y of X
    op = M[:3, 1]
    M = T(*(-d*op)) @ M
    P = base + [dict(name='X', part='3873', col=0, M=M, body='X')]
    r = lcheck.run(P, verbose=False)
    hits = {}
    for e in r['expected'] + r['clashes']:
        if 'X' in (e[0], e[1]): hits[e[1] if e[0] == 'X' else e[0]] = e[2]
    rear = (M @ np.array([0, 0, P_, 1.0]))[:3]
    return hits, rear
if __name__ == '__main__':
    for sgn in (1, -1):
        h, rear = chk(30, 0, sgn); print('sign', sgn, 'beta 30: X rear pin at y', rear[1].round(1), '(19 = pitch line; smaller = inward)', h)
