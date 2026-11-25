import sys; sys.path.insert(0, '/home/claude/scripts'); sys.path.insert(0, '/home/claude/lego-factory/sim/closing')
import numpy as np, chainst as cs, depth2 as D
from asm2 import T, R
P_ = cs.PITCH; S = 16*P_
NB = {n: cs.link_M(s) for n, s in (('l0', S - P_), ('l1', S - 2*P_), ('l32', S + P_), ('l31', S + 2*P_))}
MX = cs.link_M(S); J = (MX @ np.array([0, 0, 0, 1.0]))[:3]
MOUTH = np.array([0, np.cos(np.radians(17)), np.sin(np.radians(17))])
def pose(beta, d=0.0, h=0.0, s=0.0):
    M = T(*J) @ R('x', beta) @ T(*(-J)) @ MX
    mw = M[:3, :3] @ MOUTH
    return T(0, h, s) @ T(*(-d*mw)) @ M
def dep(M): return {n: D.depth(M, Mn) for n, Mn in NB.items() if D.depth(M, Mn) >= 0}
if __name__ == '__main__':
    B = float(sys.argv[1]); Dd = float(sys.argv[2])
    print('post (h)'); [print('  ', h, dep(pose(B, Dd, h))) for h in (30, 20, 15, 10, 6, 3, 0)]
    print('hook (d)'); [print('  ', d, dep(pose(B, d))) for d in np.arange(Dd, -0.1, -1)]
    print('swing'); [print('  ', b, dep(pose(b))) for b in range(int(B), -1, -5)]
