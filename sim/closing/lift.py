"""Closing with the head lifted (outward backbend A deg at joints 0-1, 1-2, 2-3: the real ~12 deg limit) instead of slack."""
import sys; sys.path.insert(0, '/home/claude/lego-factory/sim/closing')
import numpy as np, path2 as p, depth2 as D, chainst as cs
from asm2 import T, R
P_ = cs.PITCH; S = p.S
def joint(k):                               # joint between link k and k+1 (rear pin of link k): world point
    return (cs.link_M(S - (k + 1)*P_) @ np.array([0, 0, 16, 1.0]))[:3]   # link k at S-(k+1)P, pin at part z 16
def head(a):
    """world matrices of links 0..2 with outward backbend a at joints 2-3, 1-2, 0-1 (front lifts outward = +x rotation)."""
    Ms = {}; G = np.eye(4)
    for k in (2, 1, 0):
        J = (G @ np.r_[joint(k), 1])[:3]; G = T(*J) @ R('x', a) @ T(*(-J)) @ G
        Ms[f'l{k}'] = G @ cs.link_M(S - (k + 1)*P_)
    return Ms
A = float(sys.argv[1]); B0 = float(sys.argv[2]); B1 = float(sys.argv[3]); PHI = float(sys.argv[4])
NB32 = {'l32': p.NB['l32'], 'l31': p.NB['l31']}
def dep(M, a):
    nb = dict(NB32); nb.update(head(a)); out = {}
    for n, Mn in nb.items():
        d = D.depth(M, Mn)
        if d >= 0: out[n] = d
    return out
# sanity: head links must not collide with each other / link 3 more than the hinge contact
h = head(A); l3 = cs.link_M(S - 4*P_)
print('head self-check', D.depth(h['l0'], h['l1']), D.depth(h['l1'], h['l2']), D.depth(h['l2'], l3), ' link0 front at', np.round((h['l0'] @ np.array([0, 0, 0, 1.0]))[1:], 1))
u = np.array([0, np.cos(np.radians(PHI)), -np.sin(np.radians(PHI))]); worst = {}
def rec(st, dd):
    for n, v in dd.items():
        if v > worst.get(st, (-1, ''))[0]: worst[st] = (v, n)
for hh in (30, 20, 14, 10, 7, 4, 2, 0): rec('post', dep(T(*(hh*u)) @ p.pose(B0, 4.0), A))
for d in (3, 2, 1, 0): rec('hook', dep(p.pose(B0, d), A))
for b in np.arange(B0, B1 - 0.1, -5): rec('swing1', dep(p.pose(b), A))
for a in np.arange(A, -0.1, -2): rec('lower head', dep(p.pose(B1), a))
for b in np.arange(B1, -0.1, -2): rec('swing2', dep(p.pose(b), 0))
print(f'A {A} B0 {B0} B1 {B1} PHI {PHI}:', worst)
