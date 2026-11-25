"""Workbench animation of the kinematically validated closing sequence for the 34th link (real LDraw meshes checked by
erosion depth, sim/closing/*). usage: python3 prev_close.py OUTDIR"""
import sys, os, numpy as np
sys.path.insert(0, '/home/claude/scripts'); sys.path.insert(0, '/home/claude/lego-factory/sim/closing')
import bpy, mathutils, lblend, cprev, chainst as cs
from asm2 import T, R
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
P_ = cs.PITCH; S = 16*P_
A, B0, B1 = 12.0, 50.0, 15.0
MOUTH = np.array([0, np.cos(np.radians(17)), np.sin(np.radians(17))])
MX = cs.link_M(S); J = (MX @ np.array([0, 0, 0, 1.0]))[:3]
def xpose(beta, d=0.0, h=0.0):
    M = T(*J) @ R('x', beta) @ T(*(-J)) @ MX; return T(0, h, 0) @ T(*(-d*(M[:3, :3] @ MOUTH))) @ M
def joint(k): return (cs.link_M(S - (k + 1)*P_) @ np.array([0, 0, 16, 1.0]))[:3]
def head(a):
    Ms = {}; G = np.eye(4)
    for k in (2, 1, 0):
        Jk = (G @ np.r_[joint(k), 1])[:3]; G = T(*Jk) @ R('x', a) @ T(*(-Jk)) @ G; Ms[k] = G @ cs.link_M(S - (k + 1)*P_)
    return Ms
def ez(x): return 0.5 - 0.5*np.cos(np.pi*np.clip(x, 0, 1))
STEPS = [('head region lifted: three joints bent outward 12 deg (the most a 3873 joint allows)', 18),
         ('link 34 comes in tilted 50 deg, rear end first, through the opened gap', 30),
         ('its hooks catch the pins of link 33 (moving along the hook mouth)', 12),
         ('it swings down to 15 deg about those pins', 18),
         ('the head is lowered: link 1 hooks close over link 34 pins from the inside', 18),
         ('link 34 swings flat: both joints snapped, loop closed', 14)]
frames = []
for i, (cap, n) in enumerate(STEPS):
    for f in range(n):
        x = ez((f + 1)/n); a, beta, d, h = A, B0, 4.0, 30.0
        if i == 0: a, h = A*x, 30.0
        elif i == 1: h = 30*(1 - x)
        elif i == 2: h, d = 0, 4*(1 - x)
        elif i == 3: h, d, beta = 0, 0, B0 + (B1 - B0)*x
        elif i == 4: h, d, beta, a = 0, 0, B1, A*(1 - x)
        else: h, d, beta, a = 0, 0, B1*(1 - x), 0
        frames.append((cap, a, beta, d, h))
P = []
for k in range(3, 12): P.append(dict(name=f'l{k}', part='3873', col=8, M=cs.link_M(S - (k + 1)*P_)))
for k in range(28, 33): P.append(dict(name=f'l{k}', part='3873', col=8, M=cs.link_M(S + (33 - k)*P_)))
for k in range(3): P.append(dict(name=f'l{k}', part='3873', col=1, M=head(0)[k]))
P.append(dict(name='X', part='3873', col=2, M=xpose(B0, 4, 30)))
sc, cam, objs = cprev.setup(P, (960, 540)); sc.display.render_aa = 'FXAA'
c = np.array([-136, 19, -3.0])
def bl(p): return np.array([p[0], -p[2], -p[1]])*0.0004
tgt = bl(c); pos = bl(c + np.array([-160, 120, 40]))
cam.location = mathutils.Vector(pos); cam.rotation_euler = (mathutils.Vector(tgt) - mathutils.Vector(pos)).to_track_quat('-Z', 'Y').to_euler(); cam.data.lens = 50
for k, (cap, a, beta, d, h) in enumerate(frames):
    hm = head(a)
    for j in range(3): objs[f'l{j}'].matrix_world = lblend.M_bl(hm[j])
    objs['X'].matrix_world = lblend.M_bl(xpose(beta, d, h))
    sc.render.filepath = f'{out}/f{k:04d}.png'; bpy.ops.render.render(write_still=True)
    open(f'{out}/f{k:04d}.txt', 'w').write(cap)
print('frames', len(frames))
