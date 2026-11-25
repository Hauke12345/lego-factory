"""Where does X touch link 0 along the swing? contact points in link 0's part frame (fcl, triangle meshes)."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, trimesh, ldr, chainst as cs
from asm2 import T, R
Tr = np.array(ldr.part('3873')['tris'], float)
mesh = trimesh.Trimesh(vertices=Tr.reshape(-1, 3), faces=np.arange(len(Tr)*3).reshape(-1, 3))
P_ = cs.PITCH; S = 16*P_
M0 = cs.link_M(S - P_); MX = cs.link_M(S); J = (MX @ np.array([0, 0, 0, 1.0]))[:3]
def contacts(beta, extra=np.eye(4)):
    M = T(*J) @ R('x', beta) @ T(*(-J)) @ MX @ extra
    cm = trimesh.collision.CollisionManager(); cm.add_object('l0', mesh, transform=M0)
    hit, data = cm.in_collision_single(mesh, transform=M, return_data=True)
    pts = np.array([np.linalg.inv(M0) @ np.r_[c.point, 1] for c in data])[:, :3] if hit else np.zeros((0, 3))
    ptsX = np.array([np.linalg.inv(M) @ np.r_[c.point, 1] for c in data])[:, :3] if hit else np.zeros((0, 3))
    return pts, ptsX
if __name__ == '__main__':
    for b in (3, 10, 20, 30, 45):
        p0, pX = contacts(b)
        print(b, len(p0), 'in link0 frame x/y/z ranges', np.round(p0.min(0), 1) if len(p0) else '', np.round(p0.max(0), 1) if len(p0) else '',
              '| in X frame', np.round(pX.min(0), 1) if len(pX) else '', np.round(pX.max(0), 1) if len(pX) else '')
