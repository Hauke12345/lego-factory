"""Interference depth between links by erosion: both meshes are shrunk by e/2 along their vertex normals; the depth is
the largest e for which they still intersect (fcl on triangle meshes)."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, trimesh, ldr, fcl
Tr = np.array(ldr.part('3873')['tris'], float)
base = trimesh.Trimesh(vertices=Tr.reshape(-1, 3), faces=np.arange(len(Tr)*3).reshape(-1, 3)); base.merge_vertices()
base.fix_normals()
ES = [0.0, 0.3, 0.6, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0]
def _bvh(m):
    b = fcl.BVHModel(); b.beginModel(len(m.vertices), len(m.faces)); b.addSubModel(m.vertices, m.faces); b.endModel(); return b
ER = {}
for e in ES:
    v = base.vertices - base.vertex_normals*(e/2); ER[e] = _bvh(trimesh.Trimesh(vertices=v, faces=base.faces, process=False))
def _hit(e, Ma, Mb):
    oa = fcl.CollisionObject(ER[e], fcl.Transform(Ma[:3, :3], Ma[:3, 3])); ob = fcl.CollisionObject(ER[e], fcl.Transform(Mb[:3, :3], Mb[:3, 3]))
    return fcl.collide(oa, ob, fcl.CollisionRequest(), fcl.CollisionResult()) > 0
def depth(Ma, Mb):
    d = None
    for e in ES:
        if _hit(e, Ma, Mb): d = e
        else: break
    return -1.0 if d is None else d          # -1: no contact; 0: touching/overlap < 0.3
