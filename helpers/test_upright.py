import sys; sys.path.insert(0, '/home/claude/scripts')
import bpy, numpy as np, cprev
from cell import T
MH = np.eye(4); MH[:3, :3] = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]]).T   # columns: local x->-x, y->+z, z->+y
MH[:3, :3] = np.column_stack([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])
P = [dict(name='hub', part='26910', col=15, M=T(0, -211, -66) @ MH), dict(name='floor', part='3811', col=71, M=T(0, 4, 0))]
sc, cam, _ = cprev.setup(P, (1200, 900))
for nm, v in [('f', ((-0.10, 0.20, 0.12), (0, 0.0, 0.09), 40)), ('s', ((0.22, -0.02, 0.10), (0, 0.0, 0.09), 40))]:
    cprev.aim(cam, v); sc.render.filepath = f'/home/claude/factory/prev/up_{nm}.png'; bpy.ops.render.render(write_still=True)
