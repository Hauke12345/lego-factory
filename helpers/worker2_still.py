import sys; sys.path.insert(0, '/home/claude/scripts')
import bpy, numpy as np, cprev, worker2
from cell import T
P = worker2.build()
P.append(dict(name='floor', part='3811', col=71, M=T(0, worker2.GROUND + 4, 0)))
sc, cam, _ = cprev.setup(P, (1000, 1200))
for nm, v in [('front', ((-0.12, 0.26, 0.15), (0.0, 0.0, 0.085), 40)), ('back', ((0.20, -0.18, 0.17), (0.0, 0.0, 0.085), 40))]:
    cprev.aim(cam, v); sc.render.filepath = f'/home/claude/factory/prev/w2_{nm}.png'; bpy.ops.render.render(write_still=True)
