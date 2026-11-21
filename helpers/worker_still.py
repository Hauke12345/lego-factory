import sys; sys.path.insert(0, '/home/claude/scripts')
import bpy, numpy as np, cprev, worker, lblend
from cell import T
P = worker.build(P=[])
P.append(dict(name='floor', part='3811', col=71, M=T(0, worker.GROUND + 4, 0)))   # baseplate as floor
sc, cam, _ = cprev.setup(P, (1400, 1000))
for nm, v in [('front', ((-0.10, 0.17, 0.09), (0.0, 0.01, 0.025), 45)), ('back', ((0.15, -0.14, 0.10), (0.0, -0.005, 0.025), 45))]:
    cprev.aim(cam, v); sc.render.filepath = f'/home/claude/factory/prev/worker_{nm}.png'; bpy.ops.render.render(write_still=True)
