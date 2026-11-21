import sys; sys.path.insert(0, '/home/claude/scripts')
import bpy, numpy as np, cprev
from cell import T, R
P = [dict(name='barrel', part='18588', col=72, M=T(0, 0, 0)), dict(name='trig', part='18587', col=4, M=T(80, 0, 0)),
     dict(name='both', part='18588', col=72, M=T(-90, 0, 0)), dict(name='both2', part='18587', col=4, M=T(-90, 0, 0)),
     dict(name='ax', part='3705', col=0, M=T(0, 60, 0)),                 # axle 4 along x for scale, below (y+ = down)
     dict(name='mot', part='68488', col=15, M=T(200, 0, 0))]
sc, cam, _ = cprev.setup(P, (1500, 700))
for nm, v in [('a', ((0.02, -0.14, 0.05), (0.02, 0.0, 0.0), 45)), ('b', ((0.02, 0.14, 0.05), (0.02, 0.0, 0.0), 45)), ('c', ((0.02, 0.0, 0.16), (0.02, 0.0, 0.0), 45))]:
    cprev.aim(cam, v); sc.render.filepath = f'/home/claude/factory/prev/look_{nm}.png'; bpy.ops.render.render(write_still=True)
