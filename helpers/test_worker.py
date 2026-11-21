import sys; sys.path.insert(0, '/home/claude/scripts')
import bpy, numpy as np, cprev, worker
from cell import T, R
P = worker.build(P=[])
P.append(dict(name='gun', part='15391c01', col=72, M=T(0, -120, -260)))
sc, cam, _ = cprev.setup(P, (1200, 900))
for nm, v in [('a', ((0.16, 0.20, 0.12), (0, 0.0, 0.03), 40)), ('b', ((-0.17, 0.12, 0.10), (0, 0.02, 0.03), 40))]:
    cprev.aim(cam, v); sc.render.filepath = f'/home/claude/factory/prev/worker_{nm}.png'; bpy.ops.render.render(write_still=True)
