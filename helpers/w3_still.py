import sys; sys.path.insert(0, '/home/claude/scripts')
import bpy, numpy as np, cprev, worker2, worker3
from cell import T, R
G = worker2.GROUND
P = []
worker3.build('loader', M0=T(-230, -G, 0) @ R('y', 18), tag='L', P=P, lift=38)
worker3.build('guard', M0=T(230, -G, 0) @ R('y', -18), tag='G', P=P)
for dx in (-320, 320): P.append(dict(name=f'floor{dx}', part='3811', col=71, M=T(dx, 4, 0)))
sc, cam, _ = cprev.setup(P, (1600, 1100))
sc.display.shading.background_color = (0.80, 0.82, 0.86)
for nm, v in [('front', ((-0.05, 0.50, 0.20), (0.0, 0.0, 0.085), 42)), ('side', ((0.42, 0.22, 0.20), (0.03, 0.0, 0.09), 42)), ('back', ((0.10, -0.46, 0.24), (0.0, 0.0, 0.09), 42))]:
    cprev.aim(cam, v); sc.render.filepath = f'/home/claude/factory/prev/w3_{nm}.png'; bpy.ops.render.render(write_still=True)
