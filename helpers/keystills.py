import sys; sys.path.insert(0, '/home/claude/scripts')
import bpy, cell, cprev, lblend
union = {}
ts = [0.12, 0.33, 0.68, 0.92]
for t in ts:
    for p in cell.build(cell.cycle(t)): union.setdefault(p['name'], p)
sc, cam, objs = cprev.setup(list(union.values()), (1280, 800))
for t in ts:
    P = {p['name']: p for p in cell.build(cell.cycle(t))}
    for n, ob in objs.items():
        ob.hide_render = n not in P
        if n in P: ob.matrix_world = lblend.M_bl(P[n]['M'])
    for v in ('iso', 'close'):
        cprev.aim(cam, v); sc.render.filepath = f'/home/claude/factory/prev/key_{t:.2f}_{v}.png'; bpy.ops.render.render(write_still=True)
