"""Workbench animation of the cell cycle. Usage: python3 canim.py module out_dir nframes [cycles] [view] [res]"""
import sys, importlib, os, numpy as np
sys.path.insert(0, '/home/claude/scripts')
import bpy, mathutils, lblend, cprev
mod = importlib.import_module(sys.argv[1]); out = sys.argv[2]; N = int(sys.argv[3])
cycles = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
view = sys.argv[5] if len(sys.argv) > 5 else 'iso'
res = tuple(map(int, sys.argv[6].split('x'))) if len(sys.argv) > 6 else (1280, 720)
os.makedirs(out, exist_ok=True)
states = [mod.cycle(k/N*cycles) for k in range(N)]
# belt phase continuous across cycles
for k, st in enumerate(states): st['belt'] = st['belt'] + 400*int(k/N*cycles)
union = {}
for st in states[::max(1, N//60)] + states[-1:]:
    for p in mod.build(st): union.setdefault(p['name'], p)
sc, cam, objs = cprev.setup(list(union.values()), res)
cprev.aim(cam, view if view in cprev.VIEWS else eval(view))
for k, st in enumerate(states):
    P = {p['name']: p for p in mod.build(st)}
    for n, ob in objs.items():
        if n in P:
            ob.hide_render = False; ob.matrix_world = lblend.M_bl(P[n]['M'])
        else: ob.hide_render = True
    sc.render.filepath = f'{out}/f{k:04d}.png'
    bpy.ops.render.render(write_still=True)
    print('FRAME', k, st['phase'], flush=True)
