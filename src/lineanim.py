"""Looping Workbench animation: the head-station line (3 modules) with worker v2 robots watching.
Usage: python3 lineanim.py out_dir nframes [res]"""
import sys, os, numpy as np
sys.path.insert(0, '/home/claude/scripts')
import bpy, mathutils, lblend, cprev, cell, worker2
from cell import T, R
out = sys.argv[1]; N = int(sys.argv[2]); res = tuple(map(int, sys.argv[3].split('x'))) if len(sys.argv) > 3 else (800, 450)
CYC = 2.0
os.makedirs(out, exist_ok=True)
CONV = ('base', 'w1_', 'w2_', 'w3_', 'w4_', 'w5_', 'w6_', 'ax_idl', 'ax_drv', 'bush_', 'spr_', 'tread', 'g24_', 'motC', 'ax_mC', 'g8_mC', 'bed')
WORKERS = [(-600, -500, 180, 70), (560, -520, 180, 25), (-380, 520, 0, 60), (420, 500, 0, 85)]   # x, z, yaw, arm angle

def scene(k):
    t = k/N*CYC; st = cell.cycle(t % 1.0); st['belt'] += 400*int(t)
    P = cell.build(st)
    station = list(P)
    for dx in (-640, 640):                                  # bare conveyor modules up/downstream
        for p in station:
            if p['name'].startswith(CONV):
                q = dict(p); q['name'] = f"{p['name']}@{dx}"; q['M'] = T(dx, 0, 0) @ p['M']; P.append(q)
    for kk in (-2, -1, 1, 2):                               # pallets queued before / leaving after the station
        x = st['pallet_x'] + 400*kk
        n0 = len(cell.P); cell.pallet(x, body=f'pal{kk}', head=kk > 0)
        for p in cell.P[n0:]: p['name'] += f'_k{kk}'
    P = cell.P
    for zf in (-640, 640):
        for dx in (-640, 0, 640): P.append(dict(name=f'floor{zf}_{dx}', part='3811', col=71, M=T(dx, 0, zf), body='frame'))
    for i, (x, z, yaw, arm) in enumerate(WORKERS):
        a = arm + (12*np.sin(2*np.pi*t/CYC*2 + i) if i == 1 else 0)          # one worker points and waves a little
        worker2.build(M0=T(x, -worker2.GROUND, z) @ R('y', yaw), tag=f'wk{i}', P=P, arm=np.radians(a), track_phase=0.0)
    return P

union = {}
for k in range(0, N, max(1, N//24)):
    for p in scene(k): union.setdefault(p['name'], p)
sc, cam, objs = cprev.setup(list(union.values()), res)
sc.display.shading.background_color = (0.80, 0.82, 0.86); sc.display.render_aa = '5'
FR = [int(x) for x in sys.argv[4].split(',')] if len(sys.argv) > 4 else range(N)
for k in FR:
    t = k/N
    P = {p['name']: p for p in scene(k)}
    for n, ob in objs.items():
        if n in P: ob.hide_render = False; ob.matrix_world = lblend.M_bl(P[n]['M'])
        else: ob.hide_render = True
    phi = np.radians(38)*np.sin(2*np.pi*t)                   # slow seamless swing around the front
    r, el = 0.74, np.radians(37)
    tgt = mathutils.Vector((0.0, 0.03, 0.05))
    loc = tgt + mathutils.Vector((r*np.cos(el)*np.sin(phi), r*np.cos(el)*np.cos(phi), r*np.sin(el)))
    cam.location = loc; cam.rotation_euler = (tgt - loc).to_track_quat('-Z', 'Y').to_euler(); cam.data.lens = 32
    sc.render.filepath = f'{out}/f{k:04d}.png'
    bpy.ops.render.render(write_still=True)
    print('FRAME', k, flush=True)
