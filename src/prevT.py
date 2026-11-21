"""Workbench preview of the track-unit line (side A) over one robot cycle. usage: python3 prevT.py OUTDIR N [K0 K1]"""
import sys, os, numpy as np
sys.path.insert(0, '/home/claude/scripts')
import bpy, mathutils
import line3, tline, lblend, cprev

out, N = sys.argv[1], int(sys.argv[2])
k0, k1 = (int(sys.argv[3]), int(sys.argv[4])) if len(sys.argv) > 4 else (0, N)
os.makedirs(out, exist_ok=True)
LDU = 0.0004
def bl(p): return np.array([p[0], -p[2], -p[1]])*LDU
TOT = sum(d for _, d in line3.PHASES)
T0 = tline.T_LEAVE                                     # start the preview when the empty build wagon sets off
TS = [(T0 + TOT*(k + 0.5)/N) % TOT for k in range(N)]

def caption(t, ul):
    ta = t % TOT; tau = (ta - tline.T_LEAVE) % TOT
    for t0, t1, kind, dat in tline.SEQ:
        if t0 <= tau < t1:
            names = {'pin': 'S0 · hub pin into the build wagon\'s holder', 'vbeam': 'S1 · 13L beam onto the stub guides (and the pin)',
                     'hbeam': 'S2 · 9L beam with its pin', 'axles': 'S3 · drive + idler axles', 'driveset': 'S4 · drive set: 3 sprockets + chain (prefab)'}
            if kind == 'move': return 'Build wagon moves on' if tau > 1 else 'Empty build wagon heads back to S0'
            return names[dat] + (' (push)' if kind == 'push' else ' (crank back)')
    if tline.T_HAND <= ta < tline.T_HAND + 1.5: return 'Handover · build-wagon plungers slide the unit onto the track wagon\'s guides'
    if tline.T_HAND + 1.5 <= ta < tline.T_LEAVE: return 'Track wagon loaded · plungers back'
    if 19.0 <= ta < 26.0: return 'Track wagon reverses back to the lead-in'
    if 11.7 <= ta < 19.0: return 'Finished unit waits at the handover · supply train goes to be restocked'
    return 'Unit complete, waiting for the track wagon'

def cam_for(st, t):
    ul = st['tl']; ta = t % TOT
    bz = ul['bz']; tgt = np.array([-1660.0, -250.0, bz - 60]); off = np.array([-420.0, -950.0, -640.0]); lens = 38
    if bz < 2000:                                         # handover zone: show both wagons side by side
        tgt = np.array([-1600.0, -250.0, tline.ZREF - 40]); off = np.array([-380.0, -900.0, -700.0]); lens = 38
    return bl(tgt + off), bl(tgt), lens

names = {}
for t in np.linspace(0, TOT, 30, endpoint=False):
    for p in line3.build(line3.cycle(t/TOT, 'builder')): names.setdefault(p['name'], dict(p))
for k in range(k0, k1):
    for p in line3.build(line3.cycle(TS[k]/TOT, 'builder')): names.setdefault(p['name'], dict(p))
sc, cam, objs = cprev.setup(list(names.values()), (960, 540))
sc.display.render_aa = 'FXAA'; cam.data.clip_start = 0.005
for k in range(k0, k1):
    t = TS[k]; st = line3.cycle(t/TOT, 'builder'); P = {p['name']: p for p in line3.build(st)}
    pos, tgt, lens = cam_for(st, t)
    for n, ob in objs.items():
        p = P.get(n); vis = p is not None
        if vis: vis = np.linalg.norm(bl(p['M'][:3, 3]) - tgt) < 1.4
        ob.hide_render = not vis
        if vis: ob.matrix_world = lblend.M_bl(p['M'])
    cam.location = mathutils.Vector(pos); cam.rotation_euler = (mathutils.Vector(tgt) - mathutils.Vector(pos)).to_track_quat('-Z', 'Y').to_euler()
    cam.data.lens = lens
    sc.render.filepath = f'{out}/f{k:04d}.png'; bpy.ops.render.render(write_still=True)
    with open(f'{out}/f{k:04d}.txt', 'w') as fh: fh.write(caption(t, st['tl']))
    print('FRAME', k, flush=True)
